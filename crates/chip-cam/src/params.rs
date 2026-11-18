use chip_math::{DEFAULT_ARC_FIT_TOL_MM, DVec2, DVec3, EvalContext, ParamLength};
use serde::{Deserialize, Serialize};

use crate::{cl_data::DrillCycleKind, operation::OperationKind};

fn param_expr(expr: &str, default_mm: f64) -> ParamLength {
    ParamLength {
        expr: Some(expr.to_string()),
        value_mm: default_mm,
    }
}

fn default_true() -> bool {
    true
}

fn default_tolerance_mm() -> f64 {
    DEFAULT_ARC_FIT_TOL_MM
}

fn default_ramp_angle_deg() -> f64 {
    2.5
}

fn default_lead_sweep_deg() -> f64 {
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

/// Common multi-depth stepdown, stock-to-leave, Rest Machining, and tolerance parameters (`05-operations.md`).
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
    #[serde(default = "default_tolerance_mm")]
    pub tolerance_mm: f64,
}

fn default_max_stepdown() -> ParamLength {
    param_expr("tool_d * 0.5", 3.0)
}

fn default_zero_len() -> ParamLength {
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

/// Facing cut direction policy (`05-operations.md` §1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FacingCutMode {
    #[default]
    BothWaysZigZag,
    OneWayClimb,
    OneWayConventional,
}

impl FacingCutMode {
    pub const ALL: [Self; 3] = [
        Self::BothWaysZigZag,
        Self::OneWayClimb,
        Self::OneWayConventional,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::BothWaysZigZag => "Both Ways (Zig-Zag + U-Arcs)",
            Self::OneWayClimb => "One Way (Climb)",
            Self::OneWayConventional => "One Way (Conventional)",
        }
    }
}

/// Strategy parameters for **Facing** (`05-operations.md` §1).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FacingParams {
    #[serde(default)]
    pub pass_direction_deg: f64,
    #[serde(default = "default_true")]
    pub auto_longest_axis: bool,
    #[serde(default = "default_facing_stepover")]
    pub stepover: ParamLength,
    #[serde(default = "default_facing_ext")]
    pub pass_extension: ParamLength,
    #[serde(default)]
    pub cut_mode: FacingCutMode,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub finishing_stepdown: Option<ParamLength>,
}

fn default_facing_stepover() -> ParamLength {
    param_expr("tool_d * 0.7", 4.2)
}

fn default_facing_ext() -> ParamLength {
    param_expr("tool_d * 0.55", 3.3)
}

impl Default for FacingParams {
    fn default() -> Self {
        Self {
            pass_direction_deg: 0.0,
            auto_longest_axis: true,
            stepover: default_facing_stepover(),
            pass_extension: default_facing_ext(),
            cut_mode: FacingCutMode::BothWaysZigZag,
            finishing_stepdown: None,
        }
    }
}

impl FacingParams {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        let _ = self.stepover.reevaluate(ctx);
        let _ = self.pass_extension.reevaluate(ctx);
        if let Some(fs) = &mut self.finishing_stepdown {
            let _ = fs.reevaluate(ctx);
        }
    }
}

/// Computer cutter compensation side (`05-operations.md` §2).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ContourCompensation {
    #[default]
    LeftClimb,
    RightConventional,
    CenterOff,
}

impl ContourCompensation {
    pub const ALL: [Self; 3] = [Self::LeftClimb, Self::RightConventional, Self::CenterOff];

    pub const fn label(self) -> &'static str {
        match self {
            Self::LeftClimb => "Left (Climb Milling)",
            Self::RightConventional => "Right (Conventional)",
            Self::CenterOff => "Center (On Curve)",
        }
    }
}

/// Automatic vs. manual interactive placement of holding tabs (`05-operations.md` §2).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum TabPlacementMode {
    Automatic {
        #[serde(default = "default_tab_count")]
        count: u32,
        #[serde(default)]
        by_spacing: bool,
        #[serde(default = "default_tab_spacing")]
        target_spacing: ParamLength,
    },
    Manual,
}

fn default_tab_count() -> u32 {
    4
}

fn default_tab_spacing() -> ParamLength {
    ParamLength::from_mm(60.0)
}

impl Default for TabPlacementMode {
    fn default() -> Self {
        Self::Automatic {
            count: 4,
            by_spacing: false,
            target_spacing: default_tab_spacing(),
        }
    }
}

/// Persisted manual holding-tab anchor placed interactively on a 3D contour edge (`05-operations.md` §2).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ManualTabAnchor {
    pub contour_index: usize,
    pub s_norm: f64,
    pub world_point_mm: DVec3,
}

/// Cross-sectional Z profile of a holding tab bridge (`05-operations.md` §2).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "shape", rename_all = "snake_case")]
pub enum TabProfileKind {
    Rectangular,
    Triangular {
        #[serde(default = "default_tab_ramp_deg")]
        ramp_angle_deg: f64,
    },
}

fn default_tab_ramp_deg() -> f64 {
    25.0
}

impl Default for TabProfileKind {
    fn default() -> Self {
        Self::Triangular {
            ramp_angle_deg: default_tab_ramp_deg(),
        }
    }
}

/// Holding tab configuration on a 2D Contour operation (`05-operations.md` §2).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HoldingTabsConfig {
    #[serde(default)]
    pub enabled: bool,
    #[serde(default)]
    pub placement: TabPlacementMode,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub manual_anchors: Vec<ManualTabAnchor>,
    #[serde(default)]
    pub profile: TabProfileKind,
    #[serde(default = "default_tab_width")]
    pub width: ParamLength,
    #[serde(default = "default_tab_height")]
    pub height: ParamLength,
}

fn default_tab_width() -> ParamLength {
    ParamLength::from_mm(6.0)
}

fn default_tab_height() -> ParamLength {
    ParamLength::from_mm(1.6)
}

impl Default for HoldingTabsConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            placement: TabPlacementMode::default(),
            manual_anchors: Vec::new(),
            profile: TabProfileKind::default(),
            width: default_tab_width(),
            height: default_tab_height(),
        }
    }
}

impl HoldingTabsConfig {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        if let TabPlacementMode::Automatic { target_spacing, .. } = &mut self.placement {
            let _ = target_spacing.reevaluate(ctx);
        }
        let _ = self.width.reevaluate(ctx);
        let _ = self.height.reevaluate(ctx);
    }
}

/// Strategy parameters for **2D Contour / Profile** (`05-operations.md` §2).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Contour2DParams {
    #[serde(default)]
    pub compensation: ContourCompensation,
    #[serde(default)]
    pub wear_comp_metadata: bool,
    #[serde(default)]
    pub continuous_spiral_ramp: bool,
    #[serde(default = "default_one_u32")]
    pub roughing_passes: u32,
    #[serde(default = "default_contour_rough_stepover")]
    pub roughing_stepover: ParamLength,
    #[serde(default)]
    pub finish_passes: u32,
    #[serde(default = "default_finish_stepover")]
    pub finish_stepover: ParamLength,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub finish_feed_mm_min: Option<f64>,
    #[serde(default)]
    pub repeat_finish_pass: bool,
    #[serde(default)]
    pub tabs: HoldingTabsConfig,
}

fn default_one_u32() -> u32 {
    1
}

fn default_contour_rough_stepover() -> ParamLength {
    param_expr("tool_d * 0.45", 2.7)
}

fn default_finish_stepover() -> ParamLength {
    ParamLength::from_mm(0.2)
}

impl Default for Contour2DParams {
    fn default() -> Self {
        Self {
            compensation: ContourCompensation::LeftClimb,
            wear_comp_metadata: false,
            continuous_spiral_ramp: false,
            roughing_passes: 1,
            roughing_stepover: default_contour_rough_stepover(),
            finish_passes: 0,
            finish_stepover: default_finish_stepover(),
            finish_feed_mm_min: None,
            repeat_finish_pass: false,
            tabs: HoldingTabsConfig::default(),
        }
    }
}

impl Contour2DParams {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        let _ = self.roughing_stepover.reevaluate(ctx);
        let _ = self.finish_stepover.reevaluate(ctx);
        self.tabs.reevaluate(ctx);
    }
}

/// Cut direction policy for **2D Adaptive / Trochoidal Clearing** (`05-operations.md` §3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AdaptiveCutDirection {
    #[default]
    Climb,
    Conventional,
    BothWays,
}

impl AdaptiveCutDirection {
    pub const ALL: [Self; 3] = [Self::Climb, Self::Conventional, Self::BothWays];

    pub const fn label(self) -> &'static str {
        match self {
            Self::Climb => "Climb (Down-Milling)",
            Self::Conventional => "Conventional (Up-Milling)",
            Self::BothWays => "Both Ways (Alternating)",
        }
    }
}

fn default_adaptive_optimal_load() -> ParamLength {
    param_expr("tool_d * 0.2", 1.2)
}

fn default_micro_lift_z() -> ParamLength {
    ParamLength::from_mm(0.25)
}

fn default_no_eng_feed_mult() -> f64 {
    2.5
}

fn default_min_trochoid_r() -> ParamLength {
    param_expr("tool_d * 0.1", 0.6)
}

fn default_max_thinning_mult() -> f64 {
    2.0
}

/// Constant-engagement **2D Adaptive / Trochoidal Clearing** parameters (`05-operations.md` §3).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AdaptiveClearingParams {
    #[serde(default = "default_adaptive_optimal_load")]
    pub optimal_load: ParamLength,
    #[serde(default)]
    pub cut_direction: AdaptiveCutDirection,
    #[serde(default = "default_micro_lift_z")]
    pub micro_lift_z: ParamLength,
    #[serde(default = "default_no_eng_feed_mult")]
    pub no_engagement_feed_multiplier: f64,
    #[serde(default = "default_min_trochoid_r")]
    pub min_trochoid_radius: ParamLength,
    #[serde(default = "default_true")]
    pub auto_chip_thinning: bool,
    #[serde(default = "default_max_thinning_mult")]
    pub max_thinning_feed_multiplier: f64,
}

impl Default for AdaptiveClearingParams {
    fn default() -> Self {
        Self {
            optimal_load: default_adaptive_optimal_load(),
            cut_direction: AdaptiveCutDirection::Climb,
            micro_lift_z: default_micro_lift_z(),
            no_engagement_feed_multiplier: default_no_eng_feed_mult(),
            min_trochoid_radius: default_min_trochoid_r(),
            auto_chip_thinning: true,
            max_thinning_feed_multiplier: default_max_thinning_mult(),
        }
    }
}

impl AdaptiveClearingParams {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        let _ = self.optimal_load.reevaluate(ctx);
        let _ = self.micro_lift_z.reevaluate(ctx);
        let _ = self.min_trochoid_radius.reevaluate(ctx);
    }
}

/// 2D Pocket clearing algorithm selector (`05-operations.md` §3).
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum PocketStrategyKind {
    #[default]
    ContourParallel,
    ZigZag {
        #[serde(default)]
        pass_angle_deg: f64,
    },
    Adaptive(AdaptiveClearingParams),
}

impl PocketStrategyKind {
    pub const fn label(&self) -> &'static str {
        match self {
            Self::ContourParallel => "Contour-Parallel Offset",
            Self::ZigZag { .. } => "Zig-Zag / Raster",
            Self::Adaptive(_) => "2D Adaptive / Trochoidal",
        }
    }
}

/// Ring ordering direction for Contour-Parallel pocketing (`05-operations.md` §3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PocketPassOrder {
    #[default]
    Auto,
    InsideOut,
    OutsideIn,
}

impl PocketPassOrder {
    pub const ALL: [Self; 3] = [Self::Auto, Self::InsideOut, Self::OutsideIn];

    pub const fn label(self) -> &'static str {
        match self {
            Self::Auto => "Automatic (Inside-Out Closed / Outside-In Open)",
            Self::InsideOut => "Inside -> Out",
            Self::OutsideIn => "Outside -> In",
        }
    }
}

/// Strategy parameters for **2D Pocket** (`05-operations.md` §3).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Pocket2DParams {
    #[serde(default)]
    pub strategy: PocketStrategyKind,
    #[serde(default = "default_pocket_stepover")]
    pub stepover: ParamLength,
    #[serde(default)]
    pub pass_order: PocketPassOrder,
    #[serde(default = "default_open_overhang")]
    pub open_edge_overhang: ParamLength,
    #[serde(default = "default_true")]
    pub finish_wall_pass: bool,
    #[serde(default = "default_finish_stepover")]
    pub finish_wall_stepover: ParamLength,
}

fn default_pocket_stepover() -> ParamLength {
    param_expr("tool_d * 0.4", 2.4)
}

fn default_open_overhang() -> ParamLength {
    ParamLength::from_mm(0.8)
}

impl Default for Pocket2DParams {
    fn default() -> Self {
        Self {
            strategy: PocketStrategyKind::ContourParallel,
            stepover: default_pocket_stepover(),
            pass_order: PocketPassOrder::Auto,
            open_edge_overhang: default_open_overhang(),
            finish_wall_pass: true,
            finish_wall_stepover: default_finish_stepover(),
        }
    }
}

impl Pocket2DParams {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        if let PocketStrategyKind::Adaptive(adaptive) = &mut self.strategy {
            adaptive.reevaluate(ctx);
        }
        let _ = self.stepover.reevaluate(ctx);
        let _ = self.open_edge_overhang.reevaluate(ctx);
        let _ = self.finish_wall_stepover.reevaluate(ctx);
    }
}

/// Hole machining mode selector (`05-operations.md` §4).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HoleMachiningMode {
    #[default]
    AutoByTool,
    Drill,
    HelicalBore,
}

impl HoleMachiningMode {
    pub const ALL: [Self; 3] = [Self::AutoByTool, Self::Drill, Self::HelicalBore];

    pub const fn label(self) -> &'static str {
        match self {
            Self::AutoByTool => "Automatic (Drill if D==Hole / Helical Bore if Endmill D<Hole)",
            Self::Drill => "Drilling Cycle",
            Self::HelicalBore => "Helical Bore Milling",
        }
    }
}

/// Strategy parameters for **Drilling & Helical Bore Milling** (`05-operations.md` §4).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DrillBoreParams {
    #[serde(default)]
    pub mode: HoleMachiningMode,
    #[serde(default)]
    pub cycle_kind: DrillCycleKind,
    #[serde(default = "default_peck_depth")]
    pub peck_depth: ParamLength,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub decreasing_peck_factor: Option<f64>,
    #[serde(default = "default_min_peck")]
    pub min_peck_depth: ParamLength,
    #[serde(default = "default_chip_break_retract")]
    pub chip_break_retract: ParamLength,
    #[serde(default)]
    pub dwell_seconds: f64,
    #[serde(default = "default_true")]
    pub tip_breakthrough: bool,
    #[serde(default = "default_breakthrough_extra")]
    pub breakthrough_extra: ParamLength,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bore_pitch_per_rev: Option<ParamLength>,
    #[serde(default = "default_ramp_angle_deg")]
    pub bore_ramp_angle_deg: f64,
    #[serde(default = "default_true")]
    pub bore_finish_pass: bool,
    #[serde(default = "default_bore_finish_stepover")]
    pub bore_finish_stepover: ParamLength,
    #[serde(default)]
    pub bore_spring_pass: bool,
    #[serde(default = "default_true")]
    pub optimize_hole_order: bool,
}

fn default_peck_depth() -> ParamLength {
    param_expr("tool_d * 0.8", 4.0)
}

fn default_min_peck() -> ParamLength {
    ParamLength::from_mm(0.5)
}

fn default_chip_break_retract() -> ParamLength {
    ParamLength::from_mm(0.5)
}

fn default_breakthrough_extra() -> ParamLength {
    ParamLength::from_mm(0.5)
}

fn default_bore_finish_stepover() -> ParamLength {
    ParamLength::from_mm(0.15)
}

impl Default for DrillBoreParams {
    fn default() -> Self {
        Self {
            mode: HoleMachiningMode::AutoByTool,
            cycle_kind: DrillCycleKind::DeepPeck,
            peck_depth: default_peck_depth(),
            decreasing_peck_factor: None,
            min_peck_depth: default_min_peck(),
            chip_break_retract: default_chip_break_retract(),
            dwell_seconds: 0.0,
            tip_breakthrough: true,
            breakthrough_extra: default_breakthrough_extra(),
            bore_pitch_per_rev: None,
            bore_ramp_angle_deg: default_ramp_angle_deg(),
            bore_finish_pass: true,
            bore_finish_stepover: default_bore_finish_stepover(),
            bore_spring_pass: false,
            optimize_hole_order: true,
        }
    }
}

impl DrillBoreParams {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        let _ = self.peck_depth.reevaluate(ctx);
        let _ = self.min_peck_depth.reevaluate(ctx);
        let _ = self.chip_break_retract.reevaluate(ctx);
        let _ = self.breakthrough_extra.reevaluate(ctx);
        if let Some(p) = &mut self.bore_pitch_per_rev {
            let _ = p.reevaluate(ctx);
        }
        let _ = self.bore_finish_stepover.reevaluate(ctx);
    }
}

/// Sub-mode for **Edge Chamfering & 2D Curve Trace** (`05-operations.md` §5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ChamferTraceMode {
    #[default]
    EdgeChamfer,
    CurveTrace,
}

impl ChamferTraceMode {
    pub const ALL: [Self; 2] = [Self::EdgeChamfer, Self::CurveTrace];

    pub const fn label(self) -> &'static str {
        match self {
            Self::EdgeChamfer => "Edge Chamfering",
            Self::CurveTrace => "2D Curve Trace / Engraving",
        }
    }
}

/// Sharp CAD edge vs. modeled chamfer face selection mode (`05-operations.md` §5A).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "source", rename_all = "snake_case")]
pub enum ChamferSourceMode {
    SharpEdge {
        #[serde(default = "default_chamfer_width")]
        width: ParamLength,
    },
    ModeledFace,
}

fn default_chamfer_width() -> ParamLength {
    ParamLength::from_mm(0.5)
}

impl Default for ChamferSourceMode {
    fn default() -> Self {
        Self::SharpEdge {
            width: default_chamfer_width(),
        }
    }
}

/// Strategy parameters for **Edge Chamfering & 2D Curve Trace** (`05-operations.md` §5).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ChamferTraceParams {
    #[serde(default)]
    pub mode: ChamferTraceMode,
    #[serde(default)]
    pub chamfer_source: ChamferSourceMode,
    #[serde(default = "default_tip_offset")]
    pub tip_offset: ParamLength,
    #[serde(default = "default_true")]
    pub check_adjacent_walls: bool,
    #[serde(default = "default_trace_comp")]
    pub trace_compensation: ContourCompensation,
    #[serde(default = "default_zero_len")]
    pub trace_axial_offset: ParamLength,
}

fn default_tip_offset() -> ParamLength {
    ParamLength::from_mm(0.8)
}

fn default_trace_comp() -> ContourCompensation {
    ContourCompensation::CenterOff
}

impl Default for ChamferTraceParams {
    fn default() -> Self {
        Self {
            mode: ChamferTraceMode::EdgeChamfer,
            chamfer_source: ChamferSourceMode::default(),
            tip_offset: default_tip_offset(),
            check_adjacent_walls: true,
            trace_compensation: ContourCompensation::CenterOff,
            trace_axial_offset: default_zero_len(),
        }
    }
}

impl ChamferTraceParams {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        if let ChamferSourceMode::SharpEdge { width } = &mut self.chamfer_source {
            let _ = width.reevaluate(ctx);
        }
        let _ = self.tip_offset.reevaluate(ctx);
        let _ = self.trace_axial_offset.reevaluate(ctx);
    }
}

/// Strategy-specific parameter block inside [`crate::operation::Operation`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "strategy_kind", rename_all = "snake_case")]
pub enum StrategyParams {
    Facing(FacingParams),
    #[serde(rename = "contour_2d")]
    Contour2D(Contour2DParams),
    #[serde(rename = "pocket_2d")]
    Pocket2D(Pocket2DParams),
    DrillBore(DrillBoreParams),
    ChamferTrace(ChamferTraceParams),
}

impl StrategyParams {
    pub fn default_for_kind(kind: OperationKind) -> Self {
        match kind {
            OperationKind::Facing => Self::Facing(FacingParams::default()),
            OperationKind::Contour2D => Self::Contour2D(Contour2DParams::default()),
            OperationKind::Pocket2D => Self::Pocket2D(Pocket2DParams::default()),
            OperationKind::DrillBore => Self::DrillBore(DrillBoreParams::default()),
            OperationKind::ChamferTrace => Self::ChamferTrace(ChamferTraceParams::default()),
        }
    }

    pub fn ensure_matches_kind(&mut self, kind: OperationKind) {
        let matches = matches!(
            (kind, &self),
            (OperationKind::Facing, Self::Facing(_))
                | (OperationKind::Contour2D, Self::Contour2D(_))
                | (OperationKind::Pocket2D, Self::Pocket2D(_))
                | (OperationKind::DrillBore, Self::DrillBore(_))
                | (OperationKind::ChamferTrace, Self::ChamferTrace(_))
        );
        if !matches {
            *self = Self::default_for_kind(kind);
        }
    }

    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        match self {
            Self::Facing(p) => p.reevaluate(ctx),
            Self::Contour2D(p) => p.reevaluate(ctx),
            Self::Pocket2D(p) => p.reevaluate(ctx),
            Self::DrillBore(p) => p.reevaluate(ctx),
            Self::ChamferTrace(p) => p.reevaluate(ctx),
        }
    }
}
