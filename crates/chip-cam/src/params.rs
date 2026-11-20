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

/// Area-clearing strategy for Stage 1 of a flat-bottom two-tool [`VCarveParams`] (`05-operations.md` §6).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VCarveClearingKind {
    #[default]
    ContourParallel,
    Adaptive,
}

impl VCarveClearingKind {
    pub const ALL: [Self; 2] = [Self::ContourParallel, Self::Adaptive];

    pub const fn label(self) -> &'static str {
        match self {
            Self::ContourParallel => "Contour-Parallel Offset",
            Self::Adaptive => "2D Adaptive Trochoidal",
        }
    }
}

/// Optional V-Inlay male plug generation mode (`05-operations.md` §6C).
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum VCarveInlayMode {
    #[default]
    Disabled,
    MalePlug {
        start_depth: ParamLength,
        glue_gap_bottom: ParamLength,
        border_margin: ParamLength,
        #[serde(default = "default_true")]
        auto_mirror_x: bool,
    },
}

impl VCarveInlayMode {
    pub const fn is_male_plug(&self) -> bool {
        matches!(self, Self::MalePlug { .. })
    }
}

fn default_vcarve_clearing_allowance() -> ParamLength {
    ParamLength::from_mm(0.05)
}

fn default_vbit_flat_stepover() -> ParamLength {
    ParamLength::from_mm(0.5)
}

/// Strategy parameters for **V-Carving, Flat-Bottom Clearance, & V-Inlays** (`05-operations.md` §6).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VCarveParams {
    /// Optional maximum flat floor depth below `Top Height` (`None` = pure unlimited medial V-carve).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub flat_depth: Option<ParamLength>,
    /// Optional secondary **Flat Endmill** for Stage 1 wide flat-bottom pocket clearing.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub clearing_tool_id: Option<chip_tools::ToolId>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub clearing_preset_id: Option<String>,
    #[serde(default)]
    pub clearing_strategy: VCarveClearingKind,
    #[serde(default = "default_vcarve_clearing_allowance")]
    pub clearing_radial_allowance: ParamLength,
    #[serde(default = "default_vbit_flat_stepover")]
    pub vbit_flat_stepover: ParamLength,
    #[serde(default)]
    pub inlay_mode: VCarveInlayMode,
}

impl Default for VCarveParams {
    fn default() -> Self {
        Self {
            flat_depth: Some(ParamLength::from_mm(2.0)),
            clearing_tool_id: None,
            clearing_preset_id: None,
            clearing_strategy: VCarveClearingKind::ContourParallel,
            clearing_radial_allowance: default_vcarve_clearing_allowance(),
            vbit_flat_stepover: default_vbit_flat_stepover(),
            inlay_mode: VCarveInlayMode::Disabled,
        }
    }
}

impl VCarveParams {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        if let Some(fd) = &mut self.flat_depth {
            let _ = fd.reevaluate(ctx);
        }
        let _ = self.clearing_radial_allowance.reevaluate(ctx);
        let _ = self.vbit_flat_stepover.reevaluate(ctx);
        if let VCarveInlayMode::MalePlug {
            start_depth,
            glue_gap_bottom,
            border_margin,
            ..
        } = &mut self.inlay_mode
        {
            let _ = start_depth.reevaluate(ctx);
            let _ = glue_gap_bottom.reevaluate(ctx);
            let _ = border_margin.reevaluate(ctx);
        }
    }
}

/// 2D XY machining containment boundary source for 3-axis surface operations (`Waterline3D`, `Adaptive3D`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MachiningBoundaryMode {
    #[default]
    StockBox,
    ModelSilhouette,
    SelectedRegions,
}

impl MachiningBoundaryMode {
    pub const ALL: [Self; 3] = [Self::StockBox, Self::ModelSilhouette, Self::SelectedRegions];

    pub const fn label(self) -> &'static str {
        match self {
            Self::StockBox => "Stock Bounds",
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

fn default_zero_length() -> ParamLength {
    ParamLength::from_mm(0.0)
}

/// Strategy parameters for **3D Waterline / Z-Level Machining** (`OperationKind::Waterline3D`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Waterline3DParams {
    #[serde(default)]
    pub boundary_mode: MachiningBoundaryMode,
    #[serde(default)]
    pub tool_containment: ToolContainmentMode,
    #[serde(default = "default_zero_length")]
    pub boundary_offset: ParamLength,
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
            boundary_mode: MachiningBoundaryMode::ModelSilhouette,
            tool_containment: ToolContainmentMode::ToolOutsideBoundary,
            boundary_offset: ParamLength::from_mm(0.5),
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
        let _ = self.boundary_offset.reevaluate(ctx);
        let _ = self.min_stepdown.reevaluate(ctx);
        let _ = self.max_cusp_height.reevaluate(ctx);
    }
}

/// Strategy parameters for **3D Adaptive Clearing / Roughing** (`OperationKind::Adaptive3D`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Adaptive3DParams {
    #[serde(default)]
    pub boundary_mode: MachiningBoundaryMode,
    #[serde(default)]
    pub tool_containment: ToolContainmentMode,
    #[serde(default = "default_zero_length")]
    pub boundary_offset: ParamLength,
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
            boundary_mode: MachiningBoundaryMode::StockBox,
            tool_containment: ToolContainmentMode::ToolCenterOnBoundary,
            boundary_offset: ParamLength::from_mm(0.5),
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
        let _ = self.boundary_offset.reevaluate(ctx);
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

fn default_finishing_cusp_height() -> ParamLength {
    ParamLength::from_mm(0.025)
}

fn default_parallel3d_sample_step() -> ParamLength {
    ParamLength::from_mm(0.35)
}

fn default_finishing_smooth_tol() -> ParamLength {
    ParamLength::from_mm(0.005)
}

/// Strategy parameters for **3D Parallel / Raster Finishing** (`OperationKind::Parallel3D`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Parallel3DParams {
    #[serde(default)]
    pub boundary_mode: MachiningBoundaryMode,
    #[serde(default)]
    pub tool_containment: ToolContainmentMode,
    #[serde(default = "default_zero_length")]
    pub boundary_offset: ParamLength,
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
            boundary_mode: MachiningBoundaryMode::ModelSilhouette,
            tool_containment: ToolContainmentMode::ToolCenterOnBoundary,
            boundary_offset: ParamLength::from_mm(0.0),
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
        let _ = self.boundary_offset.reevaluate(ctx);
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
    #[serde(default)]
    pub boundary_mode: MachiningBoundaryMode,
    #[serde(default)]
    pub tool_containment: ToolContainmentMode,
    #[serde(default = "default_zero_length")]
    pub boundary_offset: ParamLength,
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
            boundary_mode: MachiningBoundaryMode::ModelSilhouette,
            tool_containment: ToolContainmentMode::ToolCenterOnBoundary,
            boundary_offset: ParamLength::from_mm(0.0),
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
        let _ = self.boundary_offset.reevaluate(ctx);
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
    #[serde(default)]
    pub boundary_mode: MachiningBoundaryMode,
    #[serde(default)]
    pub tool_containment: ToolContainmentMode,
    #[serde(default = "default_zero_length")]
    pub boundary_offset: ParamLength,
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
            boundary_mode: MachiningBoundaryMode::ModelSilhouette,
            tool_containment: ToolContainmentMode::ToolCenterOnBoundary,
            boundary_offset: ParamLength::from_mm(0.0),
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
        let _ = self.boundary_offset.reevaluate(ctx);
        let _ = self.reference_tool_diameter.reevaluate(ctx);
        let _ = self.valley_threshold.reevaluate(ctx);
        let _ = self.offset_stepover.reevaluate(ctx);
        let _ = self.smoothing_tolerance.reevaluate(ctx);
    }
}

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
    #[serde(rename = "vcarve")]
    VCarve(VCarveParams),
    #[serde(rename = "waterline_3d")]
    Waterline3D(Waterline3DParams),
    #[serde(rename = "adaptive_3d")]
    Adaptive3D(Adaptive3DParams),
    #[serde(rename = "parallel_3d")]
    Parallel3D(Parallel3DParams),
    #[serde(rename = "scallop_3d")]
    Scallop3D(Scallop3DParams),
    #[serde(rename = "pencil_3d")]
    Pencil3D(Pencil3DParams),
    #[serde(rename = "rotary_2linear")]
    Rotary2Linear(Rotary2LinearParams),
    #[serde(rename = "rotary_4axis_sim")]
    Rotary4AxisSim(Rotary4AxisSimParams),
}

impl StrategyParams {
    pub fn default_for_kind(kind: OperationKind) -> Self {
        match kind {
            OperationKind::Facing => Self::Facing(FacingParams::default()),
            OperationKind::Contour2D => Self::Contour2D(Contour2DParams::default()),
            OperationKind::Pocket2D => Self::Pocket2D(Pocket2DParams::default()),
            OperationKind::DrillBore => Self::DrillBore(DrillBoreParams::default()),
            OperationKind::ChamferTrace => Self::ChamferTrace(ChamferTraceParams::default()),
            OperationKind::VCarve => Self::VCarve(VCarveParams::default()),
            OperationKind::Waterline3D => Self::Waterline3D(Waterline3DParams::default()),
            OperationKind::Adaptive3D => Self::Adaptive3D(Adaptive3DParams::default()),
            OperationKind::Parallel3D => Self::Parallel3D(Parallel3DParams::default()),
            OperationKind::Scallop3D => Self::Scallop3D(Scallop3DParams::default()),
            OperationKind::Pencil3D => Self::Pencil3D(Pencil3DParams::default()),
            OperationKind::Rotary2Linear => Self::Rotary2Linear(Rotary2LinearParams::default()),
            OperationKind::Rotary4AxisSim => Self::Rotary4AxisSim(Rotary4AxisSimParams::default()),
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
                | (OperationKind::VCarve, Self::VCarve(_))
                | (OperationKind::Waterline3D, Self::Waterline3D(_))
                | (OperationKind::Adaptive3D, Self::Adaptive3D(_))
                | (OperationKind::Parallel3D, Self::Parallel3D(_))
                | (OperationKind::Scallop3D, Self::Scallop3D(_))
                | (OperationKind::Pencil3D, Self::Pencil3D(_))
                | (OperationKind::Rotary2Linear, Self::Rotary2Linear(_))
                | (OperationKind::Rotary4AxisSim, Self::Rotary4AxisSim(_))
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
            Self::VCarve(p) => p.reevaluate(ctx),
            Self::Waterline3D(p) => p.reevaluate(ctx),
            Self::Adaptive3D(p) => p.reevaluate(ctx),
            Self::Parallel3D(p) => p.reevaluate(ctx),
            Self::Scallop3D(p) => p.reevaluate(ctx),
            Self::Pencil3D(p) => p.reevaluate(ctx),
            Self::Rotary2Linear(p) => p.reevaluate(ctx),
            Self::Rotary4AxisSim(p) => p.reevaluate(ctx),
        }
    }
}
