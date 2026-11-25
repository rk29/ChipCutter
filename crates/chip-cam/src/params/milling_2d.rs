use chip_math::{DVec3, EvalContext, ParamLength};
use serde::{Deserialize, Serialize};

use crate::cl_data::DrillCycleKind;

use super::{
    common::{default_one_u32, default_ramp_angle_deg, default_true, default_zero_len, param_expr},
    surface_3d::{MachiningBoundaryConfig, MachiningBoundaryMode, ToolContainmentMode},
};

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
    #[serde(flatten)]
    pub boundary: MachiningBoundaryConfig,
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
            boundary: MachiningBoundaryConfig::new(
                MachiningBoundaryMode::StockBox,
                ToolContainmentMode::ToolOutsideBoundary,
                0.0,
            ),
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
        self.boundary.reevaluate(ctx);
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
    pub clip_to_stock_bounds: bool,
    #[serde(default = "default_zero_len")]
    pub open_chain_extension: ParamLength,
    #[serde(default)]
    pub tabs: HoldingTabsConfig,
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
            clip_to_stock_bounds: false,
            open_chain_extension: default_zero_len(),
            tabs: HoldingTabsConfig::default(),
        }
    }
}

impl Contour2DParams {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        let _ = self.roughing_stepover.reevaluate(ctx);
        let _ = self.finish_stepover.reevaluate(ctx);
        let _ = self.open_chain_extension.reevaluate(ctx);
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

/// 2D Pocket / Adaptive boundary interpretation (`05-operations.md` §3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PocketBoundaryMode {
    /// Clear inside selected planar faces or closed 2D loops, preserving any raised islands.
    #[default]
    SelectedPockets,
    /// Treat selected planar faces or closed 2D loops as raised bosses (islands) to leave standing,
    /// and clear all stock material outside them up to the Setup Stock XY boundary with open-air entry.
    ClearOutsideSelectedToStock,
}

impl PocketBoundaryMode {
    pub const ALL: [Self; 2] = [Self::SelectedPockets, Self::ClearOutsideSelectedToStock];

    pub const fn label(self) -> &'static str {
        match self {
            Self::SelectedPockets => "Inside Selected Pockets",
            Self::ClearOutsideSelectedToStock => "Outside Selected Bosses (To Stock)",
        }
    }
}

/// Strategy parameters for **2D Pocket** (`05-operations.md` §3).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Pocket2DParams {
    #[serde(default)]
    pub boundary_mode: PocketBoundaryMode,
    #[serde(default)]
    pub treat_outer_boundary_as_open: bool,
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
            boundary_mode: PocketBoundaryMode::SelectedPockets,
            treat_outer_boundary_as_open: false,
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
    /// Scale helical bore center feeds by `(r_orbit / R_hole).clamp(0.20, 1.0)` so actual
    /// cutting-edge wall chipload matches the programmed feedrate.
    /// Defaults to `false` on legacy TOML deserialization for golden parity; enabled by
    /// default on newly created `DrillBore` operations via `Operation::new`.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub bore_inner_arc_feed_comp: bool,
    /// Optional axial Z depth interval between full retracts to `r_plane_z` for deep
    /// helical bore chip evacuation (`None` = continuous helix without intermediate retracts).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bore_chip_clear_depth: Option<ParamLength>,
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
            bore_inner_arc_feed_comp: false,
            bore_chip_clear_depth: None,
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
        if let Some(d) = &mut self.bore_chip_clear_depth {
            let _ = d.reevaluate(ctx);
        }
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
