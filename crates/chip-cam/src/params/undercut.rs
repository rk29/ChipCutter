use chip_math::{EvalContext, ParamLength};
use serde::{Deserialize, Serialize};

use super::surface_3d::{MachiningBoundaryConfig, MachiningBoundaryMode, ToolContainmentMode};

fn default_true() -> bool {
    true
}

fn default_slot_width() -> ParamLength {
    ParamLength::from_mm(14.5)
}

fn default_throat_width() -> ParamLength {
    ParamLength::from_mm(8.5)
}

fn default_slot_height() -> ParamLength {
    ParamLength::from_mm(6.0)
}

fn default_slot_radial_stepover() -> ParamLength {
    ParamLength::from_mm(0.8)
}

fn default_slot_axial_stepdown() -> ParamLength {
    ParamLength {
        expr: Some("flute_len * 0.8".to_string()),
        value_mm: 3.2,
    }
}

fn default_slot_finish_allowance() -> ParamLength {
    ParamLength::from_mm(0.15)
}

fn default_entry_extension() -> ParamLength {
    ParamLength {
        expr: Some("tool_r + 2.0".to_string()),
        value_mm: 8.0,
    }
}

fn default_dovetail_top_throat_width() -> ParamLength {
    ParamLength::from_mm(8.5)
}

fn default_dovetail_depth() -> ParamLength {
    ParamLength::from_mm(5.0)
}

fn default_dovetail_side_angle_deg() -> f64 {
    60.0
}

fn default_dovetail_radial_stepover() -> ParamLength {
    ParamLength::from_mm(0.6)
}

fn default_dovetail_finish_allowance() -> ParamLength {
    ParamLength::from_mm(0.1)
}

fn default_undercut3d_z_stepdown() -> ParamLength {
    ParamLength::from_mm(0.4)
}

fn default_undercut3d_target_cusp() -> ParamLength {
    ParamLength::from_mm(0.025)
}

fn default_min_overhang_angle_deg() -> f64 {
    5.0
}

fn default_neck_clearance() -> ParamLength {
    ParamLength::from_mm(0.25)
}

fn default_zero_length() -> ParamLength {
    ParamLength::from_mm(0.0)
}

fn default_lead_out_distance() -> ParamLength {
    ParamLength {
        expr: Some("tool_r + 1.0".to_string()),
        value_mm: 4.0,
    }
}

/// Safe entry & exit mode for undercut slot and dovetail operations (`SlotUndercut`, `Dovetail`).
///
/// Undercut tools MUST NEVER plunge or retract vertically beneath an overhanging roof.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum UndercutEntryMode {
    /// Descend in open air past an open slot end (`entry_extension`), cut horizontally, return
    /// laterally to throat centerline, and exit horizontally back out the open end before `+Z` retract.
    #[default]
    OpenEnd,
    /// Descend vertically strictly inside a verified pre-cleared entry bore (`bore_dia >= cutter_dia + 0.5 mm`),
    /// cut horizontally, return to centerline & entry bore center, then retract `+Z`.
    EntryBoreCenter,
    /// Enter in open air past the start end and exit in open air past the opposite through-end
    /// (both ends verified clear of the stock/roof).
    BothEndsThrough,
}

impl UndercutEntryMode {
    pub const ALL: [Self; 3] = [Self::OpenEnd, Self::EntryBoreCenter, Self::BothEndsThrough];

    pub const fn label(self) -> &'static str {
        match self {
            Self::OpenEnd => "Open End (Enter & Exit Same End)",
            Self::EntryBoreCenter => "Pre-Cleared Entry Bore",
            Self::BothEndsThrough => "Both Ends Through (Open Air)",
        }
    }
}

/// Geometric family machined by `OperationKind::SlotUndercut`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum SlotUndercutMode {
    /// Inverted-T undercut slot below a narrower pre-cleared vertical throat.
    #[default]
    TSlot,
    /// Precision straight shaft/hub keyway (`DIN 6885` / Woodruff / parallel keyway).
    Keyway,
    /// Open-sided horizontal groove / O-ring / retaining-ring side undercut along an exterior wall.
    SideGroove,
}

impl SlotUndercutMode {
    pub const ALL: [Self; 3] = [Self::TSlot, Self::Keyway, Self::SideGroove];

    pub const fn label(self) -> &'static str {
        match self {
            Self::TSlot => "T-Slot (Undercut)",
            Self::Keyway => "Keyway (Parallel Slot)",
            Self::SideGroove => "Side Groove (Open Side)",
        }
    }
}

/// Standard metric (DIN 650 / DIN 6885) and imperial (ASME B5.1) T-slot and keyway presets.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum SlotUndercutPreset {
    #[default]
    Custom,
    TSlotM6,
    TSlotM8,
    TSlotM10,
    TSlotQuarterIn,
    TSlotThreeEighthIn,
    KeywayDin6885_4mm,
    KeywayDin6885_6mm,
    KeywayDin6885_8mm,
}

/// Canonical millimeter dimensions for a standard [`SlotUndercutPreset`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SlotUndercutPresetSpec {
    pub mode: SlotUndercutMode,
    pub slot_width_mm: f64,
    pub throat_width_mm: f64,
    pub slot_height_mm: f64,
}

impl SlotUndercutPreset {
    pub const ALL: [Self; 9] = [
        Self::Custom,
        Self::TSlotM6,
        Self::TSlotM8,
        Self::TSlotM10,
        Self::TSlotQuarterIn,
        Self::TSlotThreeEighthIn,
        Self::KeywayDin6885_4mm,
        Self::KeywayDin6885_6mm,
        Self::KeywayDin6885_8mm,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::Custom => "Custom Dimensions",
            Self::TSlotM6 => "DIN 650 M6 T-Slot (11.5 × 5.0 / 6.5 throat)",
            Self::TSlotM8 => "DIN 650 M8 T-Slot (14.5 × 6.0 / 8.5 throat)",
            Self::TSlotM10 => "DIN 650 M10 T-Slot (18.0 × 7.5 / 10.5 throat)",
            Self::TSlotQuarterIn => "ASME 1/4\" T-Slot (12.7 × 5.56 / 7.14 throat)",
            Self::TSlotThreeEighthIn => "ASME 3/8\" T-Slot (17.46 × 7.14 / 10.32 throat)",
            Self::KeywayDin6885_4mm => "DIN 6885 4 mm Keyway (4.0 × 2.5 mm)",
            Self::KeywayDin6885_6mm => "DIN 6885 6 mm Keyway (6.0 × 3.5 mm)",
            Self::KeywayDin6885_8mm => "DIN 6885 8 mm Keyway (8.0 × 4.0 mm)",
        }
    }

    pub const fn spec(self) -> Option<SlotUndercutPresetSpec> {
        match self {
            Self::Custom => None,
            Self::TSlotM6 => Some(SlotUndercutPresetSpec {
                mode: SlotUndercutMode::TSlot,
                slot_width_mm: 11.5,
                throat_width_mm: 6.5,
                slot_height_mm: 5.0,
            }),
            Self::TSlotM8 => Some(SlotUndercutPresetSpec {
                mode: SlotUndercutMode::TSlot,
                slot_width_mm: 14.5,
                throat_width_mm: 8.5,
                slot_height_mm: 6.0,
            }),
            Self::TSlotM10 => Some(SlotUndercutPresetSpec {
                mode: SlotUndercutMode::TSlot,
                slot_width_mm: 18.0,
                throat_width_mm: 10.5,
                slot_height_mm: 7.5,
            }),
            Self::TSlotQuarterIn => Some(SlotUndercutPresetSpec {
                mode: SlotUndercutMode::TSlot,
                slot_width_mm: 12.7,
                throat_width_mm: 7.14,
                slot_height_mm: 5.56,
            }),
            Self::TSlotThreeEighthIn => Some(SlotUndercutPresetSpec {
                mode: SlotUndercutMode::TSlot,
                slot_width_mm: 17.46,
                throat_width_mm: 10.32,
                slot_height_mm: 7.14,
            }),
            Self::KeywayDin6885_4mm => Some(SlotUndercutPresetSpec {
                mode: SlotUndercutMode::Keyway,
                slot_width_mm: 4.0,
                throat_width_mm: 4.0,
                slot_height_mm: 2.5,
            }),
            Self::KeywayDin6885_6mm => Some(SlotUndercutPresetSpec {
                mode: SlotUndercutMode::Keyway,
                slot_width_mm: 6.0,
                throat_width_mm: 6.0,
                slot_height_mm: 3.5,
            }),
            Self::KeywayDin6885_8mm => Some(SlotUndercutPresetSpec {
                mode: SlotUndercutMode::Keyway,
                slot_width_mm: 8.0,
                throat_width_mm: 8.0,
                slot_height_mm: 4.0,
            }),
        }
    }
}

/// Strategy parameters for **Slot / Keyway / T-Slot Undercut Milling** (`OperationKind::SlotUndercut`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SlotUndercutParams {
    #[serde(default)]
    pub mode: SlotUndercutMode,
    #[serde(default)]
    pub preset: SlotUndercutPreset,
    /// Total undercut head width (`mm`, e.g. `14.5 mm` for M8 T-slot).
    #[serde(default = "default_slot_width")]
    pub slot_width: ParamLength,
    /// Pre-cleared upper vertical throat width (`mm`, e.g. `8.5 mm` for M8 T-slot).
    #[serde(default = "default_throat_width")]
    pub throat_width: ParamLength,
    /// Axial height of the undercut slot head (`mm`, e.g. `6.0 mm`).
    #[serde(default = "default_slot_height")]
    pub slot_height: ParamLength,
    /// Progressive lateral roughing stepover per side (`mm`, default `0.8 mm`).
    #[serde(default = "default_slot_radial_stepover")]
    pub radial_stepover: ParamLength,
    /// Vertical overlap step when `slot_height > cutting_thickness_mm` (default `flute_len * 0.8`).
    #[serde(default = "default_slot_axial_stepdown")]
    pub axial_stepdown: ParamLength,
    /// Lateral stock reserved for the final full-width finishing pass (`mm`, default `0.15 mm`).
    #[serde(default = "default_slot_finish_allowance")]
    pub finish_allowance: ParamLength,
    /// Append an extra zero-stock spring pass on both walls to eliminate tool deflection.
    #[serde(default)]
    pub spring_pass: bool,
    /// Safe entry/exit choreography guaranteeing zero vertical plunges/retracts under an undercut roof.
    #[serde(default)]
    pub entry_mode: UndercutEntryMode,
    /// Horizontal extension past open slot end(s) for vertical entry/retract in open air (`mm`, default `tool_r + 2.0`).
    #[serde(default = "default_entry_extension")]
    pub entry_extension: ParamLength,
    /// Prefer climb milling (`true`) vs. conventional milling (`false`) on both left & right walls.
    #[serde(default = "default_true")]
    pub climb_direction: bool,
}

impl Default for SlotUndercutParams {
    fn default() -> Self {
        Self {
            mode: SlotUndercutMode::TSlot,
            preset: SlotUndercutPreset::Custom,
            slot_width: default_slot_width(),
            throat_width: default_throat_width(),
            slot_height: default_slot_height(),
            radial_stepover: default_slot_radial_stepover(),
            axial_stepdown: default_slot_axial_stepdown(),
            finish_allowance: default_slot_finish_allowance(),
            spring_pass: false,
            entry_mode: UndercutEntryMode::OpenEnd,
            entry_extension: default_entry_extension(),
            climb_direction: true,
        }
    }
}

impl SlotUndercutParams {
    /// Applies a standard metric/imperial preset to `self`, updating mode and canonical dimensions.
    pub fn apply_preset(&mut self, preset: SlotUndercutPreset) {
        self.preset = preset;
        if let Some(spec) = preset.spec() {
            self.mode = spec.mode;
            self.slot_width = ParamLength::from_mm(spec.slot_width_mm);
            self.throat_width = ParamLength::from_mm(spec.throat_width_mm);
            self.slot_height = ParamLength::from_mm(spec.slot_height_mm);
        }
    }

    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        let _ = self.slot_width.reevaluate(ctx);
        let _ = self.throat_width.reevaluate(ctx);
        let _ = self.slot_height.reevaluate(ctx);
        let _ = self.radial_stepover.reevaluate(ctx);
        let _ = self.axial_stepdown.reevaluate(ctx);
        let _ = self.finish_allowance.reevaluate(ctx);
        let _ = self.entry_extension.reevaluate(ctx);
    }
}

/// Dovetail feature polarity (`InternalSlot` cavity vs. `ExternalRail` male guideway).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum DovetailMode {
    /// Internal female dovetail slot milled beneath a pre-cleared vertical throat.
    #[default]
    InternalSlot,
    /// External male dovetail rail / guideway milled from open air around an exterior profile.
    ExternalRail,
}

impl DovetailMode {
    pub const ALL: [Self; 2] = [Self::InternalSlot, Self::ExternalRail];

    pub const fn label(self) -> &'static str {
        match self {
            Self::InternalSlot => "Internal Dovetail Slot",
            Self::ExternalRail => "External Dovetail Rail",
        }
    }
}

/// Strategy parameters for **Dovetail Slot & Rail Milling** (`OperationKind::Dovetail`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DovetailParams {
    #[serde(default)]
    pub mode: DovetailMode,
    /// Pre-cleared upper throat width (`mm`, validated `> neck_diameter_mm`) for `InternalSlot`.
    #[serde(default = "default_dovetail_top_throat_width")]
    pub top_throat_width: ParamLength,
    /// Axial depth of the angled dovetail flank (`mm`, validated `<= cutter.cutting_height_mm`).
    #[serde(default = "default_dovetail_depth")]
    pub dovetail_depth: ParamLength,
    /// Included flank angle from horizontal floor (`deg`, default `60.0°`, validated within `±0.5°` of cutter).
    #[serde(default = "default_dovetail_side_angle_deg")]
    pub side_angle_deg: f64,
    /// Progressive lateral roughing stepover into the angled dovetail flank (`mm`, default `0.6 mm`).
    #[serde(default = "default_dovetail_radial_stepover")]
    pub radial_stepover: ParamLength,
    /// Lateral stock reserved for the final angled flank finishing pass (`mm`, default `0.1 mm`).
    #[serde(default = "default_dovetail_finish_allowance")]
    pub finish_allowance: ParamLength,
    /// Append an extra zero-stock spring pass along the final angled flank.
    #[serde(default)]
    pub spring_pass: bool,
    /// Safe entry/exit choreography guaranteeing zero vertical plunges/retracts through the dovetail roof.
    #[serde(default)]
    pub entry_mode: UndercutEntryMode,
    /// Horizontal extension past open slot end(s) or away from rail edge before `+Z` retract (`mm`).
    #[serde(default = "default_entry_extension")]
    pub entry_extension: ParamLength,
    /// Prefer climb milling (`true`) vs. conventional milling (`false`).
    #[serde(default = "default_true")]
    pub climb_direction: bool,
}

impl Default for DovetailParams {
    fn default() -> Self {
        Self {
            mode: DovetailMode::InternalSlot,
            top_throat_width: default_dovetail_top_throat_width(),
            dovetail_depth: default_dovetail_depth(),
            side_angle_deg: default_dovetail_side_angle_deg(),
            radial_stepover: default_dovetail_radial_stepover(),
            finish_allowance: default_dovetail_finish_allowance(),
            spring_pass: false,
            entry_mode: UndercutEntryMode::OpenEnd,
            entry_extension: default_entry_extension(),
            climb_direction: true,
        }
    }
}

impl DovetailParams {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        let _ = self.top_throat_width.reevaluate(ctx);
        let _ = self.dovetail_depth.reevaluate(ctx);
        let _ = self.radial_stepover.reevaluate(ctx);
        let _ = self.finish_allowance.reevaluate(ctx);
        let _ = self.entry_extension.reevaluate(ctx);
    }
}

fn default_undercut3d_boundary() -> MachiningBoundaryConfig {
    MachiningBoundaryConfig::new(
        MachiningBoundaryMode::ModelSilhouette,
        ToolContainmentMode::ToolOutsideBoundary,
        0.0,
    )
}

#[derive(Debug, Clone, Deserialize)]
struct Undercut3DParamsDe {
    #[serde(default)]
    boundary_mode: Option<MachiningBoundaryMode>,
    #[serde(default)]
    tool_containment: Option<ToolContainmentMode>,
    #[serde(default)]
    boundary_offset: Option<ParamLength>,
    #[serde(default)]
    silhouette_fill_holes: Option<bool>,
    #[serde(default = "default_undercut3d_z_stepdown")]
    z_stepdown: ParamLength,
    #[serde(default)]
    use_cusp_height: bool,
    #[serde(default = "default_undercut3d_target_cusp")]
    target_cusp: ParamLength,
    #[serde(default = "default_min_overhang_angle_deg")]
    min_overhang_angle_deg: f64,
    #[serde(default = "default_neck_clearance")]
    neck_clearance: ParamLength,
    #[serde(default = "default_zero_length")]
    stock_to_leave: ParamLength,
    #[serde(default = "default_lead_out_distance")]
    lead_out_distance: ParamLength,
    #[serde(default = "default_true")]
    climb_direction: bool,
}

impl From<Undercut3DParamsDe> for Undercut3DParams {
    fn from(raw: Undercut3DParamsDe) -> Self {
        let def = default_undercut3d_boundary();
        Self {
            boundary: MachiningBoundaryConfig {
                boundary_mode: raw.boundary_mode.unwrap_or(def.boundary_mode),
                tool_containment: raw.tool_containment.unwrap_or(def.tool_containment),
                boundary_offset: raw.boundary_offset.unwrap_or(def.boundary_offset),
                silhouette_fill_holes: raw
                    .silhouette_fill_holes
                    .unwrap_or(def.silhouette_fill_holes),
            },
            z_stepdown: raw.z_stepdown,
            use_cusp_height: raw.use_cusp_height,
            target_cusp: raw.target_cusp,
            min_overhang_angle_deg: raw.min_overhang_angle_deg,
            neck_clearance: raw.neck_clearance,
            stock_to_leave: raw.stock_to_leave,
            lead_out_distance: raw.lead_out_distance,
            climb_direction: raw.climb_direction,
        }
    }
}

/// Strategy parameters for **3D Lollipop Undercut Finishing** (`OperationKind::Undercut3D`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(from = "Undercut3DParamsDe")]
pub struct Undercut3DParams {
    #[serde(flatten)]
    pub boundary: MachiningBoundaryConfig,
    /// Constant horizontal Z-slice stepdown across overhanging surfaces (`mm`, default `0.4 mm`).
    #[serde(default = "default_undercut3d_z_stepdown")]
    pub z_stepdown: ParamLength,
    /// Derive Z-level stepdown automatically from `target_cusp` and Lollipop sphere radius (`R`).
    #[serde(default)]
    pub use_cusp_height: bool,
    /// Target spherical scallop / cusp height (`mm`, default `0.025 mm`) when `use_cusp_height = true`.
    #[serde(default = "default_undercut3d_target_cusp")]
    pub target_cusp: ParamLength,
    /// Minimum downward overhang angle below the horizontal equator (`deg`, default `5.0°`, selecting `n_z <= -sin(angle)`).
    #[serde(default = "default_min_overhang_angle_deg")]
    pub min_overhang_angle_deg: f64,
    /// Minimum radial safety clearance (`mm`, default `0.25 mm`) between the relieved neck/shank and overhanging ceilings above.
    #[serde(default = "default_neck_clearance")]
    pub neck_clearance: ParamLength,
    /// Uniform radial/normal stock allowance left on the overhanging surface (`mm`, default `0.0 mm`).
    #[serde(default = "default_zero_length")]
    pub stock_to_leave: ParamLength,
    /// Safe horizontal pull-away distance into open air away from the undercut wall before any `+Z` retract (`mm`).
    #[serde(default = "default_lead_out_distance")]
    pub lead_out_distance: ParamLength,
    /// Prefer climb milling (`true`) vs. conventional milling (`false`) along undercut contour passes.
    #[serde(default = "default_true")]
    pub climb_direction: bool,
}

impl Default for Undercut3DParams {
    fn default() -> Self {
        Self {
            boundary: default_undercut3d_boundary(),
            z_stepdown: default_undercut3d_z_stepdown(),
            use_cusp_height: false,
            target_cusp: default_undercut3d_target_cusp(),
            min_overhang_angle_deg: default_min_overhang_angle_deg(),
            neck_clearance: default_neck_clearance(),
            stock_to_leave: default_zero_length(),
            lead_out_distance: default_lead_out_distance(),
            climb_direction: true,
        }
    }
}

impl Undercut3DParams {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        self.boundary.reevaluate(ctx);
        let _ = self.z_stepdown.reevaluate(ctx);
        let _ = self.target_cusp.reevaluate(ctx);
        let _ = self.neck_clearance.reevaluate(ctx);
        let _ = self.stock_to_leave.reevaluate(ctx);
        let _ = self.lead_out_distance.reevaluate(ctx);
    }
}
