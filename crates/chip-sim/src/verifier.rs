use chip_math::DVec3;
use serde::{Deserialize, Serialize};

/// Severity level of a simulation diagnostic (`06-simulation.md` §4).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SimSeverity {
    /// Blocks `chipcutter build` and GUI G-code export unless explicitly overridden.
    CriticalError,
    /// Non-blocking machining advisory (shank rub or aggressive vertical plunge).
    Warning,
}

impl SimSeverity {
    pub const fn badge(self) -> &'static str {
        match self {
            Self::CriticalError => "[!]",
            Self::Warning => "[WARN]",
        }
    }
}

/// Specific safety rule triggered during simulation verification (`06-simulation.md` §4).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SimIssueKind {
    /// `G0` rapid move removes stock deeper than rapid tolerance.
    RapidIntoStock,
    /// Cutting or rapid move penetrates below the target STEP B-Rep upper surface (`> gouge_tol`).
    TargetModelGouge,
    /// Engagement depth exceeds `tool.flute_length_mm` (non-cutting shank rub).
    FluteLengthExceeded,
    /// Uncut stock protrudes above `z_tip + tool.stickout_length_mm` within holder/collet radius.
    HolderCollision,
    /// Vertical plunge (`ΔXY == 0`) into material with non-center-cutting tool or excessive depth.
    UnsafeVerticalPlunge,
    /// Full cutter assembly (flute, shank, or collet/holder) collided with an enabled 3D fixture.
    FixtureCollision,
}

impl SimIssueKind {
    pub const fn label(self) -> &'static str {
        match self {
            Self::RapidIntoStock => "Rapid-into-Stock Crash",
            Self::TargetModelGouge => "Target Model Gouge",
            Self::FluteLengthExceeded => "Flute Length Exceeded (Shank Rub)",
            Self::HolderCollision => "Holder / Collet Collision",
            Self::UnsafeVerticalPlunge => "Excessive Vertical Plunge",
            Self::FixtureCollision => "Fixture / Clamp Collision",
        }
    }

    pub const fn default_severity(self) -> SimSeverity {
        match self {
            Self::RapidIntoStock
            | Self::TargetModelGouge
            | Self::HolderCollision
            | Self::FixtureCollision => SimSeverity::CriticalError,
            Self::FluteLengthExceeded | Self::UnsafeVerticalPlunge => SimSeverity::Warning,
        }
    }
}

/// Structured simulation diagnostic clickable in the UI to jump the camera + timeline scrubber
/// straight to the offending motion span (`06-simulation.md` §4).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SimDiagnostic {
    pub severity: SimSeverity,
    pub kind: SimIssueKind,
    pub op_idx: usize,
    pub op_id: String,
    pub op_name: String,
    pub tool_number: u32,
    pub global_seg_idx: usize,
    pub timeline_time_sec: f64,
    pub location_wcs: DVec3,
    /// Worst violation magnitude in millimeters (crash depth, gouge depth, shank rub, or plunge excess).
    pub max_excess_mm: f64,
    pub message: String,
    /// Offending workholding fixture ID when `kind == SimIssueKind::FixtureCollision`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fixture_id: Option<String>,
    /// Human-readable workholding fixture name when `kind == SimIssueKind::FixtureCollision`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fixture_name: Option<String>,
}

/// Tolerances governing automated crash & gouge detection (`06-simulation.md`).
///
/// Unified with [`chip_stock::StockVerifyTolerances`] so verification thresholds have a single
/// canonical definition across `chip-stock`, `chip-sim`, `chip-project`, and `chip-gui`.
pub type SimVerificationConfig = chip_stock::StockVerifyTolerances;
