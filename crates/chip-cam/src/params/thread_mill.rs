use chip_math::{EvalContext, ParamLength};
use serde::{Deserialize, Serialize};

use super::common::{default_one_u32, default_true, default_zero_len};

/// Internal bore vs. external boss thread form (`OperationKind::ThreadMill`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ThreadFormKind {
    #[default]
    InternalHole,
    ExternalBoss,
}

impl ThreadFormKind {
    pub const ALL: [Self; 2] = [Self::InternalHole, Self::ExternalBoss];

    pub const fn label(self) -> &'static str {
        match self {
            Self::InternalHole => "Internal (Hole)",
            Self::ExternalBoss => "External (Boss)",
        }
    }
}

/// Thread helix handedness (`RightHand` vs `LeftHand`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ThreadHandedness {
    #[default]
    RightHand,
    LeftHand,
}

impl ThreadHandedness {
    pub const ALL: [Self; 2] = [Self::RightHand, Self::LeftHand];

    pub const fn label(self) -> &'static str {
        match self {
            Self::RightHand => "Right-Hand (RH)",
            Self::LeftHand => "Left-Hand (LH)",
        }
    }
}

/// Cutting engagement direction for helical thread milling (`Climb` vs `Conventional`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ThreadCutDirection {
    #[default]
    Climb,
    Conventional,
}

impl ThreadCutDirection {
    pub const ALL: [Self; 2] = [Self::Climb, Self::Conventional];

    pub const fn label(self) -> &'static str {
        match self {
            Self::Climb => "Climb (Down-Milling)",
            Self::Conventional => "Conventional (Up-Milling)",
        }
    }
}

fn default_thread_standard_preset() -> Option<String> {
    Some("M8 x 1.25".to_string())
}

fn default_thread_major_diameter() -> ParamLength {
    ParamLength::from_mm(8.0)
}

fn default_thread_pitch() -> ParamLength {
    ParamLength::from_mm(1.25)
}

fn default_thread_lead_clearance() -> ParamLength {
    ParamLength::from_mm(0.5)
}

/// Strategy parameters for **Helical Thread Milling** (`OperationKind::ThreadMill`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ThreadMillParams {
    #[serde(default)]
    pub form: ThreadFormKind,
    #[serde(default)]
    pub handedness: ThreadHandedness,
    #[serde(default)]
    pub cut_direction: ThreadCutDirection,
    #[serde(
        default = "default_thread_standard_preset",
        skip_serializing_if = "Option::is_none"
    )]
    pub standard_preset: Option<String>,
    #[serde(default = "default_thread_major_diameter")]
    pub major_diameter: ParamLength,
    #[serde(default = "default_thread_pitch")]
    pub pitch: ParamLength,
    #[serde(default = "default_zero_len")]
    pub pitch_diameter_offset: ParamLength,
    #[serde(default = "default_one_u32")]
    pub radial_passes: u32,
    #[serde(default)]
    pub spring_passes: u32,
    #[serde(default = "default_thread_lead_clearance")]
    pub lead_in_clearance: ParamLength,
    #[serde(default = "default_true")]
    pub compensate_centerline_feed: bool,
    #[serde(default = "default_true")]
    pub optimize_hole_order: bool,
}

impl Default for ThreadMillParams {
    fn default() -> Self {
        Self {
            form: ThreadFormKind::InternalHole,
            handedness: ThreadHandedness::RightHand,
            cut_direction: ThreadCutDirection::Climb,
            standard_preset: default_thread_standard_preset(),
            major_diameter: default_thread_major_diameter(),
            pitch: default_thread_pitch(),
            pitch_diameter_offset: default_zero_len(),
            radial_passes: 1,
            spring_passes: 0,
            lead_in_clearance: default_thread_lead_clearance(),
            compensate_centerline_feed: true,
            optimize_hole_order: true,
        }
    }
}

impl ThreadMillParams {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        let _ = self.major_diameter.reevaluate(ctx);
        let _ = self.pitch.reevaluate(ctx);
        let _ = self.pitch_diameter_offset.reevaluate(ctx);
        let _ = self.lead_in_clearance.reevaluate(ctx);
    }
}
