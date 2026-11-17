use chip_tools::{CoolantMode, CuttingPreset, Tool, ToolId, ToolOperationClass};
use serde::{Deserialize, Serialize};

use crate::{heights::OperationHeights, selection::GeometrySelectionItem};

/// The 5 core v1 2.5D CAM operation types (`05-operations.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OperationKind {
    Facing,
    #[serde(rename = "contour_2d")]
    Contour2D,
    #[serde(rename = "pocket_2d")]
    Pocket2D,
    DrillBore,
    ChamferTrace,
}

impl OperationKind {
    pub const ALL: [Self; 5] = [
        Self::Facing,
        Self::Contour2D,
        Self::Pocket2D,
        Self::DrillBore,
        Self::ChamferTrace,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::Facing => "Facing",
            Self::Contour2D => "2D Contour / Profile",
            Self::Pocket2D => "2D Pocket",
            Self::DrillBore => "Drill / Helical Bore",
            Self::ChamferTrace => "Edge Chamfer / Trace",
        }
    }

    pub const fn default_name(self) -> &'static str {
        match self {
            Self::Facing => "Face Top",
            Self::Contour2D => "2D Profile",
            Self::Pocket2D => "2D Pocket",
            Self::DrillBore => "Holes / Bore",
            Self::ChamferTrace => "Edge Chamfer",
        }
    }

    pub fn is_compatible_with_tool(self, tool: &Tool) -> bool {
        match self {
            Self::Facing => tool.geometry.supports_operation(ToolOperationClass::Facing),
            Self::Contour2D => tool
                .geometry
                .supports_operation(ToolOperationClass::Contour2D),
            Self::Pocket2D => tool
                .geometry
                .supports_operation(ToolOperationClass::Pocket2D),
            Self::DrillBore => {
                tool.geometry.supports_operation(ToolOperationClass::Drill)
                    || tool
                        .geometry
                        .supports_operation(ToolOperationClass::HelicalBore)
            }
            Self::ChamferTrace => {
                tool.geometry
                    .supports_operation(ToolOperationClass::Chamfer)
                    || tool.geometry.supports_operation(ToolOperationClass::Trace)
            }
        }
    }
}

/// Runtime dirty/clean lifecycle state of an [`Operation`] (`05-operations.md` & `08-ui-viewport.md`).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum OperationStatus {
    #[default]
    Dirty,
    Generating,
    Clean,
    Error {
        message: String,
    },
    NeedsRepick {
        message: String,
    },
}

impl OperationStatus {
    pub const fn badge_icon(&self) -> &'static str {
        match self {
            Self::Clean => "[OK]",
            Self::Dirty => "[*]",
            Self::Generating => "[..]",
            Self::Error { .. } | Self::NeedsRepick { .. } => "[!]",
        }
    }
}

/// Optional per-operation overrides on top of a [`Tool`]'s [`CuttingPreset`] (`04-tool-library.md`).
/// Any `None` field inherits dynamically from the selected preset; `Some(val)` marks an explicit override.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct OperationFeedOverrides {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spindle_rpm: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cutting_feed_mm_min: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lead_feed_mm_min: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ramp_feed_mm_min: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub plunge_feed_mm_min: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stepdown_mm: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stepover_mm: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub coolant: Option<CoolantMode>,
}

impl OperationFeedOverrides {
    pub fn is_all_inherited(&self) -> bool {
        self.spindle_rpm.is_none()
            && self.cutting_feed_mm_min.is_none()
            && self.lead_feed_mm_min.is_none()
            && self.ramp_feed_mm_min.is_none()
            && self.plunge_feed_mm_min.is_none()
            && self.stepdown_mm.is_none()
            && self.stepover_mm.is_none()
            && self.coolant.is_none()
    }

    pub fn clear_all(&mut self) {
        *self = Self::default();
    }
}

/// Resolved cutting parameters combining a [`Tool`] + [`CuttingPreset`] + [`OperationFeedOverrides`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EffectiveCuttingParams {
    pub spindle_rpm: f64,
    pub cutting_feed_mm_min: f64,
    pub lead_feed_mm_min: f64,
    pub ramp_feed_mm_min: f64,
    pub plunge_feed_mm_min: f64,
    pub stepdown_mm: f64,
    pub stepover_mm: f64,
    pub coolant: CoolantMode,
}

/// Parametric 2.5D machining operation belonging to a [`crate::setup::Setup`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Operation {
    pub id: String,
    pub name: String,
    pub kind: OperationKind,
    #[serde(default)]
    pub suppressed: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool_id: Option<ToolId>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub preset_id: Option<String>,
    #[serde(default)]
    pub feed_overrides: OperationFeedOverrides,
    #[serde(default)]
    pub selections: Vec<GeometrySelectionItem>,
    #[serde(default)]
    pub heights: OperationHeights,
    #[serde(skip)]
    pub runtime_status: OperationStatus,
}

impl Operation {
    pub fn new(id: impl Into<String>, name: impl Into<String>, kind: OperationKind) -> Self {
        Self {
            id: id.into(),
            name: name.into(),
            kind,
            suppressed: false,
            tool_id: None,
            preset_id: None,
            feed_overrides: OperationFeedOverrides::default(),
            selections: Vec::new(),
            heights: OperationHeights::default(),
            runtime_status: OperationStatus::Dirty,
        }
    }

    pub fn mark_dirty(&mut self) {
        self.runtime_status = OperationStatus::Dirty;
    }

    /// Resolves effective feeds, speeds, stepdown, stepover, and coolant from `tool` + `preset_id` + `feed_overrides`.
    pub fn effective_cutting_params(&self, tool: &Tool) -> EffectiveCuttingParams {
        let preset: Option<&CuttingPreset> = self
            .preset_id
            .as_deref()
            .and_then(|pid| tool.presets.iter().find(|p| p.id == pid))
            .or_else(|| tool.presets.first());

        let d = tool.geometry.cutting_diameter_mm();
        let (rpm, feed, lead, ramp, plunge, ap, ae, p_cool) = match preset {
            Some(p) => (
                p.spindle_rpm,
                p.cutting_feed_mm_min,
                p.lead_feed_mm_min,
                p.ramp_feed_mm_min,
                p.plunge_feed_mm_min,
                p.default_stepdown_mm,
                p.default_stepover_mm,
                p.coolant_override,
            ),
            None => (10_000.0, 800.0, 640.0, 400.0, 260.0, 0.5 * d, 0.4 * d, None),
        };

        EffectiveCuttingParams {
            spindle_rpm: self.feed_overrides.spindle_rpm.unwrap_or(rpm),
            cutting_feed_mm_min: self.feed_overrides.cutting_feed_mm_min.unwrap_or(feed),
            lead_feed_mm_min: self.feed_overrides.lead_feed_mm_min.unwrap_or(lead),
            ramp_feed_mm_min: self.feed_overrides.ramp_feed_mm_min.unwrap_or(ramp),
            plunge_feed_mm_min: self.feed_overrides.plunge_feed_mm_min.unwrap_or(plunge),
            stepdown_mm: self.feed_overrides.stepdown_mm.unwrap_or(ap),
            stepover_mm: self.feed_overrides.stepover_mm.unwrap_or(ae),
            coolant: self
                .feed_overrides
                .coolant
                .or(p_cool)
                .unwrap_or(tool.default_coolant),
        }
    }
}
