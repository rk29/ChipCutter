use chip_math::EvalContext;
use chip_tools::{CoolantMode, CuttingPreset, Tool, ToolId, ToolOperationClass};
use serde::{Deserialize, Serialize};

use crate::{
    heights::OperationHeights,
    params::{LinkingParams, PassesParams, StrategyParams},
    selection::GeometrySelectionItem,
};

/// Supported 2.5D and 3-axis CAM operation types (`05-operations.md`).
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
    #[serde(rename = "vcarve")]
    VCarve,
    #[serde(rename = "waterline_3d")]
    Waterline3D,
    #[serde(rename = "adaptive_3d")]
    Adaptive3D,
}

impl OperationKind {
    pub const ALL: [Self; 8] = [
        Self::Facing,
        Self::Contour2D,
        Self::Pocket2D,
        Self::DrillBore,
        Self::ChamferTrace,
        Self::VCarve,
        Self::Adaptive3D,
        Self::Waterline3D,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::Facing => "Facing",
            Self::Contour2D => "2D Contour / Profile",
            Self::Pocket2D => "2D Pocket",
            Self::DrillBore => "Drill / Helical Bore",
            Self::ChamferTrace => "Edge Chamfer / Trace",
            Self::VCarve => "V-Carve / Bevel Inlay",
            Self::Waterline3D => "3D Waterline / Z-Level",
            Self::Adaptive3D => "3D Adaptive Roughing",
        }
    }

    pub const fn default_name(self) -> &'static str {
        match self {
            Self::Facing => "Face Top",
            Self::Contour2D => "2D Profile",
            Self::Pocket2D => "2D Pocket",
            Self::DrillBore => "Holes / Bore",
            Self::ChamferTrace => "Edge Chamfer",
            Self::VCarve => "V-Carve",
            Self::Waterline3D => "3D Waterline",
            Self::Adaptive3D => "3D Adaptive",
        }
    }

    /// Returns `true` for 3-axis surface operations operating on the full 3D model / mesh.
    pub const fn is_3d_surface(self) -> bool {
        matches!(self, Self::Waterline3D | Self::Adaptive3D)
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
            Self::VCarve => tool.geometry.supports_operation(ToolOperationClass::VCarve),
            Self::Waterline3D => tool
                .geometry
                .supports_operation(ToolOperationClass::Waterline3D),
            Self::Adaptive3D => tool
                .geometry
                .supports_operation(ToolOperationClass::Adaptive3D),
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
    #[serde(default)]
    pub passes: PassesParams,
    #[serde(default)]
    pub linking: LinkingParams,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub strategy: Option<StrategyParams>,
    #[serde(skip)]
    pub runtime_status: OperationStatus,
}

impl Operation {
    pub fn new(id: impl Into<String>, name: impl Into<String>, kind: OperationKind) -> Self {
        let mut heights = OperationHeights::default();
        let mut passes = PassesParams::default();
        match kind {
            OperationKind::Adaptive3D => {
                heights.top =
                    crate::heights::HeightSpec::new(crate::heights::HeightReference::StockTop, 0.0);
                heights.bottom = crate::heights::HeightSpec::new(
                    crate::heights::HeightReference::ModelBottom,
                    0.0,
                );
                passes.max_stepdown = chip_math::ParamLength::from_mm(5.0);
                passes.stock_to_leave_radial = chip_math::ParamLength::from_mm(0.5);
                passes.stock_to_leave_axial = chip_math::ParamLength::from_mm(0.3);
            }
            OperationKind::Waterline3D => {
                heights.top =
                    crate::heights::HeightSpec::new(crate::heights::HeightReference::ModelTop, 0.0);
                heights.bottom = crate::heights::HeightSpec::new(
                    crate::heights::HeightReference::ModelBottom,
                    0.0,
                );
                passes.max_stepdown = chip_math::ParamLength::from_mm(1.25);
            }
            _ => {}
        }
        Self {
            id: id.into(),
            name: name.into(),
            kind,
            suppressed: false,
            tool_id: None,
            preset_id: None,
            feed_overrides: OperationFeedOverrides::default(),
            selections: Vec::new(),
            heights,
            passes,
            linking: LinkingParams::default(),
            strategy: Some(StrategyParams::default_for_kind(kind)),
            runtime_status: OperationStatus::Dirty,
        }
    }

    /// Creates a **2D Adaptive Clearing** operation (`OperationKind::Pocket2D` pre-configured
    /// with `PocketStrategyKind::Adaptive`, deep axial engagement `tool_d * 1.5`, and `tool_d * 0.20` radial load).
    pub fn new_adaptive_2d(id: impl Into<String>, name: impl Into<String>) -> Self {
        let mut op = Self::new(id, name, OperationKind::Pocket2D);
        op.passes.max_stepdown = chip_math::ParamLength {
            expr: Some("tool_d * 1.5".to_string()),
            value_mm: 9.0,
        };
        op.strategy = Some(StrategyParams::Pocket2D(crate::params::Pocket2DParams {
            strategy: crate::params::PocketStrategyKind::Adaptive(
                crate::params::AdaptiveClearingParams::default(),
            ),
            ..crate::params::Pocket2DParams::default()
        }));
        op
    }

    /// Returns `true` when this operation is a `2D Pocket` using `PocketStrategyKind::Adaptive`.
    pub fn is_adaptive_2d(&self) -> bool {
        matches!(
            &self.strategy,
            Some(StrategyParams::Pocket2D(p))
                if matches!(p.strategy, crate::params::PocketStrategyKind::Adaptive(_))
        )
    }

    /// Returns `true` when 2.5D Rest Machining is enabled on this operation.
    pub fn has_rest_machining(&self) -> bool {
        !self.passes.rest_machining.is_disabled()
    }

    /// Returns the optional secondary Flat Area Clearance [`ToolId`] when `self` is a two-stage `VCarve` op.
    pub fn clearing_tool_id(&self) -> Option<&ToolId> {
        match &self.strategy {
            Some(StrategyParams::VCarve(p)) => p.clearing_tool_id.as_ref(),
            _ => None,
        }
    }

    pub fn mark_dirty(&mut self) {
        self.runtime_status = OperationStatus::Dirty;
    }

    /// Returns concrete [`StrategyParams`] matching `self.kind` (falling back to defaults when loading M2 files).
    pub fn resolved_strategy(&self) -> StrategyParams {
        match &self.strategy {
            Some(s) => {
                let mut clone = s.clone();
                clone.ensure_matches_kind(self.kind);
                clone
            }
            None => StrategyParams::default_for_kind(self.kind),
        }
    }

    /// Mutable access to [`StrategyParams`], initializing defaults if absent or mismatched with `self.kind`.
    pub fn strategy_mut(&mut self) -> &mut StrategyParams {
        let s = self
            .strategy
            .get_or_insert_with(|| StrategyParams::default_for_kind(self.kind));
        s.ensure_matches_kind(self.kind);
        s
    }

    /// Re-evaluates all inline parameter formulas (`heights`, `passes`, `linking`, `strategy`) under `ctx`.
    pub fn reevaluate_all(&mut self, ctx: &EvalContext) {
        self.heights.reevaluate(ctx);
        self.passes.reevaluate(ctx);
        self.linking.reevaluate(ctx);
        if let Some(s) = &mut self.strategy {
            s.reevaluate(ctx);
        }
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
