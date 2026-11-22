mod common;
mod deburr3d;
mod milling_2d;
mod rotary;
mod surface_3d;
mod thread_mill;

pub use common::*;
pub use deburr3d::*;
pub use milling_2d::*;
pub use rotary::*;
pub use surface_3d::*;
pub use thread_mill::*;

use chip_math::EvalContext;
use serde::{Deserialize, Serialize};

use crate::operation::OperationKind;

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
    #[serde(rename = "thread_mill")]
    ThreadMill(ThreadMillParams),
    #[serde(rename = "deburr_3d")]
    Deburr3D(Deburr3DParams),
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
            OperationKind::Deburr3D => Self::Deburr3D(Deburr3DParams::default()),
            OperationKind::Rotary2Linear => Self::Rotary2Linear(Rotary2LinearParams::default()),
            OperationKind::Rotary4AxisSim => Self::Rotary4AxisSim(Rotary4AxisSimParams::default()),
            OperationKind::ThreadMill => Self::ThreadMill(ThreadMillParams::default()),
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
                | (OperationKind::Deburr3D, Self::Deburr3D(_))
                | (OperationKind::Rotary2Linear, Self::Rotary2Linear(_))
                | (OperationKind::Rotary4AxisSim, Self::Rotary4AxisSim(_))
                | (OperationKind::ThreadMill, Self::ThreadMill(_))
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
            Self::Deburr3D(p) => p.reevaluate(ctx),
            Self::Rotary2Linear(p) => p.reevaluate(ctx),
            Self::Rotary4AxisSim(p) => p.reevaluate(ctx),
            Self::ThreadMill(p) => p.reevaluate(ctx),
        }
    }
}
