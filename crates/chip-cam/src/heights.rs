use chip_math::{Aabb3, EvalContext, ParamLength};
use serde::{Deserialize, Serialize};

use crate::wcs::CamError;

/// Named Z reference plane from which a [`HeightSpec`] offset is measured (`03-setups-wcs.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HeightReference {
    FromSelection,
    StockTop,
    StockBottom,
    ModelTop,
    ModelBottom,
    WcsOrigin,
    SetupClearance,
    SetupRetract,
    OperationTop,
}

impl HeightReference {
    pub const OPERATION_REFS: [Self; 9] = [
        Self::FromSelection,
        Self::StockTop,
        Self::StockBottom,
        Self::ModelTop,
        Self::ModelBottom,
        Self::WcsOrigin,
        Self::SetupClearance,
        Self::SetupRetract,
        Self::OperationTop,
    ];

    pub const SETUP_REFS: [Self; 5] = [
        Self::StockTop,
        Self::StockBottom,
        Self::ModelTop,
        Self::ModelBottom,
        Self::WcsOrigin,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::FromSelection => "From Selection",
            Self::StockTop => "Stock Top",
            Self::StockBottom => "Stock Bottom",
            Self::ModelTop => "Model Top",
            Self::ModelBottom => "Model Bottom",
            Self::WcsOrigin => "WCS Origin (Z0)",
            Self::SetupClearance => "Setup Clearance",
            Self::SetupRetract => "Setup Retract",
            Self::OperationTop => "Top Height",
        }
    }
}

/// Parametric Z plane specification (`reference plane + signed offset expression`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HeightSpec {
    pub reference: HeightReference,
    pub offset: ParamLength,
}

impl HeightSpec {
    pub fn new(reference: HeightReference, offset_mm: f64) -> Self {
        Self {
            reference,
            offset: ParamLength::from_mm(offset_mm),
        }
    }

    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        let _ = self.offset.reevaluate(ctx);
    }
}

/// Setup-level default Z clearance and retract planes (`03-setups-wcs.md`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SetupHeights {
    pub clearance: HeightSpec,
    pub retract: HeightSpec,
    pub default_feed_offset: ParamLength,
}

impl Default for SetupHeights {
    fn default() -> Self {
        Self {
            clearance: HeightSpec::new(HeightReference::StockTop, 10.0),
            retract: HeightSpec::new(HeightReference::StockTop, 5.0),
            default_feed_offset: ParamLength::from_mm(2.0),
        }
    }
}

impl SetupHeights {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        self.clearance.reevaluate(ctx);
        self.retract.reevaluate(ctx);
        let _ = self.default_feed_offset.reevaluate(ctx);
    }

    /// Resolves `(clearance_z_wcs, retract_z_wcs)` and validates `Clearance >= Retract`.
    pub fn resolve(
        &self,
        model_aabb_wcs: &Aabb3,
        stock_aabb_wcs: &Aabb3,
    ) -> Result<(f64, f64), CamError> {
        let eval_base = |r: HeightReference| -> f64 {
            match r {
                HeightReference::StockTop | HeightReference::FromSelection => stock_aabb_wcs.max.z,
                HeightReference::StockBottom => stock_aabb_wcs.min.z,
                HeightReference::ModelTop | HeightReference::OperationTop => model_aabb_wcs.max.z,
                HeightReference::ModelBottom => model_aabb_wcs.min.z,
                HeightReference::WcsOrigin => 0.0,
                HeightReference::SetupClearance => stock_aabb_wcs.max.z + 10.0,
                HeightReference::SetupRetract => stock_aabb_wcs.max.z + 5.0,
            }
        };
        let clearance_z = eval_base(self.clearance.reference) + self.clearance.offset.value_mm;
        let retract_base = if self.retract.reference == HeightReference::SetupClearance {
            clearance_z
        } else {
            eval_base(self.retract.reference)
        };
        let retract_z = retract_base + self.retract.offset.value_mm;

        if clearance_z + 1e-6 < retract_z {
            return Err(CamError::HeightHierarchyViolation {
                upper_name: "Setup Clearance Height",
                upper_z: clearance_z,
                lower_name: "Setup Retract Height",
                lower_z: retract_z,
            });
        }
        Ok((clearance_z, retract_z))
    }
}

/// Operation-level 5-plane Z hierarchy (`Clearance >= Retract >= Feed >= Top >= Bottom`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OperationHeights {
    pub clearance: HeightSpec,
    pub retract: HeightSpec,
    pub feed: HeightSpec,
    pub top: HeightSpec,
    pub bottom: HeightSpec,
}

impl Default for OperationHeights {
    fn default() -> Self {
        Self {
            clearance: HeightSpec::new(HeightReference::SetupClearance, 0.0),
            retract: HeightSpec::new(HeightReference::SetupRetract, 0.0),
            feed: HeightSpec::new(HeightReference::OperationTop, 2.0),
            top: HeightSpec::new(HeightReference::FromSelection, 0.0),
            bottom: HeightSpec::new(HeightReference::FromSelection, 0.0),
        }
    }
}

impl OperationHeights {
    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        self.clearance.reevaluate(ctx);
        self.retract.reevaluate(ctx);
        self.feed.reevaluate(ctx);
        self.top.reevaluate(ctx);
        self.bottom.reevaluate(ctx);
    }
}

/// Concrete evaluated Z heights in active Setup WCS millimeters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ResolvedHeights {
    pub clearance_z_wcs: f64,
    pub retract_z_wcs: f64,
    pub feed_z_wcs: f64,
    pub top_z_wcs: f64,
    pub bottom_z_wcs: f64,
}

impl ResolvedHeights {
    /// Resolves all 5 operation Z planes in topological dependency order and validates
    /// `Clearance >= Retract >= Feed >= Top >= Bottom`.
    pub fn resolve(
        heights: &OperationHeights,
        model_aabb_wcs: &Aabb3,
        stock_aabb_wcs: &Aabb3,
        setup_clearance_z: f64,
        setup_retract_z: f64,
        inferred_top_z: f64,
        inferred_bottom_z: f64,
    ) -> Result<Self, CamError> {
        let base_plane = |r: HeightReference, sel_default: f64, op_top: Option<f64>| -> f64 {
            match r {
                HeightReference::FromSelection => sel_default,
                HeightReference::StockTop => stock_aabb_wcs.max.z,
                HeightReference::StockBottom => stock_aabb_wcs.min.z,
                HeightReference::ModelTop => model_aabb_wcs.max.z,
                HeightReference::ModelBottom => model_aabb_wcs.min.z,
                HeightReference::WcsOrigin => 0.0,
                HeightReference::SetupClearance => setup_clearance_z,
                HeightReference::SetupRetract => setup_retract_z,
                HeightReference::OperationTop => op_top.unwrap_or(inferred_top_z),
            }
        };

        let bottom_z_wcs = base_plane(heights.bottom.reference, inferred_bottom_z, None)
            + heights.bottom.offset.value_mm;
        let top_z_wcs =
            base_plane(heights.top.reference, inferred_top_z, None) + heights.top.offset.value_mm;
        let feed_z_wcs = base_plane(heights.feed.reference, top_z_wcs + 2.0, Some(top_z_wcs))
            + heights.feed.offset.value_mm;
        let retract_z_wcs = base_plane(heights.retract.reference, setup_retract_z, Some(top_z_wcs))
            + heights.retract.offset.value_mm;
        let clearance_z_wcs = base_plane(
            heights.clearance.reference,
            setup_clearance_z,
            Some(top_z_wcs),
        ) + heights.clearance.offset.value_mm;

        let out = Self {
            clearance_z_wcs,
            retract_z_wcs,
            feed_z_wcs,
            top_z_wcs,
            bottom_z_wcs,
        };
        out.validate()?;
        Ok(out)
    }

    /// Explicitly validates `Clearance >= Retract >= Feed >= Top >= Bottom` per `AGENTS.md` §4.
    pub fn validate(&self) -> Result<(), CamError> {
        let tol = 1e-6;
        let pairs = [
            (
                "Clearance Height",
                self.clearance_z_wcs,
                "Retract Height",
                self.retract_z_wcs,
            ),
            (
                "Retract Height",
                self.retract_z_wcs,
                "Feed Height",
                self.feed_z_wcs,
            ),
            ("Feed Height", self.feed_z_wcs, "Top Height", self.top_z_wcs),
            (
                "Top Height",
                self.top_z_wcs,
                "Bottom Height",
                self.bottom_z_wcs,
            ),
        ];
        for (u_name, u_val, l_name, l_val) in pairs {
            if !u_val.is_finite() || !l_val.is_finite() || u_val + tol < l_val {
                return Err(CamError::HeightHierarchyViolation {
                    upper_name: u_name,
                    upper_z: u_val,
                    lower_name: l_name,
                    lower_z: l_val,
                });
            }
        }
        Ok(())
    }
}
