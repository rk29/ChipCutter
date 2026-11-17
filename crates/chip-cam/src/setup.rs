use chip_cad::{BodyId, CadDocument};
use chip_math::{Aabb3, DVec3, DisplayUnit, EvalContext, RigidFrame3};
use serde::{Deserialize, Serialize};

use crate::{
    heights::SetupHeights,
    operation::Operation,
    stock::{ResolvedStockGeometry, StockConfig, compute_tight_oriented_model_aabb},
    wcs::{CamError, WcsConfig, WcsOriginMode, WorkOffsetSlot},
};

fn default_post_profile() -> String {
    "grbl".to_string()
}

/// Single physical CNC clamping setup (`03-setups-wcs.md`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Setup {
    pub id: String,
    pub name: String,
    #[serde(default)]
    pub target_bodies: Vec<BodyId>,
    #[serde(default)]
    pub wcs_slot: WorkOffsetSlot,
    #[serde(default = "default_post_profile")]
    pub post_profile: String,
    #[serde(default)]
    pub wcs: WcsConfig,
    #[serde(default)]
    pub stock: StockConfig,
    #[serde(default)]
    pub heights: SetupHeights,
    #[serde(default)]
    pub operations: Vec<Operation>,
}

impl Setup {
    pub fn new(id: impl Into<String>, name: impl Into<String>, wcs_slot: WorkOffsetSlot) -> Self {
        Self {
            id: id.into(),
            name: name.into(),
            target_bodies: Vec::new(),
            wcs_slot,
            post_profile: default_post_profile(),
            wcs: WcsConfig::default(),
            stock: StockConfig::default(),
            heights: SetupHeights::default(),
            operations: Vec::new(),
        }
    }

    /// Resolves this Setup's orthonormal WCS [`RigidFrame3`], tight oriented model AABB,
    /// concrete [`ResolvedStockGeometry`], and Setup Z planes against `doc`.
    pub fn resolve(
        &mut self,
        doc: &CadDocument,
        display_unit: DisplayUnit,
    ) -> Result<ResolvedSetup, CamError> {
        self.resolve_immutable(doc, display_unit)
    }

    pub fn resolve_immutable(
        &self,
        doc: &CadDocument,
        display_unit: DisplayUnit,
    ) -> Result<ResolvedSetup, CamError> {
        // 1. Orthonormal rotation frame at world origin (0, 0, 0)
        let rot_frame = self.wcs.orientation.resolve_rotation(doc)?;

        // 2. Tight bounding box of target B-Rep body/bodies in rotated axes
        let model_rot_aabb =
            compute_tight_oriented_model_aabb(doc, &self.target_bodies, &rot_frame)?;

        // 3. Re-evaluate stock expressions with model dimensions
        let mut stock_clone = self.stock.clone();
        let base_ctx = EvalContext::new(display_unit)
            .with_var("model_top_z", model_rot_aabb.max.z)
            .with_var("model_bottom_z", model_rot_aabb.min.z);
        stock_clone.reevaluate(&base_ctx);
        let stock_rot = stock_clone.compute_oriented_bounds(&model_rot_aabb)?;
        let stock_rot_aabb = stock_rot.aabb_wcs();

        // 4. Compute anchor point in rotated coordinates (before additional offset)
        let base_anchor_rot = match &self.wcs.origin {
            WcsOriginMode::StockBoxPoint { anchor } => anchor.eval_on_aabb(&stock_rot_aabb),
            WcsOriginMode::ModelBoxPoint { anchor } => anchor.eval_on_aabb(&model_rot_aabb),
            WcsOriginMode::SelectedCadFeature { feature } => {
                let pt_world = feature.resolve_world_point(doc, rot_frame.z_axis)?;
                rot_frame.world_to_local_point(pt_world)
            }
        };

        let mut wcs_clone = self.wcs.clone();
        let stock_sz = stock_rot_aabb.size();
        let wcs_ctx = EvalContext::new(display_unit)
            .with_var("stock_x", stock_sz.x)
            .with_var("stock_y", stock_sz.y)
            .with_var("stock_z", stock_sz.z);
        wcs_clone.reevaluate_offsets(&wcs_ctx);

        let extra_rot = DVec3::new(
            wcs_clone.additional_offset_wcs[0].value_mm,
            wcs_clone.additional_offset_wcs[1].value_mm,
            wcs_clone.additional_offset_wcs[2].value_mm,
        );

        let origin_rot = base_anchor_rot + extra_rot;
        let origin_world = rot_frame.local_to_world_point(origin_rot);
        let frame = rot_frame.with_origin(origin_world);

        // Shift rotated model & stock bounds by `-origin_rot` so `(0, 0, 0)` is the WCS origin!
        let shift_wcs = -origin_rot;
        let model_aabb_wcs = Aabb3::new(
            model_rot_aabb.min + shift_wcs,
            model_rot_aabb.max + shift_wcs,
        );
        let stock = stock_rot.shifted_by(shift_wcs);
        let stock_aabb_wcs = stock.aabb_wcs();

        // 5. Resolve Setup Clearance & Retract heights
        let mut heights_clone = self.heights.clone();
        let setup_eval = build_setup_eval_context(display_unit, &model_aabb_wcs, &stock_aabb_wcs);
        heights_clone.reevaluate(&setup_eval);
        let (clearance_z_wcs, retract_z_wcs) =
            heights_clone.resolve(&model_aabb_wcs, &stock_aabb_wcs)?;

        Ok(ResolvedSetup {
            frame,
            model_aabb_wcs,
            stock,
            clearance_z_wcs,
            retract_z_wcs,
        })
    }
}

/// Concrete evaluated Setup WCS frame, bounds, stock geometry, and Z planes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ResolvedSetup {
    pub frame: RigidFrame3,
    pub model_aabb_wcs: Aabb3,
    pub stock: ResolvedStockGeometry,
    pub clearance_z_wcs: f64,
    pub retract_z_wcs: f64,
}

impl ResolvedSetup {
    /// Builds an [`EvalContext`] populated with Setup-scoped variables (`stock_x`, `stock_y`,
    /// `stock_z`, `model_top_z`, `model_bottom_z`).
    pub fn eval_context(&self, display_unit: DisplayUnit) -> EvalContext {
        build_setup_eval_context(display_unit, &self.model_aabb_wcs, &self.stock.aabb_wcs())
    }
}

fn build_setup_eval_context(
    display_unit: DisplayUnit,
    model_aabb_wcs: &Aabb3,
    stock_aabb_wcs: &Aabb3,
) -> EvalContext {
    let sz = stock_aabb_wcs.size();
    EvalContext::new(display_unit)
        .with_var("stock_x", sz.x)
        .with_var("stock_y", sz.y)
        .with_var("stock_z", sz.z)
        .with_var("model_top_z", model_aabb_wcs.max.z)
        .with_var("model_bottom_z", model_aabb_wcs.min.z)
}
