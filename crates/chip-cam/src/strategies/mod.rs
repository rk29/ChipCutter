pub mod adaptive2d;
pub mod chamfer_trace;
pub mod contour2d;
pub mod drill_bore;
pub mod facing;
pub mod pocket2d;

use std::sync::atomic::{AtomicBool, Ordering};

use chip_cad::CadDocument;
use chip_math::{DisplayUnit, EvalContext};
use chip_tools::Tool;

use crate::{
    cl_data::ClEvent,
    heights::ResolvedHeights,
    operation::{EffectiveCuttingParams, Operation, OperationKind},
    params::{LinkingParams, PassesParams, RetractPolicy, StrategyParams},
    rest2d::UpstreamSweptInput,
    selection::{ExtractedSelectionSummary, extract_selection_features},
    setup::ResolvedSetup,
    toolpath::{GeneratedToolpath, ToolpathBuilder},
    wcs::CamError,
};

/// Resolved immutable context passed into each 2.5D toolpath strategy generator.
pub struct StrategyExecContext<'a> {
    pub setup: &'a ResolvedSetup,
    pub op: &'a Operation,
    pub tool: &'a Tool,
    pub cutting: EffectiveCuttingParams,
    pub heights: ResolvedHeights,
    pub passes: PassesParams,
    pub linking: LinkingParams,
    pub selection: ExtractedSelectionSummary,
    pub upstream_rest: &'a [UpstreamSweptInput],
    pub cancel: &'a AtomicBool,
}

impl StrategyExecContext<'_> {
    #[inline]
    pub fn check_cancel(&self) -> Result<(), CamError> {
        if self.cancel.load(Ordering::Relaxed) {
            Err(CamError::Cancelled)
        } else {
            Ok(())
        }
    }

    #[inline]
    pub fn safe_retract_z(&self) -> f64 {
        match self.linking.retract_policy {
            RetractPolicy::MinimumRetract => self.heights.retract_z_wcs,
            RetractPolicy::FullClearance => self.heights.clearance_z_wcs,
        }
    }
}

/// Input bundle for [`generate_operation_toolpath_with_rest`] (supporting upstream Rest Machining).
pub struct OperationGenRequest<'a> {
    pub doc: &'a CadDocument,
    pub setup: &'a ResolvedSetup,
    pub op: &'a Operation,
    pub tool: &'a Tool,
    pub display_unit: DisplayUnit,
    pub is_first_tool_in_setup: bool,
    pub upstream_rest: &'a [UpstreamSweptInput],
    pub cancel: &'a AtomicBool,
}

/// Generates a verified [`GeneratedToolpath`] (typed `ClEvent` stream + exact 3D WCS motion segments)
/// for a single [`Operation`] (`05-operations.md`).
pub fn generate_operation_toolpath(
    doc: &CadDocument,
    setup: &ResolvedSetup,
    op: &Operation,
    tool: &Tool,
    display_unit: DisplayUnit,
    is_first_tool_in_setup: bool,
    cancel: &AtomicBool,
) -> Result<GeneratedToolpath, CamError> {
    generate_operation_toolpath_with_rest(&OperationGenRequest {
        doc,
        setup,
        op,
        tool,
        display_unit,
        is_first_tool_in_setup,
        upstream_rest: &[],
        cancel,
    })
}

/// Generates a verified [`GeneratedToolpath`] with optional upstream 2D swept coverage for
/// `RestMachiningMode::FromPrecedingOperations` (`05-operations.md` §Rest Machining).
pub fn generate_operation_toolpath_with_rest(
    req: &OperationGenRequest<'_>,
) -> Result<GeneratedToolpath, CamError> {
    let &OperationGenRequest {
        doc,
        setup,
        op,
        tool,
        display_unit,
        is_first_tool_in_setup,
        upstream_rest,
        cancel,
    } = req;
    if cancel.load(Ordering::Relaxed) {
        return Err(CamError::Cancelled);
    }

    if !op.kind.is_compatible_with_tool(tool) {
        return Err(CamError::IncompatibleTool {
            op_name: op.name.clone(),
            op_kind: op.kind.label(),
            tool_name: tool.name.clone(),
            tool_kind: tool.geometry.kind_label(),
        });
    }

    // 1. Extract B-Rep features in Setup WCS coordinates
    let selection = extract_selection_features(doc, setup, &op.selections)?;
    if op.kind != OperationKind::Facing && selection.features.is_empty() {
        return Err(CamError::EmptySelection {
            op_name: op.name.clone(),
            op_kind: op.kind.label(),
        });
    }

    // 2. Build scoped EvalContext (`setup` + `tool` + `feature` variables) and evaluate all expressions
    let mut eval_ctx: EvalContext = setup.eval_context(display_unit);
    tool.populate_eval_context(&mut eval_ctx);
    let (inferred_top_z, inferred_bottom_z) =
        if op.kind == OperationKind::Facing && selection.features.is_empty() {
            (setup.stock.aabb_wcs().max.z, setup.model_aabb_wcs.max.z)
        } else {
            (
                selection.inferred_top_z_wcs,
                selection.inferred_bottom_z_wcs,
            )
        };
    eval_ctx.set_var("feature_top_z", inferred_top_z);
    eval_ctx.set_var("feature_bottom_z", inferred_bottom_z);
    if let Some(hd) = selection.primary_hole_diameter_mm {
        eval_ctx.set_var("hole_d", hd);
    }

    let mut heights_spec = op.heights.clone();
    heights_spec.reevaluate(&eval_ctx);
    let heights = ResolvedHeights::resolve(
        &heights_spec,
        &setup.model_aabb_wcs,
        &setup.stock.aabb_wcs(),
        setup.clearance_z_wcs,
        setup.retract_z_wcs,
        inferred_top_z,
        inferred_bottom_z,
    )?;

    let mut passes = op.passes.clone();
    passes.reevaluate(&eval_ctx);
    let mut linking = op.linking.clone();
    linking.reevaluate(&eval_ctx);
    let mut strategy = op.resolved_strategy();
    strategy.reevaluate(&eval_ctx);

    let cutting = op.effective_cutting_params(tool);
    if !cutting.cutting_feed_mm_min.is_finite() || cutting.cutting_feed_mm_min <= 0.0 {
        return Err(CamError::InvalidParameter {
            param: "cutting_feed_mm_min",
            value: cutting.cutting_feed_mm_min,
            reason: "must be finite and > 0",
        });
    }

    let ctx = StrategyExecContext {
        setup,
        op,
        tool,
        cutting,
        heights,
        passes,
        linking,
        selection,
        upstream_rest,
        cancel,
    };

    let mut builder = ToolpathBuilder::new(&op.id);
    builder.push_event(ClEvent::OperationStart {
        op_id: op.id.clone(),
        op_name: op.name.clone(),
        strategy_name: op.kind.label().to_string(),
        est_time_sec: 0.0,
    });
    builder.push_event(ClEvent::ToolChange {
        tool_number: tool.tool_number,
        length_offset: tool.length_offset_number,
        diameter_offset: tool.diameter_offset_number,
        tool_desc: format!(
            "{} (Ø{:.3} mm)",
            tool.name,
            tool.geometry.cutting_diameter_mm()
        ),
        is_first_tool: is_first_tool_in_setup,
    });
    builder.push_event(ClEvent::SpindleStart {
        rpm: cutting.spindle_rpm,
        direction: tool.spindle_direction,
    });
    builder.push_event(ClEvent::Coolant {
        mode: cutting.coolant,
    });

    match &strategy {
        StrategyParams::Facing(p) => facing::generate_facing(&ctx, p, &mut builder)?,
        StrategyParams::Contour2D(p) => contour2d::generate_contour2d(&ctx, p, &mut builder)?,
        StrategyParams::Pocket2D(p) => pocket2d::generate_pocket2d(&ctx, p, &mut builder)?,
        StrategyParams::DrillBore(p) => drill_bore::generate_drill_bore(&ctx, p, &mut builder)?,
        StrategyParams::ChamferTrace(p) => {
            chamfer_trace::generate_chamfer_trace(&ctx, p, &mut builder)?;
        }
    }

    ctx.check_cancel()?;

    // Ensure final safe vertical retract up to Clearance Z before OperationEnd
    if let Some(last_pos) = builder.current_pos()
        && last_pos.z < ctx.heights.clearance_z_wcs - 1e-6
    {
        builder.rapid_to(chip_math::DVec3::new(
            last_pos.x,
            last_pos.y,
            ctx.heights.clearance_z_wcs,
        ));
    }
    builder.push_event(ClEvent::OperationEnd);

    let finished = builder.finish();
    if finished.segments_wcs.is_empty() {
        return Err(CamError::EmptyToolpath {
            op_name: op.name.clone(),
        });
    }

    Ok(finished)
}

/// Computes descending Z cutting levels from `top_z_wcs` down to `target_bottom_z_wcs`
/// (`<= top_z_wcs`), honoring `multiple_depths`, `max_stepdown_mm`, and `even_stepdowns`.
pub fn compute_z_levels(
    top_z_wcs: f64,
    target_bottom_z_wcs: f64,
    multiple_depths: bool,
    max_stepdown_mm: f64,
    even_stepdowns: bool,
) -> Result<Vec<f64>, CamError> {
    if !max_stepdown_mm.is_finite() || max_stepdown_mm <= 1e-4 {
        return Err(CamError::InvalidParameter {
            param: "max_stepdown",
            value: max_stepdown_mm,
            reason: "must be finite and > 0.0001 mm",
        });
    }
    let total_depth = (top_z_wcs - target_bottom_z_wcs).max(0.0);
    if !multiple_depths || total_depth <= max_stepdown_mm + 1e-6 {
        return Ok(vec![target_bottom_z_wcs]);
    }

    let n_steps = (total_depth / max_stepdown_mm).ceil().max(1.0) as usize;
    let mut levels = Vec::with_capacity(n_steps);
    if even_stepdowns {
        let step = total_depth / (n_steps as f64);
        for i in 1..=n_steps {
            let z = if i == n_steps {
                target_bottom_z_wcs
            } else {
                top_z_wcs - step * (i as f64)
            };
            levels.push(z);
        }
    } else {
        let mut z = top_z_wcs;
        for _ in 0..n_steps {
            z = (z - max_stepdown_mm).max(target_bottom_z_wcs);
            levels.push(z);
            if (z - target_bottom_z_wcs).abs() <= 1e-7 {
                break;
            }
        }
    }
    Ok(levels)
}
