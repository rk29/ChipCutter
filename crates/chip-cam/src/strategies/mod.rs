pub mod adaptive2d;
pub mod adaptive3d;
pub mod chamfer_trace;
pub mod contour2d;
pub mod deburr3d;
pub mod dovetail;
pub mod drill_bore;
pub mod facing;
pub mod parallel3d;
pub mod pencil3d;
pub mod pocket2d;
pub mod rotary2linear;
pub mod rotary4axis_sim;
pub mod rotary_wrap;
pub mod scallop3d;
pub mod slot_undercut;
pub mod thread_mill;
pub mod undercut3d;
pub mod vcarve;
pub mod waterline3d;

use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

use chip_cad::CadDocument;
use chip_math::{Aabb3, DVec3, DisplayUnit, EvalContext};
use chip_sketch::SketchLayer;
use chip_stock::{SimResolution, TriDexelStock};
use chip_tools::Tool;

use crate::{
    cl_data::ClEvent,
    fixture_avoidance::route_toolpath_around_fixtures,
    heights::ResolvedHeights,
    operation::{EffectiveCuttingParams, Operation, OperationKind},
    params::{
        ChamferSourceMode, ChamferTraceMode, ContourCompensation, LinkingParams, PassesParams,
        RetractPolicy, StrategyParams,
    },
    rest2d::{UpstreamSweptInput, filter_upstream_for_indexed_angle},
    rotary::{OperationRotaryMode, RotaryAxisKind, build_indexed_virtual_setup},
    selection::{ExtractedSelectionSummary, extract_selection_features_with_sketches},
    setup::ResolvedSetup,
    stock::ResolvedStockGeometry,
    surface3d::{SurfaceMesh3D, check_2d_toolpath_against_overhangs},
    toolpath::{GeneratedToolpath, ToolpathBuilder},
    wcs::CamError,
    workholding::ResolvedFixtureSolid,
};

/// Resolved immutable context passed into each 2.5D / 3-axis toolpath strategy generator.
pub struct StrategyExecContext<'a> {
    pub doc: &'a CadDocument,
    pub setup: &'a ResolvedSetup,
    pub op: &'a Operation,
    pub tool: &'a Tool,
    pub cutting: EffectiveCuttingParams,
    pub heights: ResolvedHeights,
    pub passes: PassesParams,
    pub linking: LinkingParams,
    pub selection: ExtractedSelectionSummary,
    pub upstream_rest: &'a [UpstreamSweptInput],
    pub carryover_stock: Option<&'a TriDexelStock>,
    pub is_first_tool_in_setup: bool,
    pub clearing_tool: Option<&'a Tool>,
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

/// Input bundle for [`generate_operation_toolpath_with_rest`] (supporting 2D sketches, upstream Rest Machining,
/// and multi-setup carried-over `TriDexelStock` rest roughing).
pub struct OperationGenRequest<'a> {
    pub doc: &'a CadDocument,
    pub sketches: &'a [SketchLayer],
    pub setup: &'a ResolvedSetup,
    pub op: &'a Operation,
    pub tool: &'a Tool,
    pub clearing_tool: Option<&'a Tool>,
    pub display_unit: DisplayUnit,
    pub is_first_tool_in_setup: bool,
    pub upstream_rest: &'a [UpstreamSweptInput],
    pub carryover_stock: Option<Arc<TriDexelStock>>,
    pub cancel: &'a AtomicBool,
}

/// Generates a verified [`GeneratedToolpath`] with optional 3D workholding fixtures and automatic
/// safe-rapid avoidance routing (`M61`), validating lifted rapid Z heights against active machine
/// Z travel limits when provided.
///
/// Exact no-op wrapper over [`generate_operation_toolpath_with_rest`] when `fixtures.is_empty()`
/// or `!auto_avoid_fixtures_on_rapids`.
pub fn generate_operation_toolpath_with_fixtures(
    req: &OperationGenRequest<'_>,
    fixtures: &[ResolvedFixtureSolid],
    auto_avoid_fixtures_on_rapids: bool,
    machine_z_travel_mm: Option<f64>,
) -> Result<GeneratedToolpath, CamError> {
    let mut tp = generate_operation_toolpath_with_rest(req)?;
    if auto_avoid_fixtures_on_rapids && !fixtures.is_empty() {
        route_toolpath_around_fixtures(
            &mut tp,
            req.tool,
            req.setup,
            fixtures,
            true,
            machine_z_travel_mm,
        );
        if let (Some(stage), Some(clear_tool)) = (tp.clearing_stage.as_mut(), req.clearing_tool) {
            let stage_summary = route_toolpath_around_fixtures(
                &mut stage.toolpath,
                clear_tool,
                req.setup,
                fixtures,
                true,
                machine_z_travel_mm,
            );
            tp.rapid_lifts_inserted += stage_summary.rapid_lifts_inserted;
            if stage_summary.rapid_lifts_inserted > 0 {
                tp.max_lifted_z_wcs = tp.max_lifted_z_wcs.max(stage_summary.max_lifted_z_wcs);
            }
        }
    }
    Ok(tp)
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
        sketches: &[],
        setup,
        op,
        tool,
        clearing_tool: None,
        display_unit,
        is_first_tool_in_setup,
        upstream_rest: &[],
        carryover_stock: None,
        cancel,
    })
}

/// Infers the matching [`SimResolution`] tier of an existing [`TriDexelStock`] for oblique
/// rigid-frame re-voxelization.
fn infer_tridexel_resolution(td: &TriDexelStock) -> SimResolution {
    let max_dim = td.dims[0].max(td.dims[1]).max(td.dims[2]);
    if max_dim >= 360 {
        SimResolution::Ultra
    } else if max_dim >= 220 {
        SimResolution::Fine
    } else {
        SimResolution::Standard
    }
}

/// Returns `true` when two rigid frames coincide within `1e-9`.
fn frames_approx_equal(a: &chip_math::RigidFrame3, b: &chip_math::RigidFrame3) -> bool {
    a.origin_mm.distance_squared(b.origin_mm) <= 1e-18
        && a.x_axis.distance_squared(b.x_axis) <= 1e-18
        && a.y_axis.distance_squared(b.y_axis) <= 1e-18
        && a.z_axis.distance_squared(b.z_axis) <= 1e-18
}

/// Generates a verified [`GeneratedToolpath`] with optional 2D sketches and upstream 2D swept coverage for
/// `RestMachiningMode::FromPrecedingOperations` (`05-operations.md` §Rest Machining).
pub fn generate_operation_toolpath_with_rest(
    req: &OperationGenRequest<'_>,
) -> Result<GeneratedToolpath, CamError> {
    if req.cancel.load(Ordering::Relaxed) {
        return Err(CamError::Cancelled);
    }

    if !req.op.kind.is_compatible_with_tool(req.tool) {
        return Err(CamError::IncompatibleTool {
            op_name: req.op.name.clone(),
            op_kind: req.op.kind.label(),
            tool_name: req.tool.name.clone(),
            tool_kind: req.tool.geometry.kind_label(),
        });
    }

    // 0. Per-operation 4th-axis rotary wrappers (Indexed 3+1 & 2D Cylindrical Wrapping) and
    //    rotary-angle-scoped 2D rest filtering.
    if req.op.kind.is_rotary_native() {
        return generate_operation_toolpath_inner(&OperationGenRequest {
            upstream_rest: &[],
            carryover_stock: req.carryover_stock.clone(),
            ..*req
        });
    }

    match &req.op.rotary {
        OperationRotaryMode::None => {
            let filtered_upstream = filter_upstream_for_indexed_angle(req.upstream_rest, None);
            generate_operation_toolpath_inner(&OperationGenRequest {
                upstream_rest: &filtered_upstream,
                carryover_stock: req.carryover_stock.clone(),
                ..*req
            })
        }
        OperationRotaryMode::Indexed3Plus1 { angle_deg } => {
            let Some(rot) = req.setup.rotary else {
                return Err(CamError::InvalidRotarySetup {
                    op_name: req.op.name.clone(),
                    reason: "Indexed 3+1 machining requires a Setup with 4th-Axis Rotary enabled"
                        .to_string(),
                });
            };
            let virtual_setup = build_indexed_virtual_setup(req.setup, &rot, *angle_deg);
            let filtered_upstream =
                filter_upstream_for_indexed_angle(req.upstream_rest, Some(*angle_deg));
            let rotated_carryover = if let Some(td) = req.carryover_stock.as_ref() {
                if frames_approx_equal(&req.setup.frame, &virtual_setup.frame) {
                    Some(Arc::clone(td))
                } else {
                    let res = infer_tridexel_resolution(td);
                    let transformed = td
                        .transform_to_frame(req.setup.frame, virtual_setup.frame, res)
                        .map_err(|e| CamError::InvalidRotarySetup {
                            op_name: req.op.name.clone(),
                            reason: format!(
                                "failed to transform carryover stock to {angle_deg:.2}°: {e}"
                            ),
                        })?;
                    Some(Arc::new(transformed))
                }
            } else {
                None
            };
            let mut inner_op = req.op.clone();
            inner_op.rotary = OperationRotaryMode::None;
            let local_tp = generate_operation_toolpath_inner(&OperationGenRequest {
                doc: req.doc,
                sketches: req.sketches,
                setup: &virtual_setup,
                op: &inner_op,
                tool: req.tool,
                clearing_tool: req.clearing_tool,
                display_unit: req.display_unit,
                is_first_tool_in_setup: req.is_first_tool_in_setup,
                upstream_rest: &filtered_upstream,
                carryover_stock: rotated_carryover,
                cancel: req.cancel,
            })?;
            Ok(rotary_wrap::wrap_indexed_3plus1_toolpath(
                &local_tp, &rot, *angle_deg,
            ))
        }
        OperationRotaryMode::WrapCylinder {
            wrap_radius,
            axial_offset,
            angle_offset_deg,
        } => {
            let Some(rot) = req.setup.rotary else {
                return Err(CamError::InvalidRotarySetup {
                    op_name: req.op.name.clone(),
                    reason: "2D Cylindrical Wrapping requires a Setup with 4th-Axis Rotary enabled"
                        .to_string(),
                });
            };
            if rot.axis == RotaryAxisKind::ZAxis {
                return Err(CamError::InvalidRotarySetup {
                    op_name: req.op.name.clone(),
                    reason:
                        "2D Cylindrical Wrapping requires a horizontal rotary axis (A/X or B/Y)"
                            .to_string(),
                });
            }
            let mut r_ctx = req.setup.eval_context(req.display_unit);
            req.tool.populate_eval_context(&mut r_ctx);
            let mut wr = wrap_radius.clone();
            let _ = wr.reevaluate(&r_ctx);
            let mut ao = axial_offset.clone();
            let _ = ao.reevaluate(&r_ctx);
            let r_wrap = wr.value_mm.max(1.0);
            let ax_off = ao.value_mm;

            // Build an unwrapped planar virtual Setup with Z=0 at cylinder OD (R = r_wrap)
            // and Z=-r_wrap at the rotary centerline, spanning the cylinder & sketch footprint.
            let half_u = ((rot.stock_u_max - rot.stock_u_min).abs() * 0.5).max(50.0);
            let half_circ = (std::f64::consts::PI * r_wrap).max(50.0);
            let (mut min_x, mut max_x, mut min_y, mut max_y) = match rot.axis {
                RotaryAxisKind::XAxis | RotaryAxisKind::ZAxis => (
                    rot.stock_u_min.min(-half_u),
                    rot.stock_u_max.max(half_u),
                    -half_circ,
                    half_circ,
                ),
                RotaryAxisKind::YAxis => (
                    -half_circ,
                    half_circ,
                    rot.stock_u_min.min(-half_u),
                    rot.stock_u_max.max(half_u),
                ),
            };
            for sk in req.sketches {
                let b = sk.exact_aabb();
                if b.is_valid() {
                    min_x = min_x.min(b.min.x - 5.0);
                    max_x = max_x.max(b.max.x + 5.0);
                    min_y = min_y.min(b.min.y - 5.0);
                    max_y = max_y.max(b.max.y + 5.0);
                }
            }
            let flat_aabb = Aabb3::new(
                DVec3::new(min_x, min_y, -r_wrap),
                DVec3::new(max_x, max_y, 0.0),
            );
            let flat_clearance = (rot.clearance_radius_mm - r_wrap).max(5.0);
            let flat_retract = 3.0_f64.min(flat_clearance);
            let flat_setup = ResolvedSetup {
                frame: req.setup.frame,
                model_aabb_wcs: flat_aabb,
                stock: ResolvedStockGeometry::Box {
                    aabb_wcs: flat_aabb,
                },
                rotary: None,
                clearance_z_wcs: flat_clearance,
                retract_z_wcs: flat_retract,
            };
            let mut inner_op = req.op.clone();
            inner_op.rotary = OperationRotaryMode::None;
            // Exclude cross-angle/non-planar upstream capsules from unwrapped cylindrical coords.
            let flat_tp = generate_operation_toolpath_inner(&OperationGenRequest {
                doc: req.doc,
                sketches: req.sketches,
                setup: &flat_setup,
                op: &inner_op,
                tool: req.tool,
                clearing_tool: req.clearing_tool,
                display_unit: req.display_unit,
                is_first_tool_in_setup: req.is_first_tool_in_setup,
                upstream_rest: &[],
                carryover_stock: None,
                cancel: req.cancel,
            })?;
            Ok(rotary_wrap::wrap_planar_toolpath_to_cylinder(
                &flat_tp,
                &rot,
                r_wrap,
                ax_off,
                *angle_offset_deg,
            ))
        }
    }
}

fn generate_operation_toolpath_inner(
    req: &OperationGenRequest<'_>,
) -> Result<GeneratedToolpath, CamError> {
    let OperationGenRequest {
        doc,
        sketches,
        setup,
        op,
        tool,
        clearing_tool,
        display_unit,
        is_first_tool_in_setup,
        upstream_rest,
        carryover_stock,
        cancel,
    } = req;
    let doc = *doc;
    let sketches = *sketches;
    let setup = *setup;
    let op = *op;
    let tool = *tool;
    let clearing_tool = *clearing_tool;
    let display_unit = *display_unit;
    let is_first_tool_in_setup = *is_first_tool_in_setup;
    let upstream_rest = *upstream_rest;
    let carryover_stock = carryover_stock.as_deref();
    let cancel = *cancel;
    if cancel.load(Ordering::Relaxed) {
        return Err(CamError::Cancelled);
    }

    // 1. Extract 3D B-Rep + 2D Sketch features in Setup WCS coordinates
    let selection = extract_selection_features_with_sketches(doc, sketches, setup, &op.selections)?;
    if op.kind != OperationKind::Facing && !op.kind.is_3d_surface() && selection.features.is_empty()
    {
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
        } else if op.kind.is_3d_surface() && selection.features.is_empty() {
            (setup.stock.aabb_wcs().max.z, setup.model_aabb_wcs.min.z)
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
        doc,
        setup,
        op,
        tool,
        cutting,
        heights,
        passes,
        linking,
        selection,
        upstream_rest,
        carryover_stock,
        is_first_tool_in_setup,
        clearing_tool,
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
        StrategyParams::VCarve(p) => {
            vcarve::generate_vcarve(&ctx, p, &mut builder)?;
        }
        StrategyParams::Waterline3D(p) => {
            waterline3d::generate_waterline3d(&ctx, p, &mut builder)?;
        }
        StrategyParams::Adaptive3D(p) => {
            adaptive3d::generate_adaptive3d(&ctx, p, &mut builder)?;
        }
        StrategyParams::Parallel3D(p) => {
            parallel3d::generate_parallel3d(&ctx, p, &mut builder)?;
        }
        StrategyParams::Scallop3D(p) => {
            scallop3d::generate_scallop3d(&ctx, p, &mut builder)?;
        }
        StrategyParams::Pencil3D(p) => {
            pencil3d::generate_pencil3d(&ctx, p, &mut builder)?;
        }
        StrategyParams::Deburr3D(p) => {
            deburr3d::generate_deburr3d(&ctx, p, &mut builder)?;
        }
        StrategyParams::Rotary2Linear(p) => {
            rotary2linear::generate_rotary2linear(&ctx, p, &mut builder)?;
        }
        StrategyParams::Rotary4AxisSim(p) => {
            rotary4axis_sim::generate_rotary4axis_sim(&ctx, p, &mut builder)?;
        }
        StrategyParams::ThreadMill(p) => {
            thread_mill::generate_thread_mill(&ctx, p, &mut builder)?;
        }
        StrategyParams::SlotUndercut(p) => {
            slot_undercut::generate_slot_undercut(&ctx, p, &mut builder)?;
        }
        StrategyParams::Dovetail(p) => {
            dovetail::generate_dovetail(&ctx, p, &mut builder)?;
        }
        StrategyParams::Undercut3D(p) => {
            undercut3d::generate_undercut3d(&ctx, p, &mut builder)?;
        }
    }

    ctx.check_cancel()?;

    // Ensure final safe vertical retract up to Clearance Z before OperationEnd
    if let Some(last_pos) = builder.current_pos()
        && last_pos.z < ctx.heights.clearance_z_wcs - 1e-6
    {
        if op.kind.is_rotary_native() {
            builder.rotary_rapid_to(
                DVec3::new(last_pos.x, last_pos.y, ctx.heights.clearance_z_wcs),
                builder.current_rotary_deg(),
            );
        } else {
            builder.rapid_to(DVec3::new(
                last_pos.x,
                last_pos.y,
                ctx.heights.clearance_z_wcs,
            ));
        }
    }
    builder.push_event(ClEvent::OperationEnd);

    // Engagement-aware feedrate optimization post-pass (fast no-op when disabled)
    if ctx.passes.feed_optimization.enabled {
        crate::feed_opt::optimize_toolpath_feedrates(
            &mut builder.events,
            &ctx.passes.feed_optimization,
            ctx.tool,
            &ctx.cutting,
            op.kind,
        );
        builder.segments_wcs = crate::feed_opt::rebuild_segments_from_events(&builder.events);
    }

    // Automatic 2.5D overhang collision guard when a 3D model is loaded and opt-in avoidance is off
    if ctx.doc.has_3d_geometry()
        && !ctx.passes.avoid_model_overhangs
        && matches!(
            op.kind,
            OperationKind::Facing
                | OperationKind::Contour2D
                | OperationKind::Pocket2D
                | OperationKind::DrillBore
                | OperationKind::ChamferTrace
        )
    {
        let surface = SurfaceMesh3D::from_cad_document(ctx.doc, &[], &ctx.setup.frame);
        let min_ignored_z = match &strategy {
            StrategyParams::ChamferTrace(p)
                if p.mode == ChamferTraceMode::EdgeChamfer
                    && matches!(p.chamfer_source, ChamferSourceMode::SharpEdge { .. }) =>
            {
                Some(ctx.selection.inferred_top_z_wcs)
            }
            StrategyParams::ChamferTrace(p)
                if p.mode == ChamferTraceMode::CurveTrace
                    && p.trace_compensation == ContourCompensation::CenterOff =>
            {
                Some(ctx.selection.inferred_top_z_wcs)
            }
            _ => None,
        };
        if let Some(report) = check_2d_toolpath_against_overhangs(
            &surface,
            ctx.tool,
            &builder.segments_wcs,
            min_ignored_z,
        ) {
            builder
                .warnings
                .push(report.format_warning(op.kind.label()));
        }
    }

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
