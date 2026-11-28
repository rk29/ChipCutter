//! UI-agnostic, physics-backed **Machinability & Tool-Breakage Advisor** (`chip-cam`).
//!
//! Evaluates every non-suppressed [`Operation`] against the project's [`WorkpieceMaterialProfile`],
//! the active [`MachineDefinition`]'s spindle power/torque curve, and the cutter's true
//! flute-count-aware stepped cantilever stiffness—surfacing spindle stall risks, carbide snap
//! hazards, thin-chip rubbing / dry aluminum chip-welding, helical bore inner-arc wall overload,
//! deep full-depth bore finish passes, deep-hole chip packing, and slow open-air plunges,
//! together with deterministic 1-click [`MachinabilityFix`] mutations.

use chip_cad::CadDocument;
use chip_machines::MachineDefinition;
use chip_math::{DisplayUnit, EvalContext, ParamLength};
use chip_sketch::SketchLayer;
use chip_tools::{
    CoolantMode, CutterGeometry, Tool, WorkpieceMaterialProfile, compute_adaptive_thinning_feed,
};
use serde::{Deserialize, Serialize};

use crate::{
    heights::ResolvedHeights,
    machinability_physics::{
        CutPhysicsInput, CutPhysicsMetrics, ToolSubstrate, available_spindle_power_watts,
        cantilever_deflection_and_stress, compute_engagement_physics,
    },
    operation::{Operation, OperationKind, ToolOperationClass, cutter_supports_operation},
    params::{
        ChamferSourceMode, DrillBoreParams, EntryStrategy, HoleMachiningMode, PocketBoundaryMode,
        PocketStrategyKind, StrategyParams,
    },
    selection::{ExtractedFeature25D, extract_selection_features_with_sketches},
    setup::{ResolvedSetup, Setup},
    strategies::compute_z_levels,
};

/// Ordered severity level for a machinability check or operation summary.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default, Serialize, Deserialize,
)]
#[serde(rename_all = "snake_case")]
pub enum MachinabilitySeverity {
    #[default]
    Ok,
    Info,
    Warning,
    Critical,
}

impl MachinabilitySeverity {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Ok => "OK",
            Self::Info => "INFO",
            Self::Warning => "WARN",
            Self::Critical => "CRITICAL",
        }
    }
}

/// Classification of a structured physics-backed machinability finding.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MachinabilityCheckKind {
    SpindlePowerStall,
    ToolDeflectionSnap,
    ThinChipRubbingWeld,
    ChiploadOverload,
    HelicalInnerArcOverload,
    DeepBoreFullDepthFinish,
    DeepHoleChipEvacuation,
    SlowOpenAirPlunge,
}

/// Deterministic, typed 1-click fix applicable to an [`Operation`] via [`apply_machinability_fix`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum MachinabilityFix {
    SetMaxStepdown { stepdown_mm: f64 },
    SetStepover { stepover_mm: f64 },
    SetCuttingFeed { feed_mm_min: f64 },
    SetRampFeed { feed_mm_min: f64 },
    SetPlungeFeed { feed_mm_min: f64 },
    EnableHelicalInnerArcComp,
    DisableDeepBoreFinishPass,
    EnableBorePeckRetract { clear_depth_mm: f64 },
}

impl MachinabilityFix {
    /// Concise operator-facing label for GUI buttons and CLI reports.
    pub fn label(&self) -> String {
        match *self {
            Self::SetMaxStepdown { stepdown_mm } => {
                format!("Set max stepdown to {stepdown_mm:.2} mm")
            }
            Self::SetStepover { stepover_mm } => {
                format!("Set stepover to {stepover_mm:.2} mm")
            }
            Self::SetCuttingFeed { feed_mm_min } => {
                format!("Set cutting feed to {feed_mm_min:.0} mm/min")
            }
            Self::SetRampFeed { feed_mm_min } => {
                format!("Set ramp feed to {feed_mm_min:.0} mm/min")
            }
            Self::SetPlungeFeed { feed_mm_min } => {
                format!("Set plunge feed to {feed_mm_min:.0} mm/min")
            }
            Self::EnableHelicalInnerArcComp => {
                "Enable helical inner-arc feed compensation".to_string()
            }
            Self::DisableDeepBoreFinishPass => {
                "Disable deep full-depth bore finish pass".to_string()
            }
            Self::EnableBorePeckRetract { clear_depth_mm } => {
                format!("Enable {clear_depth_mm:.1} mm bore chip-clear retract")
            }
        }
    }
}

/// Single structured machinability diagnostic attached to an operation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MachinabilityFinding {
    pub severity: MachinabilitySeverity,
    pub kind: MachinabilityCheckKind,
    pub title: String,
    pub detail: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub suggested_fix: Option<MachinabilityFix>,
}

/// Full cutting-physics telemetry and advisory findings for one [`Operation`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OperationMachinabilityReport {
    pub setup_id: String,
    pub setup_name: String,
    pub op_id: String,
    pub op_name: String,
    pub op_kind: OperationKind,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool_name: Option<String>,
    pub suppressed: bool,
    pub severity: MachinabilitySeverity,
    pub axial_depth_mm: f64,
    pub radial_engagement_mm: f64,
    pub total_cut_depth_mm: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub metrics: Option<CutPhysicsMetrics>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub findings: Vec<MachinabilityFinding>,
}

impl OperationMachinabilityReport {
    /// Returns the highest-severity finding that carries a deterministic 1-click safe fix.
    pub fn primary_fixable_finding(&self) -> Option<&MachinabilityFinding> {
        self.findings
            .iter()
            .filter(|f| f.suggested_fix.is_some())
            .max_by_key(|f| f.severity)
    }
}

/// Aggregated machinability analysis across all operations in a [`Setup`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SetupMachinabilityReport {
    pub setup_id: String,
    pub setup_name: String,
    pub severity: MachinabilitySeverity,
    pub critical_count: usize,
    pub warning_count: usize,
    pub info_count: usize,
    pub operations: Vec<OperationMachinabilityReport>,
}

/// Context passed into [`analyze_operation_machinability`].
#[derive(Debug, Clone, Copy)]
pub struct OperationMachinabilityContext<'a> {
    pub setup_id: &'a str,
    pub setup_name: &'a str,
    pub resolved_setup: Option<&'a ResolvedSetup>,
    pub cad_doc: Option<&'a CadDocument>,
    pub sketches: &'a [SketchLayer],
    pub material: &'a WorkpieceMaterialProfile,
    pub machine: &'a MachineDefinition,
    pub display_unit: DisplayUnit,
}

#[derive(Debug, Clone, Copy)]
struct ResolvedHoleMetrics {
    smallest_diameter_mm: f64,
    max_depth_mm: f64,
}

#[derive(Debug, Clone, Copy)]
struct BoreAnalysisGeometry {
    hole_d: f64,
    hole_depth: f64,
    r_final: f64,
    r_core: f64,
    fin_allowance: f64,
    pitch_mm: f64,
    has_finish_pass: bool,
    inner_arc_comp: bool,
    has_chip_clear: bool,
}

fn round_clean_mm(v: f64) -> f64 {
    (v * 100.0).round() / 100.0
}

fn round_clean_feed(v: f64) -> f64 {
    (v / 5.0).round().max(1.0) * 5.0
}

fn resolve_cut_depth_and_holes(
    op: &Operation,
    ctx: &OperationMachinabilityContext<'_>,
) -> (Option<f64>, Option<ResolvedHoleMetrics>) {
    let Some(rs) = ctx.resolved_setup else {
        return (None, None);
    };

    let empty_cad = CadDocument::empty();
    let doc = ctx.cad_doc.unwrap_or(&empty_cad);
    let extracted = if !op.selections.is_empty() {
        extract_selection_features_with_sketches(doc, ctx.sketches, rs, &op.selections).ok()
    } else {
        None
    };

    let stock_aabb = rs.stock.aabb_wcs();
    let sel_top = extracted
        .as_ref()
        .map_or(stock_aabb.max.z, |e| e.inferred_top_z_wcs);
    let sel_bot = extracted
        .as_ref()
        .map_or(rs.model_aabb_wcs.min.z, |e| e.inferred_bottom_z_wcs);

    let resolved_heights = ResolvedHeights::resolve(
        &op.heights,
        &rs.model_aabb_wcs,
        &stock_aabb,
        rs.clearance_z_wcs,
        rs.retract_z_wcs,
        sel_top,
        sel_bot,
    )
    .ok();

    let total_depth = resolved_heights.map(|h| (h.top_z_wcs - h.bottom_z_wcs).max(0.0));

    let holes = extracted.as_ref().and_then(|e| {
        let z_top_shift = resolved_heights.map_or(0.0, |h| h.top_z_wcs - e.inferred_top_z_wcs);
        let z_bot_shift =
            resolved_heights.map_or(0.0, |h| h.bottom_z_wcs - e.inferred_bottom_z_wcs);

        let mut min_d = f64::INFINITY;
        let mut max_depth = 0.0_f64;
        let mut found = false;
        for feat in &e.features {
            if let ExtractedFeature25D::CylindricalHole {
                diameter_mm,
                top_z_wcs,
                bottom_z_wcs,
                ..
            } = *feat
            {
                found = true;
                min_d = min_d.min(diameter_mm);
                let h_top = top_z_wcs + z_top_shift;
                let h_bot = bottom_z_wcs + z_bot_shift + op.passes.stock_to_leave_axial.value_mm;
                max_depth = max_depth.max((h_top - h_bot).max(0.0));
            }
        }
        found.then_some(ResolvedHoleMetrics {
            smallest_diameter_mm: min_d,
            max_depth_mm: max_depth,
        })
    });

    (total_depth, holes)
}

fn effective_pass_stepdown(op: &Operation, total_depth_opt: Option<f64>, fallback_ap: f64) -> f64 {
    let max_sd = op.passes.max_stepdown.value_mm;
    let max_sd_valid = if max_sd.is_finite() && max_sd > 1e-4 {
        max_sd
    } else {
        fallback_ap.max(0.1)
    };
    let Some(total_depth) = total_depth_opt.filter(|&d| d.is_finite() && d > 1e-4) else {
        return max_sd_valid;
    };
    if !op.passes.multiple_depths {
        return total_depth;
    }
    if let Ok(levels) = compute_z_levels(
        total_depth,
        0.0,
        op.passes.multiple_depths,
        max_sd_valid,
        op.passes.even_stepdowns,
    ) && !levels.is_empty()
    {
        let mut prev = total_depth;
        let mut max_dz = 0.0_f64;
        for z in levels {
            max_dz = max_dz.max((prev - z).max(0.0));
            prev = z;
        }
        if max_dz > 1e-4 {
            return max_dz;
        }
    }
    total_depth.min(max_sd_valid)
}

fn is_helical_bore_op(tool: &Tool, params: &DrillBoreParams, hole_d: Option<f64>) -> bool {
    match params.mode {
        HoleMachiningMode::Drill => false,
        HoleMachiningMode::HelicalBore => true,
        HoleMachiningMode::AutoByTool => {
            let can_bore =
                cutter_supports_operation(&tool.geometry, ToolOperationClass::HelicalBore);
            let is_drill_geom = matches!(
                tool.geometry,
                CutterGeometry::TwistDrill { .. } | CutterGeometry::ChamferMill { .. }
            );
            let td = tool.geometry.cutting_diameter_mm();
            can_bore && !is_drill_geom && hole_d.is_none_or(|hd| td < hd - 0.05)
        }
    }
}

fn resolve_bore_geometry(
    op: &Operation,
    tool: &Tool,
    params: &DrillBoreParams,
    holes: Option<ResolvedHoleMetrics>,
    total_depth_opt: Option<f64>,
) -> Option<BoreAnalysisGeometry> {
    let tool_d = tool.geometry.cutting_diameter_mm().max(1e-4);
    let tool_r = 0.5 * tool_d;
    let hole_d = holes.map(|h| h.smallest_diameter_mm)?;
    if !hole_d.is_finite() || hole_d <= tool_d + 0.04 {
        return None;
    }
    let hole_depth = holes
        .map(|h| h.max_depth_mm)
        .filter(|&d| d > 1e-4)
        .or(total_depth_opt)
        .unwrap_or_else(|| op.passes.max_stepdown.value_mm.max(tool_d));

    let radial_leave = op.passes.stock_to_leave_radial.value_mm.max(0.0);
    let r_final = (0.5 * hole_d - tool_r - radial_leave).max(0.02);
    let fin_allowance =
        if params.bore_finish_pass && r_final > params.bore_finish_stepover.value_mm + 0.05 {
            params.bore_finish_stepover.value_mm.max(0.02)
        } else {
            0.0
        };
    let r_rough = (r_final - fin_allowance).max(0.02);
    let r_core = r_rough.min(0.42 * tool_d);
    let angle_rad = params.bore_ramp_angle_deg.clamp(0.5, 20.0).to_radians();
    let circumference = std::f64::consts::TAU * r_core;
    let pitch_mm = match &params.bore_pitch_per_rev {
        Some(p) if p.value_mm > 0.05 => p.value_mm,
        _ => {
            (circumference * angle_rad.tan()).clamp(0.15, op.passes.max_stepdown.value_mm.max(0.5))
        }
    };

    Some(BoreAnalysisGeometry {
        hole_d,
        hole_depth,
        r_final,
        r_core,
        fin_allowance,
        pitch_mm,
        has_finish_pass: params.bore_finish_pass && fin_allowance > 1e-5,
        inner_arc_comp: params.bore_inner_arc_feed_comp,
        has_chip_clear: params
            .bore_chip_clear_depth
            .as_ref()
            .is_some_and(|d| d.value_mm.is_finite() && d.value_mm > 0.5),
    })
}

fn should_report_slow_open_air_plunge(
    op: &Operation,
    strategy: &StrategyParams,
    tool_d: f64,
    cutting_feed: f64,
    plunge_feed: f64,
) -> bool {
    if !cutting_feed.is_finite()
        || !plunge_feed.is_finite()
        || cutting_feed <= 100.0
        || plunge_feed <= 0.0
    {
        return false;
    }

    match strategy {
        StrategyParams::Contour2D(p) => {
            p.open_chain_extension.value_mm.is_finite()
                && p.open_chain_extension.value_mm > 0.1
                && plunge_feed < 0.40 * cutting_feed
        }
        StrategyParams::Pocket2D(p) => {
            matches!(
                p.boundary_mode,
                PocketBoundaryMode::ClearOutsideSelectedToStock
            ) && matches!(op.linking.entry_mode, EntryStrategy::Plunge)
                && plunge_feed < 0.40 * cutting_feed
        }
        StrategyParams::Facing(p) => {
            matches!(op.linking.entry_mode, EntryStrategy::Plunge)
                && p.pass_extension.value_mm.is_finite()
                && p.pass_extension.value_mm > 0.5 * tool_d
                && plunge_feed < 0.30 * cutting_feed
        }
        _ => false,
    }
}

/// Evaluates cutting physics and all 7 machinability & tool-breakage safety checks for a single
/// [`Operation`].
#[allow(clippy::too_many_lines)]
pub fn analyze_operation_machinability(
    op: &Operation,
    tool: Option<&Tool>,
    ctx: &OperationMachinabilityContext<'_>,
) -> OperationMachinabilityReport {
    let base_report = OperationMachinabilityReport {
        setup_id: ctx.setup_id.to_string(),
        setup_name: ctx.setup_name.to_string(),
        op_id: op.id.clone(),
        op_name: op.name.clone(),
        op_kind: op.kind,
        tool_id: op.tool_id.as_ref().map(|id| id.0.clone()),
        tool_name: tool.map(|t| t.name.clone()),
        suppressed: op.suppressed,
        severity: MachinabilitySeverity::Ok,
        axial_depth_mm: 0.0,
        radial_engagement_mm: 0.0,
        total_cut_depth_mm: 0.0,
        metrics: None,
        findings: Vec::new(),
    };

    if op.suppressed {
        return base_report;
    }
    let Some(tool) = tool else {
        return base_report;
    };

    // Re-evaluate inline parameter formulas against the active tool & stock (on a local clone)
    let mut op_eval = op.clone();
    let mut eval_ctx = ctx.resolved_setup.map_or_else(
        || EvalContext::new(ctx.display_unit),
        |rs| rs.eval_context(ctx.display_unit),
    );
    tool.populate_eval_context(&mut eval_ctx);
    op_eval.reevaluate_all(&eval_ctx);

    let cutting = op_eval.effective_cutting_params(tool);
    let strategy = op_eval.resolved_strategy();
    let tool_d = tool.geometry.cutting_diameter_mm().max(1e-4);
    let tool_r = 0.5 * tool_d;
    let flutes = tool.geometry.flutes().max(1);

    let (total_depth_opt, holes_opt) = resolve_cut_depth_and_holes(&op_eval, ctx);
    let total_cut_depth_mm = holes_opt
        .map(|h| h.max_depth_mm)
        .filter(|&d| d > 1e-4)
        .or(total_depth_opt)
        .unwrap_or_else(|| {
            op_eval
                .passes
                .max_stepdown
                .value_mm
                .max(cutting.stepdown_mm)
        });

    // Determine effective ap, ae, feedrate, and bore parameters per strategy kind
    let mut is_pure_drill = false;
    let mut bore_geom: Option<BoreAnalysisGeometry> = None;
    let mut effective_feed = cutting.cutting_feed_mm_min;
    let (ap, ae) = match &strategy {
        StrategyParams::Facing(p) => {
            let ap = effective_pass_stepdown(&op_eval, total_depth_opt, cutting.stepdown_mm);
            let ae = p.stepover.value_mm.clamp(0.02, tool_d);
            (ap, ae)
        }
        StrategyParams::Contour2D(p) => {
            let ap = effective_pass_stepdown(&op_eval, total_depth_opt, cutting.stepdown_mm);
            let ae = if let Some(ov_ae) = op_eval.feed_overrides.stepover_mm {
                ov_ae.clamp(0.02, tool_d)
            } else if p.roughing_passes > 1 {
                p.roughing_stepover.value_mm.clamp(0.02, tool_d)
            } else if p.finish_passes > 0 && p.roughing_passes == 0 {
                p.finish_stepover.value_mm.clamp(0.02, tool_d)
            } else if p.open_chain_extension.value_mm > 0.1 {
                p.roughing_stepover.value_mm.clamp(0.02, tool_d)
            } else {
                tool_d
            };
            (ap, ae)
        }
        StrategyParams::Pocket2D(p) => {
            let ap = effective_pass_stepdown(&op_eval, total_depth_opt, cutting.stepdown_mm);
            let ae = match &p.strategy {
                PocketStrategyKind::Adaptive(a) => {
                    let opt_load = a.optimal_load.value_mm.clamp(0.02, tool_d);
                    if a.auto_chip_thinning {
                        effective_feed = compute_adaptive_thinning_feed(
                            tool_d,
                            opt_load,
                            cutting.cutting_feed_mm_min,
                            a.max_thinning_feed_multiplier,
                        );
                    }
                    opt_load
                }
                PocketStrategyKind::ContourParallel | PocketStrategyKind::ZigZag { .. } => {
                    p.stepover.value_mm.clamp(0.02, tool_d)
                }
            };
            (ap, ae)
        }
        StrategyParams::DrillBore(p) => {
            let hole_d = holes_opt.map(|h| h.smallest_diameter_mm);
            if is_helical_bore_op(tool, p, hole_d) {
                bore_geom = resolve_bore_geometry(&op_eval, tool, p, holes_opt, total_depth_opt);
                if let Some(bg) = bore_geom {
                    (bg.pitch_mm, (bg.r_core + tool_r).min(tool_d))
                } else {
                    (
                        op_eval
                            .passes
                            .max_stepdown
                            .value_mm
                            .min(tool_d * 0.5)
                            .max(0.15),
                        tool_d * 0.5,
                    )
                }
            } else {
                is_pure_drill = true;
                (
                    p.peck_depth
                        .value_mm
                        .clamp(0.2, total_cut_depth_mm.max(0.5)),
                    0.5 * tool_d,
                )
            }
        }
        StrategyParams::ChamferTrace(p) => {
            let w = match &p.chamfer_source {
                ChamferSourceMode::SharpEdge { width } => width.value_mm.clamp(0.05, 0.5 * tool_d),
                ChamferSourceMode::ModeledFace => (0.15 * tool_d).clamp(0.1, 1.5),
            };
            (w, w)
        }
        StrategyParams::VCarve(p) => {
            let depth = p
                .flat_depth
                .as_ref()
                .map_or(total_cut_depth_mm, |fd| fd.value_mm)
                .clamp(0.1, op_eval.passes.max_stepdown.value_mm.max(0.5));
            (depth, (0.5 * tool_d).max(0.1))
        }
        StrategyParams::Adaptive3D(p) => {
            let ap = effective_pass_stepdown(&op_eval, total_depth_opt, cutting.stepdown_mm);
            let ae = p.adaptive.optimal_load.value_mm.clamp(0.02, tool_d);
            (ap, ae)
        }
        StrategyParams::Waterline3D(_) => {
            let ap = effective_pass_stepdown(&op_eval, total_depth_opt, cutting.stepdown_mm);
            let ae = cutting.stepover_mm.clamp(0.02, tool_d);
            (ap, ae)
        }
        StrategyParams::Parallel3D(p) => (
            cutting.stepdown_mm.clamp(0.05, 0.35 * tool_d),
            p.stepover.value_mm.clamp(0.02, tool_d),
        ),
        StrategyParams::Scallop3D(p) => (
            cutting.stepdown_mm.clamp(0.05, 0.35 * tool_d),
            p.max_stepover.value_mm.clamp(0.02, tool_d),
        ),
        StrategyParams::Pencil3D(p) => (
            cutting.stepdown_mm.clamp(0.05, 0.25 * tool_d),
            p.offset_stepover.value_mm.clamp(0.02, tool_d),
        ),
        StrategyParams::Deburr3D(p) => {
            let w = p.chamfer_width.value_mm.clamp(0.05, 0.4 * tool_d);
            (w, w)
        }
        StrategyParams::Undercut3D(p) => (
            p.z_stepdown.value_mm.clamp(0.05, tool_d),
            cutting.stepover_mm.clamp(0.02, 0.5 * tool_d),
        ),
        StrategyParams::ThreadMill(p) => {
            let pitch = p.pitch.value_mm.clamp(0.2, 5.0);
            (
                pitch,
                (0.6134 * pitch / f64::from(p.radial_passes.max(1))).clamp(0.02, 0.5 * tool_d),
            )
        }
        StrategyParams::SlotUndercut(p) => (
            tool.geometry.flute_length_mm().clamp(0.1, tool_d),
            p.radial_stepover.value_mm.clamp(0.02, tool_d),
        ),
        StrategyParams::Dovetail(p) => (
            tool.geometry.flute_length_mm().clamp(0.1, tool_d),
            p.radial_stepover.value_mm.clamp(0.02, tool_d),
        ),
        StrategyParams::Rotary2Linear(p) => (
            effective_pass_stepdown(&op_eval, total_depth_opt, cutting.stepdown_mm),
            p.stepover.value_mm.clamp(0.02, tool_d),
        ),
        StrategyParams::Rotary4AxisSim(p) => (
            effective_pass_stepdown(&op_eval, total_depth_opt, cutting.stepdown_mm),
            p.stepover.value_mm.clamp(0.02, tool_d),
        ),
    };

    // Compute primary physics metrics
    let metrics = if is_pure_drill {
        // Axial twist-drill cutting mechanics: MRR = pi * r_tool^2 * plunge_feed
        let mrr_mm3_min =
            std::f64::consts::PI * tool_r * tool_r * cutting.plunge_feed_mm_min.max(0.0);
        let u_s = ctx.material.specific_cutting_energy_w_s_mm3.max(0.0);
        let cutting_power_watts = u_s * (mrr_mm3_min / 60.0);
        let avail_w = available_spindle_power_watts(&ctx.machine.spindle, cutting.spindle_rpm);
        let spindle_load_pct = avail_w
            .filter(|&w| w > 1e-6)
            .map(|w| (cutting_power_watts / w) * 100.0);
        let vc = chip_tools::surface_speed_from_rpm(cutting.spindle_rpm, tool_d).unwrap_or(0.0);
        let f_t = if vc > 1e-6 {
            cutting_power_watts / (vc / 60.0)
        } else {
            0.0
        };
        // Balanced symmetric flutes on twist drills produce minimal net radial bending (~10% F_t)
        let f_r = 0.10 * f_t;
        let fz =
            chip_tools::chipload_from_feed(cutting.spindle_rpm, flutes, cutting.plunge_feed_mm_min)
                .unwrap_or(0.0);
        let deflection = cantilever_deflection_and_stress(tool, f_r, 0.5 * ap, false);
        CutPhysicsMetrics {
            surface_speed_m_min: vc,
            programmed_center_feed_mm_min: cutting.plunge_feed_mm_min,
            effective_wall_feed_mm_min: cutting.plunge_feed_mm_min,
            inner_arc_feed_multiplier: 1.0,
            nominal_chipload_mm: fz,
            wall_chipload_mm: fz,
            engagement_arc_deg: 180.0,
            chip_thinning_ratio: 1.0,
            effective_chip_thickness_mm: fz,
            avg_teeth_in_cut: f64::from(flutes),
            interrupted_cut: false,
            mrr_cm3_min: mrr_mm3_min / 1_000.0,
            cutting_power_watts,
            available_spindle_power_watts: avail_w,
            spindle_load_pct,
            tangential_force_n: f_t,
            radial_force_n: f_r,
            peak_radial_force_n: f_r,
            deflection,
        }
    } else if let Some(bg) = bore_geom {
        // Helical bore roughing ramp metrics
        let rough_wall_r = bg.r_core + tool_r;
        let ramp_scale = if bg.inner_arc_comp {
            (bg.r_core / rough_wall_r.max(1e-4)).clamp(0.20, 1.0)
        } else {
            1.0
        };
        let ramp_metrics = compute_engagement_physics(&CutPhysicsInput {
            tool,
            material: ctx.material,
            spindle: Some(&ctx.machine.spindle),
            substrate: ToolSubstrate::Carbide,
            spindle_rpm: cutting.spindle_rpm,
            feed_mm_min: cutting.ramp_feed_mm_min * ramp_scale,
            axial_depth_mm: bg.pitch_mm,
            radial_engagement_mm: rough_wall_r.min(tool_d),
            bore_diameter_mm: Some(2.0 * rough_wall_r),
            is_helical_bore: true,
        });

        // If full-depth bore finish pass is enabled, also evaluate the concave full-depth wall pass
        // and retain the higher-risk metrics so deflection/stress/load hazards are never hidden!
        if bg.has_finish_pass {
            let finish_scale = if bg.inner_arc_comp {
                (bg.r_final / (0.5 * bg.hole_d).max(1e-4)).clamp(0.20, 1.0)
            } else {
                1.0
            };
            let finish_metrics = compute_engagement_physics(&CutPhysicsInput {
                tool,
                material: ctx.material,
                spindle: Some(&ctx.machine.spindle),
                substrate: ToolSubstrate::Carbide,
                spindle_rpm: cutting.spindle_rpm,
                feed_mm_min: cutting.cutting_feed_mm_min * finish_scale,
                axial_depth_mm: bg.hole_depth,
                radial_engagement_mm: bg.fin_allowance,
                bore_diameter_mm: Some(bg.hole_d),
                is_helical_bore: false,
            });
            if finish_metrics.deflection.bending_safety_factor
                < ramp_metrics.deflection.bending_safety_factor
                || finish_metrics.deflection.tip_deflection_mm
                    > ramp_metrics.deflection.tip_deflection_mm
                || finish_metrics.cutting_power_watts > ramp_metrics.cutting_power_watts
            {
                finish_metrics
            } else {
                ramp_metrics
            }
        } else {
            ramp_metrics
        }
    } else {
        compute_engagement_physics(&CutPhysicsInput {
            tool,
            material: ctx.material,
            spindle: Some(&ctx.machine.spindle),
            substrate: ToolSubstrate::Carbide,
            spindle_rpm: cutting.spindle_rpm,
            feed_mm_min: effective_feed,
            axial_depth_mm: ap,
            radial_engagement_mm: ae,
            bore_diameter_mm: None,
            is_helical_bore: false,
        })
    };

    let mut findings: Vec<MachinabilityFinding> = Vec::new();

    // 1. Spindle Power / Stall Risk (>70% Warning, >95% Critical)
    if let (Some(load_pct), Some(avail_w)) = (
        metrics.spindle_load_pct,
        metrics.available_spindle_power_watts,
    ) {
        if load_pct > 95.0 {
            let safe_scale = (60.0 / load_pct).clamp(0.15, 0.85);
            let suggested_fix = if bore_geom.is_none() && !is_pure_drill && ap > 0.2 {
                Some(MachinabilityFix::SetMaxStepdown {
                    stepdown_mm: round_clean_mm((ap * safe_scale).max(0.15)),
                })
            } else {
                Some(MachinabilityFix::SetCuttingFeed {
                    feed_mm_min: round_clean_feed(
                        (cutting.cutting_feed_mm_min * safe_scale).max(20.0),
                    ),
                })
            };
            findings.push(MachinabilityFinding {
                severity: MachinabilitySeverity::Critical,
                kind: MachinabilityCheckKind::SpindlePowerStall,
                title: format!("Spindle stall risk ({load_pct:.0}% load)"),
                detail: format!(
                    "Estimated cutting power {:.1} W exceeds available spindle power {:.1} W at {:.0} RPM (MRR {:.2} cm³/min on {}).",
                    metrics.cutting_power_watts,
                    avail_w,
                    cutting.spindle_rpm,
                    metrics.mrr_cm3_min,
                    ctx.machine.name
                ),
                suggested_fix,
            });
        } else if load_pct > 70.0 {
            let safe_scale = (60.0 / load_pct).clamp(0.35, 0.90);
            let suggested_fix = if bore_geom.is_none() && !is_pure_drill && ap > 0.2 {
                Some(MachinabilityFix::SetMaxStepdown {
                    stepdown_mm: round_clean_mm((ap * safe_scale).max(0.15)),
                })
            } else {
                Some(MachinabilityFix::SetCuttingFeed {
                    feed_mm_min: round_clean_feed(
                        (cutting.cutting_feed_mm_min * safe_scale).max(20.0),
                    ),
                })
            };
            findings.push(MachinabilityFinding {
                severity: MachinabilitySeverity::Warning,
                kind: MachinabilityCheckKind::SpindlePowerStall,
                title: format!("High spindle load ({load_pct:.0}%)"),
                detail: format!(
                    "Estimated cutting power {:.1} W is {:.0}% of available {:.1} W at {:.0} RPM on {}.",
                    metrics.cutting_power_watts,
                    load_pct,
                    avail_w,
                    cutting.spindle_rpm,
                    ctx.machine.name
                ),
                suggested_fix,
            });
        }
    }

    // 2. Tool Deflection & Carbide Bending-Stress Snap Hazard
    let defl = metrics.deflection;
    let is_critical_defl = defl.tip_deflection_mm > 0.060
        || defl.deflection_pct_diameter > 2.0
        || defl.bending_safety_factor < 1.5;
    let is_warn_defl = defl.tip_deflection_mm > 0.025
        || defl.deflection_pct_diameter > 0.8
        || defl.bending_safety_factor < 2.5;
    if !is_pure_drill && (is_critical_defl || is_warn_defl) {
        let severity = if is_critical_defl {
            MachinabilitySeverity::Critical
        } else {
            MachinabilitySeverity::Warning
        };
        let high_ld_note = if defl.stickout_ld_ratio > 6.0 {
            format!(
                " High stickout L/D = {:.1}x amplifies bending compliance cubically.",
                defl.stickout_ld_ratio
            )
        } else {
            String::new()
        };
        let shock_note = if metrics.interrupted_cut {
            " Interrupted tooth impact knockdown active."
        } else {
            ""
        };
        let suggested_fix = if let Some(bg) = bore_geom {
            if bg.has_finish_pass {
                Some(MachinabilityFix::DisableDeepBoreFinishPass)
            } else {
                Some(MachinabilityFix::SetRampFeed {
                    feed_mm_min: round_clean_feed((cutting.ramp_feed_mm_min * 0.60).max(20.0)),
                })
            }
        } else {
            let sf_ratio =
                if defl.bending_safety_factor.is_finite() && defl.bending_safety_factor > 0.0 {
                    (defl.bending_safety_factor / 2.8).min(1.0)
                } else {
                    1.0
                };
            let defl_ratio = if defl.tip_deflection_mm > 1e-6 {
                (0.020_f64.min(0.007 * tool_d) / defl.tip_deflection_mm).min(1.0)
            } else {
                1.0
            };
            let safe_ap = round_clean_mm((ap * sf_ratio.min(defl_ratio)).clamp(0.10, ap));
            Some(MachinabilityFix::SetMaxStepdown {
                stepdown_mm: safe_ap,
            })
        };
        findings.push(MachinabilityFinding {
            severity,
            kind: MachinabilityCheckKind::ToolDeflectionSnap,
            title: if is_critical_defl {
                format!(
                    "Tool snap / severe deflection risk ({:.0} µm, SF {:.2})",
                    defl.tip_deflection_um, defl.bending_safety_factor
                )
            } else {
                format!(
                    "Elevated tool deflection ({:.0} µm, SF {:.2})",
                    defl.tip_deflection_um, defl.bending_safety_factor
                )
            },
            detail: format!(
                "Tip deflection {:.1} µm ({:.2}% D), peak bending stress {:.0} MPa vs {:.0} MPa allowable (SF {:.2}, core Ø{:.2} mm).{}{}",
                defl.tip_deflection_um,
                defl.deflection_pct_diameter,
                defl.max_bending_stress_mpa,
                defl.allowable_stress_mpa,
                defl.bending_safety_factor,
                defl.effective_core_diameter_mm,
                high_ld_note,
                shock_note
            ),
            suggested_fix,
        });
    }

    // 3. Thin-Chip Rubbing & Dry Aluminum/6063 Chip-Welding + Chipload Overload
    let skip_thin_chip = is_pure_drill
        || matches!(
            op_eval.kind,
            OperationKind::ChamferTrace | OperationKind::Deburr3D | OperationKind::VCarve
        );
    if !skip_thin_chip && metrics.effective_chip_thickness_mm > 0.0 {
        let h_min = ctx.material.min_chip_thickness_mm;
        if metrics.effective_chip_thickness_mm < h_min * 0.99 {
            let weld_note = if ctx.material.gummy_chip_weld_risk {
                let dry_note = if matches!(
                    cutting.coolant,
                    CoolantMode::Disabled | CoolantMode::AirBlast
                ) {
                    " (dry/air-blast cut increases flute welding risk)"
                } else {
                    ""
                };
                format!(" {} is prone to chip-welding{dry_note}.", ctx.material.name)
            } else {
                String::new()
            };
            let ratio = metrics.chip_thinning_ratio.max(0.15);
            let target_fz = (1.20 * h_min) / ratio;
            let max_safe_fz = ctx.material.max_recommended_chipload_factor * tool_d;
            let target_feed =
                (target_fz.min(max_safe_fz) * cutting.spindle_rpm * f64::from(flutes))
                    .min(ctx.machine.kinematics.max_cut_feed_xy_mm_min);
            findings.push(MachinabilityFinding {
                severity: MachinabilitySeverity::Warning,
                kind: MachinabilityCheckKind::ThinChipRubbingWeld,
                title: format!(
                    "Thin-chip rubbing (h_max {:.4} < h_min {:.4} mm)",
                    metrics.effective_chip_thickness_mm, h_min
                ),
                detail: format!(
                    "Effective chip thickness {:.4} mm (thinning {:.0}% at ae={:.2} mm) is below {} minimum {:.4} mm; edge will rub and generate heat.{}",
                    metrics.effective_chip_thickness_mm,
                    metrics.chip_thinning_ratio * 100.0,
                    ae,
                    ctx.material.name,
                    h_min,
                    weld_note
                ),
                suggested_fix: (target_feed > cutting.cutting_feed_mm_min + 5.0).then_some(
                    MachinabilityFix::SetCuttingFeed {
                        feed_mm_min: round_clean_feed(target_feed),
                    },
                ),
            });
        }

        // Nominal/wall chipload overload (for straight cuts or already-compensated bores)
        let max_fz = ctx.material.max_recommended_chipload_factor * tool_d;
        let is_uncomp_bore = bore_geom.is_some_and(|bg| !bg.inner_arc_comp);
        if !is_uncomp_bore && max_fz > 1e-6 && metrics.wall_chipload_mm > max_fz * 1.15 {
            let sev = if metrics.wall_chipload_mm > max_fz * 1.50 {
                MachinabilitySeverity::Critical
            } else {
                MachinabilitySeverity::Warning
            };
            let safe_feed = round_clean_feed(
                (max_fz * 0.90 * cutting.spindle_rpm * f64::from(flutes)).max(20.0),
            );
            findings.push(MachinabilityFinding {
                severity: sev,
                kind: MachinabilityCheckKind::ChiploadOverload,
                title: format!(
                    "Chipload overload ({:.4} > {:.4} mm/tooth)",
                    metrics.wall_chipload_mm, max_fz
                ),
                detail: format!(
                    "Effective chipload {:.4} mm/tooth exceeds recommended maximum {:.4} mm/tooth for {} with Ø{:.2} mm cutter.",
                    metrics.wall_chipload_mm, max_fz, ctx.material.name, tool_d
                ),
                suggested_fix: Some(MachinabilityFix::SetCuttingFeed {
                    feed_mm_min: safe_feed,
                }),
            });
        }
    }

    // 4, 5, 6. Helical Bore Specific Safety Checks
    if let Some(bg) = bore_geom {
        // Check 4: Helical bore inner-arc wall overload (`R_hole / r_orbit > 1.35x` when uncompensated)
        let r_hole = 0.5 * bg.hole_d;
        let worst_mult =
            (r_hole / bg.r_final.max(1e-4)).max((bg.r_core + tool_r) / bg.r_core.max(1e-4));
        if !bg.inner_arc_comp && worst_mult > 1.35 {
            let wall_feed = cutting.cutting_feed_mm_min * worst_mult;
            let wall_fz = chip_tools::chipload_from_feed(cutting.spindle_rpm, flutes, wall_feed)
                .unwrap_or(0.0);
            let max_fz = ctx.material.max_recommended_chipload_factor * tool_d;
            let sev = if wall_fz > max_fz * 1.15 || worst_mult > 2.0 {
                MachinabilitySeverity::Critical
            } else {
                MachinabilitySeverity::Warning
            };
            findings.push(MachinabilityFinding {
                severity: sev,
                kind: MachinabilityCheckKind::HelicalInnerArcOverload,
                title: format!(
                    "Helical bore inner-arc wall overload ({worst_mult:.2}x feed amplification)"
                ),
                detail: format!(
                    "Uncompensated Ø{:.2} mm bore with Ø{:.3} mm cutter drives cutting edge at {:.0} mm/min ({:.4} mm/tooth vs {:.4} mm/tooth limit) because R_hole/r_orbit = {:.2}x.",
                    bg.hole_d, tool_d, wall_feed, wall_fz, max_fz, worst_mult
                ),
                suggested_fix: Some(MachinabilityFix::EnableHelicalInnerArcComp),
            });
        }

        // Check 5: Helical bore full-depth radial finish hazard (`bore_finish_pass = true` when depth > 2.5 * tool_d)
        if bg.has_finish_pass && bg.hole_depth > 2.5 * tool_d {
            let sev = if defl.stickout_ld_ratio > 6.0 || flutes <= 1 || bg.hole_depth > 4.0 * tool_d
            {
                MachinabilitySeverity::Critical
            } else {
                MachinabilitySeverity::Warning
            };
            findings.push(MachinabilityFinding {
                severity: sev,
                kind: MachinabilityCheckKind::DeepBoreFullDepthFinish,
                title: format!(
                    "Full-depth bore finish pass hazard ({:.1} mm > 2.5x D)",
                    bg.hole_depth
                ),
                detail: format!(
                    "bore_finish_pass takes the full {:.1} mm depth ({:.1}x tool diameter) in one concave 360° wall lap (L/D = {:.1}x, {}FL); high chatter & cutter-snap hazard.",
                    bg.hole_depth,
                    bg.hole_depth / tool_d,
                    defl.stickout_ld_ratio,
                    flutes
                ),
                suggested_fix: Some(MachinabilityFix::DisableDeepBoreFinishPass),
            });
        }

        // Check 6: Deep-hole chip-evacuation warning (`depth > 2.5 * hole_d` & annulus < 1.5 mm/side without chip-clear depth)
        let annulus_mm = 0.5 * (bg.hole_d - tool_d);
        if bg.hole_depth > 2.5 * bg.hole_d && annulus_mm < 1.5 && !bg.has_chip_clear {
            let clear_depth_mm = (2.0 * bg.hole_d).clamp(3.0, 10.0);
            findings.push(MachinabilityFinding {
                severity: MachinabilitySeverity::Warning,
                kind: MachinabilityCheckKind::DeepHoleChipEvacuation,
                title: format!(
                    "Deep-hole chip packing risk ({:.1} mm depth, {:.2} mm annulus)",
                    bg.hole_depth, annulus_mm
                ),
                detail: format!(
                    "Helical bore depth {:.1} mm exceeds 2.5x hole diameter (Ø{:.2} mm) with only {:.2} mm/side radial chip clearance and no intermediate chip-clearing retract.",
                    bg.hole_depth, bg.hole_d, annulus_mm
                ),
                suggested_fix: Some(MachinabilityFix::EnableBorePeckRetract { clear_depth_mm }),
            });
        }
    }

    // 7. Slow Open-Air Plunge Advisory (`Info`)
    if should_report_slow_open_air_plunge(
        &op_eval,
        &strategy,
        tool_d,
        cutting.cutting_feed_mm_min,
        cutting.plunge_feed_mm_min,
    ) {
        let faster_plunge = round_clean_feed(
            (0.80 * cutting.cutting_feed_mm_min)
                .min(ctx.machine.kinematics.max_cut_feed_z_mm_min)
                .max(cutting.plunge_feed_mm_min + 20.0),
        );
        findings.push(MachinabilityFinding {
            severity: MachinabilitySeverity::Info,
            kind: MachinabilityCheckKind::SlowOpenAirPlunge,
            title: format!(
                "Slow open-air plunge ({:.0} mm/min)",
                cutting.plunge_feed_mm_min
            ),
            detail: format!(
                "Entry plunge occurs outside stock bounds at {:.0} mm/min ({:.0}% of cutting feed {:.0} mm/min); safe to speed up open-air plunge to {:.0} mm/min.",
                cutting.plunge_feed_mm_min,
                (cutting.plunge_feed_mm_min / cutting.cutting_feed_mm_min.max(1e-6)) * 100.0,
                cutting.cutting_feed_mm_min,
                faster_plunge
            ),
            suggested_fix: Some(MachinabilityFix::SetPlungeFeed {
                feed_mm_min: faster_plunge,
            }),
        });
    }

    let severity = findings
        .iter()
        .map(|f| f.severity)
        .max()
        .unwrap_or(MachinabilitySeverity::Ok);

    OperationMachinabilityReport {
        setup_id: ctx.setup_id.to_string(),
        setup_name: ctx.setup_name.to_string(),
        op_id: op.id.clone(),
        op_name: op.name.clone(),
        op_kind: op.kind,
        tool_id: op.tool_id.as_ref().map(|id| id.0.clone()),
        tool_name: Some(tool.name.clone()),
        suppressed: false,
        severity,
        axial_depth_mm: ap,
        radial_engagement_mm: ae,
        total_cut_depth_mm,
        metrics: Some(metrics),
        findings,
    }
}

/// Evaluates machinability across every operation in a [`Setup`].
pub fn analyze_setup_machinability(
    setup: &Setup,
    tools: &[Tool],
    material: &WorkpieceMaterialProfile,
    machine: &MachineDefinition,
    cad_doc: Option<&CadDocument>,
    sketches: &[SketchLayer],
    display_unit: DisplayUnit,
) -> SetupMachinabilityReport {
    let empty_cad = CadDocument::empty();
    let doc = cad_doc.unwrap_or(&empty_cad);
    let resolved_setup = setup
        .resolve_with_sketches(doc, sketches, display_unit)
        .ok();

    let ctx = OperationMachinabilityContext {
        setup_id: &setup.id,
        setup_name: &setup.name,
        resolved_setup: resolved_setup.as_ref(),
        cad_doc,
        sketches,
        material,
        machine,
        display_unit,
    };

    let mut operations = Vec::with_capacity(setup.operations.len());
    let mut severity = MachinabilitySeverity::Ok;
    let mut critical_count = 0usize;
    let mut warning_count = 0usize;
    let mut info_count = 0usize;

    for op in &setup.operations {
        let tool = op
            .tool_id
            .as_ref()
            .and_then(|tid| tools.iter().find(|t| &t.id == tid));
        let rep = analyze_operation_machinability(op, tool, &ctx);
        severity = severity.max(rep.severity);
        for f in &rep.findings {
            match f.severity {
                MachinabilitySeverity::Critical => critical_count += 1,
                MachinabilitySeverity::Warning => warning_count += 1,
                MachinabilitySeverity::Info => info_count += 1,
                MachinabilitySeverity::Ok => {}
            }
        }
        operations.push(rep);
    }

    SetupMachinabilityReport {
        setup_id: setup.id.clone(),
        setup_name: setup.name.clone(),
        severity,
        critical_count,
        warning_count,
        info_count,
        operations,
    }
}

/// Applies a deterministic [`MachinabilityFix`] to `op` and marks it `Dirty`.
///
/// Returns `true` when the operation was mutated, or `false` if the fix is invalid or does not
/// apply to `op.kind`.
pub fn apply_machinability_fix(op: &mut Operation, fix: &MachinabilityFix) -> bool {
    match *fix {
        MachinabilityFix::SetMaxStepdown { stepdown_mm } => {
            if !stepdown_mm.is_finite() || stepdown_mm <= 0.0 {
                return false;
            }
            op.passes.multiple_depths = true;
            op.passes.max_stepdown = ParamLength::from_mm(stepdown_mm);
            if op.feed_overrides.stepdown_mm.is_some() {
                op.feed_overrides.stepdown_mm = Some(stepdown_mm);
            }
            op.mark_dirty();
            true
        }
        MachinabilityFix::SetStepover { stepover_mm } => {
            if !stepover_mm.is_finite() || stepover_mm <= 0.0 {
                return false;
            }
            match op.strategy_mut() {
                StrategyParams::Facing(p) => p.stepover = ParamLength::from_mm(stepover_mm),
                StrategyParams::Contour2D(p) => {
                    p.roughing_stepover = ParamLength::from_mm(stepover_mm);
                    if p.roughing_passes <= 1 || op.feed_overrides.stepover_mm.is_some() {
                        op.feed_overrides.stepover_mm = Some(stepover_mm);
                    }
                }
                StrategyParams::Pocket2D(p) => match &mut p.strategy {
                    PocketStrategyKind::Adaptive(a) => {
                        a.optimal_load = ParamLength::from_mm(stepover_mm);
                    }
                    PocketStrategyKind::ContourParallel | PocketStrategyKind::ZigZag { .. } => {
                        p.stepover = ParamLength::from_mm(stepover_mm);
                    }
                },
                StrategyParams::Adaptive3D(p) => {
                    p.adaptive.optimal_load = ParamLength::from_mm(stepover_mm);
                }
                StrategyParams::Parallel3D(p) => p.stepover = ParamLength::from_mm(stepover_mm),
                StrategyParams::Scallop3D(p) => {
                    p.max_stepover = ParamLength::from_mm(stepover_mm);
                }
                StrategyParams::Pencil3D(p) => {
                    p.offset_stepover = ParamLength::from_mm(stepover_mm);
                }
                StrategyParams::SlotUndercut(p) => {
                    p.radial_stepover = ParamLength::from_mm(stepover_mm);
                }
                StrategyParams::Dovetail(p) => {
                    p.radial_stepover = ParamLength::from_mm(stepover_mm);
                }
                StrategyParams::Rotary2Linear(p) => p.stepover = ParamLength::from_mm(stepover_mm),
                StrategyParams::Rotary4AxisSim(p) => {
                    p.stepover = ParamLength::from_mm(stepover_mm);
                }
                _ => {
                    op.feed_overrides.stepover_mm = Some(stepover_mm);
                }
            }
            op.mark_dirty();
            true
        }
        MachinabilityFix::SetCuttingFeed { feed_mm_min } => {
            if !feed_mm_min.is_finite() || feed_mm_min <= 0.0 {
                return false;
            }
            op.feed_overrides.cutting_feed_mm_min = Some(feed_mm_min);
            op.mark_dirty();
            true
        }
        MachinabilityFix::SetRampFeed { feed_mm_min } => {
            if !feed_mm_min.is_finite() || feed_mm_min <= 0.0 {
                return false;
            }
            op.feed_overrides.ramp_feed_mm_min = Some(feed_mm_min);
            op.mark_dirty();
            true
        }
        MachinabilityFix::SetPlungeFeed { feed_mm_min } => {
            if !feed_mm_min.is_finite() || feed_mm_min <= 0.0 {
                return false;
            }
            op.feed_overrides.plunge_feed_mm_min = Some(feed_mm_min);
            op.mark_dirty();
            true
        }
        MachinabilityFix::EnableHelicalInnerArcComp => {
            if op.kind != OperationKind::DrillBore {
                return false;
            }
            if let StrategyParams::DrillBore(p) = op.strategy_mut() {
                p.bore_inner_arc_feed_comp = true;
                op.mark_dirty();
                true
            } else {
                false
            }
        }
        MachinabilityFix::DisableDeepBoreFinishPass => {
            if op.kind != OperationKind::DrillBore {
                return false;
            }
            if let StrategyParams::DrillBore(p) = op.strategy_mut() {
                p.bore_finish_pass = false;
                op.mark_dirty();
                true
            } else {
                false
            }
        }
        MachinabilityFix::EnableBorePeckRetract { clear_depth_mm } => {
            if op.kind != OperationKind::DrillBore
                || !clear_depth_mm.is_finite()
                || clear_depth_mm <= 0.5
            {
                return false;
            }
            if let StrategyParams::DrillBore(p) = op.strategy_mut() {
                p.bore_chip_clear_depth = Some(ParamLength::from_mm(clear_depth_mm));
                op.mark_dirty();
                true
            } else {
                false
            }
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_machines::{
        builtin_carvera_air_machine, builtin_carvera_z1_machine, builtin_milo_1_5_machine,
    };
    use chip_math::{DisplayUnit, ParamLength};
    use chip_sketch::{SketchEntity, SketchEntityEntry, SketchLayer};
    use chip_tools::{
        ALUMINUM_6061_T6_ID, ALUMINUM_6063_T5_ID, CutterGeometry, ProjectMaterialSelection,
        SpindleDirection, ToolId, resolve_workpiece_material,
    };

    use crate::{
        GeometrySelectionItem, HeightReference, HeightSpec, Setup, StockConfig, WorkOffsetSlot,
    };

    use super::*;

    fn test_setup_with_box_stock(w: f64, d: f64, h: f64) -> Setup {
        let mut setup = Setup::new("setup_1", "Op 1", WorkOffsetSlot::G54);
        setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(w),
            depth_y: ParamLength::from_mm(d),
            height_z: ParamLength::from_mm(h),
            xy_align: crate::FixedBoxXyAlign::CenterXy,
            z_align: crate::FixedBoxZAlign::OffsetFromTop {
                top_allowance: ParamLength::from_mm(0.0),
            },
        };
        setup
    }

    #[test]
    fn bartrim_facing_failure_is_critical_and_tuned_parameters_drop_to_ok() {
        let machine = builtin_carvera_air_machine();
        let al6063 = resolve_workpiece_material(Some(&ProjectMaterialSelection::builtin(
            ALUMINUM_6063_T5_ID,
        )));
        let t2 = machine.tools.iter().find(|t| t.tool_number == 2).unwrap();

        let mut setup = test_setup_with_box_stock(80.0, 25.0, 20.2);

        // Failure case: ap = 4.04 mm x ae = 0.50 mm on Ø3.175 x 25 mm LOC 1FL (28 mm stickout)
        let mut op_bad = Operation::new(
            "op_face_bad",
            "BarTrim Face (Aggressive)",
            OperationKind::Facing,
        );
        op_bad.tool_id = Some(t2.id.clone());
        op_bad.preset_id = Some("carvera_al6061_deep".to_string());
        op_bad.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        op_bad.heights.bottom = HeightSpec::new(HeightReference::StockTop, -8.08);
        op_bad.passes.multiple_depths = true;
        op_bad.passes.max_stepdown = ParamLength::from_mm(4.04);
        if let StrategyParams::Facing(p) = op_bad.strategy_mut() {
            p.stepover = ParamLength::from_mm(0.50);
        }

        // Tuned safe case: ap = 2.02 mm x ae = 0.30 mm
        let mut op_safe = op_bad.clone();
        op_safe.id = "op_face_safe".to_string();
        op_safe.name = "BarTrim Face (Tuned)".to_string();
        op_safe.passes.max_stepdown = ParamLength::from_mm(2.02);
        if let StrategyParams::Facing(p) = op_safe.strategy_mut() {
            p.stepover = ParamLength::from_mm(0.30);
        }

        setup.operations = vec![op_bad, op_safe];
        let report = analyze_setup_machinability(
            &setup,
            &machine.tools,
            &al6063,
            &machine,
            None,
            &[],
            DisplayUnit::Millimeter,
        );

        let r_bad = &report.operations[0];
        let r_safe = &report.operations[1];
        assert_eq!(
            r_bad.severity,
            MachinabilitySeverity::Critical,
            "4.04 x 0.50 BarTrim facing must be Critical, got {:?}",
            r_bad.findings
        );
        assert!(
            r_bad
                .findings
                .iter()
                .any(|f| f.kind == MachinabilityCheckKind::ToolDeflectionSnap
                    && f.severity == MachinabilitySeverity::Critical)
        );

        assert_eq!(
            r_safe.severity,
            MachinabilitySeverity::Ok,
            "2.02 x 0.30 tuned BarTrim facing must drop cleanly to Ok, got {:?}",
            r_safe.findings
        );

        // Applying the 1-click suggested fix to `op_bad` must reduce severity cleanly!
        let mut op_fixed = setup.operations[0].clone();
        let fix = r_bad
            .findings
            .iter()
            .find_map(|f| f.suggested_fix.clone())
            .expect("critical deflection finding must provide a 1-click fix");
        assert!(apply_machinability_fix(&mut op_fixed, &fix));
        let mut setup_fixed = test_setup_with_box_stock(80.0, 25.0, 20.2);
        setup_fixed.operations.push(op_fixed);
        let rep_fixed = analyze_setup_machinability(
            &setup_fixed,
            &machine.tools,
            &al6063,
            &machine,
            None,
            &[],
            DisplayUnit::Millimeter,
        );
        assert_eq!(
            rep_fixed.operations[0].severity,
            MachinabilitySeverity::Ok,
            "1-click fix must resolve BarTrim facing snap hazard, got {:?}",
            rep_fixed.operations[0].findings
        );
    }

    #[test]
    fn bartrim_helical_bore_failure_and_tuned_safety_checks_fire_accurately() {
        let machine = builtin_carvera_air_machine();
        let al6063 = resolve_workpiece_material(Some(&ProjectMaterialSelection::builtin(
            ALUMINUM_6063_T5_ID,
        )));
        let t2 = machine.tools.iter().find(|t| t.tool_number == 2).unwrap();

        let mut sk = SketchLayer::new("sk_holes", "Holes", "setup_1");
        sk.entities.push(SketchEntityEntry {
            id: "c1".to_string(),
            name: "Ø5.0 mm Hole".to_string(),
            sub_layer: None,
            is_construction: false,
            entity: SketchEntity::new_circle(
                ParamLength::from_mm(0.0),
                ParamLength::from_mm(0.0),
                ParamLength::from_mm(5.0),
            ),
        });

        let mut setup = test_setup_with_box_stock(40.0, 40.0, 20.2);

        // Failure case: Ø5.0 x 20.2 mm uncompensated 309 mm/min + bore_finish_pass = true
        let mut op_bad = Operation::new(
            "op_bore_bad",
            "Ø5 Through-Bore (Bad)",
            OperationKind::DrillBore,
        );
        op_bad.tool_id = Some(t2.id.clone());
        op_bad.preset_id = Some("carvera_al6061_deep".to_string());
        op_bad.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        op_bad.heights.bottom = HeightSpec::new(HeightReference::StockTop, -20.2);
        op_bad.selections.push(GeometrySelectionItem::SketchEntity {
            sketch_id: "sk_holes".to_string(),
            entity_id: "c1".to_string(),
            reverse: false,
            side_flip: false,
        });
        if let StrategyParams::DrillBore(p) = op_bad.strategy_mut() {
            p.mode = HoleMachiningMode::HelicalBore;
            p.bore_finish_pass = true;
            p.bore_inner_arc_feed_comp = false;
            p.bore_chip_clear_depth = None;
        }

        setup.operations = vec![op_bad.clone()];
        let rep_bad = analyze_setup_machinability(
            &setup,
            &machine.tools,
            &al6063,
            &machine,
            None,
            &[sk.clone()],
            DisplayUnit::Millimeter,
        );
        let findings = &rep_bad.operations[0].findings;
        assert_eq!(
            rep_bad.operations[0].severity,
            MachinabilitySeverity::Critical
        );
        assert!(
            findings.iter().any(
                |f| f.kind == MachinabilityCheckKind::HelicalInnerArcOverload
                    && f.severity == MachinabilitySeverity::Critical
            ),
            "must flag Critical inner-arc wall overload: {findings:?}"
        );
        assert!(
            findings.iter().any(
                |f| f.kind == MachinabilityCheckKind::DeepBoreFullDepthFinish
                    && f.severity == MachinabilitySeverity::Critical
            ),
            "must flag Critical full-depth bore finish pass: {findings:?}"
        );
        assert!(
            findings
                .iter()
                .any(|f| f.kind == MachinabilityCheckKind::DeepHoleChipEvacuation
                    && f.severity == MachinabilitySeverity::Warning),
            "must flag Warning deep-hole chip packing: {findings:?}"
        );

        // Apply all 3 suggested fixes atomically and verify 0 Critical and 0 Warning!
        let mut op_tuned = op_bad;
        for f in findings {
            if let Some(fix) = &f.suggested_fix {
                apply_machinability_fix(&mut op_tuned, fix);
            }
        }
        setup.operations = vec![op_tuned];
        let rep_tuned = analyze_setup_machinability(
            &setup,
            &machine.tools,
            &al6063,
            &machine,
            None,
            &[sk],
            DisplayUnit::Millimeter,
        );
        assert_eq!(
            rep_tuned.critical_count, 0,
            "tuned helical bore must have 0 critical findings: {:?}",
            rep_tuned.operations[0].findings
        );
        assert_eq!(
            rep_tuned.warning_count, 0,
            "tuned helical bore must have 0 warnings: {:?}",
            rep_tuned.operations[0].findings
        );
        assert_eq!(rep_tuned.operations[0].severity, MachinabilitySeverity::Ok);
    }

    #[test]
    fn spindle_stall_open_air_plunge_and_thin_chip_rubbing_checks() {
        let z1 = builtin_carvera_z1_machine();
        let milo = builtin_milo_1_5_machine();
        let al6061 = resolve_workpiece_material(Some(&ProjectMaterialSelection::builtin(
            ALUMINUM_6061_T6_ID,
        )));
        let al6063 = resolve_workpiece_material(Some(&ProjectMaterialSelection::builtin(
            ALUMINUM_6063_T5_ID,
        )));

        let em6 = Tool {
            id: ToolId("em6_3fl".to_string()),
            tool_number: 1,
            length_offset_number: 1,
            diameter_offset_number: 1,
            name: "Ø6mm 3FL Carbide Flat".to_string(),
            vendor: None,
            product_id: None,
            native_unit: DisplayUnit::Millimeter,
            spindle_direction: SpindleDirection::Cw,
            default_coolant: CoolantMode::AirBlast,
            stickout_length_mm: 20.0,
            geometry: CutterGeometry::FlatEndmill {
                diameter_mm: 6.0,
                flute_length_mm: 16.0,
                overall_length_mm: 50.0,
                shank_diameter_mm: 6.0,
                flutes: 3,
            },
            presets: Vec::new(),
        };

        // Heavy Ø6 mm full-width slotting: stalls Carvera Z1 (150 W), passes Milo (1.5 kW)
        let mut op_slot = Operation::new("op_slot", "Heavy Slot", OperationKind::Contour2D);
        op_slot.tool_id = Some(em6.id.clone());
        op_slot.feed_overrides.spindle_rpm = Some(12_000.0);
        op_slot.feed_overrides.cutting_feed_mm_min = Some(800.0);
        op_slot.feed_overrides.plunge_feed_mm_min = Some(350.0);
        op_slot.passes.max_stepdown = ParamLength::from_mm(3.0);

        let mut setup = test_setup_with_box_stock(100.0, 100.0, 20.0);
        setup.operations = vec![op_slot.clone()];

        let rep_z1 = analyze_setup_machinability(
            &setup,
            std::slice::from_ref(&em6),
            &al6061,
            &z1,
            None,
            &[],
            DisplayUnit::Millimeter,
        );
        let rep_milo = analyze_setup_machinability(
            &setup,
            std::slice::from_ref(&em6),
            &al6061,
            &milo,
            None,
            &[],
            DisplayUnit::Millimeter,
        );
        assert!(
            rep_z1.operations[0]
                .findings
                .iter()
                .any(|f| f.kind == MachinabilityCheckKind::SpindlePowerStall
                    && f.severity == MachinabilitySeverity::Critical),
            "150W Carvera Z1 must trigger Critical spindle stall: {:?}",
            rep_z1.operations[0].findings
        );
        assert!(
            !rep_milo.operations[0]
                .findings
                .iter()
                .any(|f| f.kind == MachinabilityCheckKind::SpindlePowerStall),
            "1.5kW Milo must not warn on spindle power: {:?}",
            rep_milo.operations[0].findings
        );

        // Open-air plunge Info advisory fires ONLY when open_chain_extension > 0 at slow plunge feed
        let mut op_closed = Operation::new("op_closed", "Closed Profile", OperationKind::Contour2D);
        op_closed.tool_id = Some(em6.id.clone());
        op_closed.feed_overrides.spindle_rpm = Some(12_000.0);
        op_closed.feed_overrides.cutting_feed_mm_min = Some(800.0);
        op_closed.feed_overrides.plunge_feed_mm_min = Some(100.0);
        op_closed.passes.max_stepdown = ParamLength::from_mm(1.0);

        let mut op_open = op_closed.clone();
        op_open.id = "op_open".to_string();
        op_open.name = "Open Chain Edge".to_string();
        if let StrategyParams::Contour2D(p) = op_open.strategy_mut() {
            p.open_chain_extension = ParamLength::from_mm(3.0);
        }

        setup.operations = vec![op_closed, op_open];
        let rep_plunge = analyze_setup_machinability(
            &setup,
            std::slice::from_ref(&em6),
            &al6061,
            &milo,
            None,
            &[],
            DisplayUnit::Millimeter,
        );
        assert!(
            !rep_plunge.operations[0]
                .findings
                .iter()
                .any(|f| f.kind == MachinabilityCheckKind::SlowOpenAirPlunge)
        );
        assert!(
            rep_plunge.operations[1]
                .findings
                .iter()
                .any(|f| f.kind == MachinabilityCheckKind::SlowOpenAirPlunge
                    && f.severity == MachinabilitySeverity::Info)
        );

        // Thin-chip rubbing & dry 6063 chip-welding warning
        let mut op_rub = Operation::new("op_rub", "Too-Light Rubbing Cut", OperationKind::Facing);
        op_rub.tool_id = Some(em6.id.clone());
        op_rub.feed_overrides.spindle_rpm = Some(15_000.0);
        op_rub.feed_overrides.cutting_feed_mm_min = Some(400.0); // fz = 0.0089 mm/tooth
        op_rub.passes.max_stepdown = ParamLength::from_mm(0.5);
        if let StrategyParams::Facing(p) = op_rub.strategy_mut() {
            p.stepover = ParamLength::from_mm(0.25); // severe thinning -> h_max ~ 0.0035 < h_min 0.010!
        }
        setup.operations = vec![op_rub];
        let rep_rub = analyze_setup_machinability(
            &setup,
            &[em6],
            &al6063,
            &milo,
            None,
            &[],
            DisplayUnit::Millimeter,
        );
        assert!(
            rep_rub.operations[0]
                .findings
                .iter()
                .any(|f| f.kind == MachinabilityCheckKind::ThinChipRubbingWeld
                    && f.severity == MachinabilitySeverity::Warning)
        );
    }
}
