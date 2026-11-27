//! CAM-domain operation strategy, undercut, deburr, and rotary/wrap validation (`chip-cam`).
//!
//! Returns structured [`CamValidationIssue`] diagnostics consumed by `chip-project::validation`
//! without coupling `chip-cam` to project persistence or reporting types.

use chip_math::{Aabb2, ArcPolyline2D, DVec2, EvalContext};
use chip_tools::{CutterGeometry, Tool};

use crate::{
    heights::ResolvedHeights,
    operation::{Operation, OperationKind},
    params::{
        DovetailMode, MachiningBoundaryMode, RotaryAxialBoundaryMode, SlotUndercutMode,
        StrategyParams,
    },
    rotary::{OperationRotaryMode, RotaryAxisKind, RotaryDriveMode, rotary_chuck_guard_u},
    selection::{ExtractedFeature25D, ExtractedSelectionSummary},
    setup::ResolvedSetup,
};

/// Severity of a CAM strategy/operation validation diagnostic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CamValidationSeverity {
    Error,
    Warning,
}

/// Structured CAM validation diagnostic emitted for a single [`Operation`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CamValidationIssue {
    pub severity: CamValidationSeverity,
    pub message: String,
}

impl CamValidationIssue {
    pub fn error(message: impl Into<String>) -> Self {
        Self {
            severity: CamValidationSeverity::Error,
            message: message.into(),
        }
    }

    pub fn warning(message: impl Into<String>) -> Self {
        Self {
            severity: CamValidationSeverity::Warning,
            message: message.into(),
        }
    }
}

/// Validates strategy-specific parameters (`Deburr3D`, `SlotUndercut`, `Dovetail`, `Undercut3D`),
/// rotary drive/setup requirements, and native 4th-axis rotary axial/angular boundary settings.
///
/// Returns `(issues, abort_operation)` where `abort_operation == true` signals that downstream
/// selection extraction and Z-height resolution should be skipped for this operation.
pub fn validate_operation_strategy_and_rotary(
    op: &Operation,
    tool: Option<&Tool>,
    resolved_setup: &ResolvedSetup,
    eval_ctx: &EvalContext,
) -> (Vec<CamValidationIssue>, bool) {
    let mut issues = Vec::new();

    // Strategy-specific validation for `OperationKind::Deburr3D`
    if op.kind == OperationKind::Deburr3D
        && let StrategyParams::Deburr3D(mut p) = op.resolved_strategy()
    {
        p.reevaluate(eval_ctx);
        if !p.chamfer_width.value_mm.is_finite() || p.chamfer_width.value_mm <= 1e-4 {
            issues.push(CamValidationIssue::error(format!(
                "3D Deburr chamfer width ({:.3} mm) must be finite and > 0.",
                p.chamfer_width.value_mm
            )));
        }
        if !p.tip_offset.value_mm.is_finite() || p.tip_offset.value_mm < 0.0 {
            issues.push(CamValidationIssue::error(format!(
                "3D Deburr tip offset ({:.3} mm) must be finite and >= 0.",
                p.tip_offset.value_mm
            )));
        }
        if !p.wall_clearance.value_mm.is_finite() || p.wall_clearance.value_mm < 0.0 {
            issues.push(CamValidationIssue::error(format!(
                "3D Deburr wall clearance ({:.3} mm) must be finite and >= 0.",
                p.wall_clearance.value_mm
            )));
        }
        if !p.edge_extension.value_mm.is_finite() || p.edge_extension.value_mm < 0.0 {
            issues.push(CamValidationIssue::error(format!(
                "3D Deburr edge extension ({:.3} mm) must be finite and >= 0.",
                p.edge_extension.value_mm
            )));
        }
        if !p.chord_tolerance_mm.is_finite() || p.chord_tolerance_mm <= 1e-5 {
            issues.push(CamValidationIssue::error(format!(
                "3D Deburr chord tolerance ({:.4} mm) must be finite and > 0.",
                p.chord_tolerance_mm
            )));
        }
        if !p.min_dihedral_angle_deg.is_finite()
            || p.min_dihedral_angle_deg <= 0.0
            || p.min_dihedral_angle_deg >= 179.0
        {
            issues.push(CamValidationIssue::error(format!(
                "3D Deburr min edge angle ({:.1}°) must be within (0°, 179°).",
                p.min_dihedral_angle_deg
            )));
        }
        if p.multiple_passes == 0 {
            issues.push(CamValidationIssue::error(
                "3D Deburr passes count must be >= 1.",
            ));
        }
        if let Some(tool) = tool {
            if p.allow_underside_back_chamfer
                && !matches!(tool.geometry, CutterGeometry::LollipopEndmill { .. })
            {
                issues.push(CamValidationIssue::error(format!(
                    "Underside back-chamfering requires a Lollipop Endmill cutter (assigned: {}).",
                    tool.geometry.kind_label()
                )));
            }
            if let CutterGeometry::ChamferMill {
                diameter_mm,
                tip_diameter_mm,
                included_angle_deg,
                ..
            } = tool.geometry
            {
                let half_a = (0.5 * included_angle_deg).clamp(5.0, 85.0).to_radians();
                let max_r = 0.5 * (diameter_mm - tip_diameter_mm).max(0.0);
                let req_r = p.tip_offset.value_mm.max(0.0) * half_a.tan()
                    + p.chamfer_width.value_mm.max(0.0);
                if req_r > max_r + 1e-3 {
                    issues.push(CamValidationIssue::error(format!(
                        "Chamfer width + tip offset radial reach ({req_r:.2} mm) exceeds chamfer mill flute span ({max_r:.2} mm)."
                    )));
                }
            }
        }
    }

    // Strategy-specific validation for Undercut operations
    if op.kind == OperationKind::SlotUndercut
        && let StrategyParams::SlotUndercut(mut p) = op.resolved_strategy()
    {
        p.reevaluate(eval_ctx);
        if !p.slot_width.value_mm.is_finite() || p.slot_width.value_mm <= 1e-4 {
            issues.push(CamValidationIssue::error(format!(
                "Slot/T-Slot width ({:.3} mm) must be finite and > 0.",
                p.slot_width.value_mm
            )));
        }
        if !p.throat_width.value_mm.is_finite() || p.throat_width.value_mm <= 1e-4 {
            issues.push(CamValidationIssue::error(format!(
                "Slot/T-Slot throat width ({:.3} mm) must be finite and > 0.",
                p.throat_width.value_mm
            )));
        }
        if !p.slot_height.value_mm.is_finite() || p.slot_height.value_mm <= 1e-4 {
            issues.push(CamValidationIssue::error(format!(
                "Slot/T-Slot height ({:.3} mm) must be finite and > 0.",
                p.slot_height.value_mm
            )));
        }
        if !p.radial_stepover.value_mm.is_finite() || p.radial_stepover.value_mm <= 1e-4 {
            issues.push(CamValidationIssue::error(format!(
                "Slot/T-Slot radial stepover ({:.3} mm) must be finite and > 0.",
                p.radial_stepover.value_mm
            )));
        }
        if !p.axial_stepdown.value_mm.is_finite() || p.axial_stepdown.value_mm <= 1e-4 {
            issues.push(CamValidationIssue::error(format!(
                "Slot/T-Slot axial stepdown ({:.3} mm) must be finite and > 0.",
                p.axial_stepdown.value_mm
            )));
        }
        if let Some(tool) = tool {
            let needs_undercut = p.mode == SlotUndercutMode::TSlot
                || p.throat_width.value_mm < p.slot_width.value_mm - 1e-4;
            match tool.geometry {
                CutterGeometry::SlotMill {
                    diameter_mm,
                    cutting_thickness_mm,
                    neck_diameter_mm,
                    ..
                } => {
                    if diameter_mm > p.slot_width.value_mm + 1e-4 {
                        issues.push(CamValidationIssue::error(format!(
                            "Slot mill diameter (Ø{diameter_mm:.3} mm) exceeds slot width ({:.3} mm).",
                            p.slot_width.value_mm
                        )));
                    }
                    if cutting_thickness_mm > p.slot_height.value_mm + 1e-4 {
                        issues.push(CamValidationIssue::error(format!(
                            "Slot mill cutting thickness ({cutting_thickness_mm:.3} mm) exceeds slot height ({:.3} mm).",
                            p.slot_height.value_mm
                        )));
                    }
                    if p.mode != SlotUndercutMode::SideGroove && needs_undercut {
                        if neck_diameter_mm >= p.throat_width.value_mm - 1e-4 {
                            issues.push(CamValidationIssue::error(format!(
                                "Slot mill neck diameter (Ø{neck_diameter_mm:.3} mm) must be strictly smaller than throat width ({:.3} mm).",
                                p.throat_width.value_mm
                            )));
                        } else {
                            let max_lat =
                                (0.5 * p.slot_width.value_mm - 0.5 * diameter_mm).max(0.0);
                            if max_lat + 0.5 * neck_diameter_mm
                                > 0.5 * p.throat_width.value_mm + 1e-4
                            {
                                issues.push(CamValidationIssue::error(format!(
                                    "Slot mill relieved neck collides with throat wall at full lateral offset ({max_lat:.3} mm + neck radius {:.3} mm > half-throat {:.3} mm).",
                                    0.5 * neck_diameter_mm,
                                    0.5 * p.throat_width.value_mm
                                )));
                            }
                        }
                    }
                }
                CutterGeometry::FlatEndmill { diameter_mm, .. } => {
                    if needs_undercut {
                        issues.push(CamValidationIssue::error(
                            "T-Slot undercut requires a relieved-neck Slot / T-Slot Mill cutter (assigned: Flat Endmill).",
                        ));
                    } else if diameter_mm > p.slot_width.value_mm + 1e-4 {
                        issues.push(CamValidationIssue::error(format!(
                            "Flat Endmill diameter (Ø{diameter_mm:.3} mm) exceeds keyway width ({:.3} mm).",
                            p.slot_width.value_mm
                        )));
                    }
                }
                _ => {}
            }
        }
    }

    if op.kind == OperationKind::Dovetail
        && let StrategyParams::Dovetail(mut p) = op.resolved_strategy()
    {
        p.reevaluate(eval_ctx);
        if !p.side_angle_deg.is_finite() || p.side_angle_deg <= 15.0 || p.side_angle_deg >= 85.0 {
            issues.push(CamValidationIssue::error(format!(
                "Dovetail side angle ({:.1}°) must be within (15°, 85°).",
                p.side_angle_deg
            )));
        }
        if !p.dovetail_depth.value_mm.is_finite() || p.dovetail_depth.value_mm <= 1e-4 {
            issues.push(CamValidationIssue::error(format!(
                "Dovetail depth ({:.3} mm) must be finite and > 0.",
                p.dovetail_depth.value_mm
            )));
        }
        if !p.radial_stepover.value_mm.is_finite() || p.radial_stepover.value_mm <= 1e-4 {
            issues.push(CamValidationIssue::error(format!(
                "Dovetail radial stepover ({:.3} mm) must be finite and > 0.",
                p.radial_stepover.value_mm
            )));
        }
        if let Some(tool) = tool
            && let CutterGeometry::DovetailCutter {
                diameter_mm,
                side_angle_deg,
                cutting_height_mm,
                corner_radius_mm,
                neck_diameter_mm,
                ..
            } = tool.geometry
        {
            if (side_angle_deg - p.side_angle_deg).abs() > 0.5 {
                issues.push(CamValidationIssue::error(format!(
                    "Dovetail cutter angle ({side_angle_deg:.2}°) mismatches target dovetail angle ({:.2}°, max tolerance ±0.5°).",
                    p.side_angle_deg
                )));
            }
            if cutting_height_mm + 1e-4 < p.dovetail_depth.value_mm {
                issues.push(CamValidationIssue::error(format!(
                    "Dovetail cutter cutting height ({cutting_height_mm:.3} mm) is shallower than dovetail depth ({:.3} mm).",
                    p.dovetail_depth.value_mm
                )));
            }
            if p.mode == DovetailMode::InternalSlot {
                let throat = p.top_throat_width.value_mm;
                if neck_diameter_mm >= throat - 1e-4 {
                    issues.push(CamValidationIssue::error(format!(
                        "Dovetail cutter neck diameter (Ø{neck_diameter_mm:.3} mm) must be strictly smaller than top throat width ({throat:.3} mm)."
                    )));
                }
                let top_d = CutterGeometry::dovetail_top_diameter_mm(
                    diameter_mm,
                    side_angle_deg,
                    p.dovetail_depth.value_mm.max(0.0),
                    corner_radius_mm,
                );
                if top_d > throat + 1e-4 {
                    issues.push(CamValidationIssue::error(format!(
                        "Dovetail cutter upper flank diameter (Ø{top_d:.3} mm) exceeds top throat width ({throat:.3} mm)."
                    )));
                }
            }
        }
    }

    if op.kind == OperationKind::Undercut3D
        && let StrategyParams::Undercut3D(mut p) = op.resolved_strategy()
    {
        p.reevaluate(eval_ctx);
        if !p.use_cusp_height
            && (!p.z_stepdown.value_mm.is_finite() || p.z_stepdown.value_mm <= 1e-4)
        {
            issues.push(CamValidationIssue::error(format!(
                "3D Undercut Z stepdown ({:.3} mm) must be finite and > 0.",
                p.z_stepdown.value_mm
            )));
        }
        if p.use_cusp_height
            && (!p.target_cusp.value_mm.is_finite() || p.target_cusp.value_mm <= 1e-5)
        {
            issues.push(CamValidationIssue::error(format!(
                "3D Undercut target cusp ({:.4} mm) must be finite and > 0.",
                p.target_cusp.value_mm
            )));
        }
        if !p.min_overhang_angle_deg.is_finite()
            || p.min_overhang_angle_deg <= 0.0
            || p.min_overhang_angle_deg >= 89.0
        {
            issues.push(CamValidationIssue::error(format!(
                "3D Undercut min overhang angle ({:.1}°) must be within (0°, 89°).",
                p.min_overhang_angle_deg
            )));
        }
        if !p.neck_clearance.value_mm.is_finite() || p.neck_clearance.value_mm < 0.0 {
            issues.push(CamValidationIssue::error(format!(
                "3D Undercut neck clearance ({:.3} mm) must be finite and >= 0.",
                p.neck_clearance.value_mm
            )));
        }
        if !p.lead_out_distance.value_mm.is_finite() || p.lead_out_distance.value_mm < 0.0 {
            issues.push(CamValidationIssue::error(format!(
                "3D Undercut lead-out distance ({:.3} mm) must be finite and >= 0.",
                p.lead_out_distance.value_mm
            )));
        }
    }

    if op.kind.is_rotary_native() && resolved_setup.rotary.is_none() {
        issues.push(CamValidationIssue::error(
            "4th-Axis Rotary operations require 4th-Axis Rotary to be enabled on this Setup.",
        ));
        return (issues, true);
    }
    if op.kind == OperationKind::Rotary4AxisSim
        && let Some(rot) = resolved_setup.rotary
        && rot.mode != RotaryDriveMode::Simultaneous4Axis
    {
        issues.push(CamValidationIssue::error(
            "Simultaneous 4-Axis Milling requires Setup Rotary Mode 'Simultaneous 4-Axis (X+Y+Z+Rotary)'.",
        ));
        return (issues, true);
    }
    if !op.rotary.is_none() && resolved_setup.rotary.is_none() {
        issues.push(CamValidationIssue::error(
            "Operation 4th-axis indexing/wrapping requires 4th-Axis Rotary to be enabled on this Setup.",
        ));
        return (issues, true);
    }

    // Strategy-specific axial/angular boundary validation for native 4th-axis rotary ops
    if op.kind.is_rotary_native()
        && let Some(mut ab) = op.rotary_axial_boundary().cloned()
    {
        ab.reevaluate(eval_ctx);
        if ab.axial_boundary_mode == RotaryAxialBoundaryMode::SelectedGeometry
            && op.selections.is_empty()
        {
            issues.push(CamValidationIssue::warning(
                "Rotary axial boundary mode 'Selected Geometry' has no selections; generation will fall back to Model Bounds.",
            ));
        }
        if ab.axial_boundary_mode == RotaryAxialBoundaryMode::ExplicitRange {
            let u_start = ab.explicit_u_start.value_mm;
            let u_end = ab.explicit_u_end.value_mm;
            if !u_start.is_finite() || !u_end.is_finite() {
                issues.push(CamValidationIssue::error(
                    "Rotary explicit axial range bounds must be finite.",
                ));
            } else if u_start >= u_end - 1e-6 {
                issues.push(CamValidationIssue::warning(format!(
                    "Rotary explicit axial range start ({u_start:.2} mm) is >= end ({u_end:.2} mm)."
                )));
            }
        }
        if !ab.start_angle_deg.is_finite()
            || !ab.end_angle_deg.is_finite()
            || ab.end_angle_deg <= ab.start_angle_deg + 1e-3
        {
            issues.push(CamValidationIssue::error(format!(
                "Rotary angular span [{:.1}°, {:.1}°] must be finite with end_angle_deg > start_angle_deg.",
                ab.start_angle_deg, ab.end_angle_deg
            )));
        }
    }

    (issues, false)
}

/// Validates extracted 2D boundary selection presence and `WrapCylinder` chuck keep-out / 360° span.
pub fn validate_operation_extracted_geometry(
    op: &Operation,
    tool: Option<&Tool>,
    resolved_setup: &ResolvedSetup,
    summary: &ExtractedSelectionSummary,
    eval_ctx: &EvalContext,
) -> Vec<CamValidationIssue> {
    let mut issues = Vec::new();

    if let Some(boundary) = op.resolved_strategy().machining_boundary()
        && boundary.boundary_mode == MachiningBoundaryMode::SelectedRegions
    {
        let has_2d_region = summary.features.iter().any(|feat| {
            matches!(
                feat,
                ExtractedFeature25D::PlanarFace { .. }
                    | ExtractedFeature25D::Contour {
                        polyline_wcs: ArcPolyline2D { closed: true, .. },
                        ..
                    }
            )
        });
        if !has_2d_region {
            issues.push(CamValidationIssue::warning(
                "3D Machining Boundary is set to 'Selected 2D Contours' but no closed 2D contours or planar faces are selected; falling back to Model Silhouette.",
            ));
        }
    }

    // Warn early if a `WrapCylinder` operation intrudes into the rotary chuck keep-out
    // zone or wraps beyond 360° around the cylinder circumference.
    if let (
        OperationRotaryMode::WrapCylinder {
            wrap_radius,
            axial_offset,
            ..
        },
        Some(rot),
    ) = (&op.rotary, resolved_setup.rotary)
        && rot.axis != RotaryAxisKind::ZAxis
    {
        let mut wr = wrap_radius.clone();
        let _ = wr.reevaluate(eval_ctx);
        let mut ao = axial_offset.clone();
        let _ = ao.reevaluate(eval_ctx);
        let r_wrap = wr.value_mm.max(1.0);
        let ax_off = ao.value_mm;

        let mut u_min = f64::INFINITY;
        let mut u_max = f64::NEG_INFINITY;
        let mut s_min = f64::INFINITY;
        let mut s_max = f64::NEG_INFINITY;
        let mut extend_aabb = |b: Aabb2| {
            if b.is_valid() {
                match rot.axis {
                    RotaryAxisKind::XAxis | RotaryAxisKind::ZAxis => {
                        u_min = u_min.min(b.min.x + ax_off);
                        u_max = u_max.max(b.max.x + ax_off);
                        s_min = s_min.min(b.min.y);
                        s_max = s_max.max(b.max.y);
                    }
                    RotaryAxisKind::YAxis => {
                        u_min = u_min.min(b.min.y + ax_off);
                        u_max = u_max.max(b.max.y + ax_off);
                        s_min = s_min.min(-b.max.x);
                        s_max = s_max.max(-b.min.x);
                    }
                }
            }
        };
        for feat in &summary.features {
            match feat {
                ExtractedFeature25D::PlanarFace {
                    outer_boundary_wcs, ..
                } => extend_aabb(outer_boundary_wcs.exact_aabb()),
                ExtractedFeature25D::Contour { polyline_wcs, .. } => {
                    extend_aabb(polyline_wcs.exact_aabb());
                }
                ExtractedFeature25D::ModeledChamfer {
                    upper_polyline_wcs,
                    lower_polyline_wcs,
                    ..
                } => {
                    extend_aabb(upper_polyline_wcs.exact_aabb());
                    extend_aabb(lower_polyline_wcs.exact_aabb());
                }
                ExtractedFeature25D::CylindricalHole {
                    center_wcs,
                    diameter_mm,
                    ..
                } => {
                    let r = 0.5 * diameter_mm.abs();
                    extend_aabb(Aabb2::from_points([
                        DVec2::new(center_wcs.x - r, center_wcs.y - r),
                        DVec2::new(center_wcs.x + r, center_wcs.y + r),
                    ]));
                }
            }
        }

        if u_min.is_finite()
            && u_max.is_finite()
            && (rot.chuck_u_max - rot.chuck_u_min).abs() > 1e-6
        {
            let tool_d = tool.map_or(0.1, |t| t.geometry.cutting_diameter_mm());
            let guard_u = rotary_chuck_guard_u(&rot, tool_d);
            let intrudes = if rot.chuck_end_positive {
                u_max > guard_u + 1e-4
            } else {
                u_min < guard_u - 1e-4
            };
            if intrudes {
                issues.push(CamValidationIssue::warning(format!(
                    "Wrapped 2D operation axial extent [{u_min:.2}, {u_max:.2}] mm intrudes into the rotary chuck keep-out zone (limit {guard_u:.2} mm)."
                )));
            }
        }
        if s_min.is_finite() && s_max.is_finite() {
            let span_deg = ((s_max - s_min) / r_wrap).to_degrees();
            if span_deg > 360.0 + 1e-3 {
                issues.push(CamValidationIssue::warning(format!(
                    "Wrapped 2D geometry circumferential span ({span_deg:.1}°) exceeds 360° at wrap radius {r_wrap:.2} mm."
                )));
            }
        }
    }

    issues
}

/// Validates undercut cutter relieved neck length against resolved cut depth (`SlotUndercut`, `Dovetail`).
pub fn validate_operation_undercut_depth(
    op: &Operation,
    tool: Option<&Tool>,
    rh: &ResolvedHeights,
) -> Vec<CamValidationIssue> {
    let mut issues = Vec::new();
    let total_depth = (rh.top_z_wcs
        - (rh.bottom_z_wcs + op.passes.stock_to_leave_axial.value_mm.max(0.0)))
    .max(0.0);
    if let Some(tool) = tool {
        match (op.kind, &tool.geometry) {
            (OperationKind::SlotUndercut, CutterGeometry::SlotMill { neck_length_mm, .. })
                if *neck_length_mm + 1e-4 < total_depth =>
            {
                issues.push(CamValidationIssue::error(format!(
                    "Slot mill relieved neck length ({neck_length_mm:.3} mm) is shorter than total slot depth ({total_depth:.3} mm)."
                )));
            }
            (OperationKind::Dovetail, CutterGeometry::DovetailCutter { neck_length_mm, .. })
                if *neck_length_mm + 1e-4 < total_depth =>
            {
                issues.push(CamValidationIssue::error(format!(
                    "Dovetail cutter relieved neck length ({neck_length_mm:.3} mm) is shorter than total cut depth ({total_depth:.3} mm)."
                )));
            }
            _ => {}
        }
    }
    issues
}
