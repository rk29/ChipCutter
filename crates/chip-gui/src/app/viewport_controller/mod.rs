mod measure_controller;
mod picking;
mod placement_modes;
mod sketch_authoring;
mod viewport_sync;

use std::borrow::Cow;

use chip_cam::{
    Operation, OperationRotaryMode, ResolvedRotaryConfig, ResolvedSetup,
    build_indexed_virtual_setup,
};
use chip_math::Ray3;

use crate::panels::SketchAuthorTool;

pub(crate) struct SketchAuthorInteractionInput {
    pub sketch_idx: usize,
    pub tool: SketchAuthorTool,
    pub ray: Ray3,
    pub mm_per_px: f64,
    pub primary_click: bool,
    pub right_click: bool,
}

/// Resolves the active viewport rotary angle (`deg`) and optional Indexed 3+1 HUD badge
/// `(axis_letter, angle_deg)` following strict precedence:
/// 1. Active simulation playback (`sim_active == true`) always owns `sim_rotary_deg`.
/// 2. Otherwise, when `auto_rotate_indexed_op == true` and `focused_op` is `Indexed3Plus1 { angle_deg }`
///    on a rotary-enabled Setup, auto-rotate to `angle_deg` and return the HUD badge when rotated.
/// 3. Otherwise `0.0°` with no indexed badge.
pub(crate) fn resolve_effective_viewport_rotary(
    sim_active: bool,
    sim_rotary_deg: f64,
    auto_rotate_indexed_op: bool,
    focused_op: Option<&Operation>,
    rotary: Option<ResolvedRotaryConfig>,
) -> (f64, Option<(char, f64)>) {
    if sim_active {
        return (sim_rotary_deg, None);
    }
    if auto_rotate_indexed_op
        && let Some(rot) = rotary
        && let Some(op) = focused_op
        && let OperationRotaryMode::Indexed3Plus1 { angle_deg } = op.rotary
    {
        let badge = (angle_deg.abs() > 1e-6).then_some((rot.axis_letter, angle_deg));
        return (angle_deg, badge);
    }
    (0.0, None)
}

/// Returns the effective [`ResolvedSetup`] used for `BoundaryPreviewOverlay`, `ZPlaneVisual`,
/// and `[Geometry]` / `[Heights]` inspection: when auto-rotate is active for an `Indexed3Plus1`
/// operation outside simulation playback, returns [`build_indexed_virtual_setup`] so previews and
/// heights match the exact upright `+Z` rotated orientation machined by `generate_operation_toolpath`.
pub(crate) fn effective_setup_for_indexed_preview<'a>(
    rs: &'a ResolvedSetup,
    auto_rotate_indexed_op: bool,
    sim_active: bool,
    op: &Operation,
) -> Cow<'a, ResolvedSetup> {
    if !sim_active
        && auto_rotate_indexed_op
        && let Some(rot) = rs.rotary
        && let OperationRotaryMode::Indexed3Plus1 { angle_deg } = op.rotary
    {
        return Cow::Owned(build_indexed_virtual_setup(rs, &rot, angle_deg));
    }
    Cow::Borrowed(rs)
}

#[cfg(test)]
mod tests;
