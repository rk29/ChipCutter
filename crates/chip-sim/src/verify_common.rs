//! Shared verification & sweep conversion helpers unified across Heightfield, Tri-Dexel, and
//! Cylindrical simulation engines:
//! - Arc-to-chord decomposition (`decompose_segment_to_chords`, `segment_to_sweep_chords`)
//! - Fractional segment slicing (`slice_segment_range`)
//! - Bounded diagnostic batch coalescer (`ViolationBatchCoalescer`)
//! - Rule 5 vertical plunge validation (`evaluate_vertical_plunge_violation`)

use chip_cam::{ArcPlane, SegmentMotion3D, ToolpathSegment3D};
use chip_math::{DVec2, DVec3};
use chip_stock::{
    RadialCutterProfile, SpanRuleViolations, StockVerifyTolerances, SweepChord, SweepMotion,
};

use crate::{
    timeline::MotionSpan,
    verifier::{SimDiagnostic, SimIssueKind, SimSeverity, SimVerificationConfig},
};

/// Linearized 3D micro-chord emitted by [`decompose_segment_to_chords`].
#[derive(Debug, Clone, Copy)]
pub(crate) struct LinearChord3D {
    pub(crate) p0: DVec3,
    pub(crate) p1: DVec3,
    pub(crate) rotary_deg_0: f64,
    pub(crate) rotary_deg_1: f64,
}

/// Decomposes a [`ToolpathSegment3D`] (linear or circular/helical arc) into linear micro-chords
/// whose chord sag is at most `max_sag_mm` (capped at `<= 0.05 rad` per chord).
pub(crate) fn decompose_segment_to_chords(
    seg: &ToolpathSegment3D,
    max_sag_mm: f64,
) -> Vec<LinearChord3D> {
    match seg.motion {
        SegmentMotion3D::Rapid | SegmentMotion3D::Linear { .. } => {
            vec![LinearChord3D {
                p0: seg.start_wcs,
                p1: seg.end_wcs,
                rotary_deg_0: seg.start_rotary_deg,
                rotary_deg_1: seg.end_rotary_deg,
            }]
        }
        SegmentMotion3D::Arc {
            radius_mm,
            signed_sweep_rad,
            ..
        } => {
            let r = radius_mm.max(1e-6);
            // Chord sag s = r * (1 - cos(dtheta / 2)) <= max_sag_mm, capped at <= 0.05 rad (~3°)
            let cos_half = (1.0 - (max_sag_mm.max(1e-6) / r).min(0.5)).clamp(-1.0, 1.0);
            let max_dtheta = (2.0 * cos_half.acos()).clamp(0.02, 0.05);
            let steps = ((signed_sweep_rad.abs() / max_dtheta).ceil() as usize).clamp(4, 2_048);

            let mut out = Vec::with_capacity(steps);
            let mut prev = seg.start_wcs;
            let mut prev_rot = seg.start_rotary_deg;
            for k in 1..=steps {
                let u = k as f64 / steps as f64;
                let (next, next_rot) = if k == steps {
                    (seg.end_wcs, seg.end_rotary_deg)
                } else {
                    (seg.sample_point(u), seg.sample_rotary_deg(u))
                };
                out.push(LinearChord3D {
                    p0: prev,
                    p1: next,
                    rotary_deg_0: prev_rot,
                    rotary_deg_1: next_rot,
                });
                prev = next;
                prev_rot = next_rot;
            }
            out
        }
    }
}

/// Appends [`SweepChord`] primitives for `seg` to `out`, preserving closed-form planar XY arcs
/// for exact Heightfield distance evaluation while decomposing sloped/helical arcs with `<= 0.005 mm`
/// chord sag for 3D Tri-Dexel and 4th-axis Cylindrical sweeps.
pub(crate) fn push_segment_sweep_chords(
    seg: &ToolpathSegment3D,
    op_idx: u32,
    profile_idx: usize,
    verify_tol: StockVerifyTolerances,
    out: &mut Vec<SweepChord>,
) {
    let is_rapid = matches!(seg.motion, SegmentMotion3D::Rapid);
    if let SegmentMotion3D::Arc {
        plane: ArcPlane::XY,
        center_wcs,
        radius_mm,
        signed_sweep_rad,
        ..
    } = seg.motion
        && (seg.end_wcs.z - seg.start_wcs.z).abs() <= 1e-7
    {
        let center_xy = DVec2::new(center_wcs.x, center_wcs.y);
        let a0_rad = (seg.start_wcs.y - center_xy.y).atan2(seg.start_wcs.x - center_xy.x);
        out.push(SweepChord {
            motion: SweepMotion::PlanarArcXY {
                start: seg.start_wcs,
                end: seg.end_wcs,
                center_xy,
                radius_mm,
                a0_rad,
                signed_sweep_rad,
                rotary_deg_0: seg.start_rotary_deg,
                rotary_deg_1: seg.end_rotary_deg,
            },
            op_idx,
            is_rapid,
            profile_idx,
            verify_tol,
        });
        return;
    }

    for ch in decompose_segment_to_chords(seg, 0.005) {
        out.push(SweepChord {
            motion: SweepMotion::Linear {
                p0: ch.p0,
                p1: ch.p1,
                rotary_deg_0: ch.rotary_deg_0,
                rotary_deg_1: ch.rotary_deg_1,
            },
            op_idx,
            is_rapid,
            profile_idx,
            verify_tol,
        });
    }
}

/// Slices a fractional sub-segment `[u_start, u_end]` (`0.0 <= u_start < u_end <= 1.0`) out of `seg`
/// for exact mid-segment interactive timeline scrubbing across all 3 volumetric stock engines.
pub(crate) fn slice_segment_range(
    seg: &ToolpathSegment3D,
    u_start: f64,
    u_end: f64,
) -> Option<ToolpathSegment3D> {
    let a = u_start.clamp(0.0, 1.0);
    let b = u_end.clamp(0.0, 1.0);
    if b - a <= 1e-6 {
        return None;
    }
    let start_wcs = seg.sample_point(a);
    let end_wcs = seg.sample_point(b);
    let motion = match seg.motion {
        SegmentMotion3D::Rapid => SegmentMotion3D::Rapid,
        SegmentMotion3D::Linear { feed_mm_min, role } => {
            SegmentMotion3D::Linear { feed_mm_min, role }
        }
        SegmentMotion3D::Arc {
            plane,
            dir,
            center_wcs,
            radius_mm,
            signed_sweep_rad,
            feed_mm_min,
            role,
        } => SegmentMotion3D::Arc {
            plane,
            dir,
            center_wcs,
            radius_mm,
            signed_sweep_rad: signed_sweep_rad * (b - a),
            feed_mm_min,
            role,
        },
    };
    Some(ToolpathSegment3D {
        start_wcs,
        end_wcs,
        motion,
        start_rotary_deg: seg.sample_rotary_deg(a),
        end_rotary_deg: seg.sample_rotary_deg(b),
    })
}

/// Uncoalesced per-span violation event emitted into [`ViolationBatchCoalescer`].
#[derive(Debug, Clone)]
pub(crate) struct RuleViolationHit {
    pub(crate) severity: SimSeverity,
    pub(crate) kind: SimIssueKind,
    pub(crate) span_idx: usize,
    pub(crate) op_idx: usize,
    pub(crate) op_id: String,
    pub(crate) op_name: String,
    pub(crate) tool_number: u32,
    pub(crate) time_sec: f64,
    pub(crate) location_wcs: DVec3,
    pub(crate) excess_mm: f64,
    pub(crate) detail: String,
}

/// Bounded streaming diagnostic coalescer shared across Heightfield, Tri-Dexel, and Cylindrical
/// verifiers.
///
/// Merges consecutive hits of the same `(op_idx, kind)` within `merge_gap_spans` (default `16`)
/// and caps emitted diagnostics to at most `max_per_op_kind` (default `3`) per `(op_idx, kind)`
/// while preserving onset timestamps, peak penetration depths, and critical severity upgrades.
#[derive(Debug, Clone)]
pub(crate) struct ViolationBatchCoalescer {
    diagnostics: Vec<SimDiagnostic>,
    last_seg_by_diag: Vec<usize>,
    max_per_op_kind: usize,
    merge_gap_spans: usize,
}

impl Default for ViolationBatchCoalescer {
    fn default() -> Self {
        Self::new(3, 16)
    }
}

impl ViolationBatchCoalescer {
    pub(crate) fn new(max_per_op_kind: usize, merge_gap_spans: usize) -> Self {
        Self {
            diagnostics: Vec::new(),
            last_seg_by_diag: Vec::new(),
            max_per_op_kind: max_per_op_kind.max(1),
            merge_gap_spans,
        }
    }

    pub(crate) fn push(&mut self, hit: RuleViolationHit) {
        let existing_count = self
            .diagnostics
            .iter()
            .filter(|d| d.op_idx == hit.op_idx && d.kind == hit.kind)
            .count();

        for (di, diag) in self.diagnostics.iter_mut().enumerate().rev() {
            if diag.op_idx == hit.op_idx
                && diag.kind == hit.kind
                && (hit.span_idx <= self.last_seg_by_diag[di].saturating_add(self.merge_gap_spans)
                    || existing_count >= self.max_per_op_kind)
            {
                self.last_seg_by_diag[di] = self.last_seg_by_diag[di].max(hit.span_idx);
                let upgrades_severity = hit.severity == SimSeverity::CriticalError
                    && diag.severity != SimSeverity::CriticalError;
                if upgrades_severity
                    || (hit.severity == diag.severity && hit.excess_mm > diag.max_excess_mm)
                {
                    diag.severity = hit.severity;
                    diag.max_excess_mm = hit.excess_mm;
                    diag.location_wcs = hit.location_wcs;
                    diag.message = hit.detail;
                }
                return;
            }
        }

        self.last_seg_by_diag.push(hit.span_idx);
        self.diagnostics.push(SimDiagnostic {
            severity: hit.severity,
            kind: hit.kind,
            op_idx: hit.op_idx,
            op_id: hit.op_id,
            op_name: hit.op_name,
            tool_number: hit.tool_number,
            global_seg_idx: hit.span_idx,
            timeline_time_sec: hit.time_sec,
            location_wcs: hit.location_wcs,
            max_excess_mm: hit.excess_mm,
            message: hit.detail,
        });
    }

    /// Translates a [`SpanRuleViolations`] summary for `span` into bounded coalesced diagnostics.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn push_span_violations(
        &mut self,
        violations: &SpanRuleViolations,
        span_idx: usize,
        op_idx: usize,
        op_id: &str,
        op_name: &str,
        tool_number: u32,
        time_sec: f64,
        prof: &RadialCutterProfile,
    ) {
        if let Some((depth, loc)) = violations.rapid {
            self.push(RuleViolationHit {
                severity: SimIssueKind::RapidIntoStock.default_severity(),
                kind: SimIssueKind::RapidIntoStock,
                span_idx,
                op_idx,
                op_id: op_id.to_string(),
                op_name: op_name.to_string(),
                tool_number,
                time_sec,
                location_wcs: loc,
                excess_mm: depth,
                detail: format!(
                    "G0 rapid move crashed {depth:.3} mm into remaining stock at WCS ({:.2}, {:.2}, {:.2}).",
                    loc.x, loc.y, loc.z
                ),
            });
        }
        if let Some((depth, loc)) = violations.gouge {
            self.push(RuleViolationHit {
                severity: SimIssueKind::TargetModelGouge.default_severity(),
                kind: SimIssueKind::TargetModelGouge,
                span_idx,
                op_idx,
                op_id: op_id.to_string(),
                op_name: op_name.to_string(),
                tool_number,
                time_sec,
                location_wcs: loc,
                excess_mm: depth,
                detail: format!(
                    "Tool T{tool_number} gouged target model by {depth:.3} mm at WCS ({:.2}, {:.2}, {:.2}).",
                    loc.x, loc.y, loc.z
                ),
            });
        }
        if let Some((depth, loc)) = violations.neck {
            self.push(RuleViolationHit {
                severity: SimSeverity::CriticalError,
                kind: SimIssueKind::FluteLengthExceeded,
                span_idx,
                op_idx,
                op_id: op_id.to_string(),
                op_name: op_name.to_string(),
                tool_number,
                time_sec,
                location_wcs: loc,
                excess_mm: depth,
                detail: format!(
                    "Thread mill T{tool_number} relieved neck collided {depth:.2} mm with uncut stock (missing/undersized pilot hole or insufficient neck clearance)."
                ),
            });
        }
        if let Some((depth, loc)) = violations.flute {
            let reach_label = if prof.is_thread_mill {
                "usable neck length"
            } else {
                "flute length"
            };
            self.push(RuleViolationHit {
                severity: SimIssueKind::FluteLengthExceeded.default_severity(),
                kind: SimIssueKind::FluteLengthExceeded,
                span_idx,
                op_idx,
                op_id: op_id.to_string(),
                op_name: op_name.to_string(),
                tool_number,
                time_sec,
                location_wcs: loc,
                excess_mm: depth,
                detail: format!(
                    "Engagement depth exceeded T{tool_number} {reach_label} ({:.2} mm) by {depth:.2} mm (shank rub).",
                    prof.effective_reach_mm()
                ),
            });
        }
        if let Some((depth, loc)) = violations.holder {
            self.push(RuleViolationHit {
                severity: SimIssueKind::HolderCollision.default_severity(),
                kind: SimIssueKind::HolderCollision,
                span_idx,
                op_idx,
                op_id: op_id.to_string(),
                op_name: op_name.to_string(),
                tool_number,
                time_sec,
                location_wcs: loc,
                excess_mm: depth,
                detail: format!(
                    "Collet/holder of T{tool_number} collided {depth:.2} mm into stock (stickout {:.2} mm insufficient).",
                    prof.stickout_length_mm
                ),
            });
        }
    }

    pub(crate) fn into_diagnostics(self) -> Vec<SimDiagnostic> {
        self.diagnostics
    }
}

/// Evaluates Rule 5 (vertical plunge into uncut material: missing pilot hole on a thread mill,
/// non-center-cutting cutter plunge, or excessive axial plunge depth) shared across backends.
pub(crate) fn evaluate_vertical_plunge_violation(
    span: &MotionSpan,
    op_id: &str,
    op_name: &str,
    tool_number: u32,
    prof: &RadialCutterProfile,
    cfg: &SimVerificationConfig,
    sample_plunge_depth_mm: impl FnOnce(f64, f64, f64, f64, Option<f64>) -> f64,
) -> Option<RuleViolationHit> {
    let seg = &span.segment;
    if matches!(seg.motion, SegmentMotion3D::Rapid) {
        return None;
    }
    let dxy = (seg.end_wcs.x - seg.start_wcs.x).hypot(seg.end_wcs.y - seg.start_wcs.y);
    if dxy > 1e-4 || seg.end_wcs.z >= seg.start_wcs.z - 1e-4 {
        return None;
    }

    let thread_check_r = prof
        .is_thread_mill
        .then(|| (prof.cutting_radius_mm - cfg.radial_wall_skin_mm).max(0.0));
    let plunge_in_mat = sample_plunge_depth_mm(
        seg.start_wcs.x,
        seg.start_wcs.y,
        seg.start_wcs.z,
        seg.end_wcs.z,
        thread_check_r,
    );
    if !plunge_in_mat.is_finite() || plunge_in_mat <= 0.05 {
        return None;
    }

    let (severity, excess_mm, detail) = if prof.is_thread_mill {
        (
            SimSeverity::CriticalError,
            plunge_in_mat,
            format!(
                "Thread mill T{tool_number} plunged vertically {plunge_in_mat:.2} mm into solid un-drilled stock (missing or undersized pilot hole)."
            ),
        )
    } else if !prof.is_center_cutting {
        (
            SimIssueKind::UnsafeVerticalPlunge.default_severity(),
            plunge_in_mat,
            format!(
                "Non-center-cutting tool T{tool_number} plunged vertically {plunge_in_mat:.2} mm into stock (use Helical or Linear ramp)."
            ),
        )
    } else if plunge_in_mat > prof.max_safe_plunge_depth_mm + 0.05 {
        (
            SimIssueKind::UnsafeVerticalPlunge.default_severity(),
            plunge_in_mat - prof.max_safe_plunge_depth_mm,
            format!(
                "Vertical plunge depth ({plunge_in_mat:.2} mm) exceeds safe axial plunge threshold ({:.2} mm) on T{tool_number}.",
                prof.max_safe_plunge_depth_mm
            ),
        )
    } else {
        return None;
    };

    Some(RuleViolationHit {
        severity,
        kind: SimIssueKind::UnsafeVerticalPlunge,
        span_idx: span.global_seg_idx,
        op_idx: span.op_idx,
        op_id: op_id.to_string(),
        op_name: op_name.to_string(),
        tool_number,
        time_sec: span.t_start_sec,
        location_wcs: seg.end_wcs,
        excess_mm,
        detail,
    })
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cam::{ArcDir, ArcPlane, MotionRole};

    use super::*;

    #[test]
    fn violation_batch_coalescer_merges_adjacent_hits_caps_count_and_upgrades_severity() {
        let mut coal = ViolationBatchCoalescer::default();

        for (i, depth) in [0.2, 0.9, 0.4, 0.3, 0.5].iter().enumerate() {
            coal.push(RuleViolationHit {
                severity: SimSeverity::Warning,
                kind: SimIssueKind::FluteLengthExceeded,
                span_idx: 10 + i * 2,
                op_idx: 0,
                op_id: "op_1".to_string(),
                op_name: "Op 1".to_string(),
                tool_number: 1,
                time_sec: 1.0 + i as f64,
                location_wcs: DVec3::new(i as f64, 0.0, -5.0),
                excess_mm: *depth,
                detail: format!("rub {depth:.2}"),
            });
        }
        coal.push(RuleViolationHit {
            severity: SimSeverity::CriticalError,
            kind: SimIssueKind::FluteLengthExceeded,
            span_idx: 22,
            op_idx: 0,
            op_id: "op_1".to_string(),
            op_name: "Op 1".to_string(),
            tool_number: 1,
            time_sec: 6.0,
            location_wcs: DVec3::new(9.0, 0.0, -5.0),
            excess_mm: 0.3,
            detail: "neck collision".to_string(),
        });

        for k in 1..=4 {
            coal.push(RuleViolationHit {
                severity: SimSeverity::Warning,
                kind: SimIssueKind::FluteLengthExceeded,
                span_idx: 10 + k * 100,
                op_idx: 0,
                op_id: "op_1".to_string(),
                op_name: "Op 1".to_string(),
                tool_number: 1,
                time_sec: 10.0 * k as f64,
                location_wcs: DVec3::new(0.0, 0.0, -5.0),
                excess_mm: 0.1 * k as f64,
                detail: format!("pass {k}"),
            });
        }

        let diags = coal.into_diagnostics();
        assert_eq!(
            diags.len(),
            3,
            "bounded at 3 diagnostics per (op_idx, kind)"
        );
        assert_eq!(diags[0].global_seg_idx, 10);
        assert_eq!(diags[0].severity, SimSeverity::CriticalError);
        assert_eq!(diags[0].message, "neck collision");
        assert!((diags[2].max_excess_mm - 0.4).abs() < 1e-9);
    }

    #[test]
    fn decompose_segment_to_chords_bounds_arc_sagitta_and_preserves_endpoints() {
        let seg = ToolpathSegment3D {
            start_wcs: DVec3::new(10.0, 0.0, -2.0),
            end_wcs: DVec3::new(-10.0, 0.0, -4.0),
            start_rotary_deg: 0.0,
            end_rotary_deg: 0.0,
            motion: SegmentMotion3D::Arc {
                plane: ArcPlane::XY,
                dir: ArcDir::CCW,
                center_wcs: DVec3::new(0.0, 0.0, -2.0),
                radius_mm: 10.0,
                signed_sweep_rad: std::f64::consts::PI,
                feed_mm_min: 600.0,
                role: MotionRole::Cut,
            },
        };
        let chords = decompose_segment_to_chords(&seg, 0.005);
        assert!(chords.len() >= 60);
        assert_eq!(chords.first().unwrap().p0, seg.start_wcs);
        assert_eq!(chords.last().unwrap().p1, seg.end_wcs);
    }
}
