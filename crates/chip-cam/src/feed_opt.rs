//! Engagement-aware feedrate optimization post-pass (`M58`).
//!
//! Operates directly on typed [`ClEvent`] streams after strategy generation so all 2.5D,
//! 3-axis surface, V-carve, and 4th-axis strategies benefit uniformly:
//! - **Internal Arc Scaling:** exact perimeter-to-centerline scaling
//!   `F_center = F_nom * ((R_arc - R_tool) / R_arc).clamp(min_feed_fraction, 1.0)` on concave/internal
//!   `G17` cutting arcs (`R_tool < R_arc < 4 * R_tool`) so cutting-edge chipload never spikes.
//! - **Sharp Corner Lookahead Slowdown:** detects linear cutting deflection angles
//!   `theta >= corner_angle_threshold_deg`, splits segments cleanly within `slowdown_distance`
//!   (preserving exact path geometry and endpoints with zero-length guards), decelerates
//!   monotonically approaching the apex, and recovers smoothly after exit.
//! - **Constant-Chipload Radial Chip-Thinning Boost:** optionally boosts straight/low-curvature
//!   light-engagement cutting passes up to `max_thinning_multiplier` while throttling back down
//!   inside corners and internal arcs, never accelerating plunges, ramps, or rapid moves.

use chip_math::{DVec2, DVec3};
use chip_tools::{Tool, compute_adaptive_thinning_feed};

use crate::{
    cl_data::{ArcDir, ArcPlane, ClEvent, MotionRole},
    operation::{EffectiveCuttingParams, OperationKind},
    params::FeedOptimizationConfig,
    toolpath::{ToolpathBuilder, ToolpathSegment3D, VerticalArcMove, XyArcMove},
};

const MIN_SPLIT_SPAN_MM: f64 = 0.05;
const MIN_BREAKPOINT_GAP_MM: f64 = 0.02;
const MAX_RAMP_STEPS: usize = 3;

/// Computes exact perimeter-to-centerline feed scaling for a planar internal cutting arc.
///
/// For `tool_radius_mm < arc_radius_mm < 4.0 * tool_radius_mm`, returns
/// `((arc_radius_mm - tool_radius_mm) / arc_radius_mm).clamp(min_feed_fraction, 1.0)`.
/// Returns `1.0` outside that range or on degenerate/non-finite inputs.
#[inline]
pub fn internal_arc_feed_scale(
    arc_radius_mm: f64,
    tool_radius_mm: f64,
    min_feed_fraction: f64,
) -> f64 {
    if !arc_radius_mm.is_finite()
        || !tool_radius_mm.is_finite()
        || !min_feed_fraction.is_finite()
        || tool_radius_mm <= 1e-6
        || arc_radius_mm <= 1e-6
    {
        return 1.0;
    }
    let min_f = min_feed_fraction.clamp(0.15, 1.0);
    if arc_radius_mm > tool_radius_mm && arc_radius_mm < 4.0 * tool_radius_mm {
        ((arc_radius_mm - tool_radius_mm) / arc_radius_mm).clamp(min_f, 1.0)
    } else {
        1.0
    }
}

/// Computes monotonic corner slowdown scale in `[min_feed_fraction, 1.0]` for a turn deflection
/// angle `deflection_deg` (`0°` straight, `90°` square corner, `180°` full reversal).
#[inline]
pub fn corner_feed_scale(deflection_deg: f64, threshold_deg: f64, min_feed_fraction: f64) -> f64 {
    if !deflection_deg.is_finite() || !threshold_deg.is_finite() || !min_feed_fraction.is_finite() {
        return 1.0;
    }
    let min_f = min_feed_fraction.clamp(0.15, 1.0);
    let thresh = threshold_deg.clamp(5.0, 120.0);
    if deflection_deg < thresh {
        return 1.0;
    }
    let full_deg = if thresh < 90.0 {
        90.0
    } else {
        (thresh + 30.0).min(180.0)
    };
    let t = ((deflection_deg - thresh) / (full_deg - thresh).max(1.0)).clamp(0.0, 1.0);
    (1.0 - t * (1.0 - min_f)).clamp(min_f, 1.0)
}

#[inline]
fn is_cutting_role(role: MotionRole) -> bool {
    matches!(role, MotionRole::Cut | MotionRole::FinishCut)
}

#[derive(Debug, Clone, Copy)]
struct LinearCutSeg {
    start_wcs: DVec3,
    end_wcs: DVec3,
    orig_feed: f64,
    role: MotionRole,
}

/// Optimizes cutting feedrates in-place on a generated [`ClEvent`] stream according to `cfg`.
///
/// Fast-path no-op when `!cfg.enabled` or for `OperationKind::DrillBore` / `OperationKind::ThreadMill`.
/// Never modifies `G0` rapids, `MotionRole::Plunge`/`Ramp`/`LinkTransition`/`LeadIn`/`LeadOut`/
/// `TabRamp`, or rotary rapid moves.
pub fn optimize_toolpath_feedrates(
    events: &mut Vec<ClEvent>,
    cfg: &FeedOptimizationConfig,
    tool: &Tool,
    cutting: &EffectiveCuttingParams,
    op_kind: OperationKind,
) {
    if !cfg.enabled
        || matches!(
            op_kind,
            OperationKind::DrillBore | OperationKind::ThreadMill
        )
    {
        return;
    }

    let tool_d = tool.geometry.cutting_diameter_mm().max(0.05);
    let tool_r = 0.5 * tool_d;
    let min_frac = if cfg.min_feed_fraction.is_finite() {
        cfg.min_feed_fraction.clamp(0.15, 1.0)
    } else {
        0.35
    };
    let angle_thresh_deg = if cfg.corner_angle_threshold_deg.is_finite() {
        cfg.corner_angle_threshold_deg.clamp(5.0, 120.0)
    } else {
        25.0
    };
    let slowdown_dist = if cfg.slowdown_distance.value_mm.is_finite() {
        cfg.slowdown_distance.value_mm.max(0.0)
    } else {
        0.5 * tool_d
    };
    let max_thin_mult = if cfg.max_thinning_multiplier.is_finite() {
        cfg.max_thinning_multiplier.clamp(1.0, 2.5)
    } else {
        1.5
    };

    let nom_feed = cutting.cutting_feed_mm_min.max(1.0);
    let max_feed_cap = nom_feed * max_thin_mult;
    let rct_mult = if cfg.chip_thinning_boost {
        compute_adaptive_thinning_feed(tool_d, cutting.stepover_mm, 1.0, max_thin_mult)
    } else {
        1.0
    };

    let boost_straight_feed = |orig_feed: f64, role: MotionRole, p0: DVec3, p1: DVec3| -> f64 {
        if !cfg.chip_thinning_boost || !orig_feed.is_finite() || orig_feed <= 0.0 {
            return orig_feed;
        }
        let delta = p1 - p0;
        let horiz = delta.x.hypot(delta.y);
        // Never accelerate downward Z ramps or plunges
        if delta.z < -1e-4 && (-delta.z) > 0.1 * horiz.max(1e-6) {
            return orig_feed;
        }
        let mult = if rct_mult > 1.001 {
            rct_mult
        } else if matches!(role, MotionRole::FinishCut) {
            1.15_f64.min(max_thin_mult)
        } else {
            1.0
        };
        (orig_feed * mult).min(max_feed_cap).max(orig_feed)
    };

    let src = std::mem::take(events);
    let mut out = Vec::with_capacity(src.len() + src.len() / 4);
    let mut cur_pos: Option<DVec3> = None;
    let mut prev_cut_end_tangent: Option<DVec3> = None;
    let mut i = 0usize;

    while i < src.len() {
        match &src[i] {
            ClEvent::LinearMove { role, .. } if is_cutting_role(*role) && cur_pos.is_some() => {
                // Gather contiguous run of cutting LinearMoves
                let run_entry_tangent = prev_cut_end_tangent;
                let mut run: Vec<LinearCutSeg> = Vec::new();
                while i < src.len() {
                    if let ClEvent::LinearMove {
                        end_wcs,
                        feed_mm_min,
                        role,
                    } = src[i]
                        && is_cutting_role(role)
                        && let Some(start_wcs) = cur_pos
                    {
                        run.push(LinearCutSeg {
                            start_wcs,
                            end_wcs,
                            orig_feed: feed_mm_min,
                            role,
                        });
                        cur_pos = Some(end_wcs);
                        i += 1;
                    } else {
                        break;
                    }
                }

                // Peek next event: if it is a cutting XY ArcMove, extract its start tangent
                let next_cut_start_tangent = if i < src.len()
                    && let ClEvent::ArcMove {
                        plane: ArcPlane::XY,
                        dir,
                        center_offset_ijk,
                        role,
                        ..
                    } = src[i]
                    && is_cutting_role(role)
                {
                    xy_arc_start_tangent(center_offset_ijk, dir)
                } else {
                    None
                };

                emit_optimized_linear_run(
                    &run,
                    run_entry_tangent,
                    next_cut_start_tangent,
                    cfg.corner_slowdown,
                    slowdown_dist,
                    angle_thresh_deg,
                    min_frac,
                    &boost_straight_feed,
                    &mut out,
                );

                prev_cut_end_tangent = run.last().and_then(|s| {
                    let d = s.end_wcs - s.start_wcs;
                    (d.length_squared() > 1e-12).then(|| d.normalize())
                });
            }
            ClEvent::ArcMove {
                plane,
                dir,
                end_wcs,
                center_offset_ijk,
                radius_mm,
                turns,
                feed_mm_min,
                role,
            } => {
                let mut adj_feed = *feed_mm_min;
                if is_cutting_role(*role) && cfg.internal_arc_scaling && *plane == ArcPlane::XY {
                    let scale = internal_arc_feed_scale(*radius_mm, tool_r, min_frac);
                    if scale < 0.999_999 {
                        adj_feed = (adj_feed * scale).max(1.0);
                    }
                }
                let start_opt = cur_pos;
                prev_cut_end_tangent = if is_cutting_role(*role) && *plane == ArcPlane::XY {
                    start_opt
                        .and_then(|s0| xy_arc_end_tangent(s0, *end_wcs, *center_offset_ijk, *dir))
                } else {
                    None
                };
                cur_pos = Some(*end_wcs);
                out.push(ClEvent::ArcMove {
                    plane: *plane,
                    dir: *dir,
                    end_wcs: *end_wcs,
                    center_offset_ijk: *center_offset_ijk,
                    radius_mm: *radius_mm,
                    turns: *turns,
                    feed_mm_min: adj_feed,
                    role: *role,
                });
                i += 1;
            }
            ClEvent::RotaryLinearMove {
                end_wcs,
                rotary_deg,
                feed_mm_min,
                effective_radius_mm,
                role,
            } => {
                let adj_feed = if is_cutting_role(*role)
                    && let Some(start_wcs) = cur_pos
                {
                    boost_straight_feed(*feed_mm_min, *role, start_wcs, *end_wcs)
                } else {
                    *feed_mm_min
                };
                cur_pos = Some(*end_wcs);
                prev_cut_end_tangent = None;
                out.push(ClEvent::RotaryLinearMove {
                    end_wcs: *end_wcs,
                    rotary_deg: *rotary_deg,
                    feed_mm_min: adj_feed,
                    effective_radius_mm: *effective_radius_mm,
                    role: *role,
                });
                i += 1;
            }
            ClEvent::RapidMove { end_wcs } | ClEvent::RotaryRapidMove { end_wcs, .. } => {
                cur_pos = Some(*end_wcs);
                prev_cut_end_tangent = None;
                out.push(src[i].clone());
                i += 1;
            }
            ClEvent::LinearMove { end_wcs, .. } => {
                cur_pos = Some(*end_wcs);
                prev_cut_end_tangent = None;
                out.push(src[i].clone());
                i += 1;
            }
            ClEvent::DrillCycle {
                bottom_wcs,
                clear_z_wcs,
                ..
            } => {
                cur_pos = Some(DVec3::new(bottom_wcs.x, bottom_wcs.y, *clear_z_wcs));
                prev_cut_end_tangent = None;
                out.push(src[i].clone());
                i += 1;
            }
            _ => {
                out.push(src[i].clone());
                i += 1;
            }
        }
    }

    *events = out;
}

#[allow(clippy::too_many_arguments)]
fn emit_optimized_linear_run(
    run: &[LinearCutSeg],
    entry_tangent: Option<DVec3>,
    exit_tangent: Option<DVec3>,
    corner_slowdown: bool,
    slowdown_dist: f64,
    angle_thresh_deg: f64,
    min_frac: f64,
    boost_straight_feed: &impl Fn(f64, MotionRole, DVec3, DVec3) -> f64,
    out: &mut Vec<ClEvent>,
) {
    let m = run.len();
    if m == 0 {
        return;
    }

    let mut tangents: Vec<Option<DVec3>> = Vec::with_capacity(m);
    let mut cruise_feeds: Vec<f64> = Vec::with_capacity(m);
    for seg in run {
        let d = seg.end_wcs - seg.start_wcs;
        tangents.push((d.length_squared() > 1e-12).then(|| d.normalize()));
        cruise_feeds.push(boost_straight_feed(
            seg.orig_feed,
            seg.role,
            seg.start_wcs,
            seg.end_wcs,
        ));
    }

    let mut vertex_scales = vec![1.0_f64; m + 1];
    if corner_slowdown && slowdown_dist > 1e-4 {
        if let (Some(t_in), Some(t_out)) = (entry_tangent, tangents[0]) {
            let theta_deg = t_in.dot(t_out).clamp(-1.0, 1.0).acos().to_degrees();
            vertex_scales[0] = corner_feed_scale(theta_deg, angle_thresh_deg, min_frac);
        }
        for k in 1..m {
            if let (Some(t_in), Some(t_out)) = (tangents[k - 1], tangents[k]) {
                let theta_deg = t_in.dot(t_out).clamp(-1.0, 1.0).acos().to_degrees();
                vertex_scales[k] = corner_feed_scale(theta_deg, angle_thresh_deg, min_frac);
            }
        }
        if let (Some(t_in), Some(t_out)) = (tangents[m - 1], exit_tangent) {
            let theta_deg = t_in.dot(t_out).clamp(-1.0, 1.0).acos().to_degrees();
            vertex_scales[m] = corner_feed_scale(theta_deg, angle_thresh_deg, min_frac);
        }
    }

    for (j, seg) in run.iter().enumerate() {
        let s_entry = vertex_scales[j];
        let s_exit = vertex_scales[j + 1];
        let f_cruise = cruise_feeds[j].max(1.0);
        let len = seg.start_wcs.distance(seg.end_wcs);

        if !corner_slowdown || slowdown_dist <= 1e-4 || (s_entry >= 0.999 && s_exit >= 0.999) {
            out.push(ClEvent::LinearMove {
                end_wcs: seg.end_wcs,
                feed_mm_min: f_cruise,
                role: seg.role,
            });
            continue;
        }

        let f_entry = (seg.orig_feed * s_entry).max(1.0).min(f_cruise);
        let f_exit = (seg.orig_feed * s_exit).max(1.0).min(f_cruise);

        // Zero-length / micro-segment guard: never create degenerate splits
        if len <= MIN_SPLIT_SPAN_MM {
            out.push(ClEvent::LinearMove {
                end_wcs: seg.end_wcs,
                feed_mm_min: f_cruise.min(f_entry).min(f_exit),
                role: seg.role,
            });
            continue;
        }

        let mut d_rec = if s_entry < 0.999 {
            0.5 * slowdown_dist
        } else {
            0.0
        };
        let mut d_dec = if s_exit < 0.999 { slowdown_dist } else { 0.0 };
        if d_rec + d_dec > len {
            let total = d_rec + d_dec;
            if total > 1e-9 {
                d_rec = len * (d_rec / total);
                d_dec = len * (d_dec / total);
            }
        }

        let f_env = |s: f64| -> f64 {
            let fe = if d_rec > 1e-6 {
                let u = (s / d_rec).clamp(0.0, 1.0);
                f_entry + (f_cruise - f_entry) * u
            } else {
                f_cruise
            };
            let fx = if d_dec > 1e-6 {
                let u = ((len - s) / d_dec).clamp(0.0, 1.0);
                f_exit + (f_cruise - f_exit) * u
            } else {
                f_cruise
            };
            f_cruise.min(fe).min(fx).max(1.0)
        };

        let target_step = (slowdown_dist / (MAX_RAMP_STEPS as f64)).max(0.15);
        let mut bps: Vec<f64> = Vec::with_capacity(2 * MAX_RAMP_STEPS + 2);
        bps.push(0.0);

        if d_rec > MIN_SPLIT_SPAN_MM {
            let n_rec = ((d_rec / target_step).ceil() as usize).clamp(1, MAX_RAMP_STEPS);
            for k in 1..=n_rec {
                push_breakpoint(&mut bps, d_rec * (k as f64) / (n_rec as f64), len);
            }
        }

        let dec_start = (len - d_dec).max(0.0);
        if d_dec > MIN_SPLIT_SPAN_MM {
            push_breakpoint(&mut bps, dec_start, len);
            let n_dec = ((d_dec / target_step).ceil() as usize).clamp(1, MAX_RAMP_STEPS);
            for k in 1..n_dec {
                push_breakpoint(
                    &mut bps,
                    dec_start + d_dec * (k as f64) / (n_dec as f64),
                    len,
                );
            }
        }
        bps.push(len);

        for w_idx in 0..bps.len().saturating_sub(1) {
            let s_a = bps[w_idx];
            let s_b = bps[w_idx + 1];
            if s_b - s_a <= 1e-7 {
                continue;
            }
            let is_last = w_idx + 1 == bps.len() - 1;
            let end_pt = if is_last {
                seg.end_wcs
            } else {
                seg.start_wcs.lerp(seg.end_wcs, (s_b / len).clamp(0.0, 1.0))
            };
            let sub_feed = f_env(s_a).min(f_env(s_b));
            out.push(ClEvent::LinearMove {
                end_wcs: end_pt,
                feed_mm_min: sub_feed,
                role: seg.role,
            });
        }
    }
}

fn push_breakpoint(bps: &mut Vec<f64>, s: f64, total_len: f64) {
    if s <= MIN_BREAKPOINT_GAP_MM || s >= total_len - MIN_BREAKPOINT_GAP_MM {
        return;
    }
    if bps
        .last()
        .is_none_or(|&prev| s - prev > MIN_BREAKPOINT_GAP_MM)
    {
        bps.push(s);
    }
}

fn xy_arc_start_tangent(center_offset_ijk: DVec3, dir: ArcDir) -> Option<DVec3> {
    // Radius vector from center to start point is `-center_offset_ijk`
    let rx = -center_offset_ijk.x;
    let ry = -center_offset_ijk.y;
    let r = rx.hypot(ry);
    if r <= 1e-7 {
        return None;
    }
    let sign = match dir {
        ArcDir::CCW => 1.0,
        ArcDir::CW => -1.0,
    };
    Some(DVec3::new(-sign * ry / r, sign * rx / r, 0.0))
}

fn xy_arc_end_tangent(
    start_wcs: DVec3,
    end_wcs: DVec3,
    center_offset_ijk: DVec3,
    dir: ArcDir,
) -> Option<DVec3> {
    let center = start_wcs + center_offset_ijk;
    let rx = end_wcs.x - center.x;
    let ry = end_wcs.y - center.y;
    let r = rx.hypot(ry);
    if r <= 1e-7 {
        return None;
    }
    let sign = match dir {
        ArcDir::CCW => 1.0,
        ArcDir::CW => -1.0,
    };
    Some(DVec3::new(-sign * ry / r, sign * rx / r, 0.0))
}

/// Rebuilds synchronized [`ToolpathSegment3D`] spans from an optimized [`ClEvent`] stream
/// (keeping `SimulationTimeline` and `ToolpathStats` 100% consistent with `events`).
pub fn rebuild_segments_from_events(events: &[ClEvent]) -> Vec<ToolpathSegment3D> {
    let mut tb = ToolpathBuilder::new("__feed_opt_rebuild");
    for ev in events {
        match ev {
            ClEvent::RapidMove { end_wcs } => tb.rapid_to(*end_wcs),
            ClEvent::LinearMove {
                end_wcs,
                feed_mm_min,
                role,
            } => tb.linear_to(*end_wcs, *feed_mm_min, *role),
            ClEvent::ArcMove {
                plane,
                dir,
                end_wcs,
                center_offset_ijk,
                radius_mm,
                turns,
                feed_mm_min,
                role,
            } => {
                if let Some(prev) = tb.current_pos() {
                    match plane {
                        ArcPlane::XY => {
                            let center_xy = DVec2::new(
                                prev.x + center_offset_ijk.x,
                                prev.y + center_offset_ijk.y,
                            );
                            tb.arc_xy_to(XyArcMove {
                                end_wcs: *end_wcs,
                                center_xy,
                                radius_mm: *radius_mm,
                                ccw: matches!(dir, ArcDir::CCW),
                                turns: *turns,
                                feed_mm_min: *feed_mm_min,
                                role: *role,
                            });
                        }
                        ArcPlane::XZ | ArcPlane::YZ => {
                            let center_wcs = prev + *center_offset_ijk;
                            let (a0, a1) = match plane {
                                ArcPlane::XZ => (
                                    (prev.z - center_wcs.z).atan2(prev.x - center_wcs.x),
                                    (end_wcs.z - center_wcs.z).atan2(end_wcs.x - center_wcs.x),
                                ),
                                ArcPlane::YZ => (
                                    (prev.z - center_wcs.z).atan2(prev.y - center_wcs.y),
                                    (end_wcs.z - center_wcs.z).atan2(end_wcs.y - center_wcs.y),
                                ),
                                ArcPlane::XY => (0.0, 0.0),
                            };
                            let signed_sweep_rad = match dir {
                                ArcDir::CCW => (a1 - a0).rem_euclid(std::f64::consts::TAU),
                                ArcDir::CW => -(a0 - a1).rem_euclid(std::f64::consts::TAU),
                            };
                            tb.arc_vertical_plane_to(VerticalArcMove {
                                plane: *plane,
                                end_wcs: *end_wcs,
                                center_wcs,
                                radius_mm: *radius_mm,
                                signed_sweep_rad,
                                feed_mm_min: *feed_mm_min,
                                role: *role,
                            });
                        }
                    }
                }
            }
            ClEvent::RotaryIndex {
                angle_deg,
                rapid,
                feed_deg_min,
            } => tb.rotary_index_to(*angle_deg, *rapid, *feed_deg_min),
            ClEvent::RotaryRapidMove {
                end_wcs,
                rotary_deg,
            } => tb.rotary_rapid_to(*end_wcs, *rotary_deg),
            ClEvent::RotaryLinearMove {
                end_wcs,
                rotary_deg,
                feed_mm_min,
                effective_radius_mm,
                role,
            } => tb.rotary_linear_to(
                *end_wcs,
                *rotary_deg,
                *feed_mm_min,
                *effective_radius_mm,
                *role,
            ),
            ClEvent::DrillCycle {
                kind,
                bottom_wcs,
                r_plane_z_wcs,
                clear_z_wcs,
                peck_q_mm,
                decreasing_factor,
                min_peck_mm,
                chip_break_retract_mm,
                dwell_sec,
                feed_mm_min,
            } => {
                tb.emit_drill_cycle(crate::toolpath::DrillCycleSpec {
                    kind: *kind,
                    bottom_wcs: *bottom_wcs,
                    r_plane_z_wcs: *r_plane_z_wcs,
                    clear_z_wcs: *clear_z_wcs,
                    peck_q_mm: *peck_q_mm,
                    decreasing_factor: *decreasing_factor,
                    min_peck_mm: *min_peck_mm,
                    chip_break_retract_mm: *chip_break_retract_mm,
                    dwell_sec: *dwell_sec,
                    feed_mm_min: *feed_mm_min,
                });
            }
            _ => {}
        }
    }
    tb.segments_wcs
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::{DisplayUnit, ParamLength, approx_eq};
    use chip_tools::{CoolantMode, CutterGeometry, SpindleDirection, Tool, ToolId};

    use super::*;
    use crate::{operation::Operation, params::PassesParams};

    fn test_flat_endmill_6mm() -> Tool {
        Tool {
            id: ToolId("t1_6mm".to_string()),
            tool_number: 1,
            length_offset_number: 1,
            diameter_offset_number: 1,
            name: "6mm 3FL Flat Endmill".to_string(),
            vendor: None,
            product_id: None,
            native_unit: DisplayUnit::Millimeter,
            spindle_direction: SpindleDirection::Cw,
            default_coolant: CoolantMode::Flood,
            stickout_length_mm: 24.0,
            geometry: CutterGeometry::FlatEndmill {
                diameter_mm: 6.0,
                flute_length_mm: 18.0,
                overall_length_mm: 55.0,
                shank_diameter_mm: 6.0,
                flutes: 3,
            },
            presets: Vec::new(),
        }
    }

    fn test_cutting() -> EffectiveCuttingParams {
        EffectiveCuttingParams {
            spindle_rpm: 12_000.0,
            cutting_feed_mm_min: 1_000.0,
            lead_feed_mm_min: 800.0,
            ramp_feed_mm_min: 500.0,
            plunge_feed_mm_min: 330.0,
            stepdown_mm: 3.0,
            stepover_mm: 2.4,
            coolant: CoolantMode::Flood,
        }
    }

    #[test]
    fn internal_arc_scaling_matches_exact_perimeter_formula_and_clamps_min_fraction() {
        let tool = test_flat_endmill_6mm(); // tool_r = 3.0 mm
        let cutting = test_cutting();
        let cfg = FeedOptimizationConfig {
            enabled: true,
            corner_slowdown: false,
            min_feed_fraction: 0.35,
            internal_arc_scaling: true,
            chip_thinning_boost: false,
            ..FeedOptimizationConfig::default()
        };

        let mut events = vec![
            ClEvent::RapidMove {
                end_wcs: DVec3::new(0.0, 0.0, 5.0),
            },
            ClEvent::LinearMove {
                end_wcs: DVec3::new(0.0, 0.0, -5.0),
                feed_mm_min: 330.0,
                role: MotionRole::Plunge,
            },
            // Case 1: R_arc = 6.0 mm (2 * tool_r) -> exact scale = (6 - 3) / 6 = 0.5 -> F = 500.0
            ClEvent::ArcMove {
                plane: ArcPlane::XY,
                dir: ArcDir::CCW,
                end_wcs: DVec3::new(6.0, 6.0, -5.0),
                center_offset_ijk: DVec3::new(0.0, 6.0, 0.0),
                radius_mm: 6.0,
                turns: 0,
                feed_mm_min: 1_000.0,
                role: MotionRole::Cut,
            },
            // Case 2: Tight concave arc R_arc = 3.5 mm -> (3.5 - 3.0) / 3.5 = 0.1428 < 0.35 -> clamped to 0.35 -> F = 350.0
            ClEvent::ArcMove {
                plane: ArcPlane::XY,
                dir: ArcDir::CCW,
                end_wcs: DVec3::new(9.5, 9.5, -5.0),
                center_offset_ijk: DVec3::new(0.0, 3.5, 0.0),
                radius_mm: 3.5,
                turns: 0,
                feed_mm_min: 1_000.0,
                role: MotionRole::FinishCut,
            },
            // Case 3: Gentle wide arc R_arc = 15.0 mm (>= 4 * tool_r = 12.0) -> unscaled F = 1000.0
            ClEvent::ArcMove {
                plane: ArcPlane::XY,
                dir: ArcDir::CCW,
                end_wcs: DVec3::new(24.5, 24.5, -5.0),
                center_offset_ijk: DVec3::new(0.0, 15.0, 0.0),
                radius_mm: 15.0,
                turns: 0,
                feed_mm_min: 1_000.0,
                role: MotionRole::Cut,
            },
            // Case 4: Helical Ramp arc must NEVER be modified!
            ClEvent::ArcMove {
                plane: ArcPlane::XY,
                dir: ArcDir::CCW,
                end_wcs: DVec3::new(20.0, 20.0, -6.0),
                center_offset_ijk: DVec3::new(0.0, 4.0, 0.0),
                radius_mm: 4.0,
                turns: 0,
                feed_mm_min: 500.0,
                role: MotionRole::Ramp,
            },
        ];

        optimize_toolpath_feedrates(&mut events, &cfg, &tool, &cutting, OperationKind::Pocket2D);

        match events[1] {
            ClEvent::LinearMove { feed_mm_min, .. } => assert!(approx_eq(feed_mm_min, 330.0, 1e-6)),
            _ => panic!("expected plunge unchanged"),
        }
        match events[2] {
            ClEvent::ArcMove { feed_mm_min, .. } => assert!(approx_eq(feed_mm_min, 500.0, 1e-6)),
            _ => panic!("expected R=6 arc scaled to 500"),
        }
        match events[3] {
            ClEvent::ArcMove { feed_mm_min, .. } => assert!(approx_eq(feed_mm_min, 350.0, 1e-6)),
            _ => panic!("expected R=3.5 arc clamped to 350"),
        }
        match events[4] {
            ClEvent::ArcMove { feed_mm_min, .. } => {
                assert!(approx_eq(feed_mm_min, 1_000.0, 1e-6));
            }
            _ => panic!("expected wide R=15 arc unchanged"),
        }
        match events[5] {
            ClEvent::ArcMove { feed_mm_min, .. } => assert!(approx_eq(feed_mm_min, 500.0, 1e-6)),
            _ => panic!("expected Ramp arc unchanged"),
        }
    }

    #[test]
    fn sharp_corner_lookahead_splits_segments_preserves_exact_geometry_and_decelerates_monotonically()
     {
        let tool = test_flat_endmill_6mm(); // tool_d = 6.0 mm
        let cutting = test_cutting();
        let cfg = FeedOptimizationConfig {
            enabled: true,
            corner_slowdown: true,
            min_feed_fraction: 0.35,
            corner_angle_threshold_deg: 25.0,
            slowdown_distance: ParamLength::from_mm(3.0),
            internal_arc_scaling: true,
            chip_thinning_boost: false,
            max_thinning_multiplier: 1.5,
        };

        let p0 = DVec3::new(0.0, 0.0, -5.0);
        let p_corner = DVec3::new(20.0, 0.0, -5.0);
        let p_end = DVec3::new(20.0, 20.0, -5.0);

        let mut events = vec![
            ClEvent::RapidMove {
                end_wcs: DVec3::new(0.0, 0.0, 5.0),
            },
            ClEvent::LinearMove {
                end_wcs: p0,
                feed_mm_min: 330.0,
                role: MotionRole::Plunge,
            },
            ClEvent::LinearMove {
                end_wcs: p_corner,
                feed_mm_min: 1_000.0,
                role: MotionRole::Cut,
            },
            ClEvent::LinearMove {
                end_wcs: p_end,
                feed_mm_min: 1_000.0,
                role: MotionRole::Cut,
            },
        ];

        optimize_toolpath_feedrates(&mut events, &cfg, &tool, &cutting, OperationKind::Pocket2D);

        let cut_moves: Vec<(DVec3, f64)> = events
            .iter()
            .filter_map(|ev| match *ev {
                ClEvent::LinearMove {
                    end_wcs,
                    feed_mm_min,
                    role: MotionRole::Cut,
                } => Some((end_wcs, feed_mm_min)),
                _ => None,
            })
            .collect();

        // Must split both approach leg and exit leg into multiple sub-segments
        assert!(
            cut_moves.len() >= 6,
            "expected split decel + recovery segments around 90° corner, got {}",
            cut_moves.len()
        );

        // Locate exact apex vertex (20, 0, -5)
        let apex_idx = cut_moves
            .iter()
            .position(|(pt, _)| pt.distance(p_corner) <= 1e-12)
            .expect("exact corner apex endpoint must be preserved");
        assert!(
            apex_idx >= 2,
            "approach leg must be split into cruise + ramp"
        );
        assert!(
            cut_moves.last().unwrap().0.distance(p_end) <= 1e-12,
            "exact final endpoint must be preserved"
        );

        // Approach leg geometry stays strictly on Y=0, Z=-5 with monotonic X and monotonic non-increasing feedrate
        let mut prev_x = 0.0;
        let mut prev_f = 1_000.0;
        for &(pt, f) in &cut_moves[..=apex_idx] {
            assert!(pt.y.abs() <= 1e-12, "approach deviated from Y=0: {pt:?}");
            assert!((pt.z - (-5.0)).abs() <= 1e-12);
            assert!(pt.x > prev_x + 1e-6, "non-increasing X on approach");
            assert!(
                f <= prev_f + 1e-9,
                "feedrate must decelerate monotonically into corner: {f} > {prev_f}"
            );
            prev_x = pt.x;
            prev_f = f;
        }
        assert!(
            approx_eq(cut_moves[apex_idx].1, 350.0, 1e-6),
            "apex approach feed must reach min_feed_fraction * F_nom = 350, got {}",
            cut_moves[apex_idx].1
        );

        // Exit leg stays strictly on X=20, Z=-5 with monotonic Y and monotonic non-decreasing feedrate recovery
        let mut prev_y = 0.0;
        let mut prev_exit_f = 350.0;
        for &(pt, f) in &cut_moves[(apex_idx + 1)..] {
            assert!(
                (pt.x - 20.0).abs() <= 1e-12,
                "exit deviated from X=20: {pt:?}"
            );
            assert!((pt.z - (-5.0)).abs() <= 1e-12);
            assert!(pt.y > prev_y + 1e-6, "non-increasing Y on exit");
            assert!(
                f >= prev_exit_f - 1e-9,
                "feedrate must recover monotonically after corner: {f} < {prev_exit_f}"
            );
            prev_y = pt.y;
            prev_exit_f = f;
        }
        assert!(
            approx_eq(cut_moves.last().unwrap().1, 1_000.0, 1e-6),
            "final straight exit must recover to 1000 mm/min"
        );

        // Rebuilt segments must have identical total path length = 40.0 mm (plus plunge)
        let segs = rebuild_segments_from_events(&events);
        let cut_len: f64 = segs
            .iter()
            .filter(|s| {
                matches!(
                    s.motion,
                    crate::toolpath::SegmentMotion3D::Linear {
                        role: MotionRole::Cut,
                        ..
                    }
                )
            })
            .map(ToolpathSegment3D::length_mm)
            .sum();
        assert!(approx_eq(cut_len, 40.0, 1e-9));
    }

    #[test]
    fn disabled_feed_optimization_leaves_cl_events_byte_identical() {
        let tool = test_flat_endmill_6mm();
        let cutting = test_cutting();
        let cfg = FeedOptimizationConfig::default();
        assert!(!cfg.enabled);

        let orig = vec![
            ClEvent::RapidMove {
                end_wcs: DVec3::new(0.0, 0.0, 5.0),
            },
            ClEvent::LinearMove {
                end_wcs: DVec3::new(0.0, 0.0, -5.0),
                feed_mm_min: 330.0,
                role: MotionRole::Plunge,
            },
            ClEvent::LinearMove {
                end_wcs: DVec3::new(20.0, 0.0, -5.0),
                feed_mm_min: 1_000.0,
                role: MotionRole::Cut,
            },
            ClEvent::LinearMove {
                end_wcs: DVec3::new(20.0, 20.0, -5.0),
                feed_mm_min: 1_000.0,
                role: MotionRole::Cut,
            },
            ClEvent::ArcMove {
                plane: ArcPlane::XY,
                dir: ArcDir::CCW,
                end_wcs: DVec3::new(15.0, 25.0, -5.0),
                center_offset_ijk: DVec3::new(-5.0, 0.0, 0.0),
                radius_mm: 5.0,
                turns: 0,
                feed_mm_min: 1_000.0,
                role: MotionRole::FinishCut,
            },
        ];

        let mut events = orig.clone();
        optimize_toolpath_feedrates(&mut events, &cfg, &tool, &cutting, OperationKind::Pocket2D);
        assert_eq!(events, orig);

        // Also verify DrillBore and ThreadMill remain untouched even when enabled
        let mut enabled_cfg = cfg;
        enabled_cfg.enabled = true;
        for kind in [OperationKind::DrillBore, OperationKind::ThreadMill] {
            let mut ev_copy = orig.clone();
            optimize_toolpath_feedrates(&mut ev_copy, &enabled_cfg, &tool, &cutting, kind);
            assert_eq!(ev_copy, orig);
        }
    }

    #[test]
    fn feed_optimization_config_roundtrips_and_omits_default_in_chip_toml() {
        use serde::{Serialize, ser};

        // Lightweight serde field-capture serializer proving `skip_serializing_if` behavior
        // identically to `toml::to_string` without requiring external dev-dependencies.
        #[derive(Default)]
        struct FieldRecorder {
            fields: Vec<String>,
        }

        #[derive(Debug)]
        struct DummyErr(String);
        impl std::fmt::Display for DummyErr {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                f.write_str(&self.0)
            }
        }
        impl std::error::Error for DummyErr {}
        impl ser::Error for DummyErr {
            fn custom<T: std::fmt::Display>(msg: T) -> Self {
                Self(msg.to_string())
            }
        }

        struct Sink;
        impl ser::SerializeSeq for Sink {
            type Ok = ();
            type Error = DummyErr;
            fn serialize_element<T: ?Sized + Serialize>(&mut self, _: &T) -> Result<(), DummyErr> {
                Ok(())
            }
            fn end(self) -> Result<(), DummyErr> {
                Ok(())
            }
        }
        impl ser::SerializeTuple for Sink {
            type Ok = ();
            type Error = DummyErr;
            fn serialize_element<T: ?Sized + Serialize>(&mut self, _: &T) -> Result<(), DummyErr> {
                Ok(())
            }
            fn end(self) -> Result<(), DummyErr> {
                Ok(())
            }
        }
        impl ser::SerializeTupleStruct for Sink {
            type Ok = ();
            type Error = DummyErr;
            fn serialize_field<T: ?Sized + Serialize>(&mut self, _: &T) -> Result<(), DummyErr> {
                Ok(())
            }
            fn end(self) -> Result<(), DummyErr> {
                Ok(())
            }
        }
        impl ser::SerializeTupleVariant for Sink {
            type Ok = ();
            type Error = DummyErr;
            fn serialize_field<T: ?Sized + Serialize>(&mut self, _: &T) -> Result<(), DummyErr> {
                Ok(())
            }
            fn end(self) -> Result<(), DummyErr> {
                Ok(())
            }
        }
        impl ser::SerializeMap for Sink {
            type Ok = ();
            type Error = DummyErr;
            fn serialize_key<T: ?Sized + Serialize>(&mut self, _: &T) -> Result<(), DummyErr> {
                Ok(())
            }
            fn serialize_value<T: ?Sized + Serialize>(&mut self, _: &T) -> Result<(), DummyErr> {
                Ok(())
            }
            fn end(self) -> Result<(), DummyErr> {
                Ok(())
            }
        }
        impl ser::SerializeStructVariant for Sink {
            type Ok = ();
            type Error = DummyErr;
            fn serialize_field<T: ?Sized + Serialize>(
                &mut self,
                _: &'static str,
                _: &T,
            ) -> Result<(), DummyErr> {
                Ok(())
            }
            fn end(self) -> Result<(), DummyErr> {
                Ok(())
            }
        }

        struct StructRec<'a> {
            rec: &'a mut FieldRecorder,
            depth: usize,
        }
        impl ser::SerializeStruct for StructRec<'_> {
            type Ok = ();
            type Error = DummyErr;
            fn serialize_field<T: ?Sized + Serialize>(
                &mut self,
                key: &'static str,
                value: &T,
            ) -> Result<(), DummyErr> {
                if self.depth == 0 {
                    self.rec.fields.push(key.to_string());
                }
                let mut child = FieldRecorder::default();
                let _ = value.serialize(&mut child);
                Ok(())
            }
            fn end(self) -> Result<(), DummyErr> {
                Ok(())
            }
        }

        impl<'a> ser::Serializer for &'a mut FieldRecorder {
            type Ok = ();
            type Error = DummyErr;
            type SerializeSeq = Sink;
            type SerializeTuple = Sink;
            type SerializeTupleStruct = Sink;
            type SerializeTupleVariant = Sink;
            type SerializeMap = Sink;
            type SerializeStruct = StructRec<'a>;
            type SerializeStructVariant = Sink;

            fn serialize_bool(self, _: bool) -> Result<(), DummyErr> {
                Ok(())
            }
            fn serialize_i8(self, _: i8) -> Result<(), DummyErr> {
                Ok(())
            }
            fn serialize_i16(self, _: i16) -> Result<(), DummyErr> {
                Ok(())
            }
            fn serialize_i32(self, _: i32) -> Result<(), DummyErr> {
                Ok(())
            }
            fn serialize_i64(self, _: i64) -> Result<(), DummyErr> {
                Ok(())
            }
            fn serialize_u8(self, _: u8) -> Result<(), DummyErr> {
                Ok(())
            }
            fn serialize_u16(self, _: u16) -> Result<(), DummyErr> {
                Ok(())
            }
            fn serialize_u32(self, _: u32) -> Result<(), DummyErr> {
                Ok(())
            }
            fn serialize_u64(self, _: u64) -> Result<(), DummyErr> {
                Ok(())
            }
            fn serialize_f32(self, _: f32) -> Result<(), DummyErr> {
                Ok(())
            }
            fn serialize_f64(self, _: f64) -> Result<(), DummyErr> {
                Ok(())
            }
            fn serialize_char(self, _: char) -> Result<(), DummyErr> {
                Ok(())
            }
            fn serialize_str(self, _: &str) -> Result<(), DummyErr> {
                Ok(())
            }
            fn serialize_bytes(self, _: &[u8]) -> Result<(), DummyErr> {
                Ok(())
            }
            fn serialize_none(self) -> Result<(), DummyErr> {
                Ok(())
            }
            fn serialize_some<T: ?Sized + Serialize>(self, v: &T) -> Result<(), DummyErr> {
                v.serialize(self)
            }
            fn serialize_unit(self) -> Result<(), DummyErr> {
                Ok(())
            }
            fn serialize_unit_struct(self, _: &'static str) -> Result<(), DummyErr> {
                Ok(())
            }
            fn serialize_unit_variant(
                self,
                _: &'static str,
                _: u32,
                _: &'static str,
            ) -> Result<(), DummyErr> {
                Ok(())
            }
            fn serialize_newtype_struct<T: ?Sized + Serialize>(
                self,
                _: &'static str,
                v: &T,
            ) -> Result<(), DummyErr> {
                v.serialize(self)
            }
            fn serialize_newtype_variant<T: ?Sized + Serialize>(
                self,
                _: &'static str,
                _: u32,
                _: &'static str,
                v: &T,
            ) -> Result<(), DummyErr> {
                v.serialize(self)
            }
            fn serialize_seq(self, _: Option<usize>) -> Result<Sink, DummyErr> {
                Ok(Sink)
            }
            fn serialize_tuple(self, _: usize) -> Result<Sink, DummyErr> {
                Ok(Sink)
            }
            fn serialize_tuple_struct(self, _: &'static str, _: usize) -> Result<Sink, DummyErr> {
                Ok(Sink)
            }
            fn serialize_tuple_variant(
                self,
                _: &'static str,
                _: u32,
                _: &'static str,
                _: usize,
            ) -> Result<Sink, DummyErr> {
                Ok(Sink)
            }
            fn serialize_map(self, _: Option<usize>) -> Result<Sink, DummyErr> {
                Ok(Sink)
            }
            fn serialize_struct(
                self,
                _: &'static str,
                _: usize,
            ) -> Result<StructRec<'a>, DummyErr> {
                Ok(StructRec {
                    rec: self,
                    depth: 0,
                })
            }
            fn serialize_struct_variant(
                self,
                _: &'static str,
                _: u32,
                _: &'static str,
                _: usize,
            ) -> Result<Sink, DummyErr> {
                Ok(Sink)
            }
        }

        // 1. Default PassesParams must omit `feed_optimization` completely when serialized
        let default_passes = PassesParams::default();
        let mut rec_default = FieldRecorder::default();
        default_passes.serialize(&mut rec_default).unwrap();
        assert!(
            !rec_default.fields.iter().any(|f| f == "feed_optimization"),
            "default FeedOptimizationConfig must be omitted when serializing: {:?}",
            rec_default.fields
        );

        // 2. Reevaluating default PassesParams with a larger tool must STILL omit `feed_optimization`
        let mut reeval_passes = PassesParams::default();
        let mut ctx = chip_math::EvalContext::new(DisplayUnit::Millimeter);
        ctx.set_var("tool_d", 10.0);
        reeval_passes.reevaluate(&ctx);
        assert!(reeval_passes.feed_optimization.is_default());
        assert!(approx_eq(
            reeval_passes.feed_optimization.slowdown_distance.value_mm,
            5.0,
            1e-9
        ));
        let mut rec_reeval = FieldRecorder::default();
        reeval_passes.serialize(&mut rec_reeval).unwrap();
        assert!(!rec_reeval.fields.iter().any(|f| f == "feed_optimization"));

        // 3. Enabled custom FeedOptimizationConfig emits `feed_optimization` and all config fields
        let mut op = Operation::new("op_1", "Optimized Pocket", OperationKind::Pocket2D);
        op.passes.feed_optimization = FeedOptimizationConfig {
            enabled: true,
            corner_slowdown: true,
            min_feed_fraction: 0.42,
            corner_angle_threshold_deg: 30.0,
            slowdown_distance: ParamLength {
                expr: Some("tool_d * 0.75".to_string()),
                value_mm: 4.5,
            },
            internal_arc_scaling: true,
            chip_thinning_boost: true,
            max_thinning_multiplier: 1.8,
        };
        assert!(!op.passes.feed_optimization.is_default());
        let mut rec_enabled = FieldRecorder::default();
        op.passes.serialize(&mut rec_enabled).unwrap();
        assert!(rec_enabled.fields.iter().any(|f| f == "feed_optimization"));

        let mut rec_cfg = FieldRecorder::default();
        op.passes.feed_optimization.serialize(&mut rec_cfg).unwrap();
        for expected in [
            "enabled",
            "corner_slowdown",
            "min_feed_fraction",
            "corner_angle_threshold_deg",
            "slowdown_distance",
            "internal_arc_scaling",
            "chip_thinning_boost",
            "max_thinning_multiplier",
        ] {
            assert!(
                rec_cfg.fields.iter().any(|f| f == expected),
                "missing field '{expected}' in {:?}",
                rec_cfg.fields
            );
        }

        // 4. Deserializing an empty map (legacy schema_version 1/2/3 .chip.toml) recovers exact defaults
        let empty_map: serde::de::value::MapDeserializer<
            '_,
            std::iter::Empty<(&str, &str)>,
            serde::de::value::Error,
        > = serde::de::value::MapDeserializer::new(std::iter::empty());
        let legacy_passes: PassesParams = serde::Deserialize::deserialize(empty_map).unwrap();
        assert_eq!(
            legacy_passes.feed_optimization,
            FeedOptimizationConfig::default()
        );
        assert!(!legacy_passes.feed_optimization.enabled);
    }
}
