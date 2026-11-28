use chip_math::DVec3;
use chip_tools::Tool;

use crate::{
    cl_data::{ClEvent, MotionRole},
    feed_opt::rebuild_segments_from_events,
    setup::ResolvedSetup,
    toolpath::GeneratedToolpath,
    workholding::ResolvedFixtureSolid,
};

use super::collision::{check_chord_fixture_collision, min_safe_z_over_fixtures_with_profile};
use super::{FixtureRoutingSummary, SAFE_RAPID_EXTRA_LIFT_MM, ToolAssemblyProfile};

/// Automatically re-routes colliding non-cutting transitions (`RapidMove`, `RotaryRapidMove`,
/// `MotionRole::LinkTransition`) cleanly up and over enabled 3D workholding fixtures while
/// **NEVER** altering cutting moves (`Cut`, `FinishCut`, `Plunge`, `Ramp`, `LeadIn`, `LeadOut`,
/// `TabRamp`).
///
/// Exact no-op when `!auto_avoid_fixtures_on_rapids` or `resolved_fixtures.is_empty()`.
pub fn route_toolpath_around_fixtures(
    toolpath: &mut GeneratedToolpath,
    tool: &Tool,
    setup: &ResolvedSetup,
    resolved_fixtures: &[ResolvedFixtureSolid],
    auto_avoid_fixtures_on_rapids: bool,
    machine_z_travel_mm: Option<f64>,
) -> FixtureRoutingSummary {
    if !auto_avoid_fixtures_on_rapids || resolved_fixtures.is_empty() {
        return FixtureRoutingSummary::default();
    }

    let assembly = ToolAssemblyProfile::from_tool(tool);
    let stock_top_z = setup.stock.aabb_wcs().max.z;
    let stock_min_z = setup.stock.aabb_wcs().min.z;

    let mut out: Vec<ClEvent> = Vec::with_capacity(toolpath.events.len() + 16);
    let mut cur_pos: Option<DVec3> = None;
    let mut cur_rotary_deg = 0.0_f64;
    let mut lifts_inserted = 0usize;
    let mut max_lifted_z = f64::NEG_INFINITY;

    for ev in &toolpath.events {
        match ev {
            ClEvent::RapidMove { end_wcs } => {
                if let Some(p0) = cur_pos
                    && (end_wcs.x - p0.x).hypot(end_wcs.y - p0.y) > 1e-5
                    && check_chord_fixture_collision(
                        p0,
                        *end_wcs,
                        &assembly,
                        resolved_fixtures,
                        true,
                        true,
                    )
                    .is_some()
                {
                    let req_z = min_safe_z_over_fixtures_with_profile(
                        p0,
                        *end_wcs,
                        &assembly,
                        resolved_fixtures,
                    )
                    .unwrap_or(setup.clearance_z_wcs);
                    let z_safe = setup
                        .clearance_z_wcs
                        .max(req_z + SAFE_RAPID_EXTRA_LIFT_MM)
                        .max(p0.z)
                        .max(end_wcs.z);

                    push_safe_rapid_bridge_3axis(
                        &mut out,
                        p0,
                        *end_wcs,
                        z_safe,
                        None,
                        stock_top_z,
                        setup.retract_z_wcs,
                    );
                    lifts_inserted += 1;
                    max_lifted_z = max_lifted_z.max(z_safe);
                    cur_pos = Some(*end_wcs);
                    continue;
                }
                cur_pos = Some(*end_wcs);
                out.push(ev.clone());
            }
            ClEvent::LinearMove {
                end_wcs,
                feed_mm_min,
                role,
            } if *role == MotionRole::LinkTransition => {
                if let Some(p0) = cur_pos
                    && (end_wcs.x - p0.x).hypot(end_wcs.y - p0.y) > 1e-5
                    && check_chord_fixture_collision(
                        p0,
                        *end_wcs,
                        &assembly,
                        resolved_fixtures,
                        true,
                        true,
                    )
                    .is_some()
                {
                    let req_z = min_safe_z_over_fixtures_with_profile(
                        p0,
                        *end_wcs,
                        &assembly,
                        resolved_fixtures,
                    )
                    .unwrap_or(setup.clearance_z_wcs);
                    let z_safe = setup
                        .clearance_z_wcs
                        .max(req_z + SAFE_RAPID_EXTRA_LIFT_MM)
                        .max(p0.z)
                        .max(end_wcs.z);

                    push_safe_rapid_bridge_3axis(
                        &mut out,
                        p0,
                        *end_wcs,
                        z_safe,
                        Some(*feed_mm_min),
                        stock_top_z,
                        setup.retract_z_wcs,
                    );
                    lifts_inserted += 1;
                    max_lifted_z = max_lifted_z.max(z_safe);
                    cur_pos = Some(*end_wcs);
                    continue;
                }
                cur_pos = Some(*end_wcs);
                out.push(ev.clone());
            }
            ClEvent::RotaryRapidMove {
                end_wcs,
                rotary_deg,
            } => {
                if let Some(p0) = cur_pos
                    && (end_wcs.x - p0.x).hypot(end_wcs.y - p0.y) > 1e-5
                    && check_chord_fixture_collision(
                        p0,
                        *end_wcs,
                        &assembly,
                        resolved_fixtures,
                        true,
                        true,
                    )
                    .is_some()
                {
                    let req_z = min_safe_z_over_fixtures_with_profile(
                        p0,
                        *end_wcs,
                        &assembly,
                        resolved_fixtures,
                    )
                    .unwrap_or(setup.clearance_z_wcs);
                    let rot_clear_z = setup.rotary.map_or(f64::NEG_INFINITY, |r| {
                        r.center_wcs.z + r.clearance_radius_mm
                    });
                    let z_safe = setup
                        .clearance_z_wcs
                        .max(rot_clear_z)
                        .max(req_z + SAFE_RAPID_EXTRA_LIFT_MM)
                        .max(p0.z)
                        .max(end_wcs.z);

                    if p0.z < z_safe - 1e-4 {
                        out.push(ClEvent::RotaryRapidMove {
                            end_wcs: DVec3::new(p0.x, p0.y, z_safe),
                            rotary_deg: cur_rotary_deg,
                        });
                    }
                    out.push(ClEvent::RotaryRapidMove {
                        end_wcs: DVec3::new(end_wcs.x, end_wcs.y, z_safe),
                        rotary_deg: *rotary_deg,
                    });
                    if end_wcs.z < z_safe - 1e-4 {
                        out.push(ClEvent::RotaryRapidMove {
                            end_wcs: *end_wcs,
                            rotary_deg: *rotary_deg,
                        });
                    }
                    lifts_inserted += 1;
                    max_lifted_z = max_lifted_z.max(z_safe);
                    cur_pos = Some(*end_wcs);
                    cur_rotary_deg = *rotary_deg;
                    continue;
                }
                cur_pos = Some(*end_wcs);
                cur_rotary_deg = *rotary_deg;
                out.push(ev.clone());
            }
            ClEvent::RotaryLinearMove {
                end_wcs,
                rotary_deg,
                feed_mm_min,
                effective_radius_mm,
                role,
            } if *role == MotionRole::LinkTransition => {
                if let Some(p0) = cur_pos
                    && (end_wcs.x - p0.x).hypot(end_wcs.y - p0.y) > 1e-5
                    && check_chord_fixture_collision(
                        p0,
                        *end_wcs,
                        &assembly,
                        resolved_fixtures,
                        true,
                        true,
                    )
                    .is_some()
                {
                    let req_z = min_safe_z_over_fixtures_with_profile(
                        p0,
                        *end_wcs,
                        &assembly,
                        resolved_fixtures,
                    )
                    .unwrap_or(setup.clearance_z_wcs);
                    let rot_clear_z = setup.rotary.map_or(f64::NEG_INFINITY, |r| {
                        r.center_wcs.z + r.clearance_radius_mm
                    });
                    let z_safe = setup
                        .clearance_z_wcs
                        .max(rot_clear_z)
                        .max(req_z + SAFE_RAPID_EXTRA_LIFT_MM)
                        .max(p0.z)
                        .max(end_wcs.z);

                    if p0.z < z_safe - 1e-4 {
                        out.push(ClEvent::RotaryRapidMove {
                            end_wcs: DVec3::new(p0.x, p0.y, z_safe),
                            rotary_deg: cur_rotary_deg,
                        });
                    }
                    out.push(ClEvent::RotaryRapidMove {
                        end_wcs: DVec3::new(end_wcs.x, end_wcs.y, z_safe),
                        rotary_deg: *rotary_deg,
                    });
                    if end_wcs.z < z_safe - 1e-4 {
                        out.push(ClEvent::RotaryLinearMove {
                            end_wcs: *end_wcs,
                            rotary_deg: *rotary_deg,
                            feed_mm_min: *feed_mm_min,
                            effective_radius_mm: *effective_radius_mm,
                            role: MotionRole::LinkTransition,
                        });
                    }
                    lifts_inserted += 1;
                    max_lifted_z = max_lifted_z.max(z_safe);
                    cur_pos = Some(*end_wcs);
                    cur_rotary_deg = *rotary_deg;
                    continue;
                }
                cur_pos = Some(*end_wcs);
                cur_rotary_deg = *rotary_deg;
                out.push(ev.clone());
            }
            ClEvent::RotaryIndex { angle_deg, .. } => {
                if let Some(p0) = cur_pos
                    && let Some(rot) = setup.rotary
                {
                    let rot_clear_z = rot.center_wcs.z + rot.clearance_radius_mm;
                    let needs_rot_lift = p0.z < rot_clear_z - 1e-4
                        && check_chord_fixture_collision(
                            p0,
                            p0,
                            &assembly,
                            resolved_fixtures,
                            true,
                            true,
                        )
                        .is_some();
                    if needs_rot_lift {
                        let req_z = min_safe_z_over_fixtures_with_profile(
                            p0,
                            p0,
                            &assembly,
                            resolved_fixtures,
                        )
                        .unwrap_or(rot_clear_z);
                        let z_safe = setup
                            .clearance_z_wcs
                            .max(rot_clear_z)
                            .max(req_z + SAFE_RAPID_EXTRA_LIFT_MM);
                        out.push(ClEvent::RapidMove {
                            end_wcs: DVec3::new(p0.x, p0.y, z_safe),
                        });
                        cur_pos = Some(DVec3::new(p0.x, p0.y, z_safe));
                        lifts_inserted += 1;
                        max_lifted_z = max_lifted_z.max(z_safe);
                    }
                }
                cur_rotary_deg = *angle_deg;
                out.push(ev.clone());
            }
            // Cutting moves (`Cut`, `FinishCut`, `Plunge`, `Ramp`, `LeadIn`, `LeadOut`, `TabRamp`)
            // are NEVER altered or skipped — only track current tool tip position!
            ClEvent::LinearMove { end_wcs, .. } | ClEvent::ArcMove { end_wcs, .. } => {
                cur_pos = Some(*end_wcs);
                out.push(ev.clone());
            }
            ClEvent::RotaryLinearMove {
                end_wcs,
                rotary_deg,
                ..
            } => {
                cur_pos = Some(*end_wcs);
                cur_rotary_deg = *rotary_deg;
                out.push(ev.clone());
            }
            ClEvent::DrillCycle {
                bottom_wcs,
                clear_z_wcs,
                r_plane_z_wcs,
                ..
            } => {
                cur_pos = Some(DVec3::new(
                    bottom_wcs.x,
                    bottom_wcs.y,
                    clear_z_wcs.max(*r_plane_z_wcs),
                ));
                out.push(ev.clone());
            }
            _ => {
                out.push(ev.clone());
            }
        }
    }

    if lifts_inserted == 0 {
        return FixtureRoutingSummary::default();
    }

    toolpath.events = out;
    toolpath.segments_wcs = rebuild_segments_from_events(&toolpath.events);
    toolpath.recompute_stats();
    toolpath.rapid_lifts_inserted = lifts_inserted;
    toolpath.max_lifted_z_wcs = if max_lifted_z.is_finite() {
        max_lifted_z
    } else {
        0.0
    };

    if let Some(z_travel) = machine_z_travel_mm
        && z_travel.is_finite()
        && z_travel > 0.0
        && max_lifted_z.is_finite()
    {
        let min_z = toolpath.stats.bounds_wcs.min.z.min(stock_min_z);
        let z_span = max_lifted_z - min_z;
        if z_span > z_travel + 0.01 {
            toolpath.warnings.push(format!(
                "Safe-rapid lift over fixtures (max Z = {max_lifted_z:+.1} mm, Z span {z_span:.1} mm) exceeds machine Z travel limit ({z_travel:.1} mm); lower fixture height or adjust Setup Z clearance."
            ));
        }
    }

    FixtureRoutingSummary {
        rapid_lifts_inserted: lifts_inserted,
        max_lifted_z_wcs: toolpath.max_lifted_z_wcs,
    }
}

/// Emits a clean 3-move vertical lift -> horizontal bridge -> vertical descent (with safe
/// feed-plane guard when a `LinkTransition` descends below stock top).
fn push_safe_rapid_bridge_3axis(
    out: &mut Vec<ClEvent>,
    p0: DVec3,
    p1: DVec3,
    z_safe: f64,
    link_feed_mm_min: Option<f64>,
    stock_top_z: f64,
    retract_z_wcs: f64,
) {
    // 1. Vertical lift to `(p0.x, p0.y, z_safe)` (coalescing if the immediately preceding move
    //    was already a vertical RapidMove at `(p0.x, p0.y)`!)
    if p0.z < z_safe - 1e-4 {
        let lift_pt = DVec3::new(p0.x, p0.y, z_safe);
        let merged_prev = if let Some(ClEvent::RapidMove { end_wcs }) = out.last_mut()
            && (end_wcs.x - p0.x).abs() <= 1e-5
            && (end_wcs.y - p0.y).abs() <= 1e-5
            && end_wcs.z <= z_safe + 1e-4
        {
            end_wcs.z = z_safe;
            true
        } else {
            false
        };
        if !merged_prev {
            out.push(ClEvent::RapidMove { end_wcs: lift_pt });
        }
    }

    // 2. Horizontal safe clearance bridge to `(p1.x, p1.y, z_safe)`
    out.push(ClEvent::RapidMove {
        end_wcs: DVec3::new(p1.x, p1.y, z_safe),
    });

    // 3. Vertical descent to `p1` (with safe feed-plane guard for sub-stock `LinkTransition`s!)
    if p1.z < z_safe - 1e-4 {
        if let Some(feed) = link_feed_mm_min
            && p1.z < stock_top_z - 1e-4
        {
            let feed_guard_z = retract_z_wcs.max(stock_top_z + 1.0).min(z_safe);
            if feed_guard_z < z_safe - 1e-4 && feed_guard_z > p1.z + 1e-4 {
                out.push(ClEvent::RapidMove {
                    end_wcs: DVec3::new(p1.x, p1.y, feed_guard_z),
                });
            }
            out.push(ClEvent::LinearMove {
                end_wcs: p1,
                feed_mm_min: feed.max(1.0),
                role: MotionRole::LinkTransition,
            });
        } else {
            out.push(ClEvent::RapidMove { end_wcs: p1 });
        }
    }
}

// ---------------------------------------------------------------------------
// Exact 2D / 3D Computational Geometry Primitives
// ---------------------------------------------------------------------------
