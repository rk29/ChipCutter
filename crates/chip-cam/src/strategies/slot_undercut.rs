//! **Slot / Keyway / T-Slot Undercut Milling** (`OperationKind::SlotUndercut`).
//!
//! Enforces strict physical machine safety invariants:
//! - Never vertically plunges or retracts beneath an undercut roof.
//! - Enters vertically strictly in verified open air past an open slot end (`entry_extension`)
//!   or inside a verified pre-cleared entry bore (`bore_dia >= cutter_dia + 0.5 mm`).
//! - Validates `neck_diameter_mm < throat_width_mm`, full lateral-offset neck clearance,
//!   `neck_length_mm >= total_depth`, `cutting_thickness_mm <= slot_height_mm`, and
//!   `cutter_diameter_mm <= slot_width_mm`.
//! - Generates multi-level axial Z passes and progressive lateral roughing, finish, and optional
//!   spring passes, returning laterally to throat centerline and exiting horizontally before `+Z` retract.

use chip_math::{Aabb3, ArcPolyline2D, ArcSegment2D, DVec2, DVec3};
use chip_tools::CutterGeometry;

use crate::{
    cl_data::MotionRole,
    params::{SlotUndercutMode, SlotUndercutParams, UndercutEntryMode},
    selection::ExtractedFeature25D,
    strategies::StrategyExecContext,
    toolpath::{ToolpathBuilder, XyArcMove},
    wcs::CamError,
};

#[derive(Debug, Clone)]
pub(crate) struct ExtractedSlotCenterline {
    pub(crate) polyline: ArcPolyline2D,
    pub(crate) inferred_width_mm: Option<f64>,
    pub(crate) from_brep_face: bool,
    pub(crate) start_is_blind_face_wall: bool,
    pub(crate) end_is_blind_face_wall: bool,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct EntryBoreCandidate {
    pub(crate) center_wcs: DVec2,
    pub(crate) diameter_mm: f64,
}

/// Generates a **Slot / Keyway / T-Slot Undercut** toolpath (`OperationKind::SlotUndercut`).
pub fn generate_slot_undercut(
    ctx: &StrategyExecContext<'_>,
    params: &SlotUndercutParams,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    ctx.check_cancel()?;

    // 1. Validate tool geometry & extract exact cutter dimensions
    let (cutter_d, cutting_thickness, neck_d, neck_len, is_undercut_cutter) =
        match ctx.tool.geometry {
            CutterGeometry::SlotMill {
                diameter_mm,
                cutting_thickness_mm,
                neck_diameter_mm,
                neck_length_mm,
                ..
            } => (
                diameter_mm,
                cutting_thickness_mm,
                neck_diameter_mm,
                neck_length_mm,
                true,
            ),
            CutterGeometry::FlatEndmill {
                diameter_mm,
                flute_length_mm,
                ..
            } => (
                diameter_mm,
                flute_length_mm,
                diameter_mm,
                flute_length_mm,
                false,
            ),
            _ => {
                return Err(CamError::IncompatibleTool {
                    op_name: ctx.op.name.clone(),
                    op_kind: ctx.op.kind.label(),
                    tool_name: ctx.tool.name.clone(),
                    tool_kind: ctx.tool.geometry.kind_label(),
                });
            }
        };
    let cutter_r = 0.5 * cutter_d;

    // 2. Extract centerlines and optional pre-cleared entry bores from selections
    let (mut centerlines, entry_bores) =
        extract_slot_centerlines_and_bores(&ctx.selection.features);
    if centerlines.is_empty() {
        return Err(CamError::EmptySelection {
            op_name: ctx.op.name.clone(),
            op_kind: ctx.op.kind.label(),
        });
    }

    // 3. Validate numeric parameters (auto-infer slot width from modeled B-Rep floor face when preset is Custom & default)
    let mut slot_width = params.slot_width.value_mm;
    if params.preset == crate::params::SlotUndercutPreset::Custom
        && params.slot_width.expr.is_none()
        && (slot_width - 14.5).abs() < 1e-6
        && let Some(w) = centerlines
            .iter()
            .find(|c| c.from_brep_face)
            .and_then(|c| c.inferred_width_mm)
        && w > cutter_d - 1e-4
    {
        slot_width = w;
    }
    let throat_width = params.throat_width.value_mm;
    let slot_height = params.slot_height.value_mm;
    let radial_stepover = params.radial_stepover.value_mm;
    let axial_stepdown = params.axial_stepdown.value_mm;
    let finish_allowance = params.finish_allowance.value_mm;
    let entry_extension = params.entry_extension.value_mm;

    if !slot_width.is_finite() || slot_width <= 1e-4 {
        return Err(CamError::InvalidParameter {
            param: "slot_width",
            value: slot_width,
            reason: "must be finite and > 0",
        });
    }
    if !throat_width.is_finite() || throat_width <= 1e-4 {
        return Err(CamError::InvalidParameter {
            param: "throat_width",
            value: throat_width,
            reason: "must be finite and > 0",
        });
    }
    if !slot_height.is_finite() || slot_height <= 1e-4 {
        return Err(CamError::InvalidParameter {
            param: "slot_height",
            value: slot_height,
            reason: "must be finite and > 0",
        });
    }
    if !radial_stepover.is_finite() || radial_stepover <= 1e-4 {
        return Err(CamError::InvalidParameter {
            param: "radial_stepover",
            value: radial_stepover,
            reason: "must be finite and > 0",
        });
    }
    if !axial_stepdown.is_finite() || axial_stepdown <= 1e-4 {
        return Err(CamError::InvalidParameter {
            param: "axial_stepdown",
            value: axial_stepdown,
            reason: "must be finite and > 0",
        });
    }
    if !finish_allowance.is_finite() || finish_allowance < 0.0 {
        return Err(CamError::InvalidParameter {
            param: "finish_allowance",
            value: finish_allowance,
            reason: "must be finite and >= 0",
        });
    }
    if !entry_extension.is_finite() || entry_extension < 0.0 {
        return Err(CamError::InvalidParameter {
            param: "entry_extension",
            value: entry_extension,
            reason: "must be finite and >= 0",
        });
    }

    let radial_stock = ctx.passes.stock_to_leave_radial.value_mm.max(0.0);
    let axial_stock = ctx.passes.stock_to_leave_axial.value_mm.max(0.0);
    let effective_slot_width = (slot_width - 2.0 * radial_stock).max(0.0);

    // 4. Hard physical clearance & reach safety checks
    let requires_undercut_roof_guards =
        params.mode == SlotUndercutMode::TSlot || throat_width < slot_width - 1e-4;

    if !is_undercut_cutter && requires_undercut_roof_guards {
        return Err(CamError::InvalidUndercutGeometry {
            reason: format!(
                "T-Slot / undercut groove (slot width {slot_width:.3} mm > throat {throat_width:.3} mm) requires a relieved-neck Slot / T-Slot Mill cutter; '{}' is a {}",
                ctx.tool.name,
                ctx.tool.geometry.kind_label()
            ),
        });
    }

    if cutter_d > effective_slot_width + 1e-4 {
        return Err(CamError::InvalidUndercutGeometry {
            reason: format!(
                "cutter diameter Ø{cutter_d:.3} mm exceeds target slot width {effective_slot_width:.3} mm"
            ),
        });
    }

    if is_undercut_cutter && cutting_thickness > slot_height + 1e-4 {
        return Err(CamError::InvalidUndercutGeometry {
            reason: format!(
                "slot mill cutting thickness ({cutting_thickness:.3} mm) exceeds slot height ({slot_height:.3} mm)"
            ),
        });
    }

    let z_bottom = ctx.heights.bottom_z_wcs + axial_stock;
    let total_depth = (ctx.heights.top_z_wcs - z_bottom).max(0.0);

    if is_undercut_cutter && neck_len + 1e-4 < total_depth {
        return Err(CamError::InvalidUndercutGeometry {
            reason: format!(
                "slot mill relieved neck length ({neck_len:.3} mm) is shorter than total slot depth ({total_depth:.3} mm); shank would collide with workpiece roof"
            ),
        });
    }

    let max_lat = (0.5 * effective_slot_width - cutter_r).max(0.0);
    if params.mode != SlotUndercutMode::SideGroove && requires_undercut_roof_guards {
        if neck_d >= throat_width - 1e-4 {
            return Err(CamError::InvalidUndercutGeometry {
                reason: format!(
                    "slot mill neck diameter (Ø{neck_d:.3} mm) must be strictly smaller than pre-cleared throat width ({throat_width:.3} mm)"
                ),
            });
        }
        let neck_outer_at_wall = max_lat + 0.5 * neck_d;
        let half_throat = 0.5 * throat_width;
        if neck_outer_at_wall > half_throat + 1e-4 {
            return Err(CamError::InvalidUndercutGeometry {
                reason: format!(
                    "at full lateral offset ({max_lat:.3} mm), relieved neck radius ({:.3} mm) reaches {:.3} mm from centerline, exceeding half-throat ({half_throat:.3} mm)",
                    0.5 * neck_d,
                    neck_outer_at_wall
                ),
            });
        }
    }

    // 5. Compute axial Z levels (tip Z positions spanning `[z_bottom, z_bottom + slot_height - cutting_thickness]`)
    let z_levels = compute_slot_axial_levels(
        z_bottom,
        slot_height,
        cutting_thickness,
        axial_stepdown,
        is_undercut_cutter,
    );

    // 6. Compute progressive lateral offset schedule `(offset_mm, is_finish)`
    let need_center_first =
        max_lat <= 1e-5 || cutter_r > 0.5 * throat_width - 1e-4 || !requires_undercut_roof_guards;
    let lateral_steps = build_progressive_lateral_offsets(
        max_lat,
        (0.5 * throat_width - cutter_r).max(0.0),
        radial_stepover,
        finish_allowance,
        params.spring_pass,
    );

    let stock_aabb = ctx.setup.stock.aabb_wcs();
    let safe_z = ctx.safe_retract_z();
    let feed_cut = ctx.cutting.cutting_feed_mm_min;
    let feed_lead = ctx.cutting.lead_feed_mm_min.min(feed_cut).max(1.0);
    let feed_plunge = ctx.cutting.plunge_feed_mm_min.min(feed_cut).max(1.0);

    // 7. Process each slot centerline with strict safe entry/exit choreography
    for cl in &mut centerlines {
        ctx.check_cancel()?;
        if cl.polyline.segments.is_empty() || cl.polyline.perimeter() <= 1e-4 {
            continue;
        }
        // If extracted from a planar face with blind end walls, inset blind endpoints by cutter_r
        if cl.start_is_blind_face_wall || cl.end_is_blind_face_wall {
            trim_polyline_ends_for_blind_walls(
                &mut cl.polyline,
                if cl.start_is_blind_face_wall {
                    cutter_r
                } else {
                    0.0
                },
                if cl.end_is_blind_face_wall {
                    cutter_r
                } else {
                    0.0
                },
            );
            if cl.polyline.perimeter() <= 1e-4 {
                continue;
            }
        }

        let plan = resolve_safe_slot_entry_plan(
            &cl.polyline,
            params.entry_mode,
            entry_extension,
            cutter_d,
            &stock_aabb,
            &entry_bores,
            requires_undercut_roof_guards || is_undercut_cutter,
        )?;

        emit_undercut_slot_passes(
            builder,
            &plan,
            &z_levels,
            need_center_first,
            &lateral_steps,
            params.climb_direction,
            safe_z,
            feed_plunge,
            feed_lead,
            feed_cut,
        );
    }

    Ok(())
}

/// Safe entry/exit execution plan for a single slot or internal dovetail centerline.
#[derive(Debug, Clone)]
pub(crate) enum SafeSlotEntryPlan {
    /// Enter and exit in open air at `start` (optionally breaking out into open air at `end` when `both_ends_open`).
    OpenEnd {
        path: ArcPolyline2D,
        both_ends_open: bool,
    },
    /// Enter and exit vertically strictly inside a verified pre-cleared entry bore (`bore_center`),
    /// machining 1 or 2 arms outward and returning horizontally to `bore_center` before any `+Z` retract.
    EntryBore {
        bore_center: DVec2,
        arms: Vec<ArcPolyline2D>,
    },
}

/// Resolves and strictly validates a roof-safe entry/exit plan for `centerline`.
///
/// When `strict_undercut_guard` is `true`, vertical plunges/retracts are strictly forbidden
/// beneath uncut stock/roof material: every entry/exit point must clear `stock_aabb` by at least
/// `0.5 * cutter_d` OR lie inside a verified pre-cleared entry bore (`bore_dia >= cutter_d + 0.5 mm`).
pub(crate) fn resolve_safe_slot_entry_plan(
    centerline: &ArcPolyline2D,
    entry_mode: UndercutEntryMode,
    entry_extension: f64,
    cutter_d: f64,
    stock_aabb: &Aabb3,
    entry_bores: &[EntryBoreCandidate],
    strict_undercut_guard: bool,
) -> Result<SafeSlotEntryPlan, CamError> {
    let cutter_r = 0.5 * cutter_d;
    let min_bore_dia = cutter_d + 0.5;
    let Some(p_start) = polyline_start_point(centerline) else {
        return Err(CamError::InvalidUndercutGeometry {
            reason: "slot centerline has no start point".to_string(),
        });
    };
    let Some(p_end) = polyline_end_point(centerline) else {
        return Err(CamError::InvalidUndercutGeometry {
            reason: "slot centerline has no end point".to_string(),
        });
    };
    let t_start = polyline_start_tangent(centerline).unwrap_or(DVec2::X);
    let t_end = polyline_end_tangent(centerline).unwrap_or(DVec2::X);

    match entry_mode {
        UndercutEntryMode::EntryBoreCenter => {
            let bore = find_verified_entry_bore(centerline, entry_bores, min_bore_dia)?;
            let arms = split_centerline_at_bore(centerline, bore.center_wcs);
            if arms.is_empty() {
                return Err(CamError::InvalidUndercutGeometry {
                    reason: "slot centerline is too short around entry bore".to_string(),
                });
            }
            Ok(SafeSlotEntryPlan::EntryBore {
                bore_center: bore.center_wcs,
                arms,
            })
        }
        UndercutEntryMode::BothEndsThrough => {
            let p_start_ext = p_start - t_start * entry_extension;
            let p_end_ext = p_end + t_end * entry_extension;
            let start_ok = !strict_undercut_guard
                || is_xy_safe_vertical_zone(
                    p_start_ext,
                    cutter_r,
                    stock_aabb,
                    entry_bores,
                    min_bore_dia,
                );
            let end_ok = !strict_undercut_guard
                || is_xy_safe_vertical_zone(
                    p_end_ext,
                    cutter_r,
                    stock_aabb,
                    entry_bores,
                    min_bore_dia,
                );
            if !start_ok || !end_ok {
                return Err(CamError::InvalidUndercutGeometry {
                    reason: format!(
                        "BothEndsThrough entry requires both extended slot ends to clear the stock boundary by >= {:.2} mm (cutter radius) or land inside a verified entry bore (>= Ø{min_bore_dia:.2} mm); start_clear={start_ok}, end_clear={end_ok}",
                        cutter_r
                    ),
                });
            }
            let path = extend_open_polyline(centerline, entry_extension, entry_extension);
            Ok(SafeSlotEntryPlan::OpenEnd {
                path,
                both_ends_open: true,
            })
        }
        UndercutEntryMode::OpenEnd => {
            let p_start_ext = p_start - t_start * entry_extension;
            let p_end_ext = p_end + t_end * entry_extension;
            let start_ok = is_xy_safe_vertical_zone(
                p_start_ext,
                cutter_r,
                stock_aabb,
                entry_bores,
                min_bore_dia,
            );
            let end_ok = is_xy_safe_vertical_zone(
                p_end_ext,
                cutter_r,
                stock_aabb,
                entry_bores,
                min_bore_dia,
            );

            if !strict_undercut_guard {
                let path = extend_open_polyline(
                    centerline,
                    entry_extension,
                    if end_ok { entry_extension } else { 0.0 },
                );
                return Ok(SafeSlotEntryPlan::OpenEnd {
                    path,
                    both_ends_open: end_ok,
                });
            }

            if start_ok && end_ok {
                let path = extend_open_polyline(centerline, entry_extension, entry_extension);
                Ok(SafeSlotEntryPlan::OpenEnd {
                    path,
                    both_ends_open: true,
                })
            } else if start_ok {
                let path = extend_open_polyline(centerline, entry_extension, 0.0);
                Ok(SafeSlotEntryPlan::OpenEnd {
                    path,
                    both_ends_open: false,
                })
            } else if end_ok {
                // Reverse so the open end is always the start of `path`
                let rev = centerline.reverse();
                let path = extend_open_polyline(&rev, entry_extension, 0.0);
                Ok(SafeSlotEntryPlan::OpenEnd {
                    path,
                    both_ends_open: false,
                })
            } else {
                Err(CamError::InvalidUndercutGeometry {
                    reason: format!(
                        "blind undercut slot has neither an open end clearing stock by >= {:.2} mm nor a verified pre-cleared entry bore (need bore Ø >= {min_bore_dia:.2} mm); extend the slot/entry_extension past stock or switch to EntryBoreCenter with a pre-cleared bore",
                        cutter_r
                    ),
                })
            }
        }
    }
}

/// Checks whether `(pt.x, pt.y)` is safe for a vertical plunge/retract of a cutter with radius
/// `cutter_r` (strictly outside `stock_aabb` OR inside a verified entry bore `>= min_bore_dia`).
#[inline]
pub(crate) fn is_xy_safe_vertical_zone(
    pt: DVec2,
    cutter_r: f64,
    stock_aabb: &Aabb3,
    entry_bores: &[EntryBoreCandidate],
    min_bore_dia: f64,
) -> bool {
    let outside_stock = !stock_aabb.is_valid()
        || pt.x <= stock_aabb.min.x - cutter_r + 1e-3
        || pt.x >= stock_aabb.max.x + cutter_r - 1e-3
        || pt.y <= stock_aabb.min.y - cutter_r + 1e-3
        || pt.y >= stock_aabb.max.y + cutter_r - 1e-3;
    if outside_stock {
        return true;
    }
    entry_bores.iter().any(|b| {
        b.diameter_mm >= min_bore_dia - 1e-4
            && pt.distance(b.center_wcs) <= (0.5 * b.diameter_mm - cutter_r).max(0.0) + 1e-3
    })
}

fn find_verified_entry_bore(
    centerline: &ArcPolyline2D,
    entry_bores: &[EntryBoreCandidate],
    min_bore_dia: f64,
) -> Result<EntryBoreCandidate, CamError> {
    if entry_bores.is_empty() {
        return Err(CamError::InvalidUndercutGeometry {
            reason: format!(
                "EntryBoreCenter mode requires a selected pre-cleared cylindrical entry bore with diameter >= Ø{min_bore_dia:.2} mm"
            ),
        });
    }
    let mut best_any: Option<(f64, EntryBoreCandidate)> = None;
    let mut best_valid: Option<(f64, EntryBoreCandidate)> = None;
    for &b in entry_bores {
        let dist = min_distance_to_polyline(centerline, b.center_wcs);
        if best_any.is_none_or(|(d, _)| dist < d) {
            best_any = Some((dist, b));
        }
        if b.diameter_mm >= min_bore_dia - 1e-4
            && dist <= 0.5 * b.diameter_mm + 0.5
            && best_valid.is_none_or(|(d, _)| dist < d)
        {
            best_valid = Some((dist, b));
        }
    }
    if let Some((_, bore)) = best_valid {
        return Ok(bore);
    }
    if let Some((dist, bore)) = best_any {
        if bore.diameter_mm < min_bore_dia - 1e-4 {
            return Err(CamError::InvalidUndercutGeometry {
                reason: format!(
                    "pre-cleared entry bore diameter (Ø{:.3} mm) is too small for cutter clearance (minimum required: Ø{min_bore_dia:.3} mm)",
                    bore.diameter_mm
                ),
            });
        }
        return Err(CamError::InvalidUndercutGeometry {
            reason: format!(
                "selected entry bore center is {:.2} mm away from the slot centerline",
                dist
            ),
        });
    }
    Err(CamError::InvalidUndercutGeometry {
        reason: format!(
            "no verified pre-cleared entry bore (>= Ø{min_bore_dia:.2} mm) found on slot centerline"
        ),
    })
}

/// Emits roof-safe horizontal slot / internal-dovetail passes across all Z levels and lateral steps.
///
/// Guarantees:
/// - Vertical plunge occurs ONLY at the verified open entry point or entry bore center.
/// - Every lateral pass returns to centerline (`offset = 0.0`) and exits horizontally to the
///   verified safe entry/exit zone BEFORE any `+Z` retract.
#[allow(clippy::too_many_arguments)]
pub(crate) fn emit_undercut_slot_passes(
    builder: &mut ToolpathBuilder,
    plan: &SafeSlotEntryPlan,
    z_levels: &[f64],
    need_center_first: bool,
    lateral_steps: &[(f64, bool)],
    climb_direction: bool,
    safe_z: f64,
    feed_plunge: f64,
    feed_lead: f64,
    feed_cut: f64,
) {
    match plan {
        SafeSlotEntryPlan::OpenEnd {
            path,
            both_ends_open,
        } => {
            let Some(p_entry) = polyline_start_point(path) else {
                return;
            };
            let Some(p_far) = polyline_end_point(path) else {
                return;
            };

            // Rapid above open entry point (always outside stock/roof!)
            builder.rapid_to(DVec3::new(p_entry.x, p_entry.y, safe_z));

            let mut at_far_end = false;
            for &z_cut in z_levels {
                // Descend vertically at whichever verified open end we are currently over
                let cur_xy = if at_far_end && *both_ends_open {
                    p_far
                } else {
                    p_entry
                };
                builder.linear_to(
                    DVec3::new(cur_xy.x, cur_xy.y, z_cut),
                    feed_plunge,
                    MotionRole::Plunge,
                );

                // 1. Optional centerline slotting pass (clears core material when cutter > throat)
                if need_center_first {
                    builder.pass_count += 1;
                    if at_far_end && *both_ends_open {
                        let rev = path.reverse();
                        emit_polyline_at_z(builder, &rev, z_cut, feed_cut, MotionRole::Cut);
                        at_far_end = false;
                    } else {
                        emit_polyline_at_z(builder, path, z_cut, feed_cut, MotionRole::Cut);
                        if *both_ends_open {
                            at_far_end = true;
                        } else {
                            // Blind far end: return horizontally along centerline back to open entry
                            let rev = path.reverse();
                            emit_polyline_at_z(
                                builder,
                                &rev,
                                z_cut,
                                feed_lead,
                                MotionRole::LinkTransition,
                            );
                            at_far_end = false;
                        }
                    }
                }

                // 2. Progressive lateral roughing / finish / spring passes on both left & right walls.
                //    For CW (M3) spindle:
                //    - Climb milling puts material on the RIGHT of travel:
                //      * Right wall (`offset = -d`): travel forward (`start -> end`)
                //      * Left wall  (`offset = +d`): travel backward (`end -> start`)
                //    - Conventional milling reverses the travel side pair.
                let first_sign = if climb_direction { -1.0 } else { 1.0 };
                let second_sign = -first_sign;

                for &(offset_d, is_finish) in lateral_steps {
                    if offset_d <= 1e-6 {
                        continue;
                    }
                    let role = if is_finish {
                        MotionRole::FinishCut
                    } else {
                        MotionRole::Cut
                    };

                    // If we are currently at the open far end, orient `active_path` from far -> entry
                    // so the lateral loop starts cleanly from our current open-air position!
                    let active_path = if at_far_end && *both_ends_open {
                        path.reverse()
                    } else {
                        path.clone()
                    };
                    let active_entry = polyline_start_point(&active_path).unwrap_or(p_entry);
                    let active_far = polyline_end_point(&active_path).unwrap_or(p_far);

                    let first_poly = offset_open_polyline(&active_path, first_sign * offset_d);
                    let second_poly =
                        offset_open_polyline(&active_path, second_sign * offset_d).reverse();

                    if let (Some(f_start), Some(s_start)) = (
                        polyline_start_point(&first_poly),
                        polyline_start_point(&second_poly),
                    ) {
                        builder.pass_count += 1;
                        // Lateral shift from centerline to first wall in open air at active_entry
                        builder.linear_to(
                            DVec3::new(f_start.x, f_start.y, z_cut),
                            feed_lead,
                            MotionRole::LeadIn,
                        );
                        // Cut first wall (`active_entry -> active_far`)
                        emit_polyline_at_z(builder, &first_poly, z_cut, feed_cut, role);

                        // At `active_far`: return laterally to centerline (`active_far`), then shift to second wall
                        builder.linear_to(
                            DVec3::new(active_far.x, active_far.y, z_cut),
                            feed_lead,
                            MotionRole::LinkTransition,
                        );
                        builder.linear_to(
                            DVec3::new(s_start.x, s_start.y, z_cut),
                            feed_lead,
                            MotionRole::LeadIn,
                        );
                        // Cut second wall (`active_far -> active_entry`)
                        emit_polyline_at_z(builder, &second_poly, z_cut, feed_cut, role);

                        // Return laterally to throat centerline at `active_entry` (verified open air!)
                        builder.linear_to(
                            DVec3::new(active_entry.x, active_entry.y, z_cut),
                            feed_lead,
                            MotionRole::LeadOut,
                        );
                    }
                }
            }

            // Final safe vertical retract ONLY from verified open air at centerline
            let exit_xy = if at_far_end && *both_ends_open {
                p_far
            } else {
                p_entry
            };
            builder.rapid_to(DVec3::new(exit_xy.x, exit_xy.y, safe_z));
        }

        SafeSlotEntryPlan::EntryBore { bore_center, arms } => {
            // Rapid above verified pre-cleared entry bore center
            builder.rapid_to(DVec3::new(bore_center.x, bore_center.y, safe_z));

            for &z_cut in z_levels {
                // Descend strictly inside verified pre-cleared entry bore
                builder.linear_to(
                    DVec3::new(bore_center.x, bore_center.y, z_cut),
                    feed_plunge,
                    MotionRole::Plunge,
                );

                for arm in arms {
                    let Some(arm_far) = polyline_end_point(arm) else {
                        continue;
                    };

                    if need_center_first {
                        builder.pass_count += 1;
                        emit_polyline_at_z(builder, arm, z_cut, feed_cut, MotionRole::Cut);
                        let rev = arm.reverse();
                        emit_polyline_at_z(
                            builder,
                            &rev,
                            z_cut,
                            feed_lead,
                            MotionRole::LinkTransition,
                        );
                    }

                    let first_sign = if climb_direction { -1.0 } else { 1.0 };
                    let second_sign = -first_sign;

                    for &(offset_d, is_finish) in lateral_steps {
                        if offset_d <= 1e-6 {
                            continue;
                        }
                        let role = if is_finish {
                            MotionRole::FinishCut
                        } else {
                            MotionRole::Cut
                        };
                        let first_poly = offset_open_polyline(arm, first_sign * offset_d);
                        let second_poly =
                            offset_open_polyline(arm, second_sign * offset_d).reverse();

                        if let (Some(f_start), Some(s_start)) = (
                            polyline_start_point(&first_poly),
                            polyline_start_point(&second_poly),
                        ) {
                            builder.pass_count += 1;
                            builder.linear_to(
                                DVec3::new(f_start.x, f_start.y, z_cut),
                                feed_lead,
                                MotionRole::LeadIn,
                            );
                            emit_polyline_at_z(builder, &first_poly, z_cut, feed_cut, role);
                            builder.linear_to(
                                DVec3::new(arm_far.x, arm_far.y, z_cut),
                                feed_lead,
                                MotionRole::LinkTransition,
                            );
                            builder.linear_to(
                                DVec3::new(s_start.x, s_start.y, z_cut),
                                feed_lead,
                                MotionRole::LeadIn,
                            );
                            emit_polyline_at_z(builder, &second_poly, z_cut, feed_cut, role);
                            // Always return strictly to entry bore center before next pass or retract
                            builder.linear_to(
                                DVec3::new(bore_center.x, bore_center.y, z_cut),
                                feed_lead,
                                MotionRole::LeadOut,
                            );
                        }
                    }
                }
            }

            // Retract vertically ONLY inside the verified entry bore center
            builder.rapid_to(DVec3::new(bore_center.x, bore_center.y, safe_z));
        }
    }
}

/// Computes axial Z tip levels spanning a slot height `[z_bottom, z_bottom + slot_height]` with a
/// cutter of cutting thickness `cutting_thickness`.
pub(crate) fn compute_slot_axial_levels(
    z_bottom: f64,
    slot_height: f64,
    cutting_thickness: f64,
    axial_stepdown: f64,
    is_undercut_cutter: bool,
) -> Vec<f64> {
    if !is_undercut_cutter {
        // Straight endmill keyway: step down from top of slot (`z_bottom + slot_height`) to `z_bottom`
        let step = axial_stepdown.clamp(0.05, cutting_thickness.max(0.05));
        if slot_height <= step + 1e-5 {
            return vec![z_bottom];
        }
        let n = (slot_height / step).ceil().max(1.0) as usize;
        let dz = slot_height / (n as f64);
        let z_top = z_bottom + slot_height;
        return (1..=n)
            .map(|i| {
                if i == n {
                    z_bottom
                } else {
                    z_top - dz * (i as f64)
                }
            })
            .collect();
    }

    let extra_h = (slot_height - cutting_thickness).max(0.0);
    if extra_h <= 1e-4 {
        return vec![z_bottom];
    }

    // SlotMill: top pass places upper disc edge flush with slot roof (`z_bottom + slot_height - cutting_thickness`),
    // bottom pass places lower disc edge flush with slot floor (`z_bottom`), with guaranteed flute overlap.
    let max_safe_step = (cutting_thickness * 0.95).max(0.05);
    let step = axial_stepdown.clamp(0.05, max_safe_step);
    let n_intervals = (extra_h / step).ceil().max(1.0) as usize;
    let dz = extra_h / (n_intervals as f64);
    let z_upper = z_bottom + extra_h;
    (0..=n_intervals)
        .map(|i| {
            if i == n_intervals {
                z_bottom
            } else {
                z_upper - dz * (i as f64)
            }
        })
        .collect()
}

/// Builds progressive lateral offsets `(offset_mm, is_finish)` from `start_offset` outward to
/// `max_offset`, reserving `finish_allowance` for the final full-depth finishing pass and appending
/// an optional zero-stock spring pass.
pub(crate) fn build_progressive_lateral_offsets(
    max_offset: f64,
    start_offset: f64,
    radial_stepover: f64,
    finish_allowance: f64,
    spring_pass: bool,
) -> Vec<(f64, bool)> {
    let mut out = Vec::new();
    if max_offset <= 1e-5 {
        if spring_pass {
            out.push((0.0, true));
        }
        return out;
    }

    let step = radial_stepover.max(0.01);
    let finish_allow = finish_allowance.clamp(0.0, max_offset);
    let rough_limit = (max_offset - finish_allow).max(0.0);

    if rough_limit > start_offset + 1e-5 {
        let span = rough_limit - start_offset;
        let n = (span / step).ceil().max(1.0) as usize;
        let actual_step = span / (n as f64);
        for i in 1..=n {
            let d = if i == n {
                rough_limit
            } else {
                start_offset + actual_step * (i as f64)
            };
            if d > 1e-5 {
                out.push((d, false));
            }
        }
    }

    // Final exact finishing pass at `max_offset`
    let need_finish = out
        .last()
        .is_none_or(|&(last_d, _)| (max_offset - last_d).abs() > 1e-5);
    if need_finish {
        out.push((max_offset, true));
    } else if let Some(last) = out.last_mut() {
        last.1 = true;
    }

    if spring_pass {
        out.push((max_offset, true));
    }
    out
}

/// Extracts slot centerlines and entry bore candidates from `ExtractedFeature25D` selections.
pub(crate) fn extract_slot_centerlines_and_bores(
    features: &[ExtractedFeature25D],
) -> (Vec<ExtractedSlotCenterline>, Vec<EntryBoreCandidate>) {
    let mut centerlines = Vec::new();
    let mut bores = Vec::new();
    let mut planar_candidates: Vec<(ArcPolyline2D, Vec<bool>, f64, bool)> = Vec::new();

    for feat in features {
        match feat {
            ExtractedFeature25D::Contour { polyline_wcs, .. } => {
                if !polyline_wcs.segments.is_empty() {
                    centerlines.push(ExtractedSlotCenterline {
                        polyline: polyline_wcs.clone(),
                        inferred_width_mm: None,
                        from_brep_face: false,
                        start_is_blind_face_wall: false,
                        end_is_blind_face_wall: false,
                    });
                }
            }
            ExtractedFeature25D::CylindricalHole {
                center_wcs,
                diameter_mm,
                ..
            } => {
                if diameter_mm.is_finite() && *diameter_mm > 1e-3 {
                    bores.push(EntryBoreCandidate {
                        center_wcs: *center_wcs,
                        diameter_mm: *diameter_mm,
                    });
                }
            }
            ExtractedFeature25D::PlanarFace {
                source_face_id,
                outer_boundary_wcs,
                open_outer_segments,
                area_mm2,
                ..
            } => {
                if !outer_boundary_wcs.segments.is_empty() {
                    planar_candidates.push((
                        outer_boundary_wcs.clone(),
                        open_outer_segments.clone(),
                        *area_mm2,
                        source_face_id.is_some(),
                    ));
                }
            }
            ExtractedFeature25D::ModeledChamfer { .. } => {}
        }
    }

    // If explicit contours exist, circular PlanarFace regions (e.g. sketch entry circles) act as entry bores;
    // otherwise extract slot centerlines from rectangular/obround PlanarFaces (modeled B-Rep T-slot floors!).
    for (poly, open_flags, area, is_brep) in planar_candidates {
        if !centerlines.is_empty()
            && let Some(bore) = try_classify_circular_bore(&poly, area)
        {
            bores.push(bore);
            continue;
        }
        if let Some(mut cl) = extract_centerline_from_planar_slot_face(&poly, &open_flags) {
            cl.from_brep_face = is_brep;
            centerlines.push(cl);
        }
    }

    (centerlines, bores)
}

fn try_classify_circular_bore(poly: &ArcPolyline2D, area_mm2: f64) -> Option<EntryBoreCandidate> {
    if !poly.closed || poly.segments.is_empty() {
        return None;
    }
    let pts = sample_polyline_vertices(poly);
    if pts.len() < 3 {
        return None;
    }
    let mut min_p = DVec2::splat(f64::INFINITY);
    let mut max_p = DVec2::splat(f64::NEG_INFINITY);
    for &p in &pts {
        min_p = min_p.min(p);
        max_p = max_p.max(p);
    }
    let dx = max_p.x - min_p.x;
    let dy = max_p.y - min_p.y;
    if dx <= 1e-3 || dy <= 1e-3 || (dx - dy).abs() > 0.08 * dx.max(dy) {
        return None;
    }
    let est_d = 0.5 * (dx + dy);
    let circle_area = std::f64::consts::FRAC_PI_4 * est_d * est_d;
    let eff_area = if area_mm2 > 1e-6 {
        area_mm2
    } else {
        poly.signed_area().abs()
    };
    if (eff_area - circle_area).abs() <= 0.12 * circle_area {
        Some(EntryBoreCandidate {
            center_wcs: 0.5 * (min_p + max_p),
            diameter_mm: est_d,
        })
    } else {
        None
    }
}

fn extract_centerline_from_planar_slot_face(
    outer: &ArcPolyline2D,
    open_flags: &[bool],
) -> Option<ExtractedSlotCenterline> {
    // Choose primary slot axis from the longest linear edge (falling back to AABB major axis)
    let mut best_len = 0.0_f64;
    let mut axis_u = DVec2::X;
    for seg in &outer.segments {
        if let ArcSegment2D::Line { start, end } = *seg {
            let v = end - start;
            let l = v.length();
            if l > best_len {
                best_len = l;
                axis_u = v / l;
            }
        }
    }
    let pts = sample_polyline_vertices(outer);
    if pts.len() < 2 {
        return None;
    }
    let axis_v = DVec2::new(-axis_u.y, axis_u.x);
    let mut u_min = f64::INFINITY;
    let mut u_max = f64::NEG_INFINITY;
    let mut v_min = f64::INFINITY;
    let mut v_max = f64::NEG_INFINITY;
    for &p in &pts {
        let u = p.dot(axis_u);
        let v = p.dot(axis_v);
        u_min = u_min.min(u);
        u_max = u_max.max(u);
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }
    let len = u_max - u_min;
    let width = v_max - v_min;
    if len <= 1e-4 || width <= 1e-4 {
        return None;
    }

    // Check if end segments at `u_min` / `u_max` are open vs closed walls
    let mut start_has_open = false;
    let mut end_has_open = false;
    for (idx, seg) in outer.segments.iter().enumerate() {
        let is_open = open_flags.get(idx).copied().unwrap_or(false);
        if !is_open {
            continue;
        }
        let mid = match *seg {
            ArcSegment2D::Line { start, end } => 0.5 * (start + end),
            ArcSegment2D::Arc { start, end, .. } => 0.5 * (start + end),
        };
        let u_mid = mid.dot(axis_u);
        if (u_mid - u_min).abs() < 0.25 * width.max(1.0) {
            start_has_open = true;
        }
        if (u_mid - u_max).abs() < 0.25 * width.max(1.0) {
            end_has_open = true;
        }
    }

    let v_mid = 0.5 * (v_min + v_max);
    let p_start = axis_u * u_min + axis_v * v_mid;
    let p_end = axis_u * u_max + axis_v * v_mid;
    Some(ExtractedSlotCenterline {
        polyline: ArcPolyline2D {
            segments: vec![ArcSegment2D::Line {
                start: p_start,
                end: p_end,
            }],
            closed: false,
        },
        inferred_width_mm: Some(width),
        from_brep_face: false,
        start_is_blind_face_wall: !open_flags.is_empty() && !start_has_open,
        end_is_blind_face_wall: !open_flags.is_empty() && !end_has_open,
    })
}

fn sample_polyline_vertices(poly: &ArcPolyline2D) -> Vec<DVec2> {
    let mut pts = Vec::new();
    for seg in &poly.segments {
        match *seg {
            ArcSegment2D::Line { start, end } => {
                pts.push(start);
                pts.push(end);
            }
            ArcSegment2D::Arc {
                start,
                end,
                center,
                radius,
                ccw,
            } => {
                pts.push(start);
                let a0 = (start.y - center.y).atan2(start.x - center.x);
                let a1 = (end.y - center.y).atan2(end.x - center.x);
                let mut sweep = a1 - a0;
                if ccw && sweep <= 0.0 {
                    sweep += std::f64::consts::TAU;
                } else if !ccw && sweep >= 0.0 {
                    sweep -= std::f64::consts::TAU;
                }
                for k in 1..8 {
                    let a = a0 + sweep * (k as f64 / 8.0);
                    let (s, c) = a.sin_cos();
                    pts.push(center + DVec2::new(c, s) * radius);
                }
                pts.push(end);
            }
        }
    }
    pts
}

#[inline]
pub(crate) fn polyline_start_point(poly: &ArcPolyline2D) -> Option<DVec2> {
    poly.segments.first().map(|s| s.start())
}

#[inline]
pub(crate) fn polyline_end_point(poly: &ArcPolyline2D) -> Option<DVec2> {
    poly.segments.last().map(|s| s.end())
}

pub(crate) fn polyline_start_tangent(poly: &ArcPolyline2D) -> Option<DVec2> {
    let seg = poly.segments.first()?;
    match *seg {
        ArcSegment2D::Line { start, end } => {
            let d = (end - start).normalize_or_zero();
            (d.length_squared() > 0.5).then_some(d)
        }
        ArcSegment2D::Arc {
            start, center, ccw, ..
        } => {
            let r = (start - center).normalize_or_zero();
            let t = if ccw {
                DVec2::new(-r.y, r.x)
            } else {
                DVec2::new(r.y, -r.x)
            };
            (t.length_squared() > 0.5).then_some(t)
        }
    }
}

pub(crate) fn polyline_end_tangent(poly: &ArcPolyline2D) -> Option<DVec2> {
    let seg = poly.segments.last()?;
    match *seg {
        ArcSegment2D::Line { start, end } => {
            let d = (end - start).normalize_or_zero();
            (d.length_squared() > 0.5).then_some(d)
        }
        ArcSegment2D::Arc {
            end, center, ccw, ..
        } => {
            let r = (end - center).normalize_or_zero();
            let t = if ccw {
                DVec2::new(-r.y, r.x)
            } else {
                DVec2::new(r.y, -r.x)
            };
            (t.length_squared() > 0.5).then_some(t)
        }
    }
}

/// Extends an open [`ArcPolyline2D`] tangentially at its start by `ext_start` and at its end by `ext_end`.
pub(crate) fn extend_open_polyline(
    poly: &ArcPolyline2D,
    ext_start: f64,
    ext_end: f64,
) -> ArcPolyline2D {
    let mut segs = poly.segments.clone();
    if segs.is_empty() {
        return poly.clone();
    }
    if ext_start > 1e-5
        && let (Some(p0), Some(t0)) = (polyline_start_point(poly), polyline_start_tangent(poly))
    {
        let p_new = p0 - t0 * ext_start;
        match &mut segs[0] {
            ArcSegment2D::Line { start, .. } => *start = p_new,
            ArcSegment2D::Arc { .. } => {
                segs.insert(
                    0,
                    ArcSegment2D::Line {
                        start: p_new,
                        end: p0,
                    },
                );
            }
        }
    }
    if ext_end > 1e-5
        && let (Some(p1), Some(t1)) = (polyline_end_point(poly), polyline_end_tangent(poly))
    {
        let p_new = p1 + t1 * ext_end;
        if let Some(last) = segs.last_mut() {
            match last {
                ArcSegment2D::Line { end, .. } => *end = p_new,
                ArcSegment2D::Arc { .. } => {
                    segs.push(ArcSegment2D::Line {
                        start: p1,
                        end: p_new,
                    });
                }
            }
        }
    }
    ArcPolyline2D {
        segments: segs,
        closed: false,
    }
}

fn trim_polyline_ends_for_blind_walls(poly: &mut ArcPolyline2D, trim_start: f64, trim_end: f64) {
    if poly.segments.len() == 1
        && let ArcSegment2D::Line { start, end } = &mut poly.segments[0]
    {
        let d = *end - *start;
        let len = d.length();
        if len > trim_start + trim_end + 1e-3 {
            let dir = d / len;
            *start += dir * trim_start;
            *end -= dir * trim_end;
        }
    }
}

/// Offsets an open [`ArcPolyline2D`] analytically by signed distance `signed_left_offset`
/// (`+d` = left of travel, `-d` = right of travel), preserving circular arcs.
pub(crate) fn offset_open_polyline(poly: &ArcPolyline2D, signed_left_offset: f64) -> ArcPolyline2D {
    if signed_left_offset.abs() <= 1e-7 || poly.segments.is_empty() {
        return poly.clone();
    }
    let mut out_segs: Vec<ArcSegment2D> = Vec::with_capacity(poly.segments.len() * 2);
    for seg in &poly.segments {
        let shifted = match *seg {
            ArcSegment2D::Line { start, end } => {
                let d = end - start;
                let len = d.length();
                if len <= 1e-8 {
                    continue;
                }
                let left_n = DVec2::new(-d.y / len, d.x / len);
                let shift = left_n * signed_left_offset;
                ArcSegment2D::Line {
                    start: start + shift,
                    end: end + shift,
                }
            }
            ArcSegment2D::Arc {
                start,
                end,
                center,
                radius,
                ccw,
            } => {
                // For a CCW arc, left normal points INWARD toward center (`r' = r - d`);
                // for a CW arc, left normal points OUTWARD away from center (`r' = r + d`).
                let new_r = if ccw {
                    radius - signed_left_offset
                } else {
                    radius + signed_left_offset
                };
                if new_r <= 1e-4 {
                    continue;
                }
                let r0 = (start - center).normalize_or_zero();
                let r1 = (end - center).normalize_or_zero();
                ArcSegment2D::Arc {
                    start: center + r0 * new_r,
                    end: center + r1 * new_r,
                    center,
                    radius: new_r,
                    ccw,
                }
            }
        };

        if let Some(prev) = out_segs.last() {
            let prev_end = segment_end_pt(*prev);
            let cur_start = segment_start_pt(shifted);
            if prev_end.distance(cur_start) > 1e-5 {
                out_segs.push(ArcSegment2D::Line {
                    start: prev_end,
                    end: cur_start,
                });
            }
        }
        out_segs.push(shifted);
    }

    ArcPolyline2D {
        segments: out_segs,
        closed: false,
    }
}

#[inline]
fn segment_start_pt(seg: ArcSegment2D) -> DVec2 {
    match seg {
        ArcSegment2D::Line { start, .. } | ArcSegment2D::Arc { start, .. } => start,
    }
}

#[inline]
fn segment_end_pt(seg: ArcSegment2D) -> DVec2 {
    match seg {
        ArcSegment2D::Line { end, .. } | ArcSegment2D::Arc { end, .. } => end,
    }
}

/// Emits every segment of `poly` at constant Z height `z_wcs` into `builder`, preserving true XY arcs.
pub(crate) fn emit_polyline_at_z(
    builder: &mut ToolpathBuilder,
    poly: &ArcPolyline2D,
    z_wcs: f64,
    feed_mm_min: f64,
    role: MotionRole,
) {
    for seg in &poly.segments {
        match *seg {
            ArcSegment2D::Line { end, .. } => {
                builder.linear_to(DVec3::new(end.x, end.y, z_wcs), feed_mm_min, role);
            }
            ArcSegment2D::Arc {
                end,
                center,
                radius,
                ccw,
                ..
            } => {
                builder.arc_xy_to(XyArcMove {
                    end_wcs: DVec3::new(end.x, end.y, z_wcs),
                    center_xy: center,
                    radius_mm: radius,
                    ccw,
                    turns: 0,
                    feed_mm_min,
                    role,
                });
            }
        }
    }
}

fn min_distance_to_polyline(poly: &ArcPolyline2D, pt: DVec2) -> f64 {
    let mut best = f64::INFINITY;
    for seg in &poly.segments {
        match *seg {
            ArcSegment2D::Line { start, end } => {
                let ab = end - start;
                let l2 = ab.length_squared();
                let t = if l2 > 1e-12 {
                    ((pt - start).dot(ab) / l2).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                best = best.min(pt.distance(start + ab * t));
            }
            ArcSegment2D::Arc {
                start,
                end,
                center,
                radius,
                ..
            } => {
                best = best.min(pt.distance(start)).min(pt.distance(end));
                let r_dist = (pt.distance(center) - radius).abs();
                best = best.min(r_dist);
            }
        }
    }
    best
}

/// Splits an open centerline at `bore_center` into 1 or 2 outward arms starting at `bore_center`.
fn split_centerline_at_bore(poly: &ArcPolyline2D, bore_center: DVec2) -> Vec<ArcPolyline2D> {
    let mut best_seg_idx = 0usize;
    let mut best_t = 0.0_f64;
    let mut best_dist = f64::INFINITY;
    let mut best_proj = bore_center;

    for (idx, seg) in poly.segments.iter().enumerate() {
        match *seg {
            ArcSegment2D::Line { start, end } => {
                let ab = end - start;
                let l2 = ab.length_squared();
                let t = if l2 > 1e-12 {
                    ((bore_center - start).dot(ab) / l2).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let q = start + ab * t;
                let d = bore_center.distance(q);
                if d < best_dist {
                    best_dist = d;
                    best_seg_idx = idx;
                    best_t = t;
                    best_proj = q;
                }
            }
            ArcSegment2D::Arc { start, end, .. } => {
                let d0 = bore_center.distance(start);
                if d0 < best_dist {
                    best_dist = d0;
                    best_seg_idx = idx;
                    best_t = 0.0;
                    best_proj = start;
                }
                let d1 = bore_center.distance(end);
                if d1 < best_dist {
                    best_dist = d1;
                    best_seg_idx = idx;
                    best_t = 1.0;
                    best_proj = end;
                }
            }
        }
    }

    let mut left_segs = Vec::new();
    let mut right_segs = Vec::new();
    for (idx, &seg) in poly.segments.iter().enumerate() {
        if idx < best_seg_idx {
            left_segs.push(seg);
        } else if idx > best_seg_idx {
            right_segs.push(seg);
        } else if let ArcSegment2D::Line { start, end } = seg {
            if best_t > 1e-4 && start.distance(best_proj) > 1e-4 {
                left_segs.push(ArcSegment2D::Line {
                    start,
                    end: best_proj,
                });
            }
            if best_t < 1.0 - 1e-4 && best_proj.distance(end) > 1e-4 {
                right_segs.push(ArcSegment2D::Line {
                    start: best_proj,
                    end,
                });
            }
        } else if best_t <= 0.5 {
            right_segs.push(seg);
        } else {
            left_segs.push(seg);
        }
    }

    let mut arms = Vec::new();
    if !left_segs.is_empty() {
        let mut arm = ArcPolyline2D {
            segments: left_segs,
            closed: false,
        }
        .reverse();
        if let Some(first) = polyline_start_point(&arm)
            && first.distance(bore_center) > 1e-4
        {
            arm.segments.insert(
                0,
                ArcSegment2D::Line {
                    start: bore_center,
                    end: first,
                },
            );
        }
        if arm.perimeter() > 1e-3 {
            arms.push(arm);
        }
    }
    if !right_segs.is_empty() {
        let mut arm = ArcPolyline2D {
            segments: right_segs,
            closed: false,
        };
        if let Some(first) = polyline_start_point(&arm)
            && first.distance(bore_center) > 1e-4
        {
            arm.segments.insert(
                0,
                ArcSegment2D::Line {
                    start: bore_center,
                    end: first,
                },
            );
        }
        if arm.perimeter() > 1e-3 {
            arms.push(arm);
        }
    }
    arms
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::sync::atomic::AtomicBool;

    use chip_cad::CadDocument;
    use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, DisplayUnit, ParamLength};
    use chip_sketch::{SketchEntity, SketchEntityEntry, SketchLayer};
    use chip_tools::{CutterGeometry, starter_tslot_12mm};

    use crate::{
        BoxAnchor27, CamError, FixedBoxXyAlign, FixedBoxZAlign, GeometrySelectionItem,
        HeightReference, HeightSpec, Operation, OperationGenRequest, OperationKind,
        SegmentMotion3D, Setup, SlotUndercutMode, SlotUndercutParams, SlotUndercutPreset,
        StockConfig, StrategyParams, UndercutEntryMode, WcsOriginMode, WorkOffsetSlot,
        generate_operation_toolpath_with_rest,
    };

    fn make_box_setup() -> (CadDocument, crate::ResolvedSetup) {
        let doc = CadDocument::empty();
        let mut setup = Setup::new("s_slot", "Slot Test", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        };
        setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(100.0),
            depth_y: ParamLength::from_mm(100.0),
            height_z: ParamLength::from_mm(20.0),
            xy_align: FixedBoxXyAlign::CenterXy,
            z_align: FixedBoxZAlign::CenterZ,
        };
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
        (doc, resolved)
    }

    fn make_line_sketch(x0_wcs: f64, x1_wcs: f64, bore_wcs: Option<(DVec2, f64)>) -> SketchLayer {
        // SetupXy sketches anchor (0, 0) at Stock Front-Left (`(-50, -50)` in TOP_CENTER WCS)
        let mut sk = SketchLayer::new("sk_slot", "Slot Centerline", "s_slot");
        let line = ArcPolyline2D {
            segments: vec![ArcSegment2D::Line {
                start: DVec2::new(x0_wcs + 50.0, 50.0),
                end: DVec2::new(x1_wcs + 50.0, 50.0),
            }],
            closed: false,
        };
        sk.entities.push(SketchEntityEntry {
            id: "e_center".to_string(),
            name: "Centerline".to_string(),
            sub_layer: None,
            is_construction: false,
            entity: SketchEntity::new_polyline(line),
        });
        if let Some((c, dia)) = bore_wcs {
            sk.entities.push(SketchEntityEntry {
                id: "e_bore".to_string(),
                name: "Entry Bore".to_string(),
                sub_layer: None,
                is_construction: false,
                entity: SketchEntity::new_circle(
                    ParamLength::from_mm(c.x + 50.0),
                    ParamLength::from_mm(c.y + 50.0),
                    ParamLength::from_mm(dia),
                ),
            });
        }
        sk.rebuild_regions(None);
        sk
    }

    #[test]
    fn rejects_blind_tslot_without_entry_bore_and_rejects_neck_and_reach_violations() {
        let (doc, resolved) = make_box_setup();
        let slot_12 = starter_tslot_12mm(); // Ø12, thick=4, neck=5, neck_len=18
        let cancel = AtomicBool::new(false);

        // 1. Blind T-slot completely inside stock (`x ∈ [-20, 20]` inside `[-50, 50]`) with no entry bore -> MUST FAIL!
        let sk_blind = make_line_sketch(-20.0, 20.0, None);
        let mut p_m8 = SlotUndercutParams::default();
        p_m8.apply_preset(SlotUndercutPreset::TSlotM8);
        let mut op = Operation::new("op_blind", "Blind T-Slot", OperationKind::SlotUndercut);
        op.tool_id = Some(slot_12.id.clone());
        op.selections = vec![GeometrySelectionItem::SketchEntity {
            sketch_id: sk_blind.id.clone(),
            entity_id: "e_center".to_string(),
            reverse: false,
            side_flip: false,
        }];
        op.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        op.heights.bottom = HeightSpec::new(HeightReference::StockTop, -14.0);
        op.strategy = Some(StrategyParams::SlotUndercut(p_m8.clone()));

        let err = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: std::slice::from_ref(&sk_blind),
            setup: &resolved,
            op: &op,
            tool: &slot_12,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: true,
            upstream_rest: &[],
            carryover_stock: None,
            cancel: &cancel,
        })
        .unwrap_err();
        assert!(
            matches!(err, CamError::InvalidUndercutGeometry { ref reason } if reason.contains("blind")),
            "expected blind slot rejection, got: {err}"
        );

        // 2. EntryBoreCenter with undersized entry bore (`Ø12.2 < Ø12.5`) -> MUST FAIL!
        let sk_small_bore = make_line_sketch(-20.0, 20.0, Some((DVec2::ZERO, 12.2)));
        let mut op_bore = op.clone();
        op_bore
            .selections
            .push(GeometrySelectionItem::SketchRegion {
                sketch_id: sk_small_bore.id.clone(),
                region_id: sk_small_bore.regions[0].id.clone(),
            });
        let mut p_bore = p_m8.clone();
        p_bore.entry_mode = UndercutEntryMode::EntryBoreCenter;
        op_bore.strategy = Some(StrategyParams::SlotUndercut(p_bore.clone()));
        let err_bore = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: &[sk_small_bore],
            setup: &resolved,
            op: &op_bore,
            tool: &slot_12,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: true,
            upstream_rest: &[],
            carryover_stock: None,
            cancel: &cancel,
        })
        .unwrap_err();
        assert!(
            matches!(err_bore, CamError::InvalidUndercutGeometry { .. }),
            "expected undersized entry bore rejection, got: {err_bore}"
        );

        // 3. EntryBoreCenter with valid pre-cleared Ø13.0 bore -> SUCCEEDS and plunges/retracts ONLY at bore center!
        let sk_ok_bore = make_line_sketch(-20.0, 20.0, Some((DVec2::ZERO, 13.0)));
        op_bore.selections = vec![
            GeometrySelectionItem::SketchEntity {
                sketch_id: sk_ok_bore.id.clone(),
                entity_id: "e_center".to_string(),
                reverse: false,
                side_flip: false,
            },
            GeometrySelectionItem::SketchRegion {
                sketch_id: sk_ok_bore.id.clone(),
                region_id: sk_ok_bore.regions[0].id.clone(),
            },
        ];
        let tp_bore = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: &[sk_ok_bore],
            setup: &resolved,
            op: &op_bore,
            tool: &slot_12,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: true,
            upstream_rest: &[],
            carryover_stock: None,
            cancel: &cancel,
        })
        .unwrap();
        for seg in &tp_bore.segments_wcs {
            let dz = (seg.end_wcs.z - seg.start_wcs.z).abs();
            if dz > 1e-4 {
                assert!(
                    DVec2::new(seg.start_wcs.x, seg.start_wcs.y).length() < 1e-4
                        && DVec2::new(seg.end_wcs.x, seg.end_wcs.y).length() < 1e-4,
                    "vertical motion in EntryBoreCenter MUST stay at bore center (0,0), got {:?} -> {:?}",
                    seg.start_wcs,
                    seg.end_wcs
                );
            }
        }

        // 4. `neck_diameter >= throat_width` -> MUST FAIL!
        let sk_thru = make_line_sketch(-50.0, 50.0, None);
        let mut op_thru = op.clone();
        op_thru.selections = vec![GeometrySelectionItem::SketchEntity {
            sketch_id: sk_thru.id.clone(),
            entity_id: "e_center".to_string(),
            reverse: false,
            side_flip: false,
        }];
        let mut p_narrow = p_m8.clone();
        p_narrow.throat_width = ParamLength::from_mm(5.0); // == neck_diameter 5.0!
        op_thru.strategy = Some(StrategyParams::SlotUndercut(p_narrow));
        let err_neck = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: std::slice::from_ref(&sk_thru),
            setup: &resolved,
            op: &op_thru,
            tool: &slot_12,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: true,
            upstream_rest: &[],
            carryover_stock: None,
            cancel: &cancel,
        })
        .unwrap_err();
        assert!(
            matches!(err_neck, CamError::InvalidUndercutGeometry { ref reason } if reason.contains("neck diameter")),
            "expected neck >= throat rejection, got: {err_neck}"
        );

        // 5. `neck_length < total_depth` (`depth = 20 mm > neck_len 18 mm`) -> MUST FAIL!
        let mut op_deep = op_thru.clone();
        op_deep.strategy = Some(StrategyParams::SlotUndercut(p_m8.clone()));
        op_deep.heights.bottom = HeightSpec::new(HeightReference::StockTop, -20.0);
        let err_reach = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: std::slice::from_ref(&sk_thru),
            setup: &resolved,
            op: &op_deep,
            tool: &slot_12,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: true,
            upstream_rest: &[],
            carryover_stock: None,
            cancel: &cancel,
        })
        .unwrap_err();
        assert!(
            matches!(err_reach, CamError::InvalidUndercutGeometry { ref reason } if reason.contains("neck length")),
            "expected neck reach rejection, got: {err_reach}"
        );

        // 6. Valid through M8 T-slot (6 mm tall vs 4 mm thick cutter) produces 2 overlapping Z levels
        //    and ZERO vertical plunges/retracts inside stock X ∈ [-50, +50]!
        let mut op_ok = op_thru;
        op_ok.strategy = Some(StrategyParams::SlotUndercut(SlotUndercutParams {
            mode: SlotUndercutMode::TSlot,
            entry_mode: UndercutEntryMode::BothEndsThrough,
            spring_pass: true,
            ..p_m8
        }));
        let tp_ok = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: &[sk_thru],
            setup: &resolved,
            op: &op_ok,
            tool: &slot_12,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: true,
            upstream_rest: &[],
            carryover_stock: None,
            cancel: &cancel,
        })
        .unwrap();

        let mut cut_zs: Vec<f64> = tp_ok
            .segments_wcs
            .iter()
            .filter_map(|s| match s.motion {
                SegmentMotion3D::Linear { .. }
                    if (s.start_wcs.z - s.end_wcs.z).abs() < 1e-4 && s.start_wcs.z < -1.0 =>
                {
                    Some((s.start_wcs.z * 100.0).round() / 100.0)
                }
                _ => None,
            })
            .collect();
        cut_zs.sort_by(|a, b| a.total_cmp(b));
        cut_zs.dedup();
        assert_eq!(
            cut_zs,
            vec![-14.0, -12.0],
            "expected 2 axial Z levels (-12.0 & -14.0) for 6mm slot height with 4mm thick cutter"
        );

        let r_tool = match slot_12.geometry {
            CutterGeometry::SlotMill { diameter_mm, .. } => 0.5 * diameter_mm,
            _ => unreachable!(),
        };
        for seg in &tp_ok.segments_wcs {
            if (seg.end_wcs.z - seg.start_wcs.z).abs() > 1e-4 {
                assert!(
                    seg.start_wcs.x.abs() >= 50.0 + r_tool - 1e-3,
                    "vertical motion MUST occur strictly outside stock roof (±{:.1}), got {:?}",
                    50.0 + r_tool,
                    seg
                );
            }
        }
    }
}
