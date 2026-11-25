//! **Dovetail Slot & Rail Milling** (`OperationKind::Dovetail`).
//!
//! Supports both internal female dovetail slots (`DovetailMode::InternalSlot`) and external male
//! dovetail guideways (`DovetailMode::ExternalRail`) with strict physical machine safety guards:
//! - Validates `|tool.side_angle_deg - params.side_angle_deg| <= 0.5°` (plus modeled B-Rep face
//!   angle consistency when angled faces are selected).
//! - Validates `neck_diameter_mm < top_throat_width`, `cutting_height_mm >= dovetail_depth`,
//!   `neck_length_mm >= total_depth`, and exact upper-flank throat fit (`2 * r_at_top <= top_throat_width`).
//! - Never vertically plunges or retracts beneath an overhanging dovetail roof: enters and exits
//!   strictly in verified open air past an open slot end or inside a pre-cleared entry bore.

use chip_math::{ArcPolyline2D, DVec2, DVec3};
use chip_tools::CutterGeometry;

use crate::{
    cl_data::MotionRole,
    params::{DovetailMode, DovetailParams},
    selection::ExtractedFeature25D,
    strategies::{
        StrategyExecContext,
        slot_undercut::{
            build_progressive_lateral_offsets, emit_polyline_at_z, emit_undercut_slot_passes,
            extend_open_polyline, extract_slot_centerlines_and_bores, offset_open_polyline,
            polyline_end_point, polyline_end_tangent, polyline_start_point, polyline_start_tangent,
            resolve_safe_slot_entry_plan,
        },
    },
    toolpath::ToolpathBuilder,
    wcs::CamError,
};

/// Generates a **Dovetail Slot / Rail** toolpath (`OperationKind::Dovetail`).
pub fn generate_dovetail(
    ctx: &StrategyExecContext<'_>,
    params: &DovetailParams,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    ctx.check_cancel()?;

    // 1. Require a `DovetailCutter` and extract exact analytical dimensions
    let CutterGeometry::DovetailCutter {
        diameter_mm,
        side_angle_deg: tool_angle_deg,
        cutting_height_mm,
        corner_radius_mm,
        neck_diameter_mm,
        neck_length_mm,
        ..
    } = ctx.tool.geometry
    else {
        return Err(CamError::IncompatibleTool {
            op_name: ctx.op.name.clone(),
            op_kind: ctx.op.kind.label(),
            tool_name: ctx.tool.name.clone(),
            tool_kind: ctx.tool.geometry.kind_label(),
        });
    };

    // 2. Validate numeric parameters
    let top_throat_width = params.top_throat_width.value_mm;
    let mut dovetail_depth = params.dovetail_depth.value_mm;
    let mut target_angle_deg = params.side_angle_deg;
    let radial_stepover = params.radial_stepover.value_mm;
    let finish_allowance = params.finish_allowance.value_mm;
    let entry_extension = params.entry_extension.value_mm;

    if !target_angle_deg.is_finite() || target_angle_deg <= 15.0 || target_angle_deg >= 85.0 {
        return Err(CamError::InvalidParameter {
            param: "side_angle_deg",
            value: target_angle_deg,
            reason: "must be finite and within (15°, 85°)",
        });
    }
    if !dovetail_depth.is_finite() || dovetail_depth <= 1e-4 {
        return Err(CamError::InvalidParameter {
            param: "dovetail_depth",
            value: dovetail_depth,
            reason: "must be finite and > 0",
        });
    }
    if params.mode == DovetailMode::InternalSlot
        && (!top_throat_width.is_finite() || top_throat_width <= 1e-4)
    {
        return Err(CamError::InvalidParameter {
            param: "top_throat_width",
            value: top_throat_width,
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

    // Check modeled angled faces (if selected) for exact dovetail angle / depth consistency
    let mut modeled_rail_contours: Vec<(ArcPolyline2D, bool)> = Vec::new();
    for feat in &ctx.selection.features {
        if let ExtractedFeature25D::ModeledChamfer {
            upper_polyline_wcs,
            axial_drop_mm,
            bevel_half_angle_rad,
            outward_normal_xy_wcs,
            ..
        } = feat
        {
            // Angle from horizontal plane = 90° - bevel_half_angle
            let modeled_angle_deg = (90.0 - bevel_half_angle_rad.to_degrees()).clamp(15.0, 85.0);
            if (tool_angle_deg - modeled_angle_deg).abs() > 0.5 {
                return Err(CamError::InvalidUndercutGeometry {
                    reason: format!(
                        "dovetail cutter angle ({tool_angle_deg:.2}°) mismatches selected modeled dovetail wall angle ({modeled_angle_deg:.2}°, tolerance ±0.5°)"
                    ),
                });
            }
            target_angle_deg = modeled_angle_deg;
            if *axial_drop_mm > 1e-3 {
                dovetail_depth = *axial_drop_mm;
            }
            if !upper_polyline_wcs.segments.is_empty() {
                // Orient so outward_normal points to the LEFT (+1) or RIGHT (-1) cleanly
                let flip = outward_normal_xy_wcs.is_some_and(|n_out| {
                    polyline_start_tangent(upper_polyline_wcs).is_some_and(|t| {
                        let left_n = DVec2::new(-t.y, t.x);
                        left_n.dot(n_out) < 0.0
                    })
                });
                modeled_rail_contours.push((upper_polyline_wcs.clone(), flip));
            }
        }
    }

    // 3. Hard physical angle, flute-height, and relieved-neck reach safety checks
    if (tool_angle_deg - target_angle_deg).abs() > 0.5 {
        return Err(CamError::InvalidUndercutGeometry {
            reason: format!(
                "dovetail cutter side angle ({tool_angle_deg:.2}°) does not match target dovetail angle ({target_angle_deg:.2}°, maximum tolerance ±0.5°)"
            ),
        });
    }

    if cutting_height_mm + 1e-4 < dovetail_depth {
        return Err(CamError::InvalidUndercutGeometry {
            reason: format!(
                "dovetail cutter cutting height ({cutting_height_mm:.3} mm) is shallower than target dovetail depth ({dovetail_depth:.3} mm)"
            ),
        });
    }

    let radial_stock = ctx.passes.stock_to_leave_radial.value_mm.max(0.0);
    let axial_stock = ctx.passes.stock_to_leave_axial.value_mm.max(0.0);
    let z_bottom = ctx.heights.bottom_z_wcs + axial_stock;
    let total_depth = (ctx.heights.top_z_wcs - z_bottom).max(0.0);

    if neck_length_mm + 1e-4 < total_depth {
        return Err(CamError::InvalidUndercutGeometry {
            reason: format!(
                "dovetail cutter relieved neck length ({neck_length_mm:.3} mm) is shorter than total depth ({total_depth:.3} mm); shank would collide with workpiece"
            ),
        });
    }

    // Exact analytic flank radius of the dovetail cutter at `dz = dovetail_depth`, accounting for
    // bottom corner fillet `corner_radius_mm` so the angled flank matches to sub-micron precision!
    let r_at_dovetail_top = 0.5
        * CutterGeometry::dovetail_top_diameter_mm(
            diameter_mm,
            tool_angle_deg,
            dovetail_depth,
            corner_radius_mm,
        );
    let r_bottom = 0.5 * diameter_mm;

    let safe_z = ctx.safe_retract_z();
    let feed_cut = ctx.cutting.cutting_feed_mm_min;
    let feed_lead = ctx.cutting.lead_feed_mm_min.min(feed_cut).max(1.0);
    let feed_plunge = ctx.cutting.plunge_feed_mm_min.min(feed_cut).max(1.0);

    match params.mode {
        DovetailMode::InternalSlot => {
            // Validate throat clearance & exact flank reach inside the pre-cleared throat
            if neck_diameter_mm >= top_throat_width - 1e-4 {
                return Err(CamError::InvalidUndercutGeometry {
                    reason: format!(
                        "dovetail cutter neck diameter (Ø{neck_diameter_mm:.3} mm) must be strictly smaller than top throat width ({top_throat_width:.3} mm)"
                    ),
                });
            }

            let half_throat = 0.5 * top_throat_width;
            let offset_final = half_throat - r_at_dovetail_top - radial_stock;
            if offset_final < -1e-4 {
                return Err(CamError::InvalidUndercutGeometry {
                    reason: format!(
                        "dovetail cutter top diameter at depth {dovetail_depth:.2} mm (Ø{:.3} mm) exceeds top throat width ({top_throat_width:.3} mm); use a smaller dovetail cutter",
                        2.0 * r_at_dovetail_top
                    ),
                });
            }
            let offset_final = offset_final.max(0.0);

            let neck_outer_at_wall = offset_final + 0.5 * neck_diameter_mm;
            if neck_outer_at_wall > half_throat + 1e-4 {
                return Err(CamError::InvalidUndercutGeometry {
                    reason: format!(
                        "at final dovetail lateral offset ({offset_final:.3} mm), relieved neck outer edge ({neck_outer_at_wall:.3} mm) exceeds half-throat ({half_throat:.3} mm)"
                    ),
                });
            }

            let (centerlines, entry_bores) =
                extract_slot_centerlines_and_bores(&ctx.selection.features);
            if centerlines.is_empty() {
                return Err(CamError::EmptySelection {
                    op_name: ctx.op.name.clone(),
                    op_kind: ctx.op.kind.label(),
                });
            }

            // Progressive lateral stepovers from pre-cleared throat wall outward to `offset_final`
            let start_engage = (half_throat - r_bottom).max(0.0);
            let need_center_first = offset_final <= 1e-5 || r_bottom > half_throat - 1e-4;
            let lateral_steps = build_progressive_lateral_offsets(
                offset_final,
                start_engage,
                radial_stepover,
                finish_allowance,
                params.spring_pass,
            );
            let stock_aabb = ctx.setup.stock.aabb_wcs();

            for cl in &centerlines {
                ctx.check_cancel()?;
                if cl.polyline.segments.is_empty() || cl.polyline.perimeter() <= 1e-4 {
                    continue;
                }
                let plan = resolve_safe_slot_entry_plan(
                    &cl.polyline,
                    params.entry_mode,
                    entry_extension,
                    diameter_mm,
                    &stock_aabb,
                    &entry_bores,
                    true,
                )?;
                emit_undercut_slot_passes(
                    builder,
                    &plan,
                    &[z_bottom],
                    need_center_first,
                    &lateral_steps,
                    params.climb_direction,
                    safe_z,
                    feed_plunge,
                    feed_lead,
                    feed_cut,
                );
            }
        }

        DovetailMode::ExternalRail => {
            // Collect exterior rail edge contours from `Contour`, `PlanarFace` outer boundary, or `ModeledChamfer`
            let mut rails: Vec<(ArcPolyline2D, bool)> = modeled_rail_contours;
            for feat in &ctx.selection.features {
                match feat {
                    ExtractedFeature25D::Contour {
                        polyline_wcs,
                        side_flip,
                        ..
                    } if !polyline_wcs.segments.is_empty() => {
                        rails.push((polyline_wcs.clone(), *side_flip));
                    }
                    ExtractedFeature25D::PlanarFace {
                        outer_boundary_wcs, ..
                    } if !outer_boundary_wcs.segments.is_empty() => {
                        // CW outer loop -> LEFT (`+1`) is outward into open air!
                        rails.push((outer_boundary_wcs.ensure_orientation(false), false));
                    }
                    _ => {}
                }
            }
            if rails.is_empty() {
                return Err(CamError::EmptySelection {
                    op_name: ctx.op.name.clone(),
                    op_kind: ctx.op.kind.label(),
                });
            }

            // For an external dovetail rail, the cutter approaches from open air outside the top rail
            // edge and steps INWARD progressively toward `r_final = r_at_dovetail_top + radial_stock`,
            // pulling horizontally away from the overhanging lip before any `+Z` retract!
            let alpha_rad = tool_angle_deg.to_radians();
            let undercut_run = dovetail_depth / alpha_rad.tan().max(1e-6);
            let r_final = r_at_dovetail_top + radial_stock;
            let r_clear = r_final + undercut_run + entry_extension.max(r_bottom + 1.0);

            // Build inward lateral offsets `(offset_from_rail_edge, is_finish)` ordered outermost -> innermost
            let mut rail_offsets: Vec<(f64, bool)> = Vec::new();
            let rough_target = r_final + finish_allowance;
            let rough_outer = r_final + undercut_run;
            if rough_outer > rough_target + 1e-4 {
                let span = rough_outer - rough_target;
                let n = (span / radial_stepover.max(0.01)).ceil().max(1.0) as usize;
                let step = span / (n as f64);
                for i in 1..=n {
                    let d = if i == n {
                        rough_target
                    } else {
                        rough_outer - step * (i as f64)
                    };
                    rail_offsets.push((d, false));
                }
            }
            if rail_offsets
                .last()
                .is_none_or(|&(last_d, _)| (last_d - r_final).abs() > 1e-5)
            {
                rail_offsets.push((r_final, true));
            } else if let Some(last) = rail_offsets.last_mut() {
                last.1 = true;
            }
            if params.spring_pass {
                rail_offsets.push((r_final, true));
            }

            for (raw_rail, side_flip) in rails {
                ctx.check_cancel()?;
                // Outward air side is LEFT (`+1`) by default (`-1` when `side_flip`).
                // For Climb milling (CW spindle), material must be on the RIGHT of travel, i.e.,
                // air must be on the LEFT! Flip travel direction when needed to honor `climb_direction`.
                let base_outward_sign = if side_flip { -1.0 } else { 1.0 };
                let want_air_on_left = params.climb_direction;
                let (rail_poly, outward_sign) = if (base_outward_sign > 0.0) == want_air_on_left {
                    (raw_rail, base_outward_sign)
                } else {
                    (raw_rail.reverse(), -base_outward_sign)
                };

                let extended = if rail_poly.closed {
                    rail_poly
                } else {
                    extend_open_polyline(&rail_poly, entry_extension, entry_extension)
                };

                let clear_poly = offset_open_polyline(&extended, outward_sign * r_clear);
                let Some(p_clear_start) = polyline_start_point(&clear_poly) else {
                    continue;
                };
                let Some(p_clear_end) = polyline_end_point(&clear_poly) else {
                    continue;
                };

                for &(off_mm, is_finish) in &rail_offsets {
                    let cut_poly = offset_open_polyline(&extended, outward_sign * off_mm);
                    let (Some(c_start), Some(c_end)) = (
                        polyline_start_point(&cut_poly),
                        polyline_end_point(&cut_poly),
                    ) else {
                        continue;
                    };
                    let role = if is_finish {
                        MotionRole::FinishCut
                    } else {
                        MotionRole::Cut
                    };

                    builder.pass_count += 1;
                    // 1. Rapid above safe open-air clearance point outside rail overhang
                    builder.rapid_to(DVec3::new(p_clear_start.x, p_clear_start.y, safe_z));
                    // 2. Plunge vertically in open air down to dovetail floor Z
                    builder.linear_to(
                        DVec3::new(p_clear_start.x, p_clear_start.y, z_bottom),
                        feed_plunge,
                        MotionRole::Plunge,
                    );
                    // 3. Lead horizontally inward to cutting pass start
                    builder.linear_to(
                        DVec3::new(c_start.x, c_start.y, z_bottom),
                        feed_lead,
                        MotionRole::LeadIn,
                    );
                    // 4. Cut angled dovetail rail pass
                    emit_polyline_at_z(builder, &cut_poly, z_bottom, feed_cut, role);
                    // 5. Pull horizontally outward away from overhanging rail lip BEFORE `+Z` retract!
                    let pull_end = if extended.closed {
                        // Compute exact outward pull from c_end along outward normal
                        let t_end = polyline_end_tangent(&extended).unwrap_or(DVec2::X);
                        let n_out = DVec2::new(-t_end.y, t_end.x) * outward_sign;
                        c_end + n_out * (r_clear - off_mm).max(r_bottom + 1.0)
                    } else {
                        p_clear_end
                    };
                    builder.linear_to(
                        DVec3::new(pull_end.x, pull_end.y, z_bottom),
                        feed_lead,
                        MotionRole::LeadOut,
                    );
                    // 6. Retract vertically `+Z` strictly in open air!
                    builder.rapid_to(DVec3::new(pull_end.x, pull_end.y, safe_z));
                }
            }
        }
    }

    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::sync::atomic::AtomicBool;

    use chip_cad::CadDocument;
    use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, DisplayUnit, ParamLength};
    use chip_sketch::{SketchEntity, SketchEntityEntry, SketchLayer};
    use chip_tools::{CutterGeometry, starter_dovetail_60deg_12mm};

    use crate::{
        BoxAnchor27, CamError, DovetailMode, DovetailParams, FixedBoxXyAlign, FixedBoxZAlign,
        GeometrySelectionItem, HeightReference, HeightSpec, Operation, OperationGenRequest,
        OperationKind, SegmentMotion3D, Setup, StockConfig, StrategyParams, UndercutEntryMode,
        WcsOriginMode, WorkOffsetSlot, generate_operation_toolpath_with_rest,
    };

    #[test]
    fn dovetail_validates_angle_match_depth_neck_and_emits_exact_60deg_flank_offset() {
        let doc = CadDocument::empty();
        let mut setup = Setup::new("s_dove", "Dovetail Test", WorkOffsetSlot::G54);
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

        let mut sk = SketchLayer::new("sk_dove", "Dovetail Centerline", "s_dove");
        sk.entities.push(SketchEntityEntry {
            id: "e_cl".to_string(),
            name: "Centerline".to_string(),
            sub_layer: None,
            is_construction: false,
            entity: SketchEntity::new_polyline(ArcPolyline2D {
                segments: vec![ArcSegment2D::Line {
                    start: DVec2::new(-50.0, 0.0),
                    end: DVec2::new(50.0, 0.0),
                }],
                closed: false,
            }),
        });
        sk.rebuild_regions(None);

        let dove_60 = starter_dovetail_60deg_12mm(); // Ø12, 60°, H=5, Rc=0.2, neck=5.5, neck_len=18
        let mut dove_45 = dove_60.clone();
        if let CutterGeometry::DovetailCutter { side_angle_deg, .. } = &mut dove_45.geometry {
            *side_angle_deg = 45.0;
        }

        let mut op = Operation::new("op_dove", "60° Dovetail", OperationKind::Dovetail);
        op.tool_id = Some(dove_60.id.clone());
        op.selections = vec![GeometrySelectionItem::SketchEntity {
            sketch_id: sk.id.clone(),
            entity_id: "e_cl".to_string(),
            reverse: false,
            side_flip: false,
        }];
        op.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        op.heights.bottom = HeightSpec::new(HeightReference::StockTop, -12.0);
        op.strategy = Some(StrategyParams::Dovetail(DovetailParams {
            mode: DovetailMode::InternalSlot,
            top_throat_width: ParamLength::from_mm(8.5),
            dovetail_depth: ParamLength::from_mm(5.0),
            side_angle_deg: 60.0,
            radial_stepover: ParamLength::from_mm(0.5),
            finish_allowance: ParamLength::from_mm(0.1),
            spring_pass: true,
            entry_mode: UndercutEntryMode::BothEndsThrough,
            entry_extension: ParamLength::from_mm(8.0),
            climb_direction: true,
        }));

        let cancel = AtomicBool::new(false);

        // 1. Mismatching 45° cutter on 60° slot MUST fail cleanly!
        let err_ang = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: std::slice::from_ref(&sk),
            setup: &resolved,
            op: &op,
            tool: &dove_45,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: true,
            upstream_rest: &[],
            carryover_stock: None,
            fixtures: &[],
            shared_avoid_regions: &[],
            cancel: &cancel,
        })
        .unwrap_err();
        assert!(
            matches!(err_ang, CamError::InvalidUndercutGeometry { ref reason } if reason.contains("angle")),
            "expected angle mismatch error, got: {err_ang}"
        );

        // 2. Valid 60° cutter succeeds with exact final lateral offset and zero vertical moves inside stock!
        let tp = generate_operation_toolpath_with_rest(&OperationGenRequest {
            doc: &doc,
            sketches: &[sk],
            setup: &resolved,
            op: &op,
            tool: &dove_60,
            clearing_tool: None,
            display_unit: DisplayUnit::Millimeter,
            is_first_tool_in_setup: true,
            upstream_rest: &[],
            carryover_stock: None,
            fixtures: &[],
            shared_avoid_regions: &[],
            cancel: &cancel,
        })
        .unwrap();

        let expected_top_r = 0.5 * CutterGeometry::dovetail_top_diameter_mm(12.0, 60.0, 5.0, 0.2);
        let expected_max_y = 0.5 * 8.5 - expected_top_r;
        let max_abs_y = tp
            .segments_wcs
            .iter()
            .filter_map(|s| match s.motion {
                SegmentMotion3D::Linear { .. } => Some(s.start_wcs.y.abs().max(s.end_wcs.y.abs())),
                _ => None,
            })
            .fold(0.0_f64, f64::max);
        assert!(
            (max_abs_y - expected_max_y).abs() < 1e-4,
            "expected final dovetail lateral offset {expected_max_y:.4} mm, got {max_abs_y:.4} mm"
        );

        for seg in &tp.segments_wcs {
            if (seg.end_wcs.z - seg.start_wcs.z).abs() > 1e-4 {
                assert!(
                    seg.start_wcs.x.abs() >= 56.0 - 1e-3,
                    "vertical dovetail plunge/retract must occur outside stock roof, got {seg:?}"
                );
            }
        }
    }
}
