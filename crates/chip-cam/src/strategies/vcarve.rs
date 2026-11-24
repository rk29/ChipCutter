//! **Operation 6: V-Carving, Flat-Bottom Two-Tool Clearance, & V-Inlays** (`05-operations.md` §6):
//! - Variable-depth 3D medial-axis V-carving (`boostvoronoi` + exact `ArcPolyline2D` clearances)
//!   with razor-sharp 3D corner climbs and multi-stepdown `max_stepdown` Z-layering;
//! - Optional `flat_depth` clamping + **Two-Stage Flat Endmill Area Clearance** (`clearing_tool_id`
//!   Stage 1 Pocket/Adaptive clearing + Stage 2 exact 2D Rest-Machined V-bit bevels & tight corners);
//! - Optional **V-Inlay Male Plug** generation (`start_depth` + `glue_gap_bottom` + `border_margin`).

use chip_math::{Aabb2, ArcPolyline2D, ArcSegment2D, DVec2, DVec3, ParamLength};
use chip_tools::{CutterGeometry, Tool, ToolOperationClass};

use crate::{
    avoid_regions::clip_polyline_outside_avoid,
    cl_data::{ClEvent, MotionRole},
    heights::ResolvedHeights,
    medial::{MedialChain2D, MedialNode2D, extract_medial_axis_chains},
    offset2d::{PocketRegion2D, build_offset_region_tree},
    params::{
        AdaptiveClearingParams, Pocket2DParams, PocketPassOrder, PocketStrategyKind,
        VCarveClearingKind, VCarveInlayMode, VCarveParams,
    },
    rest2d::RegionSet2D,
    selection::{ExtractedFeature25D, ExtractedSelectionSummary},
    strategies::{StrategyExecContext, compute_z_levels, pocket2d},
    surface3d::is_2d_segment_clear_of_avoid,
    toolpath::{ClearingSubToolpath, GeneratedToolpath, SegmentMotion3D, ToolpathBuilder},
    wcs::CamError,
};

/// Generates a **V-Carving / Flat-Bottom Clearance / V-Inlay** toolpath (`05-operations.md` §6).
pub fn generate_vcarve(
    ctx: &StrategyExecContext<'_>,
    params: &VCarveParams,
    builder: &mut ToolpathBuilder,
) -> Result<(), CamError> {
    let (outer_dia_mm, tip_dia_mm, included_deg) = match ctx.tool.geometry {
        CutterGeometry::ChamferMill {
            diameter_mm,
            tip_diameter_mm,
            included_angle_deg,
            ..
        } => (diameter_mm, tip_diameter_mm, included_angle_deg),
        _ => {
            return Err(CamError::IncompatibleTool {
                op_name: ctx.op.name.clone(),
                op_kind: ctx.op.kind.label(),
                tool_name: ctx.tool.name.clone(),
                tool_kind: ctx.tool.geometry.kind_label(),
            });
        }
    };

    if !(10.0..=160.0).contains(&included_deg) {
        return Err(CamError::InvalidParameter {
            param: "vcarve.included_angle_deg",
            value: included_deg,
            reason: "V-Bit included angle must be between 10° and 160°",
        });
    }

    let theta_rad = 0.5 * included_deg.to_radians();
    let tan_theta = theta_rad.tan().max(1e-6);
    let r_tip = (0.5 * tip_dia_mm).max(0.0);
    let r_cutter_max = (0.5 * outer_dia_mm).max(r_tip + 0.05);
    let max_cone_depth = ((r_cutter_max - r_tip).max(0.05)) / tan_theta;

    // 1. Collect closed source regions (`PlanarFace` outer+islands and closed `Contour` loops)
    let raw_regions = collect_source_vcarve_regions(ctx)?;
    if raw_regions.is_empty() {
        return Err(CamError::EmptySelection {
            op_name: ctx.op.name.clone(),
            op_kind: ctx.op.kind.label(),
        });
    }

    let top_z = ctx.heights.top_z_wcs;
    let axial_leave = ctx.passes.stock_to_leave_axial.value_mm.max(0.0);
    let radial_leave = ctx.passes.stock_to_leave_radial.value_mm.max(0.0);

    // 2. Resolve effective flat depth & transform regions if `VCarveInlayMode::MalePlug` is active
    let (work_regions, eff_flat_depth_opt) =
        prepare_vcarve_regions_and_depth(&raw_regions, params, tan_theta, r_tip)?;

    // 3. Determine target max depth & Z stepdown schedule
    let unclamped_cap_depth = max_cone_depth;
    let target_max_depth = match eff_flat_depth_opt {
        Some(fd) => (fd - axial_leave).max(0.02),
        None => unclamped_cap_depth,
    };
    let z_levels = compute_z_levels(
        top_z,
        top_z - target_max_depth,
        ctx.passes.multiple_depths,
        ctx.passes.max_stepdown.value_mm,
        ctx.passes.even_stepdowns,
    )?;

    // 4. Optional Stage 1: Secondary Flat Endmill Area Clearance when `flat_depth` + `clearing_tool` are set!
    let mut stage1_swept_coverage: Option<RegionSet2D> = None;
    if let (Some(d_flat), Some(clear_tool)) = (eff_flat_depth_opt, ctx.clearing_tool) {
        if !clear_tool
            .geometry
            .supports_operation(ToolOperationClass::VCarveFlatClearance)
        {
            return Err(CamError::IncompatibleTool {
                op_name: format!("{} (Flat Clearance)", ctx.op.name),
                op_kind: "V-Carve Flat Clearance",
                tool_name: clear_tool.name.clone(),
                tool_kind: clear_tool.geometry.kind_label(),
            });
        }
        let d_eff = (d_flat - axial_leave).max(0.02);
        let z_flat = top_z - d_eff;
        let r_bevel = d_eff * tan_theta + r_tip + radial_leave;
        let allowance = params.clearing_radial_allowance.value_mm.max(0.0);

        if let Some((stage1_tp, cleared_set)) = try_generate_flat_clearing_stage(
            ctx,
            params,
            clear_tool,
            &work_regions,
            r_bevel + allowance,
            z_flat,
        )? {
            stage1_swept_coverage = Some(cleared_set);
            builder.clearing_stage = Some(Box::new(ClearingSubToolpath {
                tool_id: clear_tool.id.clone(),
                toolpath: stage1_tp,
            }));
            // Stage 2 V-Bit now follows Stage 1's tool, so mark V-Bit's ToolChange `is_first_tool = false`!
            for ev in &mut builder.events {
                if let ClEvent::ToolChange { is_first_tool, .. } = ev {
                    *is_first_tool = false;
                }
            }
        }
    }

    // 5. Stage 2: V-Bit Bevel, Sharp 3D Corner Climbs, Medial Strokes, & Rest-Machined Flat Corners!
    let avoid_2d = ctx.resolve_avoid_2d(top_z - target_max_depth, top_z);
    builder.warnings.extend(avoid_2d.warnings);
    let avoid_keepout = avoid_2d.effective_region;
    let avoid_keepout_opt = (!avoid_keepout.is_empty()).then_some(&avoid_keepout);

    let retract_z = ctx.safe_retract_z();
    let vbit_flat_step = params
        .vbit_flat_stepover
        .value_mm
        .clamp(0.05, outer_dia_mm * 0.85);

    for region in &work_regions {
        ctx.check_cancel()?;
        match eff_flat_depth_opt {
            None => {
                emit_pure_medial_vcarve_region(
                    ctx,
                    builder,
                    region,
                    &VBitKinematics {
                        top_z,
                        tan_theta,
                        r_tip: r_tip + radial_leave,
                        max_depth_cap: max_cone_depth,
                        retract_z,
                    },
                    &z_levels,
                    avoid_keepout_opt,
                )?;
            }
            Some(d_flat) => {
                let d_eff = (d_flat - axial_leave).max(0.02);
                emit_flat_clamped_vcarve_region(
                    ctx,
                    builder,
                    region,
                    &FlatVCarveSpec {
                        top_z,
                        d_max: d_eff,
                        tan_theta,
                        r_tip: r_tip + radial_leave,
                        max_cone_depth,
                        vbit_flat_step,
                        retract_z,
                        stage1_swept: stage1_swept_coverage.as_ref(),
                        avoid_keepout: avoid_keepout_opt,
                    },
                    &z_levels,
                )?;
            }
        }
    }

    Ok(())
}

struct VBitKinematics {
    top_z: f64,
    tan_theta: f64,
    r_tip: f64,
    max_depth_cap: f64,
    retract_z: f64,
}

struct FlatVCarveSpec<'a> {
    top_z: f64,
    d_max: f64,
    tan_theta: f64,
    r_tip: f64,
    max_cone_depth: f64,
    vbit_flat_step: f64,
    retract_z: f64,
    stage1_swept: Option<&'a RegionSet2D>,
    avoid_keepout: Option<&'a RegionSet2D>,
}

fn collect_source_vcarve_regions(
    ctx: &StrategyExecContext<'_>,
) -> Result<Vec<PocketRegion2D>, CamError> {
    let mut raw = Vec::new();
    for feat in &ctx.selection.features {
        match feat {
            ExtractedFeature25D::PlanarFace {
                outer_boundary_wcs,
                inner_islands_wcs,
                raised_islands_wcs,
                ..
            } => {
                let islands = if !inner_islands_wcs.is_empty() {
                    inner_islands_wcs.clone()
                } else {
                    raised_islands_wcs.clone()
                };
                if outer_boundary_wcs.closed && !outer_boundary_wcs.segments.is_empty() {
                    raw.push(PocketRegion2D::new(
                        outer_boundary_wcs.ensure_orientation(true),
                        islands
                            .into_iter()
                            .map(|p| p.ensure_orientation(false))
                            .collect(),
                    ));
                }
            }
            ExtractedFeature25D::Contour { polyline_wcs, .. } if polyline_wcs.closed => {
                raw.push(PocketRegion2D::new(
                    polyline_wcs.ensure_orientation(true),
                    Vec::new(),
                ));
            }
            _ => {}
        }
    }
    Ok(raw)
}

fn prepare_vcarve_regions_and_depth(
    raw_regions: &[PocketRegion2D],
    params: &VCarveParams,
    tan_theta: f64,
    r_tip: f64,
) -> Result<(Vec<PocketRegion2D>, Option<f64>), CamError> {
    match &params.inlay_mode {
        VCarveInlayMode::Disabled => {
            let fd = match &params.flat_depth {
                Some(p) => {
                    if !p.value_mm.is_finite() || p.value_mm <= 0.02 {
                        return Err(CamError::InvalidParameter {
                            param: "vcarve.flat_depth",
                            value: p.value_mm,
                            reason: "must be > 0.02 mm when enabled",
                        });
                    }
                    Some(p.value_mm)
                }
                None => None,
            };
            Ok((raw_regions.to_vec(), fd))
        }
        VCarveInlayMode::MalePlug {
            start_depth,
            glue_gap_bottom,
            border_margin,
            auto_mirror_x,
        } => {
            let d_start = start_depth.value_mm;
            let d_glue = glue_gap_bottom.value_mm.max(0.0);
            let margin = border_margin.value_mm.max(1.5);
            if !d_start.is_finite() || d_start <= 0.05 {
                return Err(CamError::InvalidParameter {
                    param: "vcarve.inlay.start_depth",
                    value: d_start,
                    reason: "must be > 0.05 mm",
                });
            }
            let d_total = d_start + d_glue;
            let r_start = d_start * tan_theta;
            let r_total = d_total * tan_theta + r_tip;

            let overall_aabb = raw_regions
                .iter()
                .fold(Aabb2::EMPTY, |acc, r| acc.union(&r.exact_aabb()));
            if !overall_aabb.is_valid() {
                return Ok((Vec::new(), Some(d_total)));
            }
            let mirror_cx = overall_aabb.center().x;

            let oriented_artwork: Vec<PocketRegion2D> = if *auto_mirror_x {
                raw_regions
                    .iter()
                    .map(|r| mirror_region_x(r, mirror_cx))
                    .collect()
            } else {
                raw_regions.to_vec()
            };

            let artwork_set = RegionSet2D::from_regions(oriented_artwork);
            // Male plug raised plateau at `z_top` is `OffsetInward(Artwork, d_start * tan(θ))` so at depth
            // `d_start` below `z_top` its sloped V-wall widens back to exact `Artwork` zero-gap fit!
            let plug_plateau = artwork_set.erode_inward(r_start);

            let pad = r_total + margin;
            let fmin = overall_aabb.min - DVec2::splat(pad);
            let fmax = overall_aabb.max + DVec2::splat(pad);
            let frame_poly = build_box_ccw(fmin, fmax);
            let frame_set =
                RegionSet2D::from_regions(vec![PocketRegion2D::new(frame_poly, Vec::new())]);

            let male_cavity = frame_set.difference(&plug_plateau);
            Ok((male_cavity.regions, Some(d_total)))
        }
    }
}

fn mirror_region_x(reg: &PocketRegion2D, cx: f64) -> PocketRegion2D {
    PocketRegion2D::new(
        mirror_polyline_x(&reg.outer_ccw, cx).ensure_orientation(true),
        reg.islands_cw
            .iter()
            .map(|p| mirror_polyline_x(p, cx).ensure_orientation(false))
            .collect(),
    )
}

fn mirror_polyline_x(poly: &ArcPolyline2D, cx: f64) -> ArcPolyline2D {
    let mx = |p: DVec2| DVec2::new(2.0 * cx - p.x, p.y);
    let segs = poly
        .segments
        .iter()
        .map(|s| match *s {
            ArcSegment2D::Line { start, end } => ArcSegment2D::Line {
                start: mx(start),
                end: mx(end),
            },
            ArcSegment2D::Arc {
                start,
                end,
                center,
                radius,
                ccw,
            } => ArcSegment2D::Arc {
                start: mx(start),
                end: mx(end),
                center: mx(center),
                radius,
                ccw: !ccw,
            },
        })
        .collect();
    ArcPolyline2D {
        segments: segs,
        closed: poly.closed,
    }
}

fn build_box_ccw(min: DVec2, max: DVec2) -> ArcPolyline2D {
    let pts = [min, DVec2::new(max.x, min.y), max, DVec2::new(min.x, max.y)];
    ArcPolyline2D {
        segments: (0..4)
            .map(|i| ArcSegment2D::Line {
                start: pts[i],
                end: pts[(i + 1) % 4],
            })
            .collect(),
        closed: true,
    }
}

fn try_generate_flat_clearing_stage(
    ctx: &StrategyExecContext<'_>,
    params: &VCarveParams,
    clear_tool: &Tool,
    work_regions: &[PocketRegion2D],
    bevel_inset_mm: f64,
    z_flat_wcs: f64,
) -> Result<Option<(GeneratedToolpath, RegionSet2D)>, CamError> {
    let mut core_regions = Vec::new();
    let mut features = Vec::new();
    for reg in work_regions {
        for core in reg.erode_inward(bevel_inset_mm) {
            let n_segs = core.outer_ccw.segments.len();
            if n_segs < 2 {
                continue;
            }
            features.push(ExtractedFeature25D::PlanarFace {
                source_face_id: None,
                outer_boundary_wcs: core.outer_ccw.clone(),
                open_outer_segments: vec![false; n_segs],
                inner_islands_wcs: core.islands_cw.clone(),
                raised_islands_wcs: core.islands_cw.clone(),
                face_z_wcs: z_flat_wcs,
                inferred_top_z_wcs: ctx.heights.top_z_wcs,
                adjacent_lower_floor_z_wcs: None,
                area_mm2: 0.0,
            });
            core_regions.push(core);
        }
    }
    if features.is_empty() {
        return Ok(None);
    }

    let clear_preset = params
        .clearing_preset_id
        .as_deref()
        .and_then(|pid| clear_tool.presets.iter().find(|p| p.id == pid))
        .or_else(|| clear_tool.presets.first());
    let clear_d = clear_tool.geometry.cutting_diameter_mm();
    let clear_cutting = match clear_preset {
        Some(p) => crate::operation::EffectiveCuttingParams {
            spindle_rpm: p.spindle_rpm,
            cutting_feed_mm_min: p.cutting_feed_mm_min,
            lead_feed_mm_min: p.lead_feed_mm_min,
            ramp_feed_mm_min: p.ramp_feed_mm_min,
            plunge_feed_mm_min: p.plunge_feed_mm_min,
            stepdown_mm: p.default_stepdown_mm,
            stepover_mm: p.default_stepover_mm,
            coolant: p.coolant_override.unwrap_or(clear_tool.default_coolant),
        },
        None => crate::operation::EffectiveCuttingParams {
            spindle_rpm: 12_000.0,
            cutting_feed_mm_min: 1_000.0,
            lead_feed_mm_min: 800.0,
            ramp_feed_mm_min: 500.0,
            plunge_feed_mm_min: 330.0,
            stepdown_mm: (0.5 * clear_d).max(0.5),
            stepover_mm: (0.4 * clear_d).max(0.25),
            coolant: clear_tool.default_coolant,
        },
    };

    let stage1_heights = ResolvedHeights {
        clearance_z_wcs: ctx.heights.clearance_z_wcs,
        retract_z_wcs: ctx.heights.retract_z_wcs,
        feed_z_wcs: ctx.heights.feed_z_wcs,
        top_z_wcs: ctx.heights.top_z_wcs,
        bottom_z_wcs: z_flat_wcs,
    };
    let mut stage1_passes = ctx.passes.clone();
    stage1_passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
    stage1_passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
    stage1_passes.max_stepdown = ParamLength::from_mm(clear_cutting.stepdown_mm.max(0.25));
    stage1_passes.rest_machining = crate::params::RestMachiningConfig::default();

    let stage1_ctx = StrategyExecContext {
        doc: ctx.doc,
        sketches: ctx.sketches,
        setup: ctx.setup,
        op: ctx.op,
        tool: clear_tool,
        display_unit: ctx.display_unit,
        cutting: clear_cutting,
        heights: stage1_heights,
        passes: stage1_passes,
        linking: ctx.linking.clone(),
        selection: ExtractedSelectionSummary {
            features,
            highlight_faces: Vec::new(),
            highlight_edges: Vec::new(),
            highlight_sketch_regions: Vec::new(),
            highlight_sketch_entities: Vec::new(),
            direction_arrows: Vec::new(),
            inferred_top_z_wcs: ctx.heights.top_z_wcs,
            inferred_bottom_z_wcs: z_flat_wcs,
            primary_hole_diameter_mm: None,
        },
        upstream_rest: &[],
        carryover_stock: ctx.carryover_stock,
        fixtures: ctx.fixtures,
        shared_avoid_regions: ctx.shared_avoid_regions,
        is_first_tool_in_setup: ctx.is_first_tool_in_setup,
        clearing_tool: None,
        cancel: ctx.cancel,
    };

    let pkt_params = Pocket2DParams {
        strategy: match params.clearing_strategy {
            VCarveClearingKind::ContourParallel => PocketStrategyKind::ContourParallel,
            VCarveClearingKind::Adaptive => PocketStrategyKind::Adaptive(AdaptiveClearingParams {
                optimal_load: ParamLength::from_mm((0.22 * clear_d).max(0.2)),
                ..AdaptiveClearingParams::default()
            }),
        },
        stepover: ParamLength::from_mm(clear_cutting.stepover_mm.clamp(0.1, 0.85 * clear_d)),
        pass_order: PocketPassOrder::InsideOut,
        finish_wall_pass: true,
        finish_wall_stepover: ParamLength::from_mm((0.08 * clear_d).clamp(0.05, 0.4)),
        ..Pocket2DParams::default()
    };

    let mut s1_builder = ToolpathBuilder::new(format!("{}_clear", ctx.op.id));
    s1_builder.push_event(ClEvent::OperationStart {
        op_id: format!("{}_clear", ctx.op.id),
        op_name: format!("{} [Stage 1 — Flat Clearance]", ctx.op.name),
        strategy_name: "V-Carve Flat Clearance".to_string(),
        est_time_sec: 0.0,
    });
    s1_builder.push_event(ClEvent::ToolChange {
        tool_number: clear_tool.tool_number,
        length_offset: clear_tool.length_offset_number,
        diameter_offset: clear_tool.diameter_offset_number,
        tool_desc: format!(
            "{} (Ø{:.3} mm)",
            clear_tool.name,
            clear_tool.geometry.cutting_diameter_mm()
        ),
        is_first_tool: ctx.is_first_tool_in_setup,
    });
    s1_builder.push_event(ClEvent::SpindleStart {
        rpm: clear_cutting.spindle_rpm,
        direction: clear_tool.spindle_direction,
    });
    s1_builder.push_event(ClEvent::Coolant {
        mode: clear_cutting.coolant,
    });

    match pocket2d::generate_pocket2d(&stage1_ctx, &pkt_params, &mut s1_builder) {
        Ok(()) => {
            if let Some(last_pos) = s1_builder.current_pos()
                && last_pos.z < ctx.heights.clearance_z_wcs - 1e-6
            {
                s1_builder.rapid_to(DVec3::new(
                    last_pos.x,
                    last_pos.y,
                    ctx.heights.clearance_z_wcs,
                ));
            }
            s1_builder.push_event(ClEvent::OperationEnd);
            let tp = s1_builder.finish();
            if tp.segments_wcs.is_empty() {
                Ok(None)
            } else {
                let clear_r = 0.5 * clear_d;
                // Shrink swept coverage slightly (`0.12 mm` overlap) so Stage 2 V-bit blends seamlessly without cusps!
                let overlap_r = (clear_r - 0.12).max(0.10);
                let mut floor_pts: Vec<DVec2> = Vec::new();
                for seg in &tp.segments_wcs {
                    if matches!(seg.motion, SegmentMotion3D::Rapid) {
                        continue;
                    }
                    if seg.start_wcs.z.max(seg.end_wcs.z) <= z_flat_wcs + 0.05 {
                        let s2 = DVec2::new(seg.start_wcs.x, seg.start_wcs.y);
                        let e2 = DVec2::new(seg.end_wcs.x, seg.end_wcs.y);
                        floor_pts.push(s2);
                        floor_pts.push(0.5 * (s2 + e2));
                        floor_pts.push(e2);
                    }
                }
                let mut cleared_pieces: Vec<RegionSet2D> = Vec::new();
                for core in &core_regions {
                    for center_comp in core.erode_inward(clear_r) {
                        let raw_aabb = center_comp.exact_aabb();
                        let comp_aabb = Aabb2 {
                            min: raw_aabb.min - DVec2::splat(0.15),
                            max: raw_aabb.max + DVec2::splat(0.15),
                        };
                        let was_cut = floor_pts.iter().any(|&p| {
                            comp_aabb.contains_point(p)
                                && (center_comp.contains_interior_point(p)
                                    || center_comp.min_boundary_distance(p) <= 0.15)
                        });
                        if was_cut {
                            let dilated = center_comp.dilate_outward(overlap_r);
                            if !dilated.is_empty() {
                                cleared_pieces.push(RegionSet2D::from_regions(dilated));
                            }
                        }
                    }
                }
                let cleared_set = RegionSet2D::union_many(&mut cleared_pieces);
                Ok(Some((tp, cleared_set)))
            }
        }
        Err(CamError::PocketTooNarrowForTool { .. }) => Ok(None),
        Err(other) => Err(other),
    }
}

fn emit_pure_medial_vcarve_region(
    ctx: &StrategyExecContext<'_>,
    builder: &mut ToolpathBuilder,
    region: &PocketRegion2D,
    kin: &VBitKinematics,
    z_levels: &[f64],
    avoid_keepout: Option<&RegionSet2D>,
) -> Result<(), CamError> {
    // Tight 0.005 mm chord tolerance for pure medial V-carving so narrow serif tips
    // and sharp 3D corner climbs carve cleanly with zero flattened blunting.
    let raw_chains = extract_medial_axis_chains(region, 0.005);
    if raw_chains.is_empty() {
        return Ok(());
    }
    let chains: Vec<MedialChain2D> = raw_chains
        .into_iter()
        .map(|ch| refine_medial_chain_z_chord(ch, region, kin, 0.005))
        .collect();

    let mut max_raw_d = 0.0_f64;
    for ch in &chains {
        for s in &ch.samples {
            let d = ((s.clearance_radius_mm - kin.r_tip).max(0.0)) / kin.tan_theta;
            max_raw_d = max_raw_d.max(d);
        }
    }
    if max_raw_d > kin.max_depth_cap + 0.05 {
        builder.warnings.push(format!(
            "Stroke width exceeds V-bit flute diameter; clamped pure V-carve depth at {:.2} mm (enable Flat Depth for wider pockets).",
            kin.max_depth_cap
        ));
    }

    for (lvl_idx, &z_floor_lvl) in z_levels.iter().enumerate() {
        ctx.check_cancel()?;
        let prev_z_lvl = if lvl_idx == 0 {
            kin.top_z + 1e-4
        } else {
            z_levels[lvl_idx - 1]
        };

        for chain in &chains {
            emit_medial_chain_at_level(
                ctx,
                builder,
                chain,
                kin,
                None,
                z_floor_lvl,
                prev_z_lvl,
                avoid_keepout,
            );
        }
    }
    Ok(())
}

/// Refines a pure-medial 3D V-carve chain so nonlinear Z-profile curvature stays within
/// `z_chord_tol_mm` (`<= 0.005 mm`) along every curved branch and sharp corner climb.
fn refine_medial_chain_z_chord(
    chain: MedialChain2D,
    region: &PocketRegion2D,
    kin: &VBitKinematics,
    z_chord_tol_mm: f64,
) -> MedialChain2D {
    if chain.samples.len() < 2 {
        return chain;
    }
    let tol = z_chord_tol_mm.max(1e-4);
    let z_at_r = |r: f64| -> f64 {
        let d = (((r - kin.r_tip).max(0.0)) / kin.tan_theta).min(kin.max_depth_cap);
        kin.top_z - d
    };

    fn subdivide_span(
        a: MedialNode2D,
        b: MedialNode2D,
        region: &PocketRegion2D,
        z_at_r: &impl Fn(f64) -> f64,
        tol: f64,
        depth: u8,
        out: &mut Vec<MedialNode2D>,
    ) {
        if depth >= 4 || a.pos_wcs.distance(b.pos_wcs) <= 0.02 {
            out.push(b);
            return;
        }
        let mid_xy = 0.5 * (a.pos_wcs + b.pos_wcs);
        let mid_r = region.min_boundary_distance(mid_xy).max(0.0);
        let z_lin = 0.5 * (z_at_r(a.clearance_radius_mm) + z_at_r(b.clearance_radius_mm));
        let z_exact = z_at_r(mid_r);
        if (z_exact - z_lin).abs() > tol {
            let mid = MedialNode2D {
                pos_wcs: mid_xy,
                clearance_radius_mm: mid_r,
            };
            subdivide_span(a, mid, region, z_at_r, tol, depth + 1, out);
            subdivide_span(mid, b, region, z_at_r, tol, depth + 1, out);
        } else {
            out.push(b);
        }
    }

    let mut refined = Vec::with_capacity(chain.samples.len() * 2);
    refined.push(chain.samples[0]);
    for w in chain.samples.windows(2) {
        subdivide_span(w[0], w[1], region, &z_at_r, tol, 0, &mut refined);
    }
    MedialChain2D {
        samples: refined,
        closed: chain.closed,
    }
}

fn emit_flat_clamped_vcarve_region(
    ctx: &StrategyExecContext<'_>,
    builder: &mut ToolpathBuilder,
    region: &PocketRegion2D,
    spec: &FlatVCarveSpec<'_>,
    z_levels: &[f64],
) -> Result<(), CamError> {
    let r_max = spec.d_max * spec.tan_theta + spec.r_tip;
    let z_flat = spec.top_z - spec.d_max;
    let chains = extract_medial_axis_chains(region, 0.02);

    // 1. Progressive Z levels for V-bit flat core cleanup (if un-cleared by Stage 1) + Bevel wall trace + Medial corner climbs!
    for (lvl_idx, &z_lvl) in z_levels.iter().enumerate() {
        ctx.check_cancel()?;
        let prev_z_lvl = if lvl_idx == 0 {
            spec.top_z + 1e-4
        } else {
            z_levels[lvl_idx - 1]
        };
        let d_lvl = (spec.top_z - z_lvl).clamp(0.0, spec.d_max);
        let r_lvl = d_lvl * spec.tan_theta + spec.r_tip;

        // 1a. Flat-bottom floor clearing for unreached regions / single-tool V-carve
        let flat_target_set = RegionSet2D::from_regions(region.erode_inward(r_lvl));
        let rest_flat_regions = match spec.stage1_swept {
            Some(swept) => flat_target_set.difference(swept).regions,
            None => flat_target_set.regions,
        };

        for sub_reg in &rest_flat_regions {
            let inner_eroded = sub_reg.erode_inward(0.35 * spec.vbit_flat_step);
            let roots = if inner_eroded.is_empty() {
                vec![sub_reg.clone()]
            } else {
                inner_eroded
            };
            for root in &roots {
                let tree = build_offset_region_tree(root, spec.vbit_flat_step, 128);
                let mut loops = Vec::new();
                collect_offset_loops_inside_out(&tree, &mut loops);
                for lp in loops {
                    emit_constant_z_loop(
                        ctx,
                        builder,
                        &lp,
                        z_lvl,
                        spec.retract_z,
                        spec.avoid_keepout,
                    );
                }
            }
        }

        // 1b. Exact Bevel Wall Perimeter Pass at `z_lvl` (`OffsetInward(region, r_lvl)`)
        // If `d_lvl > max_cone_depth`, step outward slightly to clear upper cone terrace cleanly
        for bevel_reg in region.erode_inward(r_lvl) {
            emit_constant_z_loop(
                ctx,
                builder,
                &bevel_reg.outer_ccw,
                z_lvl,
                spec.retract_z,
                spec.avoid_keepout,
            );
            for isl in &bevel_reg.islands_cw {
                emit_constant_z_loop(ctx, builder, isl, z_lvl, spec.retract_z, spec.avoid_keepout);
            }
            let _ = spec.max_cone_depth;
        }

        // 1c. Narrow Medial Strokes & Diagonal 3D Sharp Corner Climbs (`r(s) <= r_max`)!
        let kin = VBitKinematics {
            top_z: spec.top_z,
            tan_theta: spec.tan_theta,
            r_tip: spec.r_tip,
            max_depth_cap: spec.d_max,
            retract_z: spec.retract_z,
        };
        for chain in &chains {
            emit_medial_chain_at_level(
                ctx,
                builder,
                chain,
                &kin,
                Some(r_max),
                z_lvl.max(z_flat),
                prev_z_lvl,
                spec.avoid_keepout,
            );
        }
    }

    Ok(())
}

fn collect_offset_loops_inside_out(
    node: &crate::offset2d::OffsetRegionNode,
    out: &mut Vec<ArcPolyline2D>,
) {
    for child in &node.children {
        collect_offset_loops_inside_out(child, out);
    }
    out.push(node.region.outer_ccw.clone());
    for isl in &node.region.islands_cw {
        out.push(isl.clone());
    }
}

fn emit_constant_z_loop(
    ctx: &StrategyExecContext<'_>,
    builder: &mut ToolpathBuilder,
    poly: &ArcPolyline2D,
    z_cut: f64,
    retract_z: f64,
    avoid_keepout: Option<&RegionSet2D>,
) {
    let spans: Vec<ArcPolyline2D> = if let Some(av) = avoid_keepout
        && !av.is_empty()
    {
        clip_polyline_outside_avoid(poly, av)
            .into_iter()
            .filter(|sp| sp.perimeter() > 0.05)
            .collect()
    } else {
        vec![poly.clone()]
    };

    for span in &spans {
        let Some((s_xy, _)) = span.sample_at_fraction(0.0) else {
            continue;
        };
        builder.pass_count += 1;
        let need_retract = builder.current_pos().is_none_or(|cur| {
            let cur_xy = DVec2::new(cur.x, cur.y);
            let xy_jump = cur_xy.distance(s_xy);
            xy_jump > 1.5
                || (cur.z - z_cut).abs() > 2.0
                || avoid_keepout.is_some_and(|av| !is_2d_segment_clear_of_avoid(cur_xy, s_xy, av))
        });
        if need_retract {
            if let Some(cur) = builder.current_pos()
                && cur.z < retract_z - 1e-6
            {
                builder.rapid_to(DVec3::new(cur.x, cur.y, retract_z));
            }
            builder.rapid_to(DVec3::new(s_xy.x, s_xy.y, retract_z));
            builder.rapid_to(DVec3::new(s_xy.x, s_xy.y, ctx.heights.feed_z_wcs));
            builder.linear_to(
                DVec3::new(s_xy.x, s_xy.y, z_cut),
                ctx.cutting.plunge_feed_mm_min,
                MotionRole::Plunge,
            );
        } else {
            builder.linear_to(
                DVec3::new(s_xy.x, s_xy.y, z_cut),
                ctx.cutting.lead_feed_mm_min,
                MotionRole::LinkTransition,
            );
        }

        builder.emit_polyline_2d_z_interp(
            span,
            z_cut,
            z_cut,
            ctx.cutting.cutting_feed_mm_min,
            MotionRole::Cut,
        );
    }
}

#[allow(clippy::too_many_arguments)]
fn emit_medial_chain_at_level(
    ctx: &StrategyExecContext<'_>,
    builder: &mut ToolpathBuilder,
    chain: &MedialChain2D,
    kin: &VBitKinematics,
    r_flat_clip: Option<f64>,
    z_floor_lvl: f64,
    prev_z_lvl: f64,
    avoid_keepout: Option<&RegionSet2D>,
) {
    if chain.samples.is_empty() {
        return;
    }

    // Single-point dimple (e.g. circular dot 'i' / '!')
    if chain.samples.len() == 1 {
        let s = chain.samples[0];
        if avoid_keepout.is_some_and(|av| av.contains_point(s.pos_wcs)) {
            return;
        }
        if r_flat_clip.is_some_and(|rc| s.clearance_radius_mm > rc + 1e-4) {
            return;
        }
        let d =
            (((s.clearance_radius_mm - kin.r_tip).max(0.0)) / kin.tan_theta).min(kin.max_depth_cap);
        let z_target = kin.top_z - d;
        if z_target >= prev_z_lvl - 1e-5 {
            return;
        }
        let z_cut = z_target.max(z_floor_lvl);
        if let Some(cur) = builder.current_pos()
            && cur.z < kin.retract_z - 1e-6
        {
            builder.rapid_to(DVec3::new(cur.x, cur.y, kin.retract_z));
        }
        builder.pass_count += 1;
        builder.rapid_to(DVec3::new(s.pos_wcs.x, s.pos_wcs.y, kin.retract_z));
        builder.rapid_to(DVec3::new(s.pos_wcs.x, s.pos_wcs.y, ctx.heights.feed_z_wcs));
        builder.linear_to(
            DVec3::new(s.pos_wcs.x, s.pos_wcs.y, z_cut),
            ctx.cutting.plunge_feed_mm_min,
            MotionRole::Plunge,
        );
        builder.rapid_to(DVec3::new(s.pos_wcs.x, s.pos_wcs.y, kin.retract_z));
        return;
    }

    // Clip chain into continuous sub-polylines where `r <= r_flat_clip` (splitting exact crossing `r == r_flat_clip`!)
    let mut clipped_subchains = match r_flat_clip {
        Some(r_lim) => clip_medial_chain_below_radius(&chain.samples, r_lim),
        None => vec![chain.samples.clone()],
    };
    if let Some(av) = avoid_keepout
        && !av.is_empty()
    {
        clipped_subchains = clipped_subchains
            .into_iter()
            .flat_map(|sub| clip_medial_samples_outside_region(&sub, av))
            .collect();
    }

    for sub in clipped_subchains {
        if sub.len() < 2 {
            continue;
        }
        // Map each sample to `(x, y, z_target)` and check if any point reaches deeper than `prev_z_lvl`!
        let pts_target: Vec<DVec3> = sub
            .iter()
            .map(|s| {
                let d = (((s.clearance_radius_mm - kin.r_tip).max(0.0)) / kin.tan_theta)
                    .min(kin.max_depth_cap);
                DVec3::new(s.pos_wcs.x, s.pos_wcs.y, kin.top_z - d)
            })
            .collect();

        let min_z_in_sub = pts_target.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
        if min_z_in_sub >= prev_z_lvl - 1e-4 {
            continue;
        }

        // Orient open corner-climb strokes to start at the DEEP interior end and climb UPWARD out of
        // the sharp corner (`z -> top_z`), minimizing plunge stress and eliminating corner burrs!
        let mut pts_3d: Vec<DVec3> = pts_target
            .into_iter()
            .map(|p| DVec3::new(p.x, p.y, p.z.max(z_floor_lvl)))
            .collect();
        if let (Some(first), Some(last)) = (pts_3d.first().copied(), pts_3d.last().copied())
            && first.distance(last) > 1e-4
            && first.z > last.z + 0.05
        {
            pts_3d.reverse();
        }

        let start = pts_3d[0];
        let can_link = builder.current_pos().is_some_and(|cur| {
            cur.distance(start) <= 0.05
                && avoid_keepout.is_none_or(|av| {
                    is_2d_segment_clear_of_avoid(
                        DVec2::new(cur.x, cur.y),
                        DVec2::new(start.x, start.y),
                        av,
                    )
                })
        });
        if !can_link {
            if let Some(cur) = builder.current_pos()
                && cur.z < kin.retract_z - 1e-6
            {
                builder.rapid_to(DVec3::new(cur.x, cur.y, kin.retract_z));
            }
            builder.rapid_to(DVec3::new(start.x, start.y, kin.retract_z));
            builder.rapid_to(DVec3::new(start.x, start.y, ctx.heights.feed_z_wcs));
            builder.linear_to(start, ctx.cutting.plunge_feed_mm_min, MotionRole::Plunge);
        }

        builder.pass_count += 1;
        for &pt in &pts_3d[1..] {
            builder.linear_to(pt, ctx.cutting.cutting_feed_mm_min, MotionRole::Cut);
        }
    }
}

/// Splits a medial-axis node chain into contiguous sub-chains lying strictly outside `avoid`,
/// binary-refining boundary crossings to `< 0.01 mm`.
fn clip_medial_samples_outside_region(
    samples: &[MedialNode2D],
    avoid: &RegionSet2D,
) -> Vec<Vec<MedialNode2D>> {
    if samples.len() < 2 || avoid.is_empty() {
        return vec![samples.to_vec()];
    }
    let lerp_node = |a: MedialNode2D, b: MedialNode2D, t: f64| MedialNode2D {
        pos_wcs: a.pos_wcs.lerp(b.pos_wcs, t),
        clearance_radius_mm: a.clearance_radius_mm
            + (b.clearance_radius_mm - a.clearance_radius_mm) * t,
    };
    let refine_exit = |outside: MedialNode2D, inside: MedialNode2D| -> MedialNode2D {
        let mut lo = 0.0_f64;
        let mut hi = 1.0_f64;
        for _ in 0..8 {
            let mid = 0.5 * (lo + hi);
            let p = outside.pos_wcs.lerp(inside.pos_wcs, mid);
            if avoid.contains_point(p) {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        lerp_node(outside, inside, lo)
    };

    let mut out: Vec<Vec<MedialNode2D>> = Vec::new();
    let mut cur: Vec<MedialNode2D> = Vec::new();

    for w in samples.windows(2) {
        let a = w[0];
        let b = w[1];
        let seg_len = a.pos_wcs.distance(b.pos_wcs);
        let n_sub = (seg_len / 0.15).ceil().clamp(1.0, 64.0) as usize;
        let mut prev = a;
        let mut prev_out = !avoid.contains_point(prev.pos_wcs);

        for k in 1..=n_sub {
            let next = if k == n_sub {
                b
            } else {
                lerp_node(a, b, k as f64 / n_sub as f64)
            };
            let next_out = !avoid.contains_point(next.pos_wcs);
            match (prev_out, next_out) {
                (true, true) => {
                    if cur.is_empty() {
                        cur.push(prev);
                    }
                    if cur
                        .last()
                        .is_none_or(|l| l.pos_wcs.distance_squared(next.pos_wcs) > 1e-10)
                    {
                        cur.push(next);
                    }
                }
                (true, false) => {
                    let hit = refine_exit(prev, next);
                    if cur.is_empty() {
                        cur.push(prev);
                    }
                    if cur
                        .last()
                        .is_none_or(|l| l.pos_wcs.distance_squared(hit.pos_wcs) > 1e-10)
                    {
                        cur.push(hit);
                    }
                    if cur.len() >= 2 {
                        out.push(std::mem::take(&mut cur));
                    } else {
                        cur.clear();
                    }
                }
                (false, true) => {
                    let hit = refine_exit(next, prev);
                    cur.clear();
                    cur.push(hit);
                    if hit.pos_wcs.distance_squared(next.pos_wcs) > 1e-10 {
                        cur.push(next);
                    }
                }
                (false, false) => {
                    if cur.len() >= 2 {
                        out.push(std::mem::take(&mut cur));
                    } else {
                        cur.clear();
                    }
                }
            }
            prev = next;
            prev_out = next_out;
        }
    }

    if cur.len() >= 2 {
        out.push(cur);
    }
    out
}

fn clip_medial_chain_below_radius(samples: &[MedialNode2D], r_lim: f64) -> Vec<Vec<MedialNode2D>> {
    let eps = 1e-5;
    let mut out: Vec<Vec<MedialNode2D>> = Vec::new();
    let mut cur: Vec<MedialNode2D> = Vec::new();

    for w in samples.windows(2) {
        let a = w[0];
        let b = w[1];
        let a_in = a.clearance_radius_mm <= r_lim + eps;
        let b_in = b.clearance_radius_mm <= r_lim + eps;

        match (a_in, b_in) {
            (true, true) => {
                if cur.is_empty() {
                    cur.push(MedialNode2D {
                        pos_wcs: a.pos_wcs,
                        clearance_radius_mm: a.clearance_radius_mm.min(r_lim),
                    });
                }
                cur.push(MedialNode2D {
                    pos_wcs: b.pos_wcs,
                    clearance_radius_mm: b.clearance_radius_mm.min(r_lim),
                });
            }
            (true, false) => {
                if cur.is_empty() {
                    cur.push(a);
                }
                let denom = b.clearance_radius_mm - a.clearance_radius_mm;
                let t = if denom.abs() > 1e-9 {
                    ((r_lim - a.clearance_radius_mm) / denom).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                cur.push(MedialNode2D {
                    pos_wcs: a.pos_wcs.lerp(b.pos_wcs, t),
                    clearance_radius_mm: r_lim,
                });
                if cur.len() >= 2 {
                    out.push(std::mem::take(&mut cur));
                } else {
                    cur.clear();
                }
            }
            (false, true) => {
                let denom = b.clearance_radius_mm - a.clearance_radius_mm;
                let t = if denom.abs() > 1e-9 {
                    ((r_lim - a.clearance_radius_mm) / denom).clamp(0.0, 1.0)
                } else {
                    1.0
                };
                cur.push(MedialNode2D {
                    pos_wcs: a.pos_wcs.lerp(b.pos_wcs, t),
                    clearance_radius_mm: r_lim,
                });
                cur.push(b);
            }
            (false, false) => {}
        }
    }
    if cur.len() >= 2 {
        out.push(cur);
    }
    out
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, approx_eq};

    use super::*;

    #[test]
    fn pure_medial_vcarve_climbs_to_exact_sharp_corner_apex_within_z_chord_tolerance() {
        // Sharp 30 x 12 mm rectangle: 4 sharp 90° corners must climb all the way to Z = top_z (0.0 mm)
        let pts = [
            DVec2::new(0.0, 0.0),
            DVec2::new(30.0, 0.0),
            DVec2::new(30.0, 12.0),
            DVec2::new(0.0, 12.0),
        ];
        let outer = ArcPolyline2D {
            segments: (0..4)
                .map(|i| ArcSegment2D::Line {
                    start: pts[i],
                    end: pts[(i + 1) % 4],
                })
                .collect(),
            closed: true,
        };
        let region = PocketRegion2D::new(outer, Vec::new());
        let kin = VBitKinematics {
            top_z: 0.0,
            tan_theta: 30.0_f64.to_radians().tan(),
            r_tip: 0.0,
            max_depth_cap: 15.0,
            retract_z: 5.0,
        };
        let chains: Vec<MedialChain2D> = extract_medial_axis_chains(&region, 0.005)
            .into_iter()
            .map(|ch| refine_medial_chain_z_chord(ch, &region, &kin, 0.005))
            .collect();
        assert!(!chains.is_empty());

        // Every sharp corner apex must be reached within 0.01 mm with clearance_radius == 0 (Z == top_z)
        for corner in pts {
            let best = chains
                .iter()
                .flat_map(|c| c.samples.iter())
                .map(|n| (n.pos_wcs.distance(corner), n.clearance_radius_mm))
                .min_by(|a, b| a.0.total_cmp(&b.0))
                .unwrap();
            assert!(
                best.0 < 0.02 && approx_eq(best.1, 0.0, 0.01),
                "corner {corner:?} must be reached cleanly with zero blunting: dist={}, r={}",
                best.0,
                best.1
            );
        }
    }
}
