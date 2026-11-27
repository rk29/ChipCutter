use chip_cad::CadDocument;
use chip_cam::{
    BoxAnchor27, CamError, ContourCompensation, FixedBoxXyAlign, FixedBoxZAlign,
    GeometrySelectionItem, HeightReference, HeightSpec, Operation, OperationKind,
    PocketStrategyKind, ResolvedSetup, Setup, StockConfig, StrategyParams, ViseClampAxis,
    WcsOriginMode, rect_region_2d,
};
use chip_geom2d::{PocketRegion2D, RegionSet2D};
use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, DVec3, DisplayUnit, ParamLength};
use chip_sketch::{
    RectAnchorMode, SketchAnchor, SketchEntity, SketchEntityEntry, SketchLayer, SketchPlaneRef,
};

use super::{
    SoftJawConfig, SoftJawOutputMode, SoftJawPlan, region_set_aabb,
    silhouette::extract_part_silhouette_2d, soft_jaw_mesh::build_watertight_soft_jaw_stl_bytes,
    validate_non_negative_wiz, validate_positive_wiz,
};

/// Builds a complete **Auto Soft-Jaw** plan (CNC machining setup + sketch + ops and/or watertight
/// 3D-printable binary `.stl` files) from `setup` and `doc`.
pub fn build_soft_jaw_plan(
    doc: &CadDocument,
    setup: &Setup,
    sketches: &[SketchLayer],
    display_unit: DisplayUnit,
    config: &SoftJawConfig,
) -> Result<SoftJawPlan, CamError> {
    validate_positive_wiz("Soft-Jaw Wizard", "jaw_width_mm", config.jaw_width_mm)?;
    validate_positive_wiz("Soft-Jaw Wizard", "jaw_depth_mm", config.jaw_depth_mm)?;
    validate_positive_wiz("Soft-Jaw Wizard", "jaw_height_mm", config.jaw_height_mm)?;
    validate_non_negative_wiz("Soft-Jaw Wizard", "jaw_gap_mm", config.jaw_gap_mm)?;
    validate_positive_wiz("Soft-Jaw Wizard", "grip_depth_mm", config.grip_depth_mm)?;
    validate_non_negative_wiz(
        "Soft-Jaw Wizard",
        "part_clearance_mm",
        config.part_clearance_mm,
    )?;
    validate_non_negative_wiz(
        "Soft-Jaw Wizard",
        "corner_relief_radius_mm",
        config.corner_relief_radius_mm,
    )?;
    if config.grip_depth_mm >= config.jaw_height_mm - 2.0 {
        return Err(CamError::InvalidFixture {
            fixture_name: "Soft-Jaw Wizard".to_string(),
            reason: format!(
                "grip_depth_mm ({:.2}) must leave at least 2.0 mm base below pocket (jaw_height_mm = {:.2})",
                config.grip_depth_mm, config.jaw_height_mm
            ),
        });
    }

    let resolved = setup.resolve_with_sketches(doc, sketches, display_unit)?;
    let z_bot = resolved.model_aabb_wcs.min.z + config.z_band_bottom_offset_mm;
    let z_top = z_bot + config.grip_depth_mm;
    let part_sil = extract_part_silhouette_2d(doc, setup, &resolved, Some((z_bot, z_top)))?;
    let part_aabb = region_set_aabb(&part_sil).ok_or_else(|| CamError::InvalidFixture {
        fixture_name: "Soft-Jaw Wizard".to_string(),
        reason: "could not extract non-empty 2D clamping silhouette from target part".to_string(),
    })?;
    let part_center = part_aabb.center();

    // 1. Apply radial slip-fit clearance outward around the clamping silhouette
    let cleared_part = if config.part_clearance_mm > 1e-6 {
        part_sil.dilate_outward(config.part_clearance_mm)
    } else {
        part_sil.clone()
    };

    // 2. Add internal sharp-corner dogbone reliefs detected on the pre-dilation part silhouette
    //    (before `dilate_outward` rounds sharp vertices with tiny `part_clearance_mm` corner arcs!)
    let (pocket_with_relief, corner_relief_count) = add_convex_corner_reliefs(
        &part_sil,
        &cleared_part,
        config.part_clearance_mm,
        config.corner_relief_radius_mm,
    );

    // 3. Intersect against Fixed Jaw & Movable Jaw rectangles separated by `jaw_gap_mm`
    let half_w = 0.5 * config.jaw_width_mm;
    let half_gap = 0.5 * config.jaw_gap_mm;
    let depth = config.jaw_depth_mm;

    let (fixed_rect, movable_rect) = match config.clamp_axis {
        ViseClampAxis::Y => (
            rect_region_2d(
                part_center.x - half_w,
                part_center.y + half_gap,
                part_center.x + half_w,
                part_center.y + half_gap + depth,
            ),
            rect_region_2d(
                part_center.x - half_w,
                part_center.y - half_gap - depth,
                part_center.x + half_w,
                part_center.y - half_gap,
            ),
        ),
        ViseClampAxis::X => (
            rect_region_2d(
                part_center.x + half_gap,
                part_center.y - half_w,
                part_center.x + half_gap + depth,
                part_center.y + half_w,
            ),
            rect_region_2d(
                part_center.x - half_gap - depth,
                part_center.y - half_w,
                part_center.x - half_gap,
                part_center.y + half_w,
            ),
        ),
    };

    let fixed_pocket_wcs = pocket_with_relief.intersect(&fixed_rect);
    let movable_pocket_wcs = pocket_with_relief.intersect(&movable_rect);

    if fixed_pocket_wcs.is_empty() && movable_pocket_wcs.is_empty() {
        return Err(CamError::InvalidFixture {
            fixture_name: "Soft-Jaw Wizard".to_string(),
            reason: "part clamping silhouette does not overlap either soft jaw; reduce jaw_gap_mm or increase jaw_depth_mm".to_string(),
        });
    }

    // 4. Optional CNC-Machinable Soft-Jaw Setup + Sketch + Facing / Pocket2D / Contour2D ops
    let (machining_setup, pocket_sketch) = if matches!(
        config.output_mode,
        SoftJawOutputMode::MachiningSetup | SoftJawOutputMode::Both
    ) {
        let (sj_setup, sj_sketch) = build_soft_jaw_machining_setup(
            setup,
            &resolved,
            part_center,
            &fixed_pocket_wcs,
            &movable_pocket_wcs,
            config,
        );
        (Some(sj_setup), Some(sj_sketch))
    } else {
        (None, None)
    };

    // 5. Optional Watertight 3D-Printable Binary STL Buffers (`fixed_jaw.stl` & `movable_jaw.stl`)
    let (fixed_jaw_stl, fixed_jaw_triangle_count, movable_jaw_stl, movable_jaw_triangle_count) = if matches!(
        config.output_mode,
        SoftJawOutputMode::PrintableStl | SoftJawOutputMode::Both
    ) {
        let fixed_local = map_region_to_jaw_local(
            &fixed_pocket_wcs,
            part_center,
            config.clamp_axis,
            true,
            config.jaw_width_mm,
            config.jaw_gap_mm,
        );
        let movable_local = map_region_to_jaw_local(
            &movable_pocket_wcs,
            part_center,
            config.clamp_axis,
            false,
            config.jaw_width_mm,
            config.jaw_gap_mm,
        );

        let (fixed_bytes, fixed_tris) = build_watertight_soft_jaw_stl_bytes(
            "ChipCutter Fixed Soft Jaw",
            config.jaw_width_mm,
            config.jaw_depth_mm,
            config.jaw_height_mm,
            config.grip_depth_mm,
            &fixed_local,
            config.mounting_bolt_pattern,
        )?;
        let (movable_bytes, movable_tris) = build_watertight_soft_jaw_stl_bytes(
            "ChipCutter Movable Soft Jaw",
            config.jaw_width_mm,
            config.jaw_depth_mm,
            config.jaw_height_mm,
            config.grip_depth_mm,
            &movable_local,
            config.mounting_bolt_pattern,
        )?;
        (
            Some(fixed_bytes),
            fixed_tris,
            Some(movable_bytes),
            movable_tris,
        )
    } else {
        (None, 0, None, 0)
    };

    Ok(SoftJawPlan {
        part_silhouette_2d: part_sil,
        fixed_pocket_2d: fixed_pocket_wcs,
        movable_pocket_2d: movable_pocket_wcs,
        jaw_width_mm: config.jaw_width_mm,
        jaw_depth_mm: config.jaw_depth_mm,
        jaw_height_mm: config.jaw_height_mm,
        grip_depth_mm: config.grip_depth_mm,
        part_clearance_mm: config.part_clearance_mm,
        corner_relief_count,
        machining_setup,
        pocket_sketch,
        fixed_jaw_stl,
        movable_jaw_stl,
        fixed_jaw_triangle_count,
        movable_jaw_triangle_count,
    })
}

// ── Flip-Jig Helpers ───────────────────────────────────────────────────────────────────────────

/// Adds circular dogbone corner reliefs (`radius = relief_r`) at sharp convex corners detected on
/// `raw_part_sil` (before clearance dilation rounds sharp corners with tiny tangent arcs) so
/// square part corners seat dead-flat inside CNC-machined or 3D-printed soft jaws.
fn add_convex_corner_reliefs(
    raw_part_sil: &RegionSet2D,
    cleared_part: &RegionSet2D,
    clearance_mm: f64,
    relief_r: f64,
) -> (RegionSet2D, usize) {
    if !relief_r.is_finite() || relief_r <= 0.05 || raw_part_sil.is_empty() {
        return (cleared_part.clone(), 0);
    }

    let mut relief_disks: Vec<RegionSet2D> = Vec::new();
    let min_edge_len = (2.0 * relief_r).max(1.5);
    let eff_r = relief_r + clearance_mm.max(0.0);

    for reg in &raw_part_sil.regions {
        let segs = &reg.outer_ccw.segments;
        let n = segs.len();
        if n < 3 {
            continue;
        }
        for i in 0..n {
            let prev = &segs[(i + n - 1) % n];
            let curr = &segs[i];
            if prev.length() < min_edge_len || curr.length() < min_edge_len {
                continue;
            }
            let (Some((_, t_in)), Some((_, t_out))) = (prev.eval_at(1.0), curr.eval_at(0.0)) else {
                continue;
            };
            let cross = t_in.x * t_out.y - t_in.y * t_out.x;
            let dot = t_in.dot(t_out).clamp(-1.0, 1.0);
            let turn_deg = cross.atan2(dot).to_degrees();
            // Convex corner on CCW outer loop turns left (`turn_deg >= 25°`).
            if (25.0..=165.0).contains(&turn_deg) {
                let corner_pt = curr.start();
                let disk = RegionSet2D::from_circle(corner_pt, eff_r);
                if !disk.is_empty() {
                    relief_disks.push(disk);
                }
            }
        }
    }

    let count = relief_disks.len();
    if count == 0 {
        return (cleared_part.clone(), 0);
    }
    let merged_disks = RegionSet2D::union_many(&mut relief_disks);
    (cleared_part.union(&merged_disks), count)
}

fn build_soft_jaw_machining_setup(
    source_setup: &Setup,
    resolved: &ResolvedSetup,
    part_center: DVec2,
    fixed_pocket_wcs: &RegionSet2D,
    movable_pocket_wcs: &RegionSet2D,
    config: &SoftJawConfig,
) -> (Setup, SketchLayer) {
    let setup_id = config
        .setup_id
        .clone()
        .unwrap_or_else(|| "setup_soft_jaws".to_string());
    let mut sj_setup = Setup::new(
        setup_id.clone(),
        "Soft Jaws — Fixed & Movable",
        config.wcs_slot,
    );
    sj_setup.target_bodies = source_setup.target_bodies.clone();
    sj_setup.post_profile = source_setup.post_profile.clone();
    sj_setup.wcs.orientation = source_setup.wcs.orientation.clone();
    sj_setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };

    let (raw_span_x, raw_span_y) = match config.clamp_axis {
        ViseClampAxis::Y => (
            config.jaw_width_mm,
            2.0 * config.jaw_depth_mm + config.jaw_gap_mm,
        ),
        ViseClampAxis::X => (
            2.0 * config.jaw_depth_mm + config.jaw_gap_mm,
            config.jaw_width_mm,
        ),
    };

    // Ensure FixedBox stock encloses the project's 3D target model (if any) and positions the
    // model safely below the soft-jaw pocket cut depth so simulation never reports false gouges.
    let model_sz = if resolved.model_aabb_wcs.is_valid() {
        resolved.model_aabb_wcs.size()
    } else {
        DVec3::ZERO
    };
    let skim_mm = 0.5;
    let total_cut_depth = skim_mm + config.grip_depth_mm;
    let top_allow_above_model = total_cut_depth + 1.5;

    let stock_x = raw_span_x.max(model_sz.x + 2.0);
    let stock_y = raw_span_y.max(model_sz.y + 2.0);
    let stock_z = config
        .jaw_height_mm
        .max(model_sz.z + top_allow_above_model + 1.0);

    sj_setup.stock = StockConfig::FixedBox {
        width_x: ParamLength::from_mm(stock_x),
        depth_y: ParamLength::from_mm(stock_y),
        height_z: ParamLength::from_mm(stock_z),
        xy_align: FixedBoxXyAlign::CenterXy,
        z_align: FixedBoxZAlign::OffsetFromTop {
            top_allowance: ParamLength::from_mm(top_allow_above_model),
        },
    };

    let sketch_id = format!("sketch_{setup_id}");
    let mut sketch = SketchLayer::new(
        sketch_id.clone(),
        "Soft-Jaw Pocket Contours",
        setup_id.clone(),
    );
    sketch.anchor = SketchAnchor::SetupXy {
        setup_id,
        plane_ref: SketchPlaneRef::StockTop,
        z_offset: ParamLength::from_mm(0.0),
    };
    sketch.cut_depth_default = ParamLength::from_mm(config.grip_depth_mm);

    // Construction outlines for Fixed & Movable jaw blanks centered at `(0, 0)`
    let half_gap = 0.5 * config.jaw_gap_mm;
    let (fixed_cx, fixed_cy, mov_cx, mov_cy, rect_w, rect_h) = match config.clamp_axis {
        ViseClampAxis::Y => (
            0.0,
            half_gap + 0.5 * config.jaw_depth_mm,
            0.0,
            -(half_gap + 0.5 * config.jaw_depth_mm),
            config.jaw_width_mm,
            config.jaw_depth_mm,
        ),
        ViseClampAxis::X => (
            half_gap + 0.5 * config.jaw_depth_mm,
            0.0,
            -(half_gap + 0.5 * config.jaw_depth_mm),
            0.0,
            config.jaw_depth_mm,
            config.jaw_width_mm,
        ),
    };
    sketch.entities.push(SketchEntityEntry {
        id: "fixed_jaw_outline".to_string(),
        name: "Fixed Jaw Blank Outline".to_string(),
        sub_layer: Some("JAW_BLANKS".to_string()),
        is_construction: true,
        entity: SketchEntity::Rectangle {
            anchor: RectAnchorMode::Center,
            x: ParamLength::from_mm(fixed_cx),
            y: ParamLength::from_mm(fixed_cy),
            width: ParamLength::from_mm(rect_w),
            height: ParamLength::from_mm(rect_h),
            corner_radius: ParamLength::from_mm(0.0),
            rotation_deg: 0.0,
        },
    });
    sketch.entities.push(SketchEntityEntry {
        id: "movable_jaw_outline".to_string(),
        name: "Movable Jaw Blank Outline".to_string(),
        sub_layer: Some("JAW_BLANKS".to_string()),
        is_construction: true,
        entity: SketchEntity::Rectangle {
            anchor: RectAnchorMode::Center,
            x: ParamLength::from_mm(mov_cx),
            y: ParamLength::from_mm(mov_cy),
            width: ParamLength::from_mm(rect_w),
            height: ParamLength::from_mm(rect_h),
            corner_radius: ParamLength::from_mm(0.0),
            rotation_deg: 0.0,
        },
    });

    // Insert exact shifted pocket contours into the sketch layer
    let mut pocket_entity_ids: Vec<String> = Vec::new();
    for (jaw_tag, jaw_label, reg_set) in [
        ("fixed", "Fixed Jaw Pocket", fixed_pocket_wcs),
        ("movable", "Movable Jaw Pocket", movable_pocket_wcs),
    ] {
        for (idx, reg) in reg_set.regions.iter().enumerate() {
            let shifted = shift_polyline(&reg.outer_ccw, -part_center).ensure_orientation(true);
            let ent_id = format!("jaw_pocket_{jaw_tag}_{idx}");
            sketch.entities.push(SketchEntityEntry {
                id: ent_id.clone(),
                name: format!("{jaw_label} {}", idx + 1),
                sub_layer: Some("POCKETS".to_string()),
                is_construction: false,
                entity: SketchEntity::new_polyline(shifted),
            });
            pocket_entity_ids.push(ent_id);
        }
    }
    sketch.rebuild_regions(None);

    // Op 1: Skim-face soft jaw tops (`0.5 mm`)
    let mut op_face = Operation::new(
        "op_sj_1_face",
        "1. Face Soft-Jaw Tops",
        OperationKind::Facing,
    );
    op_face.tool_id = config.roughing_tool_id.clone();
    op_face.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op_face.heights.bottom = HeightSpec::new(HeightReference::StockTop, -skim_mm);
    op_face.passes.multiple_depths = false;
    op_face.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
    sj_setup.operations.push(op_face);

    // Op 2: Pocket2D roughing of contoured soft-jaw step pockets
    let mut op_pocket = Operation::new(
        "op_sj_2_pocket",
        "2. Rough Soft-Jaw Pockets",
        OperationKind::Pocket2D,
    );
    op_pocket.tool_id = config.roughing_tool_id.clone();
    op_pocket.heights.top = HeightSpec::new(HeightReference::StockTop, -skim_mm);
    op_pocket.heights.bottom = HeightSpec::new(HeightReference::StockTop, -total_cut_depth);
    op_pocket.passes.multiple_depths = config.grip_depth_mm > 2.5;
    op_pocket.passes.max_stepdown = ParamLength::from_mm(2.0);
    op_pocket.passes.stock_to_leave_radial = ParamLength::from_mm(0.15);
    op_pocket.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
    if let StrategyParams::Pocket2D(p) = op_pocket.strategy_mut() {
        p.strategy = PocketStrategyKind::ContourParallel;
    }
    if !sketch.regions.is_empty() {
        for reg in &sketch.regions {
            op_pocket
                .selections
                .push(GeometrySelectionItem::SketchRegion {
                    sketch_id: sketch_id.clone(),
                    region_id: reg.id.clone(),
                });
        }
    } else {
        for ent_id in &pocket_entity_ids {
            op_pocket
                .selections
                .push(GeometrySelectionItem::SketchEntity {
                    sketch_id: sketch_id.clone(),
                    entity_id: ent_id.clone(),
                    reverse: false,
                    side_flip: false,
                });
        }
    }
    sj_setup.operations.push(op_pocket);

    // Op 3: Contour2D wall finishing pass (`reverse = true` -> CCW -> LeftClimb cuts INSIDE pocket!)
    let mut op_finish = Operation::new(
        "op_sj_3_finish",
        "3. Finish Soft-Jaw Pocket Walls",
        OperationKind::Contour2D,
    );
    op_finish.tool_id = config
        .finishing_tool_id
        .clone()
        .or_else(|| config.roughing_tool_id.clone());
    op_finish.heights.top = HeightSpec::new(HeightReference::StockTop, -skim_mm);
    op_finish.heights.bottom = HeightSpec::new(HeightReference::StockTop, -total_cut_depth);
    op_finish.passes.multiple_depths = false;
    op_finish.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
    op_finish.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
    if let StrategyParams::Contour2D(p) = op_finish.strategy_mut() {
        p.compensation = ContourCompensation::LeftClimb;
        p.roughing_passes = 0;
        p.finish_passes = 1;
        p.finish_stepover = ParamLength::from_mm(0.15);
    }
    for ent_id in &pocket_entity_ids {
        op_finish
            .selections
            .push(GeometrySelectionItem::SketchEntity {
                sketch_id: sketch_id.clone(),
                entity_id: ent_id.clone(),
                reverse: true,
                side_flip: false,
            });
    }
    sj_setup.operations.push(op_finish);

    (sj_setup, sketch)
}

fn shift_polyline(poly: &ArcPolyline2D, delta: DVec2) -> ArcPolyline2D {
    let segments = poly
        .segments
        .iter()
        .map(|seg| match *seg {
            ArcSegment2D::Line { start, end } => ArcSegment2D::Line {
                start: start + delta,
                end: end + delta,
            },
            ArcSegment2D::Arc {
                start,
                end,
                center,
                radius,
                ccw,
            } => ArcSegment2D::Arc {
                start: start + delta,
                end: end + delta,
                center: center + delta,
                radius,
                ccw,
            },
        })
        .collect();
    ArcPolyline2D {
        segments,
        closed: poly.closed,
    }
}

/// Maps a Setup-WCS jaw pocket [`RegionSet2D`] into jaw-local `[0..W] × [0..D]` coordinates where
/// `Y = 0` is always the inner clamping face (facing the workpiece) and `Y = D` is the outer back
/// face bolted to the vise master jaw.
fn map_region_to_jaw_local(
    region_wcs: &RegionSet2D,
    part_center: DVec2,
    clamp_axis: ViseClampAxis,
    is_fixed_positive_side: bool,
    jaw_width_mm: f64,
    jaw_gap_mm: f64,
) -> RegionSet2D {
    let half_w = 0.5 * jaw_width_mm;
    let half_gap = 0.5 * jaw_gap_mm;

    let map_pt = |p: DVec2| -> DVec2 {
        let rel = p - part_center;
        match (clamp_axis, is_fixed_positive_side) {
            // Fixed (+Y): X_local = rel.x + W/2, Y_local = rel.y - gap/2
            (ViseClampAxis::Y, true) => DVec2::new(rel.x + half_w, rel.y - half_gap),
            // Movable (-Y): rotated 180° in XY so inner clamping face is still Y_local = 0!
            (ViseClampAxis::Y, false) => DVec2::new(-rel.x + half_w, -rel.y - half_gap),
            // Fixed (+X): rotate -90° so X_local = rel.y + W/2, Y_local = rel.x - gap/2 (Wait: preserves orientation: det = +1?
            // Let's keep det = +1 (rigid rotation): X_local = -rel.y + W/2, Y_local = rel.x - gap/2!)
            (ViseClampAxis::X, true) => DVec2::new(-rel.y + half_w, rel.x - half_gap),
            // Movable (-X): rigid +90° rotation: X_local = rel.y + W/2, Y_local = -rel.x - gap/2!
            (ViseClampAxis::X, false) => DVec2::new(rel.y + half_w, -rel.x - half_gap),
        }
    };

    let map_poly = |poly: &ArcPolyline2D| -> ArcPolyline2D {
        let segments = poly
            .segments
            .iter()
            .map(|seg| match *seg {
                ArcSegment2D::Line { start, end } => ArcSegment2D::Line {
                    start: map_pt(start),
                    end: map_pt(end),
                },
                ArcSegment2D::Arc {
                    start,
                    end,
                    center,
                    radius,
                    ccw,
                } => ArcSegment2D::Arc {
                    start: map_pt(start),
                    end: map_pt(end),
                    center: map_pt(center),
                    radius,
                    ccw,
                },
            })
            .collect();
        ArcPolyline2D {
            segments,
            closed: poly.closed,
        }
    };

    let regions = region_wcs
        .regions
        .iter()
        .map(|r| {
            PocketRegion2D::new(
                map_poly(&r.outer_ccw),
                r.islands_cw.iter().map(map_poly).collect(),
            )
        })
        .collect();
    RegionSet2D::from_regions(regions)
}

// ── Watertight Binary STL Soft-Jaw Mesh Builder ────────────────────────────────────────────────
