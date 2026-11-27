use chip_cad::CadDocument;
use chip_cam::{
    BoxAnchor27, CamError, DrillCycleKind, FixedBoxZAlign, FixtureBody, FixtureKind,
    GeometrySelectionItem, HeightReference, HeightSpec, HoleMachiningMode, Operation,
    OperationKind, Setup, StockConfig, StrategyParams, WcsOrientationMode, WcsOriginMode,
};
use chip_math::{Aabb2, Aabb3, DVec2, DVec3, DisplayUnit, ParamLength};
use chip_sketch::{SketchAnchor, SketchEntity, SketchEntityEntry, SketchLayer, SketchPlaneRef};

use super::{
    FlipAxis, FlipJigConfig, FlipJigPinCount, FlipJigPlan, region_set_aabb,
    silhouette::extract_part_silhouette_2d, validate_finite_wiz, validate_non_negative_wiz,
    validate_positive_wiz,
};

/// Builds a complete **Two-Sided Flip-Jig Dowel-Pin** plan from `setup1` and `doc`.
pub fn build_two_sided_flip_jig(
    doc: &CadDocument,
    setup1: &Setup,
    sketches: &[SketchLayer],
    display_unit: DisplayUnit,
    config: &FlipJigConfig,
) -> Result<FlipJigPlan, CamError> {
    validate_positive_wiz("Flip-Jig Wizard", "pin_diameter_mm", config.pin_diameter_mm)?;
    validate_non_negative_wiz(
        "Flip-Jig Wizard",
        "pin_fit_clearance_mm",
        config.pin_fit_clearance_mm,
    )?;
    validate_non_negative_wiz("Flip-Jig Wizard", "edge_margin_mm", config.edge_margin_mm)?;
    validate_positive_wiz(
        "Flip-Jig Wizard",
        "spoilboard_pin_depth_mm",
        config.spoilboard_pin_depth_mm,
    )?;
    validate_finite_wiz(
        "Flip-Jig Wizard",
        "poka_yoke_offset_mm",
        config.poka_yoke_offset_mm,
    )?;
    validate_finite_wiz(
        "Flip-Jig Wizard",
        "stock_pin_depth_mm",
        config.stock_pin_depth_mm,
    )?;
    validate_non_negative_wiz(
        "Flip-Jig Wizard",
        "pin_protrusion_z_mm",
        config.pin_protrusion_z_mm,
    )?;

    let resolved1 = setup1.resolve_with_sketches(doc, sketches, display_unit)?;
    let part_sil = extract_part_silhouette_2d(doc, setup1, &resolved1, None)?;
    let part_aabb = region_set_aabb(&part_sil).unwrap_or_else(|| Aabb2 {
        min: DVec2::new(
            resolved1.model_aabb_wcs.min.x,
            resolved1.model_aabb_wcs.min.y,
        ),
        max: DVec2::new(
            resolved1.model_aabb_wcs.max.x,
            resolved1.model_aabb_wcs.max.y,
        ),
    });
    let stock1_aabb = resolved1.stock.aabb_wcs();
    let hole_diameter_mm = config.pin_diameter_mm + config.pin_fit_clearance_mm;
    let pin_radius = 0.5 * hole_diameter_mm;
    let req_center_dist = pin_radius + config.edge_margin_mm;

    // Flip centerline passes through the part's exact center perpendicular to the flip axis so
    // symmetric stock & flipped Setup 2 align cleanly with zero drift.
    let part_center = part_aabb.center();
    let v_flip = match config.flip_axis {
        FlipAxis::XAxis => part_center.y,
        FlipAxis::YAxis => part_center.x,
    };
    let (u_min_part, u_max_part, v_span_part) = match config.flip_axis {
        FlipAxis::XAxis => (part_aabb.min.x, part_aabb.max.x, part_aabb.size().y),
        FlipAxis::YAxis => (part_aabb.min.y, part_aabb.max.y, part_aabb.size().x),
    };

    let make_xy = |u: f64, v: f64| -> DVec2 {
        match config.flip_axis {
            FlipAxis::XAxis => DVec2::new(u, v),
            FlipAxis::YAxis => DVec2::new(v, u),
        }
    };

    // Place pins outside the part silhouette along `u` (applying optional poka-yoke outward shift
    // on one end while maintaining exact `180°` flip invariance around `v = v_flip`).
    let mut u_neg = u_min_part - req_center_dist - (-config.poka_yoke_offset_mm).max(0.0);
    let mut u_pos = u_max_part + req_center_dist + config.poka_yoke_offset_mm.max(0.0);

    let clear_of_part = |pt: DVec2| -> bool {
        !part_sil.contains_point(pt)
            && part_sil.min_distance_to_boundary(pt) >= req_center_dist - 1e-6
    };

    let pin_centers_setup1_wcs: Vec<DVec2> = match config.pin_count {
        FlipJigPinCount::TwoOnAxis => {
            for _ in 0..64 {
                if clear_of_part(make_xy(u_neg, v_flip)) {
                    break;
                }
                u_neg -= 0.5;
            }
            for _ in 0..64 {
                if clear_of_part(make_xy(u_pos, v_flip)) {
                    break;
                }
                u_pos += 0.5;
            }
            vec![make_xy(u_neg, v_flip), make_xy(u_pos, v_flip)]
        }
        FlipJigPinCount::FourSymmetric => {
            let dv = (0.35 * v_span_part).max(config.pin_diameter_mm * 1.5);
            for _ in 0..64 {
                if clear_of_part(make_xy(u_neg, v_flip - dv))
                    && clear_of_part(make_xy(u_neg, v_flip + dv))
                {
                    break;
                }
                u_neg -= 0.5;
            }
            for _ in 0..64 {
                if clear_of_part(make_xy(u_pos, v_flip - dv))
                    && clear_of_part(make_xy(u_pos, v_flip + dv))
                {
                    break;
                }
                u_pos += 0.5;
            }
            vec![
                make_xy(u_neg, v_flip - dv),
                make_xy(u_neg, v_flip + dv),
                make_xy(u_pos, v_flip - dv),
                make_xy(u_pos, v_flip + dv),
            ]
        }
    };

    // Verify exact 180° flip symmetry: `mirror(pin_set) == pin_set` within `< 1e-9 mm`
    let max_symmetry_error_mm =
        compute_flip_symmetry_error(&pin_centers_setup1_wcs, config.flip_axis, v_flip);
    if max_symmetry_error_mm > 1e-9 {
        return Err(CamError::InvalidFixture {
            fixture_name: "Flip-Jig Registration Pins".to_string(),
            reason: format!(
                "pin layout symmetry error {max_symmetry_error_mm:.3e} mm exceeded 1e-9 mm"
            ),
        });
    }

    let min_part_clearance_mm = pin_centers_setup1_wcs
        .iter()
        .map(|&p| part_sil.min_distance_to_boundary(p) - pin_radius)
        .fold(f64::INFINITY, f64::min);

    // Ensure raw stock encloses all pins with at least `pin_radius + 4.0 mm` outer wall margin
    let wall_margin = pin_radius + 4.0;
    let mut pins_aabb = Aabb2::EMPTY;
    for &p in &pin_centers_setup1_wcs {
        pins_aabb.extend_point(p - DVec2::splat(wall_margin));
        pins_aabb.extend_point(p + DVec2::splat(wall_margin));
    }

    let needs_expand = pins_aabb.min.x < stock1_aabb.min.x - 1e-4
        || pins_aabb.max.x > stock1_aabb.max.x + 1e-4
        || pins_aabb.min.y < stock1_aabb.min.y - 1e-4
        || pins_aabb.max.y > stock1_aabb.max.y + 1e-4;

    let mut effective_setup1 = setup1.clone();
    let updated_setup1_stock = if needs_expand {
        if !config.expand_stock_if_needed {
            return Err(CamError::InvalidFixture {
                fixture_name: "Flip-Jig Registration Pins".to_string(),
                reason: "dowel pins fall outside current stock footprint; enable stock expansion or widen stock margins".to_string(),
            });
        }
        let expanded = expand_stock_for_pins(
            &setup1.stock,
            &resolved1.model_aabb_wcs,
            &stock1_aabb,
            &pins_aabb,
            config.flip_axis,
        );
        effective_setup1.stock = expanded.clone();
        Some(expanded)
    } else {
        None
    };

    // Re-resolve Setup 1 with expanded stock so pin coordinates in Setup 1 WCS stay exact even if
    // Setup 1's WCS origin is anchored to a StockBoxPoint corner!
    let resolved1_eff = effective_setup1.resolve_with_sketches(doc, sketches, display_unit)?;
    let origin_shift_xy = {
        let p_world_ref = resolved1.frame.local_to_world_point(DVec3::new(
            0.0,
            0.0,
            resolved1.model_aabb_wcs.min.z,
        ));
        let p_in_new = resolved1_eff.frame.world_to_local_point(p_world_ref);
        DVec2::new(p_in_new.x, p_in_new.y)
    };
    let pin_centers_s1_final: Vec<DVec2> = pin_centers_setup1_wcs
        .iter()
        .map(|&p| p + origin_shift_xy)
        .collect();
    let v_flip_final = v_flip
        + match config.flip_axis {
            FlipAxis::XAxis => origin_shift_xy.y,
            FlipAxis::YAxis => origin_shift_xy.x,
        };
    let final_sym_err =
        compute_flip_symmetry_error(&pin_centers_s1_final, config.flip_axis, v_flip_final);

    // 1. Build `"Flip-Jig Registration Pins"` SketchLayer on Setup 1 XY
    let sketch_id = format!("sketch_flip_jig_pins_{}", setup1.id);
    let mut pin_sketch = SketchLayer::new(
        sketch_id.clone(),
        "Flip-Jig Registration Pins",
        setup1.id.clone(),
    );
    pin_sketch.anchor = SketchAnchor::SetupXy {
        setup_id: setup1.id.clone(),
        plane_ref: SketchPlaneRef::StockTop,
        z_offset: ParamLength::from_mm(0.0),
    };
    let mut pin_selections = Vec::with_capacity(pin_centers_s1_final.len());
    for (idx, &pt) in pin_centers_s1_final.iter().enumerate() {
        let ent_id = format!("pin_{}", idx + 1);
        pin_sketch.entities.push(SketchEntityEntry {
            id: ent_id.clone(),
            name: format!("Dowel Pin {} (Ø{:.2} mm)", idx + 1, hole_diameter_mm),
            sub_layer: Some("PINS".to_string()),
            is_construction: false,
            entity: SketchEntity::new_circle(
                ParamLength::from_mm(pt.x),
                ParamLength::from_mm(pt.y),
                ParamLength::from_mm(hole_diameter_mm),
            ),
        });
        pin_selections.push(GeometrySelectionItem::SketchEntity {
            sketch_id: sketch_id.clone(),
            entity_id: ent_id,
            reverse: false,
            side_flip: false,
        });
    }
    pin_sketch.rebuild_regions(None);

    // 2. Build Op-1 Stock Dowel-Pin Drilling / Helical Boring operation
    let mut setup1_pin_op = Operation::new(
        "op_flip_jig_pins",
        "Flip-Jig Dowel Pin Holes",
        OperationKind::DrillBore,
    );
    setup1_pin_op.tool_id = config.pin_tool_id.clone();
    setup1_pin_op.selections = pin_selections;
    setup1_pin_op.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    setup1_pin_op.heights.bottom = if config.stock_pin_depth_mm > 1e-4 {
        HeightSpec::new(HeightReference::StockTop, -config.stock_pin_depth_mm)
    } else {
        HeightSpec::new(HeightReference::StockBottom, 0.0)
    };
    configure_pin_drill_params(
        &mut setup1_pin_op,
        hole_diameter_mm,
        config.pin_tool_diameter_mm,
    );

    // 3. Build paired `Setup 2 — Flip Op (G55)` rotated 180° around `flip_axis` with stock carryover
    let setup2_id = config
        .setup2_id
        .clone()
        .unwrap_or_else(|| "setup_2_flip".to_string());
    let mut setup2_flip = Setup::new(
        setup2_id.clone(),
        format!(
            "Setup 2 — Flip Op ({})",
            config.setup2_wcs_slot.short_code()
        ),
        config.setup2_wcs_slot,
    );
    setup2_flip.target_bodies = setup1.target_bodies.clone();
    setup2_flip.post_profile = setup1.post_profile.clone();
    setup2_flip.heights = setup1.heights.clone();
    setup2_flip.auto_avoid_fixtures_on_rapids = true;
    setup2_flip.carryover_from_setup_id = Some(setup1.id.clone());
    setup2_flip.wcs.orientation = flip_wcs_orientation(&setup1.wcs.orientation, config.flip_axis);
    setup2_flip.wcs.origin = mirror_wcs_origin(&setup1.wcs.origin, config.flip_axis);
    setup2_flip.stock = mirror_stock_for_flip(&effective_setup1.stock, config.flip_axis);

    let resolved2 = setup2_flip.resolve_with_sketches(doc, sketches, display_unit)?;
    let stock1_eff_aabb = resolved1_eff.stock.aabb_wcs();
    let stock2_aabb = resolved2.stock.aabb_wcs();

    // Transform each Setup-1 pin center rigidly into Setup-2 WCS so CylinderKeepOut bodies
    // align to `< 1e-9 mm` with the drilled stock holes after the 180° flip!
    let mut pin_centers_setup2_wcs = Vec::with_capacity(pin_centers_s1_final.len());
    let pin_z_bottom = stock2_aabb.min.z - config.spoilboard_pin_depth_mm;
    let pin_z_top = (stock2_aabb.min.z + config.pin_protrusion_z_mm.max(1.0))
        .min(stock2_aabb.max.z - 0.5)
        .max(stock2_aabb.min.z + 1.0);
    for (idx, &p1) in pin_centers_s1_final.iter().enumerate() {
        let p_world =
            resolved1_eff
                .frame
                .local_to_world_point(DVec3::new(p1.x, p1.y, stock1_eff_aabb.min.z));
        let p2_3d = resolved2.frame.world_to_local_point(p_world);
        let p2 = DVec2::new(p2_3d.x, p2_3d.y);
        pin_centers_setup2_wcs.push(p2);

        let mut pin_fix = FixtureBody::default_cylinder_keepout(
            format!("flip_pin_{}", idx + 1),
            format!("Flip-Jig Dowel Pin {}", idx + 1),
            p2,
            pin_z_bottom,
            pin_z_top,
        );
        pin_fix.clearance_margin = ParamLength::from_mm(1.0);
        if let FixtureKind::CylinderKeepOut { diameter, .. } = &mut pin_fix.kind {
            *diameter = ParamLength::from_mm(config.pin_diameter_mm);
        }
        setup2_flip.fixtures.push(pin_fix);
    }

    // 4. Optional `"Spoilboard Dowel Holes"` Setup + companion sketch/operation
    let (spoilboard_setup, spoilboard_sketch) = if config.include_spoilboard_setup {
        let sb_id = config
            .spoilboard_setup_id
            .clone()
            .unwrap_or_else(|| "setup_spoilboard_pins".to_string());
        let mut sb_setup = Setup::new(
            sb_id.clone(),
            "Spoilboard Dowel Holes",
            config.spoilboard_wcs_slot,
        );
        sb_setup.target_bodies = setup1.target_bodies.clone();
        sb_setup.post_profile = setup1.post_profile.clone();
        sb_setup.wcs = setup1.wcs.clone();
        sb_setup.stock = effective_setup1.stock.clone();
        sb_setup.heights = setup1.heights.clone();

        let sb_sketch_id = format!("sketch_{sb_id}");
        let mut sb_sketch = SketchLayer::new(
            sb_sketch_id.clone(),
            "Spoilboard Dowel Holes",
            sb_id.clone(),
        );
        sb_sketch.anchor = SketchAnchor::SetupXy {
            setup_id: sb_id,
            plane_ref: SketchPlaneRef::StockTop,
            z_offset: ParamLength::from_mm(0.0),
        };
        let mut sb_sels = Vec::with_capacity(pin_centers_s1_final.len());
        for (idx, &pt) in pin_centers_s1_final.iter().enumerate() {
            let ent_id = format!("sb_pin_{}", idx + 1);
            sb_sketch.entities.push(SketchEntityEntry {
                id: ent_id.clone(),
                name: format!("Spoilboard Pin {} (Ø{:.2} mm)", idx + 1, hole_diameter_mm),
                sub_layer: Some("SPOILBOARD_PINS".to_string()),
                is_construction: false,
                entity: SketchEntity::new_circle(
                    ParamLength::from_mm(pt.x),
                    ParamLength::from_mm(pt.y),
                    ParamLength::from_mm(hole_diameter_mm),
                ),
            });
            sb_sels.push(GeometrySelectionItem::SketchEntity {
                sketch_id: sb_sketch_id.clone(),
                entity_id: ent_id,
                reverse: false,
                side_flip: false,
            });
        }
        sb_sketch.rebuild_regions(None);

        let mut sb_op = Operation::new(
            "op_spoilboard_dowel_holes",
            "Drill/Bore Spoilboard Dowel Holes",
            OperationKind::DrillBore,
        );
        sb_op.tool_id = config.pin_tool_id.clone();
        sb_op.selections = sb_sels;
        sb_op.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
        sb_op.heights.bottom =
            HeightSpec::new(HeightReference::StockTop, -config.spoilboard_pin_depth_mm);
        configure_pin_drill_params(&mut sb_op, hole_diameter_mm, config.pin_tool_diameter_mm);
        sb_setup.operations.push(sb_op);
        (Some(sb_setup), Some(sb_sketch))
    } else {
        (None, None)
    };

    Ok(FlipJigPlan {
        flip_axis: config.flip_axis,
        flip_centerline_mm: v_flip_final,
        hole_diameter_mm,
        pin_centers_setup1_wcs: pin_centers_s1_final,
        pin_centers_setup2_wcs,
        max_symmetry_error_mm: final_sym_err,
        min_part_clearance_mm,
        updated_setup1_stock,
        pin_sketch,
        setup1_pin_op,
        setup2_flip,
        spoilboard_setup,
        spoilboard_sketch,
    })
}

fn configure_pin_drill_params(op: &mut Operation, hole_dia_mm: f64, tool_dia_mm: Option<f64>) {
    if let StrategyParams::DrillBore(p) = op.strategy_mut() {
        p.mode = match tool_dia_mm {
            Some(td) if td < hole_dia_mm - 0.15 => HoleMachiningMode::HelicalBore,
            Some(_) => HoleMachiningMode::Drill,
            None => HoleMachiningMode::AutoByTool,
        };
        p.cycle_kind = DrillCycleKind::DeepPeck;
        p.peck_depth = ParamLength::from_mm(3.0);
        p.bore_finish_pass = true;
        p.bore_finish_stepover = ParamLength::from_mm(0.1);
        p.tip_breakthrough = false;
    }
}

fn compute_flip_symmetry_error(pins: &[DVec2], axis: FlipAxis, v_flip: f64) -> f64 {
    let mut max_err = 0.0_f64;
    for &p in pins {
        let mirrored = match axis {
            FlipAxis::XAxis => DVec2::new(p.x, 2.0 * v_flip - p.y),
            FlipAxis::YAxis => DVec2::new(2.0 * v_flip - p.x, p.y),
        };
        let nearest = pins
            .iter()
            .map(|&q| q.distance(mirrored))
            .fold(f64::INFINITY, f64::min);
        if nearest > max_err {
            max_err = nearest;
        }
    }
    max_err
}

fn expand_stock_for_pins(
    stock: &StockConfig,
    model_aabb: &Aabb3,
    stock_aabb: &Aabb3,
    pins_aabb: &Aabb2,
    flip_axis: FlipAxis,
) -> StockConfig {
    let cur_neg_x = (model_aabb.min.x - stock_aabb.min.x).max(0.0);
    let cur_pos_x = (stock_aabb.max.x - model_aabb.max.x).max(0.0);
    let cur_neg_y = (model_aabb.min.y - stock_aabb.min.y).max(0.0);
    let cur_pos_y = (stock_aabb.max.y - model_aabb.max.y).max(0.0);
    let cur_top_z = (stock_aabb.max.z - model_aabb.max.z).max(0.0);
    let cur_bot_z = (model_aabb.min.z - stock_aabb.min.z).max(0.0);

    let mut neg_x = cur_neg_x.max(model_aabb.min.x - pins_aabb.min.x);
    let mut pos_x = cur_pos_x.max(pins_aabb.max.x - model_aabb.max.x);
    let mut neg_y = cur_neg_y.max(model_aabb.min.y - pins_aabb.min.y);
    let mut pos_y = cur_pos_y.max(pins_aabb.max.y - model_aabb.max.y);

    // Keep the axis perpendicular to `flip_axis` symmetric so the stock centerline remains
    // coincident with the part flip centerline.
    match flip_axis {
        FlipAxis::XAxis => {
            let sym_y = neg_y.max(pos_y);
            neg_y = sym_y;
            pos_y = sym_y;
        }
        FlipAxis::YAxis => {
            let sym_x = neg_x.max(pos_x);
            neg_x = sym_x;
            pos_x = sym_x;
        }
    }

    let symmetric_xy = (pos_x - neg_x).abs() <= 1e-6 && (pos_y - neg_y).abs() <= 1e-6;
    let sheet_z = match stock {
        StockConfig::RelativeBox {
            sheet_thickness_z, ..
        } => sheet_thickness_z.clone(),
        _ => ParamLength::from_mm(stock_aabb.size().z.max(1.0)),
    };

    StockConfig::RelativeBox {
        side_pos_x: ParamLength::from_mm(pos_x),
        side_neg_x: ParamLength::from_mm(neg_x),
        side_pos_y: ParamLength::from_mm(pos_y),
        side_neg_y: ParamLength::from_mm(neg_y),
        symmetric_xy,
        top_offset_z: ParamLength::from_mm(cur_top_z),
        bottom_offset_z: ParamLength::from_mm(cur_bot_z),
        sheet_thickness_z: sheet_z,
        round_increment_mm: None,
    }
}

fn flip_wcs_orientation(orig: &WcsOrientationMode, axis: FlipAxis) -> WcsOrientationMode {
    match orig {
        WcsOrientationMode::ModelOrientation {
            flip_z,
            flip_x,
            clock_90,
            extra_z_rot_deg,
        } => {
            let inv_clock = (4 - (*clock_90 % 4)) % 4;
            WcsOrientationMode::ModelOrientation {
                flip_z: !*flip_z,
                flip_x: match axis {
                    FlipAxis::XAxis => *flip_x,
                    FlipAxis::YAxis => !*flip_x,
                },
                clock_90: inv_clock,
                extra_z_rot_deg: -*extra_z_rot_deg,
            }
        }
        WcsOrientationMode::SelectZAndX {
            z_ref,
            flip_z,
            x_ref,
            flip_x,
            clock_90,
            extra_z_rot_deg,
        } => {
            let inv_clock = (4 - (*clock_90 % 4)) % 4;
            WcsOrientationMode::SelectZAndX {
                z_ref: z_ref.clone(),
                flip_z: !*flip_z,
                x_ref: x_ref.clone(),
                flip_x: match axis {
                    FlipAxis::XAxis => *flip_x,
                    FlipAxis::YAxis => !*flip_x,
                },
                clock_90: inv_clock,
                extra_z_rot_deg: -*extra_z_rot_deg,
            }
        }
        WcsOrientationMode::SelectXYPlane {
            face_ref,
            flip_z,
            x_edge_ref,
            clock_90,
            extra_z_rot_deg,
        } => {
            let base_clock = (4 - (*clock_90 % 4)) % 4;
            let inv_clock = match axis {
                FlipAxis::XAxis => base_clock,
                FlipAxis::YAxis => (base_clock + 2) % 4,
            };
            WcsOrientationMode::SelectXYPlane {
                face_ref: face_ref.clone(),
                flip_z: !*flip_z,
                x_edge_ref: x_edge_ref.clone(),
                clock_90: inv_clock,
                extra_z_rot_deg: -*extra_z_rot_deg,
            }
        }
    }
}

fn mirror_wcs_origin(origin: &WcsOriginMode, _axis: FlipAxis) -> WcsOriginMode {
    match origin {
        WcsOriginMode::StockBoxPoint { .. } => WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        },
        WcsOriginMode::ModelBoxPoint { .. } => WcsOriginMode::ModelBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        },
        WcsOriginMode::SelectedCadFeature { .. } => WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        },
    }
}

fn mirror_stock_for_flip(stock: &StockConfig, axis: FlipAxis) -> StockConfig {
    match stock {
        StockConfig::RelativeBox {
            side_pos_x,
            side_neg_x,
            side_pos_y,
            side_neg_y,
            symmetric_xy,
            top_offset_z,
            bottom_offset_z,
            sheet_thickness_z,
            round_increment_mm,
        } => {
            let (px, nx, py, ny) = match axis {
                FlipAxis::XAxis => (
                    side_pos_x.clone(),
                    side_neg_x.clone(),
                    side_neg_y.clone(),
                    side_pos_y.clone(),
                ),
                FlipAxis::YAxis => (
                    side_neg_x.clone(),
                    side_pos_x.clone(),
                    side_pos_y.clone(),
                    side_neg_y.clone(),
                ),
            };
            StockConfig::RelativeBox {
                side_pos_x: px,
                side_neg_x: nx,
                side_pos_y: py,
                side_neg_y: ny,
                symmetric_xy: *symmetric_xy,
                top_offset_z: bottom_offset_z.clone(),
                bottom_offset_z: top_offset_z.clone(),
                sheet_thickness_z: sheet_thickness_z.clone(),
                round_increment_mm: *round_increment_mm,
            }
        }
        StockConfig::FixedBox {
            width_x,
            depth_y,
            height_z,
            xy_align,
            z_align,
        } => {
            let mirrored_z = match z_align {
                FixedBoxZAlign::OffsetFromTop { top_allowance } => {
                    FixedBoxZAlign::OffsetFromBottom {
                        bottom_allowance: top_allowance.clone(),
                    }
                }
                FixedBoxZAlign::OffsetFromBottom { bottom_allowance } => {
                    FixedBoxZAlign::OffsetFromTop {
                        top_allowance: bottom_allowance.clone(),
                    }
                }
                FixedBoxZAlign::CenterZ => FixedBoxZAlign::CenterZ,
            };
            StockConfig::FixedBox {
                width_x: width_x.clone(),
                depth_y: depth_y.clone(),
                height_z: height_z.clone(),
                xy_align: xy_align.clone(),
                z_align: mirrored_z,
            }
        }
        other => other.clone(),
    }
}

// ── Soft-Jaw Helpers ───────────────────────────────────────────────────────────────────────────
