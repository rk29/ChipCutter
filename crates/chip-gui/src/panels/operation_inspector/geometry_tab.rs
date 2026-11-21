use chip_cad::{CadDocument, FaceId, find_coplanar_faces, find_same_diameter_holes};
use chip_cam::{
    ExtractedFeature25D, ExtractedSelectionSummary, GeometrySelectionItem, Operation, OperationKind,
};
use chip_math::{ArcSegment2D, DisplayUnit};
use chip_sketch::SketchLayer;

use crate::{
    panels::ViewportPickMode,
    widgets::{BannerKind, grid_row, inline_banner, inspector_grid, section_card, static_card},
};

pub struct GeometryTabCtx<'a> {
    pub op: &'a mut Operation,
    pub sketches: &'a [SketchLayer],
    pub setup_idx: usize,
    pub op_idx: usize,
    pub display_unit: DisplayUnit,
    pub cad_doc: Option<&'a CadDocument>,
    pub extracted: Option<Result<&'a ExtractedSelectionSummary, &'a str>>,
    pub pick_mode: &'a mut ViewportPickMode,
    pub mutated_label: &'a mut Option<&'static str>,
}

pub fn show_geometry_tab(ui: &mut egui::Ui, ctx: GeometryTabCtx<'_>) {
    let GeometryTabCtx {
        op,
        sketches,
        setup_idx,
        op_idx,
        display_unit,
        cad_doc,
        extracted,
        pick_mode,
        mutated_label,
    } = ctx;

    if op.kind.is_3d_surface() {
        inline_banner(
            ui,
            BannerKind::Info,
            "3D Surface Operation: machines the active 3D mesh or STEP solid automatically inside Stock Bounds / Model Silhouette. Optionally pick 2D Sketch regions or B-Rep loops below and choose 'Selected 2D Contours' in Passes to constrain XY machining.",
        );
        ui.add_space(4.0);
    }

    section_card(
        ui,
        ("geom_pick_card", setup_idx, op_idx),
        "Interactive 3D Pick Mode",
        true,
        |ui| {
            let is_faces = matches!(pick_mode, ViewportPickMode::OpPlanarFaces { .. });
            let is_loops = matches!(pick_mode, ViewportPickMode::OpClosedLoops { .. });
            let is_chain = matches!(pick_mode, ViewportPickMode::OpOpenChain { .. });
            let is_holes = matches!(pick_mode, ViewportPickMode::OpCylindricalHoles { .. });

            ui.horizontal_wrapped(|ui| {
                if ui
                    .selectable_label(is_faces, "Planar Faces")
                    .on_hover_text("Pick horizontal or angled planar pocket/chamfer faces")
                    .clicked()
                {
                    *pick_mode = ViewportPickMode::OpPlanarFaces { setup_idx, op_idx };
                }
                if ui
                    .selectable_label(is_loops, "Closed Loops")
                    .on_hover_text("Pick closed outer/island boundary loops")
                    .clicked()
                {
                    *pick_mode = ViewportPickMode::OpClosedLoops { setup_idx, op_idx };
                }
                if ui
                    .selectable_label(is_chain, "Open Chain")
                    .on_hover_text("Pick connected open edge chains via start and end edges")
                    .clicked()
                {
                    *pick_mode = ViewportPickMode::OpOpenChain {
                        setup_idx,
                        op_idx,
                        start_edge: None,
                    };
                }
                if ui
                    .selectable_label(is_holes, "Cylindrical Holes")
                    .on_hover_text("Pick cylindrical bore/drill faces")
                    .clicked()
                {
                    *pick_mode = ViewportPickMode::OpCylindricalHoles { setup_idx, op_idx };
                }
            });

            if let Some(doc) = cad_doc {
                ui.add_space(4.0);
                inspector_grid(ui, ("geom_cad_helpers_grid", setup_idx, op_idx), |ui| {
                    grid_row(
                        ui,
                        "Auto-Expand",
                        Some("Propagate selection to matching features on the same CAD solid"),
                        |ui| {
                            if ui
                                .small_button("+ Same Ø")
                                .on_hover_text(
                                    "Expand hole selection to all matching diameter holes on the same body",
                                )
                                .clicked()
                            {
                                expand_same_diameter_holes(doc, op, mutated_label);
                            }
                            if ui
                                .small_button("+ Coplanar")
                                .on_hover_text(
                                    "Expand planar face selection to all coplanar pocket floors/shelves on the same body",
                                )
                                .clicked()
                            {
                                expand_coplanar_faces(doc, op, mutated_label);
                            }
                            if !op.selections.is_empty()
                                && ui
                                    .small_button("Clear All")
                                    .on_hover_text(
                                        "Remove all selected geometry entities from this operation",
                                    )
                                    .clicked()
                            {
                                op.selections.clear();
                                *mutated_label = Some("Clear Operation Selections");
                            }
                        },
                    );
                });
            } else if !op.selections.is_empty() {
                ui.add_space(4.0);
                if ui
                    .small_button("Clear All")
                    .on_hover_text("Remove all selected geometry entities from this operation")
                    .clicked()
                {
                    op.selections.clear();
                    *mutated_label = Some("Clear Operation Selections");
                }
            }
        },
    );

    if !sketches.is_empty() {
        section_card(
            ui,
            ("geom_sketch_card", setup_idx, op_idx),
            "2D Sketch Layers & Quick Select",
            true,
            |ui| {
                inspector_grid(ui, ("geom_sketch_grid", setup_idx, op_idx), |ui| {
                    for sk in sketches {
                        grid_row(
                            ui,
                            &sk.name,
                            Some(
                                "Add entire 2D sketch layer or DXF/SVG sub-layer to this operation",
                            ),
                            |ui| {
                                if ui
                                    .small_button("+ All")
                                    .on_hover_text(format!(
                                        "Select all non-construction entities in sketch '{}'",
                                        sk.name
                                    ))
                                    .clicked()
                                {
                                    let item = GeometrySelectionItem::SketchLayerAll {
                                        sketch_id: sk.id.clone(),
                                        sub_layer: None,
                                        reverse: false,
                                        side_flip: false,
                                    };
                                    if !op.selections.contains(&item) {
                                        op.selections.push(item);
                                        *mutated_label = Some("Add Sketch Layer");
                                    }
                                }
                                for lname in sk.sub_layer_names() {
                                    if ui
                                        .small_button(format!("+ [{lname}]"))
                                        .on_hover_text(format!("Select all entities tagged with sub-layer '{lname}' in sketch '{}'", sk.name))
                                        .clicked()
                                    {
                                        let item = GeometrySelectionItem::SketchLayerAll {
                                            sketch_id: sk.id.clone(),
                                            sub_layer: Some(lname),
                                            reverse: false,
                                            side_flip: false,
                                        };
                                        if !op.selections.contains(&item) {
                                            op.selections.push(item);
                                            *mutated_label = Some("Add Sketch Sub-Layer");
                                        }
                                    }
                                }
                            },
                        );
                    }
                });
            },
        );
    }

    let sel_title = format!("Selected Entities ({})", op.selections.len());
    section_card(
        ui,
        ("geom_sel_card", setup_idx, op_idx),
        &sel_title,
        true,
        |ui| {
            if op.selections.is_empty() {
                let msg = if op.kind == OperationKind::Facing {
                    "Facing defaults automatically to the Setup XY Stock boundary."
                } else {
                    "Click entities directly in the 3D viewport above to add geometry."
                };
                inline_banner(ui, BannerKind::Info, msg);
            } else {
                show_selected_entity_list(ui, op, mutated_label);
            }
        },
    );

    match extracted {
        Some(Ok(sum)) => {
            static_card(ui, Some("Extracted 2.5D WCS Features"), |ui| {
                inspector_grid(ui, ("geom_ext_grid", setup_idx, op_idx), |ui| {
                    grid_row(
                        ui,
                        "Inferred Top Z",
                        Some("Highest WCS Z datum across extracted features"),
                        |ui| {
                            ui.strong(display_unit.format_length(sum.inferred_top_z_wcs));
                        },
                    );
                    grid_row(
                        ui,
                        "Inferred Bottom Z",
                        Some("Lowest WCS Z datum across extracted features"),
                        |ui| {
                            ui.strong(display_unit.format_length(sum.inferred_bottom_z_wcs));
                        },
                    );
                });
                if !sum.features.is_empty() {
                    ui.add_space(4.0);
                    show_extracted_feature_rows(ui, &sum.features, display_unit);
                }
            });
        }
        Some(Err(err)) => {
            inline_banner(ui, BannerKind::Error, err);
        }
        None => {}
    }
}

fn expand_same_diameter_holes(
    doc: &CadDocument,
    op: &mut Operation,
    mutated_label: &mut Option<&'static str>,
) {
    let seed_hole = op.selections.iter().find_map(|s| match s {
        GeometrySelectionItem::CylindricalHole { binding } => Some(FaceId(binding.last_index)),
        _ => None,
    });
    if let Some(seed) = seed_hole {
        for fid in find_same_diameter_holes(doc, seed, 1e-3, 1e-3) {
            let already = op.selections.iter().any(|s| {
                matches!(
                    s,
                    GeometrySelectionItem::CylindricalHole { binding } if binding.last_index == fid.0
                )
            });
            if let (false, Some(b)) = (already, doc.bind_face(fid)) {
                op.selections
                    .push(GeometrySelectionItem::CylindricalHole { binding: b });
            }
        }
        *mutated_label = Some("Select Same-Diameter Holes");
    }
}

fn expand_coplanar_faces(
    doc: &CadDocument,
    op: &mut Operation,
    mutated_label: &mut Option<&'static str>,
) {
    let seed_face = op.selections.iter().find_map(|s| match s {
        GeometrySelectionItem::PlanarFace { binding } => Some(FaceId(binding.last_index)),
        _ => None,
    });
    if let Some(seed) = seed_face {
        for fid in find_coplanar_faces(doc, seed, 1e-3, 1e-2) {
            let already = op.selections.iter().any(|s| {
                matches!(
                    s,
                    GeometrySelectionItem::PlanarFace { binding } if binding.last_index == fid.0
                )
            });
            if let (false, Some(b)) = (already, doc.bind_face(fid)) {
                op.selections
                    .push(GeometrySelectionItem::PlanarFace { binding: b });
            }
        }
        *mutated_label = Some("Select Coplanar Faces");
    }
}

fn show_selected_entity_list(
    ui: &mut egui::Ui,
    op: &mut Operation,
    mutated_label: &mut Option<&'static str>,
) {
    let mut remove_idx: Option<usize> = None;
    for (idx, item) in op.selections.iter_mut().enumerate() {
        ui.horizontal(|ui| {
            let has_chain_btns = matches!(
                item,
                GeometrySelectionItem::ClosedLoop { .. }
                    | GeometrySelectionItem::OpenChain { .. }
                    | GeometrySelectionItem::SketchEntity { .. }
                    | GeometrySelectionItem::SketchLayerAll { .. }
            );
            let reserve_w = if has_chain_btns { 116.0 } else { 28.0 };
            let lbl_w = (ui.available_width() - reserve_w).max(56.0);
            ui.add_sized(
                [lbl_w, 18.0],
                egui::Label::new(format!("• {}", item.short_label())).truncate(),
            )
            .on_hover_text(item.short_label());
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui
                    .small_button("×")
                    .on_hover_text("Remove this geometry item from the operation")
                    .clicked()
                {
                    remove_idx = Some(idx);
                }
                match item {
                    GeometrySelectionItem::ClosedLoop {
                        reverse, side_flip, ..
                    }
                    | GeometrySelectionItem::OpenChain {
                        reverse, side_flip, ..
                    }
                    | GeometrySelectionItem::SketchEntity {
                        reverse, side_flip, ..
                    }
                    | GeometrySelectionItem::SketchLayerAll {
                        reverse, side_flip, ..
                    } => {
                        if ui
                            .small_button("Flip Side")
                            .on_hover_text("Flip cutter compensation side (Left <-> Right)")
                            .clicked()
                        {
                            *side_flip = !*side_flip;
                            *mutated_label = Some("Flip Chain Offset Side");
                        }
                        if ui
                            .small_button("Rev")
                            .on_hover_text("Reverse chain travel direction")
                            .clicked()
                        {
                            *reverse = !*reverse;
                            *mutated_label = Some("Reverse Chain Direction");
                        }
                    }
                    _ => {}
                }
            });
        });
    }
    if let Some(ri) = remove_idx {
        op.selections.remove(ri);
        *mutated_label = Some("Remove Selected Entity");
    }
}

fn show_extracted_feature_rows(
    ui: &mut egui::Ui,
    features: &[ExtractedFeature25D],
    display_unit: DisplayUnit,
) {
    for feat in features {
        match feat {
            ExtractedFeature25D::PlanarFace {
                source_face_id,
                face_z_wcs,
                inner_islands_wcs,
                area_mm2,
                ..
            } => {
                let tag = match source_face_id {
                    Some(fid) => format!("Face #{}", fid.0),
                    None => "2D Region".to_string(),
                };
                ui.weak(format!(
                    "• {tag}: Z={}, {} island(s), {:.2} {}²",
                    display_unit.format_length(*face_z_wcs),
                    inner_islands_wcs.len(),
                    display_unit.area_from_mm2(*area_mm2),
                    display_unit.suffix()
                ));
            }
            ExtractedFeature25D::Contour {
                polyline_wcs,
                min_z_wcs,
                ..
            } => {
                let arcs = polyline_wcs
                    .segments
                    .iter()
                    .filter(|s| matches!(s, ArcSegment2D::Arc { .. }))
                    .count();
                ui.weak(format!(
                    "• {} Contour: len={}, {} segs ({} arcs), Z={}",
                    if polyline_wcs.closed {
                        "Closed"
                    } else {
                        "Open"
                    },
                    display_unit.format_length(polyline_wcs.perimeter()),
                    polyline_wcs.segments.len(),
                    arcs,
                    display_unit.format_length(*min_z_wcs),
                ));
            }
            ExtractedFeature25D::ModeledChamfer {
                source_face_id,
                radial_width_mm,
                bevel_half_angle_rad,
                top_z_wcs,
                bottom_z_wcs,
                ..
            } => {
                let tag = match source_face_id {
                    Some(fid) => format!("Modeled Chamfer #{}", fid.0),
                    None => "Modeled Chamfer".to_string(),
                };
                ui.weak(format!(
                    "• {tag}: w={}, {:.1}°, Z={}..{}",
                    display_unit.format_length(*radial_width_mm),
                    bevel_half_angle_rad.to_degrees(),
                    display_unit.format_length(*bottom_z_wcs),
                    display_unit.format_length(*top_z_wcs),
                ));
            }
            ExtractedFeature25D::CylindricalHole {
                center_wcs,
                diameter_mm,
                top_z_wcs,
                bottom_z_wcs,
                is_through_hole,
                ..
            } => {
                ui.weak(format!(
                    "• Hole Ø{} ({}): XY=({:.2}, {:.2}), Z={}..{}",
                    display_unit.format_length(*diameter_mm),
                    if *is_through_hole { "Through" } else { "Blind" },
                    display_unit.from_mm(center_wcs.x),
                    display_unit.from_mm(center_wcs.y),
                    display_unit.format_length(*bottom_z_wcs),
                    display_unit.format_length(*top_z_wcs),
                ));
            }
        }
    }
}
