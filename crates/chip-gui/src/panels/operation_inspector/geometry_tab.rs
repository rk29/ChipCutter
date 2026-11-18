use chip_cad::{CadDocument, FaceId, find_coplanar_faces, find_same_diameter_holes};
use chip_cam::{
    ExtractedFeature25D, ExtractedSelectionSummary, GeometrySelectionItem, Operation, OperationKind,
};
use chip_math::{ArcSegment2D, DisplayUnit};

use crate::panels::ViewportPickMode;

pub struct GeometryTabCtx<'a> {
    pub op: &'a mut Operation,
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
        setup_idx,
        op_idx,
        display_unit,
        cad_doc,
        extracted,
        pick_mode,
        mutated_label,
    } = ctx;

    ui.strong("Interactive 3D Pick Mode");
    let is_faces = matches!(pick_mode, ViewportPickMode::OpPlanarFaces { .. });
    let is_loops = matches!(pick_mode, ViewportPickMode::OpClosedLoops { .. });
    let is_chain = matches!(pick_mode, ViewportPickMode::OpOpenChain { .. });
    let is_holes = matches!(pick_mode, ViewportPickMode::OpCylindricalHoles { .. });

    ui.horizontal_wrapped(|ui| {
        if ui.selectable_label(is_faces, "Planar Faces").clicked() {
            *pick_mode = ViewportPickMode::OpPlanarFaces { setup_idx, op_idx };
        }
        if ui.selectable_label(is_loops, "Closed Loops").clicked() {
            *pick_mode = ViewportPickMode::OpClosedLoops { setup_idx, op_idx };
        }
        if ui.selectable_label(is_chain, "Open Chain").clicked() {
            *pick_mode = ViewportPickMode::OpOpenChain {
                setup_idx,
                op_idx,
                start_edge: None,
            };
        }
        if ui.selectable_label(is_holes, "Cylindrical Holes").clicked() {
            *pick_mode = ViewportPickMode::OpCylindricalHoles { setup_idx, op_idx };
        }
    });

    if let Some(doc) = cad_doc {
        ui.horizontal_wrapped(|ui| {
            if ui
                .small_button("+ Same Diameter")
                .on_hover_text(
                    "Expand hole selection to all matching diameter holes on the same body",
                )
                .clicked()
            {
                let seed_hole = op.selections.iter().find_map(|s| match s {
                    GeometrySelectionItem::CylindricalHole { binding } => {
                        Some(FaceId(binding.last_index))
                    }
                    _ => None,
                });
                if let Some(seed) = seed_hole {
                    for fid in find_same_diameter_holes(doc, seed, 1e-3, 1e-3) {
                        let already = op.selections.iter().any(|s| {
                            matches!(s, GeometrySelectionItem::CylindricalHole { binding } if binding.last_index == fid.0)
                        });
                        if !already
                            && let Some(b) = doc.bind_face(fid)
                        {
                            op.selections
                                .push(GeometrySelectionItem::CylindricalHole { binding: b });
                        }
                    }
                    *mutated_label = Some("Select Same-Diameter Holes");
                }
            }

            if ui
                .small_button("+ Coplanar Faces")
                .on_hover_text(
                    "Expand planar face selection to all coplanar pocket floors/shelves on the same body",
                )
                .clicked()
            {
                let seed_face = op.selections.iter().find_map(|s| match s {
                    GeometrySelectionItem::PlanarFace { binding } => {
                        Some(FaceId(binding.last_index))
                    }
                    _ => None,
                });
                if let Some(seed) = seed_face {
                    for fid in find_coplanar_faces(doc, seed, 1e-3, 1e-2) {
                        let already = op.selections.iter().any(|s| {
                            matches!(s, GeometrySelectionItem::PlanarFace { binding } if binding.last_index == fid.0)
                        });
                        if !already
                            && let Some(b) = doc.bind_face(fid)
                        {
                            op.selections
                                .push(GeometrySelectionItem::PlanarFace { binding: b });
                        }
                    }
                    *mutated_label = Some("Select Coplanar Faces");
                }
            }

            if !op.selections.is_empty() && ui.small_button("Clear All").clicked() {
                op.selections.clear();
                *mutated_label = Some("Clear Operation Selections");
            }
        });
    }

    ui.separator();
    ui.strong(format!("Selected Entities ({})", op.selections.len()));

    if op.selections.is_empty() {
        if op.kind == OperationKind::Facing {
            ui.weak("Facing defaults automatically to the Setup XY Stock boundary.");
        } else {
            ui.weak("Click entities directly in the 3D viewport above to add geometry.");
        }
    } else {
        let mut remove_idx: Option<usize> = None;
        for (idx, item) in op.selections.iter_mut().enumerate() {
            ui.horizontal(|ui| {
                ui.label(format!("• {}", item.short_label()));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.small_button("×").clicked() {
                        remove_idx = Some(idx);
                    }
                    match item {
                        GeometrySelectionItem::ClosedLoop {
                            reverse, side_flip, ..
                        }
                        | GeometrySelectionItem::OpenChain {
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

    ui.separator();
    match extracted {
        Some(Ok(sum)) => {
            ui.strong("Extracted 2.5D WCS Features");
            ui.label(format!(
                "Inferred Top Z:    {}",
                display_unit.format_length(sum.inferred_top_z_wcs)
            ));
            ui.label(format!(
                "Inferred Bottom Z: {}",
                display_unit.format_length(sum.inferred_bottom_z_wcs)
            ));
            for feat in &sum.features {
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
                            "  {tag}: Z={}, {} inner loop(s), {:.2} {}²",
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
                            "  {} Contour: len={}, {} segs ({} true arcs), Z={}",
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
                            "  {tag}: width={}, {:.1}°, Z={}..{}",
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
                            "  Hole Ø{} ({}): XY=({:.2}, {:.2}), Z={}..{}",
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
        Some(Err(err)) => {
            ui.colored_label(egui::Color32::from_rgb(245, 96, 96), format!("[!] {err}"));
        }
        None => {}
    }
}
