use chip_cad::{CadDocument, FaceId, find_coplanar_faces, find_same_diameter_holes};
use chip_cam::{
    BoundaryPreviewOverlay, ExtractedFeature25D, ExtractedSelectionSummary, GeometrySelectionItem,
    MachiningBoundaryMode, Operation, OperationKind, ResolvedHeights, ResolvedSetup,
    RotaryAxialBoundaryMode, SharedAvoidRegion, SurfaceMesh3D,
};
use chip_math::{ArcSegment2D, DisplayUnit, EvalContext};
use chip_sketch::SketchLayer;
use chip_tools::Tool;

use super::avoid_card::{
    AvoidCardAction, AvoidCardCtx, AvoidCardSetupView, show_avoid_regions_card,
};
use crate::{
    panels::{GeometryPickTarget, ViewportPickMode},
    theme::ThemeTokens,
    widgets::{
        BannerKind, IconKind, grid_row, inline_banner, inspector_grid, section_card,
        small_icon_button, static_card,
    },
};

use super::cutting_area_cards::{
    show_contour_cutting_area_card, show_machining_boundary_card, show_pocket_cutting_area_card,
    show_rotary_cutting_area_card,
};

/// Category of Cutting Area / Machining Boundary card shown at the top of `[Geometry]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CuttingAreaCardKind {
    None,
    MachiningBoundary2D,
    RotaryAxial4Axis,
    Pocket2DArea,
    Contour2DArea,
}

/// Classifies which unified Cutting Area card `[Geometry]` displays for `op`.
pub const fn cutting_area_card_kind(op: &Operation) -> CuttingAreaCardKind {
    match op.kind {
        OperationKind::Rotary2Linear | OperationKind::Rotary4AxisSim => {
            CuttingAreaCardKind::RotaryAxial4Axis
        }
        OperationKind::Facing
        | OperationKind::Adaptive3D
        | OperationKind::Waterline3D
        | OperationKind::Parallel3D
        | OperationKind::Scallop3D
        | OperationKind::Pencil3D
        | OperationKind::Undercut3D
        | OperationKind::Deburr3D => CuttingAreaCardKind::MachiningBoundary2D,
        OperationKind::Pocket2D => CuttingAreaCardKind::Pocket2DArea,
        OperationKind::Contour2D => CuttingAreaCardKind::Contour2DArea,
        OperationKind::DrillBore
        | OperationKind::ChamferTrace
        | OperationKind::VCarve
        | OperationKind::ThreadMill
        | OperationKind::SlotUndercut
        | OperationKind::Dovetail => CuttingAreaCardKind::None,
    }
}

/// Returns whether the `Interactive 3D Pick Mode` section card should open by default for `op`.
///
/// Collapses by default on 3D surface ops (`Adaptive3D`, `Waterline3D`, `Parallel3D`, `Scallop3D`,
/// `Pencil3D`, `Undercut3D`) and `Facing` whenever `Stock Bounds`, `Model Bounds`, or
/// `Model Silhouette` is active, and on 4th-axis rotary ops unless `Selected Geometry (-U..+U)`
/// is active.
pub fn pick_card_default_open(op: &Operation) -> bool {
    if op.kind.is_rotary_native() {
        return op
            .rotary_axial_boundary()
            .is_some_and(|rb| rb.axial_boundary_mode == RotaryAxialBoundaryMode::SelectedGeometry);
    }
    if matches!(
        op.kind,
        OperationKind::Facing
            | OperationKind::Adaptive3D
            | OperationKind::Waterline3D
            | OperationKind::Parallel3D
            | OperationKind::Scallop3D
            | OperationKind::Pencil3D
            | OperationKind::Undercut3D
    ) {
        return op
            .machining_boundary()
            .is_none_or(|b| b.boundary_mode == MachiningBoundaryMode::SelectedRegions);
    }
    true
}

pub struct GeometryTabCtx<'a> {
    pub op: &'a mut Operation,
    pub shared_avoid_regions: &'a mut [SharedAvoidRegion],
    pub avoid_setup_view: &'a AvoidCardSetupView,
    pub sketches: &'a [SketchLayer],
    pub setup_idx: usize,
    pub op_idx: usize,
    pub display_unit: DisplayUnit,
    pub eval_ctx: &'a EvalContext,
    pub cad_doc: Option<&'a CadDocument>,
    pub resolved_setup: Option<&'a ResolvedSetup>,
    pub active_tool: Option<&'a Tool>,
    pub surface_mesh: Option<&'a SurfaceMesh3D>,
    pub resolved_heights: Option<&'a ResolvedHeights>,
    pub boundary_preview: Option<&'a BoundaryPreviewOverlay>,
    pub extracted: Option<Result<&'a ExtractedSelectionSummary, &'a str>>,
    pub pick_mode: &'a mut ViewportPickMode,
    pub pick_target: &'a mut GeometryPickTarget,
    pub editing_shared_avoid_id: &'a mut Option<String>,
    pub mutated_label: &'a mut Option<&'static str>,
    pub shared_mutated_label: &'a mut Option<(&'static str, String)>,
    pub deferred_avoid_action: &'a mut Option<AvoidCardAction>,
    pub create_sketch_requested: &'a mut bool,
}

pub fn show_geometry_tab(ui: &mut egui::Ui, ctx: GeometryTabCtx<'_>) {
    let GeometryTabCtx {
        op,
        shared_avoid_regions,
        avoid_setup_view,
        sketches,
        setup_idx,
        op_idx,
        display_unit,
        eval_ctx,
        cad_doc,
        resolved_setup,
        active_tool,
        surface_mesh,
        resolved_heights,
        boundary_preview,
        extracted,
        pick_mode,
        pick_target,
        editing_shared_avoid_id,
        mutated_label,
        shared_mutated_label,
        deferred_avoid_action,
        create_sketch_requested,
    } = ctx;

    // 1. Unified Cutting Area / Machining Boundary card at the top of [Geometry]
    match cutting_area_card_kind(op) {
        CuttingAreaCardKind::RotaryAxial4Axis => {
            show_rotary_cutting_area_card(
                ui,
                op,
                setup_idx,
                op_idx,
                display_unit,
                eval_ctx,
                cad_doc,
                resolved_setup,
                active_tool,
                surface_mesh,
                resolved_heights,
                boundary_preview,
                extracted.and_then(|r| r.ok()),
                mutated_label,
            );
        }
        CuttingAreaCardKind::MachiningBoundary2D => {
            show_machining_boundary_card(
                ui,
                op,
                setup_idx,
                op_idx,
                display_unit,
                eval_ctx,
                active_tool,
                boundary_preview,
                mutated_label,
            );
        }
        CuttingAreaCardKind::Pocket2DArea => {
            show_pocket_cutting_area_card(ui, op, setup_idx, op_idx, mutated_label);
        }
        CuttingAreaCardKind::Contour2DArea => {
            show_contour_cutting_area_card(ui, op, setup_idx, op_idx, eval_ctx, mutated_label);
        }
        CuttingAreaCardKind::None => {
            if matches!(
                op.kind,
                OperationKind::SlotUndercut | OperationKind::Dovetail
            ) {
                inline_banner(
                    ui,
                    BannerKind::Info,
                    "Undercut Slot / Dovetail: select a 2D Sketch centerline, B-Rep slot floor face, or open edge chain (plus a pre-cleared Cylindrical Entry Bore when using Entry Bore Center mode).",
                );
                ui.add_space(4.0);
            }
        }
    }

    // 2. Collapsible Avoid / Keep-Out Regions card (auto-expanded when `!op.avoid.is_empty()` or editing shared)
    show_avoid_regions_card(
        ui,
        AvoidCardCtx {
            op,
            shared_regions: shared_avoid_regions,
            sketches,
            setup_view: avoid_setup_view,
            setup_idx,
            op_idx,
            eval_ctx,
            pick_mode,
            pick_target,
            editing_shared_avoid_id,
            mutated_label,
            shared_mutated_label,
            deferred_action: deferred_avoid_action,
        },
    );

    let pick_default_open = pick_card_default_open(op)
        || *pick_target == GeometryPickTarget::AvoidKeepOut
        || editing_shared_avoid_id.is_some();
    section_card(
        ui,
        ("geom_pick_card", setup_idx, op_idx, pick_default_open),
        "Interactive 3D Pick Mode",
        pick_default_open,
        |ui| {
            let tok = ThemeTokens::from_ui(ui);
            let is_avoid_target = *pick_target == GeometryPickTarget::AvoidKeepOut
                || editing_shared_avoid_id.is_some();
            ui.horizontal_wrapped(|ui| {
                ui.strong("Pick Target:");
                if ui
                    .selectable_label(!is_avoid_target, "Cutting Area")
                    .on_hover_text("Route 3D viewport picks into this operation's cutting geometry")
                    .clicked()
                {
                    *pick_target = GeometryPickTarget::CuttingArea;
                    *editing_shared_avoid_id = None;
                }
                let avoid_lbl = if let Some(eid) = editing_shared_avoid_id.as_deref()
                    && let Some(sr) = shared_avoid_regions.iter().find(|r| r.id == eid)
                {
                    format!("Avoid: {}", sr.name)
                } else {
                    "Avoid / Keep-Out".to_string()
                };
                if ui
                    .selectable_label(
                        is_avoid_target,
                        egui::RichText::new(avoid_lbl).color(if is_avoid_target {
                            tok.error_fg
                        } else {
                            tok.text_primary
                        }),
                    )
                    .on_hover_text(
                        "Route 3D viewport picks into Avoid / Keep-Out regions (excludes faces, loops, holes, or sketches from toolpath)",
                    )
                    .clicked()
                {
                    *pick_target = GeometryPickTarget::AvoidKeepOut;
                    if *pick_mode == ViewportPickMode::InspectEntity {
                        *pick_mode = ViewportPickMode::OpPlanarFaces { setup_idx, op_idx };
                    }
                }
            });
            ui.add_space(3.0);

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

    if !sketches.is_empty() || pick_default_open {
        section_card(
            ui,
            ("geom_sketch_card", setup_idx, op_idx, pick_default_open),
            "2D Sketch Layers & Quick Select",
            pick_default_open,
            |ui| {
                if sketches.is_empty() {
                    inline_banner(
                        ui,
                        BannerKind::Info,
                        "No 2D sketches in this project yet — create a sketch to draw a boundary rectangle or 2D contour.",
                    );
                    ui.add_space(4.0);
                    if ui
                        .small_button("+ New Sketch")
                        .on_hover_text(
                            "Create a new 2D Sketch Layer anchored to the active Setup XY plane",
                        )
                        .clicked()
                    {
                        *create_sketch_requested = true;
                    }
                } else {
                    inspector_grid(ui, ("geom_sketch_grid", setup_idx, op_idx), |ui| {
                        for sk in sketches {
                            grid_row(
                                ui,
                                &sk.name,
                                Some(
                                    "Add entire 2D sketch layer, DXF/SVG sub-layer, or individual sketch entity to this operation",
                                ),
                                |ui| {
                                    ui.horizontal_wrapped(|ui| {
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
                                                .on_hover_text(format!(
                                                    "Select all entities tagged with sub-layer '{lname}' in sketch '{}'",
                                                    sk.name
                                                ))
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
                                        const MAX_QUICK_ENTITIES: usize = 12;
                                        let non_construction: Vec<_> = sk
                                            .entities
                                            .iter()
                                            .filter(|e| !e.is_construction)
                                            .collect();
                                        if !non_construction.is_empty() {
                                            for (e_idx, ent) in non_construction
                                                .iter()
                                                .take(MAX_QUICK_ENTITIES)
                                                .enumerate()
                                            {
                                                let short_name = if ent.name.trim().is_empty() {
                                                    format!(
                                                        "{} {}",
                                                        ent.entity.kind_label(),
                                                        e_idx + 1
                                                    )
                                                } else {
                                                    ent.name.trim().to_string()
                                                };
                                                let already_pos = op.selections.iter().position(|s| {
                                                    matches!(
                                                        s,
                                                        GeometrySelectionItem::SketchEntity {
                                                            sketch_id,
                                                            entity_id,
                                                            ..
                                                        } if sketch_id == &sk.id && entity_id == &ent.id
                                                    )
                                                });
                                                let chip_label = if already_pos.is_some() {
                                                    format!("✓ {short_name}")
                                                } else {
                                                    format!("+ {short_name}")
                                                };
                                                if ui
                                                    .small_button(chip_label)
                                                    .on_hover_text(format!(
                                                        "Toggle {} '{}' in sketch '{}'",
                                                        ent.entity.kind_label(),
                                                        short_name,
                                                        sk.name
                                                    ))
                                                    .clicked()
                                                {
                                                    if let Some(pos) = already_pos {
                                                        op.selections.remove(pos);
                                                        *mutated_label =
                                                            Some("Remove Sketch Entity");
                                                    } else {
                                                        op.selections.push(
                                                            GeometrySelectionItem::SketchEntity {
                                                                sketch_id: sk.id.clone(),
                                                                entity_id: ent.id.clone(),
                                                                reverse: false,
                                                                side_flip: false,
                                                            },
                                                        );
                                                        *mutated_label = Some("Add Sketch Entity");
                                                    }
                                                }
                                            }
                                            if non_construction.len() > MAX_QUICK_ENTITIES {
                                                ui.weak(format!(
                                                    "+{} more",
                                                    non_construction.len() - MAX_QUICK_ENTITIES
                                                ));
                                            }
                                        } else if !sk.regions.is_empty() {
                                            for (r_idx, reg) in
                                                sk.regions.iter().take(MAX_QUICK_ENTITIES).enumerate()
                                            {
                                                let already_pos = op.selections.iter().position(|s| {
                                                    matches!(
                                                        s,
                                                        GeometrySelectionItem::SketchRegion {
                                                            sketch_id,
                                                            region_id,
                                                        } if sketch_id == &sk.id && region_id == &reg.id
                                                    )
                                                });
                                                let chip_label = if already_pos.is_some() {
                                                    format!("✓ Region {}", r_idx + 1)
                                                } else {
                                                    format!("+ Region {}", r_idx + 1)
                                                };
                                                if ui
                                                    .small_button(chip_label)
                                                    .on_hover_text(format!(
                                                        "Toggle closed region '{}' in sketch '{}'",
                                                        reg.id, sk.name
                                                    ))
                                                    .clicked()
                                                {
                                                    if let Some(pos) = already_pos {
                                                        op.selections.remove(pos);
                                                        *mutated_label =
                                                            Some("Remove Sketch Region");
                                                    } else {
                                                        op.selections.push(
                                                            GeometrySelectionItem::SketchRegion {
                                                                sketch_id: sk.id.clone(),
                                                                region_id: reg.id.clone(),
                                                            },
                                                        );
                                                        *mutated_label = Some("Add Sketch Region");
                                                    }
                                                }
                                            }
                                        }
                                    });
                                },
                            );
                        }
                    });
                }
            },
        );
    }

    let sel_title = format!("Selected Entities ({})", op.selections.len());
    section_card(
        ui,
        ("geom_sel_card", setup_idx, op_idx, pick_default_open),
        &sel_title,
        pick_default_open || !op.selections.is_empty(),
        |ui| {
            if op.selections.is_empty() {
                let msg = if op.kind.is_rotary_native() {
                    "Switch Axial Boundary Source to 'Selected Geometry (-U..+U)' above and pick 3D faces, loops, edges, or sketches to bound the rotary cut axially."
                } else if op.kind == OperationKind::Facing {
                    "Facing uses the Cutting Area boundary above (defaults to Stock Bounds). Select planar faces or closed 2D contours when using 'Selected Regions'."
                } else if op.kind == OperationKind::ThreadMill {
                    "Pick cylindrical holes/bosses in 3D or select circular 2D sketch contours."
                } else if op.kind == OperationKind::Deburr3D {
                    "Pick 3D B-Rep edge loops or open edge chains above, or leave empty to auto-detect sharp convex 3D edges inside the Machining Boundary."
                } else if op.kind.is_3d_surface() || op.kind == OperationKind::Undercut3D {
                    "Using automatic boundary source above. Switch Boundary Source to 'Selected Regions' and pick 2D contours or planar faces to constrain XY machining."
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
                if small_icon_button(
                    ui,
                    IconKind::Trash,
                    "Remove this geometry item from the operation",
                )
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
