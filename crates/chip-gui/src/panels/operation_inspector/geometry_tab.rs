use chip_cad::{CadDocument, FaceId, find_coplanar_faces, find_same_diameter_holes};
use chip_cam::{
    ExtractedFeature25D, ExtractedSelectionSummary, GeometrySelectionItem, MachiningBoundaryMode,
    Operation, OperationKind, PocketBoundaryMode, ResolvedHeights, ResolvedRotaryWindow,
    ResolvedSetup, RotaryAxialBoundaryMode, SharedAvoidRegion, StrategyParams, SurfaceMesh3D,
    ToolContainmentMode,
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
    viewport::{BoundaryPreviewOverlay, resolve_rotary_window_for_preview},
    widgets::{
        BannerKind, INPUT_COL_WIDTH, edit_param_length, grid_row, inline_banner, inspector_grid,
        section_card, static_card,
    },
};

const COMBO_COL_WIDTH: f32 = INPUT_COL_WIDTH + 54.0;

const CONTAINMENT_ORDER: [ToolContainmentMode; 3] = [
    ToolContainmentMode::ToolInsideBoundary,
    ToolContainmentMode::ToolCenterOnBoundary,
    ToolContainmentMode::ToolOutsideBoundary,
];

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

fn boundary_source_ui_label(mode: MachiningBoundaryMode) -> &'static str {
    match mode {
        MachiningBoundaryMode::StockBox => "Stock Bounds",
        MachiningBoundaryMode::ModelBoundingBox => "Model Bounds",
        MachiningBoundaryMode::ModelSilhouette => "Model Silhouette",
        MachiningBoundaryMode::SelectedRegions => "Selected 2D Contours",
    }
}

fn rotary_source_ui_label(mode: RotaryAxialBoundaryMode) -> &'static str {
    match mode {
        RotaryAxialBoundaryMode::StockBounds => "Stock Bounds",
        RotaryAxialBoundaryMode::ModelBounds => "Model Bounds",
        RotaryAxialBoundaryMode::SelectedGeometry => "Selected Geometry (-U..+U)",
        RotaryAxialBoundaryMode::ExplicitRange => "Explicit Range",
    }
}

fn rotary_containment_ui_label(mode: ToolContainmentMode) -> &'static str {
    match mode {
        ToolContainmentMode::ToolInsideBoundary => "Tool Inside",
        ToolContainmentMode::ToolCenterOnBoundary => "Tool Center",
        ToolContainmentMode::ToolOutsideBoundary => "Tool Outside",
    }
}

fn pocket_area_ui_label(mode: PocketBoundaryMode) -> &'static str {
    match mode {
        PocketBoundaryMode::SelectedPockets => "Inside Selected Pockets",
        PocketBoundaryMode::ClearOutsideSelectedToStock => "Clear Outside Selected Bosses to Stock",
    }
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

#[allow(clippy::too_many_arguments)]
fn show_machining_boundary_card(
    ui: &mut egui::Ui,
    op: &mut Operation,
    setup_idx: usize,
    op_idx: usize,
    display_unit: DisplayUnit,
    eval_ctx: &EvalContext,
    active_tool: Option<&Tool>,
    boundary_preview: Option<&BoundaryPreviewOverlay>,
    mutated_label: &mut Option<&'static str>,
) {
    let op_kind = op.kind;
    let selections_empty = op.selections.is_empty();
    let Some(boundary) = op.machining_boundary_mut() else {
        return;
    };

    section_card(
        ui,
        ("geom_cutting_area_card", setup_idx, op_idx),
        "Cutting Area / Machining Boundary",
        true,
        |ui| {
            inspector_grid(ui, ("geom_cutting_area_grid", setup_idx, op_idx), |ui| {
                grid_row(
                    ui,
                    "Boundary Source",
                    Some(
                        "2D XY region constraining this operation (Stock Bounds, Model Bounds rectangle, projected Model Silhouette, or Selected 2D Contours)",
                    ),
                    |ui| {
                        let prev = boundary.boundary_mode;
                        egui::ComboBox::from_id_salt(("geom_bmode", setup_idx, op_idx))
                            .width(COMBO_COL_WIDTH)
                            .truncate()
                            .selected_text(boundary_source_ui_label(boundary.boundary_mode))
                            .show_ui(ui, |ui| {
                                for m in MachiningBoundaryMode::ALL {
                                    ui.selectable_value(
                                        &mut boundary.boundary_mode,
                                        m,
                                        boundary_source_ui_label(m),
                                    )
                                    .on_hover_text(format!(
                                        "Constrain XY cutting area using {}",
                                        boundary_source_ui_label(m)
                                    ));
                                }
                            })
                            .response
                            .on_hover_text("Select 2D XY machining boundary source");
                        if boundary.boundary_mode != prev {
                            *mutated_label = Some("Change Machining Boundary Source");
                        }
                    },
                );

                if boundary.boundary_mode == MachiningBoundaryMode::ModelBoundingBox {
                    grid_row(
                        ui,
                        "-X Offset",
                        Some(
                            "Signed offset on the -X side of Model Bounds (+ expands outward toward -X, - insets inward toward +X)",
                        ),
                        |ui| {
                            if edit_param_length(
                                ui,
                                ("geom_mb_xneg", setup_idx, op_idx),
                                &mut boundary.model_bounds_offset_x_neg,
                                eval_ctx,
                                INPUT_COL_WIDTH,
                            ) {
                                *mutated_label = Some("Edit Model Bounds -X Offset");
                            }
                        },
                    );
                    grid_row(
                        ui,
                        "+X Offset",
                        Some(
                            "Signed offset on the +X side of Model Bounds (+ expands outward toward +X, - insets inward toward -X)",
                        ),
                        |ui| {
                            if edit_param_length(
                                ui,
                                ("geom_mb_xpos", setup_idx, op_idx),
                                &mut boundary.model_bounds_offset_x_pos,
                                eval_ctx,
                                INPUT_COL_WIDTH,
                            ) {
                                *mutated_label = Some("Edit Model Bounds +X Offset");
                            }
                        },
                    );
                    grid_row(
                        ui,
                        "-Y Offset",
                        Some(
                            "Signed offset on the -Y side of Model Bounds (+ expands outward toward -Y, - insets inward toward +Y)",
                        ),
                        |ui| {
                            if edit_param_length(
                                ui,
                                ("geom_mb_yneg", setup_idx, op_idx),
                                &mut boundary.model_bounds_offset_y_neg,
                                eval_ctx,
                                INPUT_COL_WIDTH,
                            ) {
                                *mutated_label = Some("Edit Model Bounds -Y Offset");
                            }
                        },
                    );
                    grid_row(
                        ui,
                        "+Y Offset",
                        Some(
                            "Signed offset on the +Y side of Model Bounds (+ expands outward toward +Y, - insets inward toward -Y)",
                        ),
                        |ui| {
                            if edit_param_length(
                                ui,
                                ("geom_mb_ypos", setup_idx, op_idx),
                                &mut boundary.model_bounds_offset_y_pos,
                                eval_ctx,
                                INPUT_COL_WIDTH,
                            ) {
                                *mutated_label = Some("Edit Model Bounds +Y Offset");
                            }
                        },
                    );
                }

                if boundary.boundary_mode == MachiningBoundaryMode::ModelSilhouette {
                    grid_row(
                        ui,
                        "Interior Holes",
                        Some(
                            "Fill interior through-holes in the projected Model Silhouette so 3D passes machine through voids instead of leaving uncut stock pillars",
                        ),
                        |ui| {
                            if ui
                                .checkbox(&mut boundary.silhouette_fill_holes, "Fill through-holes")
                                .on_hover_text(
                                    "Checked: machine across interior through-holes inside the outer silhouette. Unchecked: keep interior through-hole voids outside the cutting boundary.",
                                )
                                .changed()
                            {
                                *mutated_label = Some("Toggle Silhouette Fill Through-Holes");
                            }
                        },
                    );
                }

                grid_row(
                    ui,
                    "Tool Containment",
                    Some(
                        "Controls whether tool edge stays inside, center rides on, or edge extends outside the nominal boundary",
                    ),
                    |ui| {
                        let prev = boundary.tool_containment;
                        egui::ComboBox::from_id_salt(("geom_tcont", setup_idx, op_idx))
                            .width(COMBO_COL_WIDTH)
                            .truncate()
                            .selected_text(boundary.tool_containment.label())
                            .show_ui(ui, |ui| {
                                for c in CONTAINMENT_ORDER {
                                    ui.selectable_value(
                                        &mut boundary.tool_containment,
                                        c,
                                        c.label(),
                                    )
                                    .on_hover_text(format!("Position cutter {}", c.label()));
                                }
                            })
                            .response
                            .on_hover_text(
                                "Control cutter position relative to the machining boundary",
                            );
                        if boundary.tool_containment != prev {
                            *mutated_label = Some("Change Tool Containment Mode");
                        }
                    },
                );

                grid_row(
                    ui,
                    "Boundary Offset",
                    Some(
                        "Signed XY offset applied to the machining boundary (+ expands outward, - insets inward)",
                    ),
                    |ui| {
                        if edit_param_length(
                            ui,
                            ("geom_boff", setup_idx, op_idx),
                            &mut boundary.boundary_offset,
                            eval_ctx,
                            INPUT_COL_WIDTH,
                        ) {
                            *mutated_label = Some("Edit Boundary Offset");
                        }
                    },
                );
            });

            // Live resolved shift / XY span readout
            let tok = ThemeTokens::from_ui(ui);
            let tool_r = active_tool.map(|t| 0.5 * t.geometry.cutting_diameter_mm());
            let shift_str = tool_r.map(|r| {
                let c_shift = match boundary.tool_containment {
                    ToolContainmentMode::ToolInsideBoundary => -r,
                    ToolContainmentMode::ToolCenterOnBoundary => 0.0,
                    ToolContainmentMode::ToolOutsideBoundary => r,
                };
                let net = c_shift + boundary.boundary_offset.value_mm;
                let sign = if net > 1e-6 { "+" } else { "" };
                format!(
                    "Tool-Center Shift: {sign}{} (Ø{})",
                    display_unit.format_length(net),
                    display_unit.format_length(2.0 * r)
                )
            });
            let span_str = if let Some(BoundaryPreviewOverlay::Planar2D { nominal_region, .. }) =
                boundary_preview
                && let Some(aabb) = region_set_aabb(nominal_region)
            {
                let dx = aabb.max.x - aabb.min.x;
                let dy = aabb.max.y - aabb.min.y;
                Some(format!(
                    "XY: {} × {}",
                    display_unit.format_length(dx),
                    display_unit.format_length(dy)
                ))
            } else {
                None
            };
            if shift_str.is_some() || span_str.is_some() {
                ui.add_space(4.0);
                let summary = match (span_str, shift_str) {
                    (Some(a), Some(b)) => format!("{a}  ·  {b}"),
                    (Some(a), None) => a,
                    (None, Some(b)) => b,
                    (None, None) => String::new(),
                };
                ui.colored_label(tok.text_secondary, summary);
            }

            if boundary.boundary_mode == MachiningBoundaryMode::SelectedRegions && selections_empty
            {
                ui.add_space(4.0);
                inline_banner(
                    ui,
                    BannerKind::Warn,
                    "Selected 2D Contours is active with no geometry selected. Pick closed 2D contours, planar faces, or sketch regions below.",
                );
            } else if let Some(BoundaryPreviewOverlay::Planar2D { warnings, .. }) = boundary_preview
            {
                for w in warnings {
                    ui.add_space(4.0);
                    inline_banner(ui, BannerKind::Warn, w);
                }
            }

            if op_kind == OperationKind::Deburr3D {
                ui.add_space(4.0);
                inline_banner(
                    ui,
                    BannerKind::Info,
                    "XY Machining Boundary constrains Auto-Detect Edges mode. Pick explicit 3D edge loops, open chains, or faces below to override.",
                );
            } else if op_kind == OperationKind::Undercut3D {
                ui.add_space(4.0);
                inline_banner(
                    ui,
                    BannerKind::Info,
                    "Scans overhanging downward-facing surfaces within the Heights Z-band and XY Machining Boundary while trimming neck/shank collisions.",
                );
            }
        },
    );
}

#[allow(clippy::too_many_arguments)]
fn show_rotary_cutting_area_card(
    ui: &mut egui::Ui,
    op: &mut Operation,
    setup_idx: usize,
    op_idx: usize,
    display_unit: DisplayUnit,
    eval_ctx: &EvalContext,
    cad_doc: Option<&CadDocument>,
    resolved_setup: Option<&ResolvedSetup>,
    active_tool: Option<&Tool>,
    surface_mesh: Option<&SurfaceMesh3D>,
    resolved_heights: Option<&ResolvedHeights>,
    boundary_preview: Option<&BoundaryPreviewOverlay>,
    extracted: Option<&ExtractedSelectionSummary>,
    mutated_label: &mut Option<&'static str>,
) {
    let selections_empty = op.selections.is_empty();
    let mut mode_after = RotaryAxialBoundaryMode::ModelBounds;
    let mut changed_any = false;

    section_card(
        ui,
        ("geom_rotary_cutting_area_card", setup_idx, op_idx),
        "4th-Axis Rotary Cutting Area (-U / +U)",
        true,
        |ui| {
            if let Some(rb) = op.rotary_axial_boundary_mut() {
                inspector_grid(ui, ("geom_rotary_area_grid", setup_idx, op_idx), |ui| {
                    grid_row(
                        ui,
                        "Axial Source",
                        Some(
                            "Source determining the nominal -U..+U axial span along the rotary centerline",
                        ),
                        |ui| {
                            let prev = rb.axial_boundary_mode;
                            egui::ComboBox::from_id_salt(("geom_rot_bmode", setup_idx, op_idx))
                                .width(COMBO_COL_WIDTH)
                                .truncate()
                                .selected_text(rotary_source_ui_label(rb.axial_boundary_mode))
                                .show_ui(ui, |ui| {
                                    for m in RotaryAxialBoundaryMode::ALL {
                                        ui.selectable_value(
                                            &mut rb.axial_boundary_mode,
                                            m,
                                            rotary_source_ui_label(m),
                                        )
                                        .on_hover_text(
                                            format!(
                                                "Bound rotary cut along -U/+U using {}",
                                                rotary_source_ui_label(m)
                                            ),
                                        );
                                    }
                                })
                                .response
                                .on_hover_text(
                                    "Select how -U..+U axial cutting limits are derived",
                                );
                            if rb.axial_boundary_mode != prev {
                                *mutated_label = Some("Change Rotary Axial Boundary Source");
                                changed_any = true;
                            }
                        },
                    );

                    grid_row(
                        ui,
                        "Axial Containment",
                        Some(
                            "Tool Inside keeps full cutter diameter inside -U/+U; Tool Center rides on -U/+U; Tool Outside extends by tool radius for shoulder relief (chuck-safe)",
                        ),
                        |ui| {
                            let prev = rb.axial_containment;
                            egui::ComboBox::from_id_salt(("geom_rot_tcont", setup_idx, op_idx))
                                .width(COMBO_COL_WIDTH)
                                .truncate()
                                .selected_text(rotary_containment_ui_label(rb.axial_containment))
                                .show_ui(ui, |ui| {
                                    for c in CONTAINMENT_ORDER {
                                        ui.selectable_value(
                                            &mut rb.axial_containment,
                                            c,
                                            rotary_containment_ui_label(c),
                                        )
                                        .on_hover_text(c.label());
                                    }
                                })
                                .response
                                .on_hover_text(
                                    "Position tool perimeter relative to -U / +U axial limits",
                                );
                            if rb.axial_containment != prev {
                                *mutated_label = Some("Change Rotary Axial Containment");
                                changed_any = true;
                            }
                        },
                    );

                    if rb.axial_boundary_mode == RotaryAxialBoundaryMode::ExplicitRange {
                        grid_row(
                            ui,
                            "Start U (-U)",
                            Some("Explicit starting coordinate along the rotary axis (-U end)"),
                            |ui| {
                                if edit_param_length(
                                    ui,
                                    ("geom_rot_u_start", setup_idx, op_idx),
                                    &mut rb.explicit_u_start,
                                    eval_ctx,
                                    INPUT_COL_WIDTH,
                                ) {
                                    *mutated_label = Some("Edit Rotary Explicit Start U");
                                    changed_any = true;
                                }
                            },
                        );
                        grid_row(
                            ui,
                            "End U (+U)",
                            Some("Explicit ending coordinate along the rotary axis (+U end)"),
                            |ui| {
                                if edit_param_length(
                                    ui,
                                    ("geom_rot_u_end", setup_idx, op_idx),
                                    &mut rb.explicit_u_end,
                                    eval_ctx,
                                    INPUT_COL_WIDTH,
                                ) {
                                    *mutated_label = Some("Edit Rotary Explicit End U");
                                    changed_any = true;
                                }
                            },
                        );
                    }

                    grid_row(
                        ui,
                        "Start Margin (-U)",
                        Some(
                            "Signed axial margin at the -U end: positive (> 0) trims inward, negative (< 0) extends outward for shoulder relief while strictly respecting the rotary chuck keep-out guard",
                        ),
                        |ui| {
                            if edit_param_length(
                                ui,
                                ("geom_rot_m_start", setup_idx, op_idx),
                                &mut rb.axial_margin_start,
                                eval_ctx,
                                INPUT_COL_WIDTH,
                            ) {
                                *mutated_label = Some("Edit Rotary Start Margin (-U)");
                                changed_any = true;
                            }
                        },
                    );

                    grid_row(
                        ui,
                        "End Margin (+U)",
                        Some(
                            "Signed axial margin at the +U end: positive (> 0) trims inward, negative (< 0) extends outward for shoulder relief while strictly respecting the rotary chuck keep-out guard",
                        ),
                        |ui| {
                            if edit_param_length(
                                ui,
                                ("geom_rot_m_end", setup_idx, op_idx),
                                &mut rb.axial_margin_end,
                                eval_ctx,
                                INPUT_COL_WIDTH,
                            ) {
                                *mutated_label = Some("Edit Rotary End Margin (+U)");
                                changed_any = true;
                            }
                        },
                    );

                    grid_row(
                        ui,
                        "Start Angle (°)",
                        Some(
                            "Starting angular boundary around the 4th rotary axis (0° to 360° = full circumference)",
                        ),
                        |ui| {
                            if ui
                                .add(
                                    egui::DragValue::new(&mut rb.start_angle_deg)
                                        .speed(2.0)
                                        .range(-360.0..=360.0)
                                        .suffix("°"),
                                )
                                .on_hover_text("Start angle around the 4th rotary axis")
                                .changed()
                            {
                                if rb.end_angle_deg <= rb.start_angle_deg + 1.0 {
                                    rb.end_angle_deg = rb.start_angle_deg + 1.0;
                                }
                                *mutated_label = Some("Edit Rotary Start Angle");
                                changed_any = true;
                            }
                        },
                    );

                    grid_row(
                        ui,
                        "End Angle (°)",
                        Some(
                            "Ending angular boundary around the 4th rotary axis (0° to 360° = full circumference)",
                        ),
                        |ui| {
                            if ui
                                .add(
                                    egui::DragValue::new(&mut rb.end_angle_deg)
                                        .speed(2.0)
                                        .range((rb.start_angle_deg + 1.0)..=720.0)
                                        .suffix("°"),
                                )
                                .on_hover_text("End angle around the 4th rotary axis")
                                .changed()
                            {
                                *mutated_label = Some("Edit Rotary End Angle");
                                changed_any = true;
                            }
                        },
                    );

                    grid_row(
                        ui,
                        "Angular Presets",
                        Some("Quick angular sector presets around the 4th rotary axis"),
                        |ui| {
                            for (lbl, s_deg, e_deg) in [
                                ("Full 360°", 0.0_f64, 360.0_f64),
                                ("0–180°", 0.0_f64, 180.0_f64),
                                ("±90°", -90.0_f64, 90.0_f64),
                            ] {
                                let active = (rb.start_angle_deg - s_deg).abs() < 0.05
                                    && (rb.end_angle_deg - e_deg).abs() < 0.05;
                                if ui
                                    .selectable_label(active, lbl)
                                    .on_hover_text(format!(
                                        "Set angular window to {s_deg:.0}°..{e_deg:.0}°"
                                    ))
                                    .clicked()
                                {
                                    rb.start_angle_deg = s_deg;
                                    rb.end_angle_deg = e_deg;
                                    *mutated_label = Some("Apply Rotary Angular Preset");
                                    changed_any = true;
                                }
                            }
                        },
                    );
                });
                mode_after = rb.axial_boundary_mode;
            }

            // Live resolved rotary window readout (reuse cached overlay unless mutated this frame)
            let resolved_window: Option<Result<ResolvedRotaryWindow, String>> = if !changed_any
                && let Some(BoundaryPreviewOverlay::RotarySector { window, .. }) = boundary_preview
            {
                Some(Ok(window.clone()))
            } else if let Some(rs) = resolved_setup {
                resolve_rotary_window_for_preview(
                    op,
                    rs,
                    cad_doc,
                    surface_mesh,
                    extracted,
                    active_tool,
                    resolved_heights,
                    eval_ctx,
                )
            } else {
                None
            };

            ui.add_space(5.0);
            let tok = ThemeTokens::from_ui(ui);
            match resolved_window {
                Some(Ok(win)) => {
                    let u_span_str = format!(
                        "Effective U: [{:.2}, {:.2}] {} · Chuck Guard: {:.2} {}",
                        display_unit.from_mm(win.u_start),
                        display_unit.from_mm(win.u_end),
                        display_unit.suffix(),
                        display_unit.from_mm(win.chuck_guard_u),
                        display_unit.suffix(),
                    );
                    ui.colored_label(tok.text_primary, u_span_str);
                    ui.colored_label(
                        tok.text_secondary,
                        format!(
                            "Sector: {:.0}°..{:.0}° · R: {:.2}..{:.2} {}",
                            win.start_angle_deg,
                            win.end_angle_deg,
                            display_unit.from_mm(win.r_floor),
                            display_unit.from_mm(win.r_ceiling),
                            display_unit.suffix(),
                        ),
                    );
                    if win.clamped_by_chuck {
                        ui.add_space(4.0);
                        inline_banner(
                            ui,
                            BannerKind::Warn,
                            format!(
                                "Clamped by Rotary Chuck Keep-Out Guard at {} (tool radius + 0.5 mm safety clearance).",
                                display_unit.format_length(win.chuck_guard_u)
                            ),
                        );
                    }
                }
                Some(Err(err)) => {
                    inline_banner(ui, BannerKind::Error, err);
                }
                None => {
                    if resolved_setup.and_then(|rs| rs.rotary).is_none() {
                        inline_banner(
                            ui,
                            BannerKind::Warn,
                            "4th-Axis Rotary is not enabled on the active Setup. Enable it in Setup → Rotary.",
                        );
                    }
                }
            }

            if mode_after == RotaryAxialBoundaryMode::SelectedGeometry && selections_empty {
                ui.add_space(4.0);
                inline_banner(
                    ui,
                    BannerKind::Warn,
                    "Selected Geometry (-U..+U) is active with no selections. Pick 3D faces, loops, edges, or sketches below (currently falling back to Model Bounds).",
                );
            }
        },
    );
}

fn show_pocket_cutting_area_card(
    ui: &mut egui::Ui,
    op: &mut Operation,
    setup_idx: usize,
    op_idx: usize,
    mutated_label: &mut Option<&'static str>,
) {
    let selections_empty = op.selections.is_empty();
    let StrategyParams::Pocket2D(p) = op.strategy_mut() else {
        return;
    };

    section_card(
        ui,
        ("geom_pocket_area_card", setup_idx, op_idx),
        "Cutting Area / Machining Boundary",
        true,
        |ui| {
            inspector_grid(ui, ("geom_pocket_area_grid", setup_idx, op_idx), |ui| {
                grid_row(
                    ui,
                    "Pocket Area Mode",
                    Some(
                        "Clear inside selected pockets, or treat selected contours/faces as raised bosses and clear outside them to Setup Stock XY bounds",
                    ),
                    |ui| {
                        let prev = p.boundary_mode;
                        egui::ComboBox::from_id_salt(("geom_pkt_bmode", setup_idx, op_idx))
                            .width(COMBO_COL_WIDTH)
                            .truncate()
                            .selected_text(pocket_area_ui_label(p.boundary_mode))
                            .show_ui(ui, |ui| {
                                for m in PocketBoundaryMode::ALL {
                                    ui.selectable_value(
                                        &mut p.boundary_mode,
                                        m,
                                        pocket_area_ui_label(m),
                                    )
                                    .on_hover_text(pocket_area_ui_label(m));
                                }
                            })
                            .response
                            .on_hover_text("Choose whether to clear inside selected pockets or outside selected bosses to stock");
                        if p.boundary_mode != prev {
                            *mutated_label = Some("Change Pocket Area Mode");
                        }
                    },
                );

                if p.boundary_mode == PocketBoundaryMode::SelectedPockets {
                    grid_row(
                        ui,
                        "Open Outer Boundary",
                        Some(
                            "Treat the outer boundary of selected 2D sketches or closed loops as open air so the cutter enters smoothly from outside",
                        ),
                        |ui| {
                            if ui
                                .checkbox(
                                    &mut p.treat_outer_boundary_as_open,
                                    "Treat Outer Loop as Open Air",
                                )
                                .on_hover_text(
                                    "Allow horizontal open-air entry across the outer boundary of 2D sketch or closed-loop selections",
                                )
                                .changed()
                            {
                                *mutated_label = Some("Toggle Open Outer Boundary");
                            }
                        },
                    );
                }
            });

            if p.boundary_mode == PocketBoundaryMode::ClearOutsideSelectedToStock {
                ui.add_space(4.0);
                if selections_empty {
                    inline_banner(
                        ui,
                        BannerKind::Warn,
                        "Select one or more planar faces or closed 2D contours below to leave standing as raised bosses while clearing surrounding stock.",
                    );
                } else {
                    inline_banner(
                        ui,
                        BannerKind::Info,
                        "Clearing stock outside selected bosses out to Setup Stock XY bounds (shown in 3D viewport) with open-air entry.",
                    );
                }
            }
        },
    );
}

fn show_contour_cutting_area_card(
    ui: &mut egui::Ui,
    op: &mut Operation,
    setup_idx: usize,
    op_idx: usize,
    eval_ctx: &EvalContext,
    mutated_label: &mut Option<&'static str>,
) {
    let StrategyParams::Contour2D(p) = op.strategy_mut() else {
        return;
    };

    section_card(
        ui,
        ("geom_contour_area_card", setup_idx, op_idx),
        "Cutting Area / Machining Boundary",
        true,
        |ui| {
            inspector_grid(ui, ("geom_contour_area_grid", setup_idx, op_idx), |ui| {
                grid_row(
                    ui,
                    "Stock Clip",
                    Some("Trim generated 2D contour passes to remain inside Setup Stock XY bounds"),
                    |ui| {
                        if ui
                            .checkbox(&mut p.clip_to_stock_bounds, "Clip to Stock Bounds")
                            .on_hover_text(
                                "Keep 2D contour toolpaths clipped inside Setup Stock XY bounds",
                            )
                            .changed()
                        {
                            *mutated_label = Some("Toggle Contour Stock Clipping");
                        }
                    },
                );
                grid_row(
                    ui,
                    "Open Chain Extension",
                    Some("Extend open 2D profile start and end tangentially past stock edges"),
                    |ui| {
                        if edit_param_length(
                            ui,
                            ("geom_contour_open_ext", setup_idx, op_idx),
                            &mut p.open_chain_extension,
                            eval_ctx,
                            INPUT_COL_WIDTH,
                        ) {
                            *mutated_label = Some("Edit Open Chain Extension");
                        }
                    },
                );
            });
        },
    );
}

fn region_set_aabb(region: &chip_cam::RegionSet2D) -> Option<chip_math::Aabb2> {
    let mut out = chip_math::Aabb2::EMPTY;
    for r in &region.regions {
        let b = r.outer_ccw.exact_aabb();
        if b.is_valid() {
            out.extend_point(b.min);
            out.extend_point(b.max);
        }
    }
    out.is_valid().then_some(out)
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
