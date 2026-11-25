pub mod avoid_card;
pub mod geometry_tab;
pub mod heights_tab;
pub mod linking_tab;
pub mod passes_tab;
pub mod tool_tab;

use chip_cad::CadDocument;
use chip_cam::{
    ExtractedSelectionSummary, OperationStatus, ResolvedHeights, ResolvedSetup, SharedAvoidRegion,
    SurfaceMesh3D,
};
use chip_math::{DVec3, EvalContext};
use chip_project::{ProjectDocument, UndoHistory};
use chip_tools::{ToolLibrary, ToolOperationClass};

use self::avoid_card::{
    AvoidCardAction, AvoidCardSetupView, create_and_link_shared_avoid,
    detach_shared_avoid_to_local, mark_ops_dirty_with_rest, mark_shared_avoid_dependents_dirty,
    promote_local_avoid_to_shared, set_shared_avoid_for_all_setup_ops, toggle_shared_avoid_link,
};
use crate::{
    panels::{
        GeometryPickTarget, OperationTab, ViewportPickMode, browser::default_pick_mode_for_op,
    },
    theme::ThemeTokens,
    viewport::BoundaryPreviewOverlay,
    widgets::{BannerKind, IconKind, inline_banner, inline_icon, static_card},
};

/// Side-effect actions emitted by [`show_operation_inspector`] back to the host shell.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OperationInspectorAction {
    /// User clicked `+ New Sketch` inside the empty-state `2D Sketch Layers & Quick Select` card.
    CreateSketch,
    /// User clicked the read-only `Project Material: <name>` pill in `[Tool]` to inspect/edit
    /// the project-level workpiece material.
    OpenProjectMaterial,
}

/// Context passed to [`show_operation_inspector`].
pub struct OperationInspectorCtx<'a> {
    pub project: &'a mut ProjectDocument,
    pub setup_idx: usize,
    pub op_idx: usize,
    pub global_tools: &'a ToolLibrary,
    pub history: &'a mut UndoHistory,
    pub cad_doc: Option<&'a CadDocument>,
    pub resolved_setup: Option<&'a ResolvedSetup>,
    pub surface_mesh: Option<&'a SurfaceMesh3D>,
    pub boundary_preview: Option<&'a BoundaryPreviewOverlay>,
    pub selected_face_normal_wcs: Option<DVec3>,
    pub extracted: Option<Result<&'a ExtractedSelectionSummary, &'a str>>,
    pub tab: &'a mut OperationTab,
    pub pick_mode: &'a mut ViewportPickMode,
    pub pick_target: &'a mut GeometryPickTarget,
    pub editing_shared_avoid_id: &'a mut Option<String>,
    pub lock_chipload: &'a mut bool,
    pub show_z_planes: &'a mut bool,
    pub auto_rotate_indexed_op: &'a mut bool,
    pub open_tool_manager: &'a mut bool,
    pub sim_engine: chip_sim::SimEngineKind,
}

/// Renders the standardized 5-tab **Operation Inspector** (`[Tool]`, `[Geometry]`, `[Heights]`, `[Passes]`, `[Linking]`).
pub fn show_operation_inspector(
    ui: &mut egui::Ui,
    ctx: OperationInspectorCtx<'_>,
) -> Option<OperationInspectorAction> {
    let OperationInspectorCtx {
        project,
        setup_idx,
        op_idx,
        global_tools,
        history,
        cad_doc,
        resolved_setup,
        surface_mesh,
        boundary_preview,
        selected_face_normal_wcs,
        extracted,
        tab,
        pick_mode,
        pick_target,
        editing_shared_avoid_id,
        lock_chipload,
        show_z_planes,
        auto_rotate_indexed_op,
        open_tool_manager,
        sim_engine,
    } = ctx;

    if setup_idx >= project.setups.len() || op_idx >= project.setups[setup_idx].operations.len() {
        inline_banner(ui, BannerKind::Info, "No Operation selected.");
        return None;
    }

    let display_unit = project.display_units;
    // Snapshot only the single active Operation (and the actively edited SharedAvoidRegion, if any)
    // so idle 60 FPS frames perform zero `ProjectDocument` clones!
    let pre_op = project.setups[setup_idx].operations[op_idx].clone();
    let pre_shared: Option<(usize, SharedAvoidRegion)> = editing_shared_avoid_id
        .as_deref()
        .and_then(|eid| project.avoid_regions.iter().position(|r| r.id == eid))
        .map(|idx| (idx, project.avoid_regions[idx].clone()));
    let pre_tools_len = project.tools.len();
    let mut mutated_label: Option<&'static str> = None;
    let mut shared_mutated_label: Option<(&'static str, String)> = None;
    let mut deferred_avoid_action: Option<AvoidCardAction> = None;
    let mut create_sketch_requested = false;
    let mut open_project_material_requested = false;

    let op_kind = project.setups[setup_idx].operations[op_idx].kind;
    let op_is_adaptive = project.setups[setup_idx].operations[op_idx].is_adaptive_2d();
    let tok = ThemeTokens::from_ui(ui);
    static_card(ui, None, |ui| {
        ui.horizontal(|ui| {
            inline_icon(
                ui,
                IconKind::for_operation(op_kind, op_is_adaptive),
                tok.accent,
                15.0,
            );
            ui.strong(op_kind.label());
            let name_ref = &mut project.setups[setup_idx].operations[op_idx].name;
            let resp = ui
                .add(
                    egui::TextEdit::singleline(name_ref)
                        .desired_width(ui.available_width().max(80.0)),
                )
                .on_hover_text(
                    "Operation display name shown in the CAM Browser and posted G-code comments",
                );
            if resp.lost_focus() && project.setups[setup_idx].operations[op_idx].name != pre_op.name
            {
                mutated_label = Some("Rename Operation");
            }
        });
        ui.add_space(4.0);
        ui.separator();
        ui.add_space(2.0);
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 4.0;
            if ui
                .selectable_value(tab, OperationTab::Tool, "Tool")
                .on_hover_text("Select cutter, material preset, spindle RPM, surface speed, feedrate, and coolant")
                .clicked()
            {
                *pick_mode = ViewportPickMode::InspectEntity;
                *pick_target = GeometryPickTarget::CuttingArea;
                *editing_shared_avoid_id = None;
            }
            if ui
                .selectable_value(tab, OperationTab::Geometry, "Geometry")
                .on_hover_text("Pick 3D CAD faces, edges, holes, or 2D sketch regions machined by this operation")
                .clicked()
            {
                *pick_mode = default_pick_mode_for_op(setup_idx, op_idx, op_kind, *tab);
            }
            if ui
                .selectable_value(tab, OperationTab::Heights, "Heights")
                .on_hover_text("Configure Clearance, Retract, Feed, Top, and Bottom Z planes for this operation")
                .clicked()
            {
                *pick_mode = ViewportPickMode::InspectEntity;
                *pick_target = GeometryPickTarget::CuttingArea;
                *editing_shared_avoid_id = None;
            }
            if ui
                .selectable_value(tab, OperationTab::Passes, "Passes")
                .on_hover_text("Configure cutting strategy parameters, stepover, stepdown, stock-to-leave, and tabs")
                .clicked()
            {
                *pick_mode = ViewportPickMode::InspectEntity;
                *pick_target = GeometryPickTarget::CuttingArea;
                *editing_shared_avoid_id = None;
            }
            if ui
                .selectable_value(tab, OperationTab::Linking, "Linking")
                .on_hover_text("Configure retract policy, stay-down distance, helical/linear ramps, and tangent lead-in/out")
                .clicked()
            {
                *pick_mode = ViewportPickMode::InspectEntity;
                *pick_target = GeometryPickTarget::CuttingArea;
                *editing_shared_avoid_id = None;
            }
        });
    });

    if let OperationStatus::NeedsRepick { message } = project.setups[setup_idx].operations[op_idx]
        .runtime_status
        .clone()
    {
        static_card(ui, Some("CAD Feature Needs Re-Pick"), |ui| {
            inline_banner(ui, BannerKind::Warn, &message);
            ui.add_space(4.0);
            if ui
                .button("Clear Orphaned & Re-Pick in 3D")
                .on_hover_text(
                    "Clears orphaned selections and activates 3D picking while preserving tool, speeds/feeds, heights, passes, and tabs.",
                )
                .clicked()
            {
                project.setups[setup_idx].operations[op_idx]
                    .selections
                    .clear();
                *tab = OperationTab::Geometry;
                *pick_mode = default_pick_mode_for_op(
                    setup_idx,
                    op_idx,
                    op_kind,
                    OperationTab::Geometry,
                );
                mutated_label = Some("Clear Orphaned Feature for Re-Pick");
            }
        });
    }

    let mut eval_ctx = resolved_setup
        .map(|rs| rs.eval_context(display_unit))
        .unwrap_or_else(|| EvalContext::new(display_unit));

    if let Some(tool) = project.setups[setup_idx].operations[op_idx]
        .tool_id
        .as_ref()
        .and_then(|tid| project.tool(tid).or_else(|| global_tools.tool(tid)))
    {
        tool.populate_eval_context(&mut eval_ctx);
    }

    let (inf_top, inf_bot) = match extracted {
        Some(Ok(sum)) => {
            eval_ctx.set_var("feature_top_z", sum.inferred_top_z_wcs);
            eval_ctx.set_var("feature_bottom_z", sum.inferred_bottom_z_wcs);
            if let Some(hd) = sum.primary_hole_diameter_mm {
                eval_ctx.set_var("hole_d", hd);
            }
            (sum.inferred_top_z_wcs, sum.inferred_bottom_z_wcs)
        }
        _ => {
            let st = resolved_setup.map_or(16.0, |rs| rs.stock.aabb_wcs().max.z);
            let mt = resolved_setup.map_or(15.0, |rs| rs.model_aabb_wcs.max.z);
            (st, mt)
        }
    };

    let resolved_heights = resolved_setup.and_then(|rs| {
        let mut h_eval = project.setups[setup_idx].operations[op_idx].heights.clone();
        h_eval.reevaluate(&eval_ctx);
        ResolvedHeights::resolve(
            &h_eval,
            &rs.model_aabb_wcs,
            &rs.stock.aabb_wcs(),
            rs.clearance_z_wcs,
            rs.retract_z_wcs,
            inf_top,
            inf_bot,
        )
        .ok()
    });

    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| match *tab {
            OperationTab::Tool => tool_tab::show_tool_tab(
                ui,
                tool_tab::ToolTabCtx {
                    project,
                    setup_idx,
                    op_idx,
                    global_tools,
                    display_unit,
                    lock_chipload,
                    open_tool_manager,
                    open_project_material: &mut open_project_material_requested,
                    mutated_label: &mut mutated_label,
                },
            ),
            OperationTab::Geometry => {
                let active_tool = project.setups[setup_idx].operations[op_idx]
                    .tool_id
                    .as_ref()
                    .and_then(|tid| project.tool(tid).or_else(|| global_tools.tool(tid)))
                    .cloned();
                let avoid_setup_view = AvoidCardSetupView::from_project(project, setup_idx);
                geometry_tab::show_geometry_tab(
                    ui,
                    geometry_tab::GeometryTabCtx {
                        op: &mut project.setups[setup_idx].operations[op_idx],
                        shared_avoid_regions: &mut project.avoid_regions,
                        avoid_setup_view: &avoid_setup_view,
                        sketches: &project.sketches,
                        setup_idx,
                        op_idx,
                        display_unit,
                        eval_ctx: &eval_ctx,
                        cad_doc,
                        resolved_setup,
                        active_tool: active_tool.as_ref(),
                        surface_mesh,
                        resolved_heights: resolved_heights.as_ref(),
                        boundary_preview,
                        extracted,
                        pick_mode,
                        pick_target,
                        editing_shared_avoid_id,
                        mutated_label: &mut mutated_label,
                        shared_mutated_label: &mut shared_mutated_label,
                        deferred_avoid_action: &mut deferred_avoid_action,
                        create_sketch_requested: &mut create_sketch_requested,
                    },
                );
            }
            OperationTab::Heights => heights_tab::show_heights_tab(
                ui,
                heights_tab::HeightsTabCtx {
                    op: &mut project.setups[setup_idx].operations[op_idx],
                    setup_idx,
                    op_idx,
                    display_unit,
                    eval_ctx: &eval_ctx,
                    resolved_setup,
                    inf_top,
                    inf_bot,
                    show_z_planes,
                    mutated_label: &mut mutated_label,
                },
            ),
            OperationTab::Passes => {
                let preceding_ops: Vec<(String, String)> = project.setups[setup_idx].operations
                    [..op_idx]
                    .iter()
                    .filter(|o| !o.suppressed)
                    .map(|o| (o.id.clone(), o.name.clone()))
                    .collect();
                let assigned_tool_geom = project.setups[setup_idx].operations[op_idx]
                    .tool_id
                    .as_ref()
                    .and_then(|tid| project.tool(tid).or_else(|| global_tools.tool(tid)))
                    .map(|t| t.geometry.clone());
                let tool_feed_preview = project.setups[setup_idx].operations[op_idx]
                    .tool_id
                    .as_ref()
                    .and_then(|tid| project.tool(tid).or_else(|| global_tools.tool(tid)))
                    .map(|t| {
                        let eff = project.setups[setup_idx].operations[op_idx]
                            .effective_cutting_params(t);
                        (t.geometry.cutting_diameter_mm(), eff.cutting_feed_mm_min)
                    });
                let resolved_thread_depth_mm = resolved_heights
                    .as_ref()
                    .map(|rh| {
                        let axial_stock = project.setups[setup_idx].operations[op_idx]
                            .passes
                            .stock_to_leave_axial
                            .value_mm;
                        rh.top_z_wcs - (rh.bottom_z_wcs + axial_stock)
                    })
                    .or_else(|| {
                        extracted.and_then(|res| {
                            res.ok().map(|sum| {
                                let axial_stock = project.setups[setup_idx].operations[op_idx]
                                    .passes
                                    .stock_to_leave_axial
                                    .value_mm;
                                sum.inferred_top_z_wcs - (sum.inferred_bottom_z_wcs + axial_stock)
                            })
                        })
                    });
                let mut clearing_tool_candidates: Vec<passes_tab::ClearingToolCandidate> =
                    Vec::new();
                for t in project.tools.iter().chain(global_tools.tools.iter()) {
                    if t.geometry
                        .supports_operation(ToolOperationClass::VCarveFlatClearance)
                        && !clearing_tool_candidates.iter().any(|cand| cand.id == t.id)
                    {
                        let presets = t
                            .presets
                            .iter()
                            .map(|p| (p.id.clone(), p.name.clone()))
                            .collect();
                        clearing_tool_candidates.push(passes_tab::ClearingToolCandidate {
                            id: t.id.clone(),
                            label: format!(
                                "T{} — {} (Ø{:.2} mm)",
                                t.tool_number,
                                t.name,
                                t.geometry.cutting_diameter_mm()
                            ),
                            presets,
                        });
                    }
                }
                passes_tab::show_passes_tab(
                    ui,
                    passes_tab::PassesTabCtx {
                        op: &mut project.setups[setup_idx].operations[op_idx],
                        has_3d_model: cad_doc.is_some_and(CadDocument::has_3d_geometry),
                        preceding_ops: &preceding_ops,
                        clearing_tool_candidates: &clearing_tool_candidates,
                        assigned_tool_geom: assigned_tool_geom.as_ref(),
                        tool_feed_preview,
                        resolved_thread_depth_mm,
                        sim_engine,
                        display_unit,
                        setup_idx,
                        op_idx,
                        eval_ctx: &eval_ctx,
                        resolved_rotary: resolved_setup.and_then(|rs| rs.rotary),
                        selected_face_normal_wcs,
                        auto_rotate_indexed_op,
                        pick_mode,
                        mutated_label: &mut mutated_label,
                    },
                );
            }
            OperationTab::Linking => linking_tab::show_linking_tab(
                ui,
                &mut project.setups[setup_idx].operations[op_idx],
                setup_idx,
                op_idx,
                &eval_ctx,
                &mut mutated_label,
            ),
        });

    if let Some(action) = mutated_label {
        let maybe_embed = project.setups[setup_idx].operations[op_idx]
            .clearing_tool_id()
            .filter(|ctid| project.tool(ctid).is_none())
            .and_then(|ctid| global_tools.tool(ctid))
            .cloned();
        if let Some(gt) = maybe_embed {
            project.ensure_tool_embedded(&gt);
        }
        let mut pre_doc = project.clone();
        pre_doc.tools.truncate(pre_tools_len);
        pre_doc.setups[setup_idx].operations[op_idx] = pre_op;
        if let Some((s_idx, prev_sr)) = pre_shared.as_ref()
            && *s_idx < pre_doc.avoid_regions.len()
        {
            pre_doc.avoid_regions[*s_idx] = prev_sr.clone();
        }
        history.record_before_edit(action, None, &pre_doc);
        mark_ops_dirty_with_rest(&mut project.setups[setup_idx], &[op_idx]);
    } else if let Some((action, shared_id)) = shared_mutated_label {
        let mut pre_doc = project.clone();
        if let Some((s_idx, prev_sr)) = pre_shared
            && s_idx < pre_doc.avoid_regions.len()
        {
            pre_doc.avoid_regions[s_idx] = prev_sr;
        }
        history.record_before_edit(action, None, &pre_doc);
        mark_shared_avoid_dependents_dirty(project, &shared_id);
    }

    if let Some(avoid_act) = deferred_avoid_action {
        match avoid_act {
            AvoidCardAction::PromoteLocalToShared { name } => {
                let _ =
                    promote_local_avoid_to_shared(project, history, setup_idx, op_idx, Some(&name));
            }
            AvoidCardAction::CreateNewShared => {
                if let Some(new_id) =
                    create_and_link_shared_avoid(project, history, setup_idx, op_idx, None)
                {
                    *editing_shared_avoid_id = Some(new_id);
                    *pick_target = GeometryPickTarget::AvoidKeepOut;
                    if *pick_mode == ViewportPickMode::InspectEntity {
                        *pick_mode = ViewportPickMode::OpPlanarFaces { setup_idx, op_idx };
                    }
                }
            }
            AvoidCardAction::ToggleLinkOnOp {
                target_op_idx,
                shared_id,
            } => {
                let _ = toggle_shared_avoid_link(
                    project,
                    history,
                    setup_idx,
                    target_op_idx,
                    &shared_id,
                );
                if target_op_idx == op_idx
                    && editing_shared_avoid_id.as_deref() == Some(shared_id.as_str())
                    && !project.setups[setup_idx].operations[op_idx]
                        .avoid
                        .shared_region_ids
                        .contains(&shared_id)
                {
                    *editing_shared_avoid_id = None;
                }
            }
            AvoidCardAction::SetLinkForAllSetupOps { shared_id, link } => {
                let _ = set_shared_avoid_for_all_setup_ops(
                    project, history, setup_idx, &shared_id, link,
                );
                if !link && editing_shared_avoid_id.as_deref() == Some(shared_id.as_str()) {
                    *editing_shared_avoid_id = None;
                }
            }
            AvoidCardAction::DetachToLocal { shared_id } => {
                let _ =
                    detach_shared_avoid_to_local(project, history, setup_idx, op_idx, &shared_id);
                if editing_shared_avoid_id.as_deref() == Some(shared_id.as_str()) {
                    *editing_shared_avoid_id = None;
                }
            }
        }
    }

    if open_project_material_requested {
        Some(OperationInspectorAction::OpenProjectMaterial)
    } else if create_sketch_requested {
        Some(OperationInspectorAction::CreateSketch)
    } else {
        None
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cam::{
        AdaptiveClearingParams, MachiningBoundaryMode, Operation, OperationKind,
        PocketStrategyKind, ResolvedRotaryConfig, ResolvedStockGeometry, RotaryAxialBoundaryMode,
        RotaryAxisKind, RotaryDirection, RotaryDriveMode, RotaryUnwindMode, Setup, StockAxis,
        StockConfig, StrategyParams, WorkOffsetSlot,
    };
    use chip_math::{Aabb3, DVec3, DisplayUnit, ParamLength};
    use chip_project::{ProjectDocument, UndoHistory};
    use chip_tools::{ToolLibrary, default_starter_tool_library};

    use super::*;

    fn collect_shape_texts(shapes: &[egui::epaint::ClippedShape], out: &mut Vec<String>) {
        fn visit(shape: &egui::Shape, out: &mut Vec<String>) {
            match shape {
                egui::Shape::Text(t) => out.push(t.galley.text().to_string()),
                egui::Shape::Vec(v) => {
                    for s in v {
                        visit(s, out);
                    }
                }
                _ => {}
            }
        }
        for cs in shapes {
            visit(&cs.shape, out);
        }
    }

    fn make_resolved_box_setup() -> ResolvedSetup {
        let doc = CadDocument::empty();
        let mut setup = Setup::new("s1", "Setup 1", WorkOffsetSlot::G54);
        setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(120.0),
            depth_y: ParamLength::from_mm(120.0),
            height_z: ParamLength::from_mm(20.0),
            xy_align: Default::default(),
            z_align: Default::default(),
        };
        setup
            .resolve_with_sketches(&doc, &[], DisplayUnit::Millimeter)
            .unwrap()
    }

    fn make_resolved_rotary_setup() -> ResolvedSetup {
        let mut rs = make_resolved_box_setup();
        rs.model_aabb_wcs = Aabb3::new(
            DVec3::new(-30.0, -18.0, -18.0),
            DVec3::new(30.0, 18.0, 18.0),
        );
        rs.stock = ResolvedStockGeometry::Cylinder {
            axis: StockAxis::X,
            center_wcs: DVec3::ZERO,
            radius_mm: 21.0,
            min_axial_mm: -40.0,
            max_axial_mm: 40.0,
            aabb_wcs: Aabb3::new(
                DVec3::new(-40.0, -21.0, -21.0),
                DVec3::new(40.0, 21.0, 21.0),
            ),
        };
        rs.rotary = Some(ResolvedRotaryConfig {
            mode: RotaryDriveMode::Simultaneous4Axis,
            axis: RotaryAxisKind::XAxis,
            direction: RotaryDirection::RightHandCcw,
            center_wcs: DVec3::ZERO,
            stock_outer_radius_mm: 21.0,
            stock_u_min: -40.0,
            stock_u_max: 40.0,
            clearance_radius_mm: 33.0,
            chuck_u_min: -60.0,
            chuck_u_max: -40.0,
            chuck_radius_mm: 32.5,
            chuck_end_positive: false,
            axis_letter: 'A',
            unwind_mode: RotaryUnwindMode::ContinuousMultiTurn,
        });
        rs
    }

    fn render_inspector_texts(
        project: &mut ProjectDocument,
        resolved_setup: &ResolvedSetup,
        tools: &ToolLibrary,
        tab: OperationTab,
    ) -> Vec<String> {
        let ctx = egui::Context::default();
        let mut history = UndoHistory::default();
        let mut cur_tab = tab;
        let mut pick_mode = ViewportPickMode::InspectEntity;
        let mut pick_target = GeometryPickTarget::CuttingArea;
        let mut editing_shared_avoid_id: Option<String> = None;
        let mut lock_chipload = true;
        let mut show_z_planes = false;
        let mut auto_rotate_indexed_op = true;
        let mut open_tool_manager = false;

        let mut out = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(420.0, 1400.0),
                )),
                ..Default::default()
            },
            |ui| {
                egui::CentralPanel::default().show(ui, |ui| {
                    show_operation_inspector(
                        ui,
                        OperationInspectorCtx {
                            project,
                            setup_idx: 0,
                            op_idx: 0,
                            global_tools: tools,
                            history: &mut history,
                            cad_doc: None,
                            resolved_setup: Some(resolved_setup),
                            surface_mesh: None,
                            boundary_preview: None,
                            selected_face_normal_wcs: None,
                            extracted: None,
                            tab: &mut cur_tab,
                            pick_mode: &mut pick_mode,
                            pick_target: &mut pick_target,
                            editing_shared_avoid_id: &mut editing_shared_avoid_id,
                            lock_chipload: &mut lock_chipload,
                            show_z_planes: &mut show_z_planes,
                            auto_rotate_indexed_op: &mut auto_rotate_indexed_op,
                            open_tool_manager: &mut open_tool_manager,
                            sim_engine: chip_sim::SimEngineKind::TriDexel,
                        },
                    );
                });
            },
        );
        out.textures_delta.clear();
        let mut texts = Vec::new();
        collect_shape_texts(&out.shapes, &mut texts);
        texts
    }

    #[test]
    fn geometry_tab_shows_cutting_area_cards_and_passes_removes_duplicates() {
        let tools = default_starter_tool_library();
        let tool_id = tools.tools[0].id.clone();
        let rs_box = make_resolved_box_setup();
        let rs_rot = make_resolved_rotary_setup();

        // 1. 3-Axis + Undercut3D + Deburr3D + Facing -> Machining Boundary in [Geometry], NOT in [Passes]
        for kind in [
            OperationKind::Adaptive3D,
            OperationKind::Waterline3D,
            OperationKind::Parallel3D,
            OperationKind::Scallop3D,
            OperationKind::Pencil3D,
            OperationKind::Undercut3D,
            OperationKind::Deburr3D,
            OperationKind::Facing,
        ] {
            let mut proj = ProjectDocument::new("test");
            proj.tools = tools.tools.clone();
            let mut op = Operation::new("op1", "Op", kind);
            op.tool_id = Some(tool_id.clone());
            proj.setups[0].operations = vec![op];

            let geo_texts =
                render_inspector_texts(&mut proj, &rs_box, &tools, OperationTab::Geometry);
            let geo_joined = geo_texts.join("\n");
            assert!(
                geo_joined.contains("Cutting Area / Machining Boundary"),
                "{kind:?} [Geometry] must render Cutting Area / Machining Boundary card"
            );
            assert!(
                geo_joined.contains("Boundary Source")
                    && geo_joined.contains("Tool Containment")
                    && geo_joined.contains("Boundary Offset"),
                "{kind:?} [Geometry] must render Boundary Source, Tool Containment, and Boundary Offset"
            );
            if kind == OperationKind::Deburr3D {
                assert!(
                    geo_joined.contains("Auto-Detect Edges"),
                    "Deburr3D [Geometry] must note Auto-Detect Edges constraint"
                );
            }

            let pass_texts =
                render_inspector_texts(&mut proj, &rs_box, &tools, OperationTab::Passes);
            let pass_joined = pass_texts.join("\n");
            assert!(
                !pass_joined.contains("3D Machining Boundary")
                    && !pass_joined.contains("Facing Machining Boundary")
                    && !pass_joined.contains("Boundary Source")
                    && !pass_joined.contains("Tool Containment"),
                "{kind:?} [Passes] must not duplicate Machining Boundary controls"
            );
        }

        // 2. 4th-Axis Rotary -> Rotary Cutting Area (-U / +U) in [Geometry], NOT in [Passes]
        for kind in [OperationKind::Rotary2Linear, OperationKind::Rotary4AxisSim] {
            let mut proj = ProjectDocument::new("test_rot");
            proj.tools = tools.tools.clone();
            let mut op = Operation::new("op_rot", "Rotary", kind);
            op.tool_id = Some(tool_id.clone());
            proj.setups[0].operations = vec![op];

            let geo_texts =
                render_inspector_texts(&mut proj, &rs_rot, &tools, OperationTab::Geometry);
            let geo_joined = geo_texts.join("\n");
            assert!(
                geo_joined.contains("4th-Axis Rotary Cutting Area (-U / +U)"),
                "{kind:?} [Geometry] must render rotary cutting area card"
            );
            assert!(
                geo_joined.contains("Axial Source")
                    && geo_joined.contains("Axial Containment")
                    && geo_joined.contains("Start Margin (-U)")
                    && geo_joined.contains("End Margin (+U)")
                    && geo_joined.contains("Full 360°")
                    && geo_joined.contains("Effective U:")
                    && geo_joined.contains("Chuck Guard:"),
                "{kind:?} [Geometry] must render axial bounds, presets, and live Effective U / Chuck Guard readout"
            );
            assert!(
                !geo_joined.contains("Selected 2D Contours' in Passes"),
                "{kind:?} [Geometry] must not show misleading generic 3D banner"
            );

            let pass_texts =
                render_inspector_texts(&mut proj, &rs_rot, &tools, OperationTab::Passes);
            let pass_joined = pass_texts.join("\n");
            assert!(
                !pass_joined.contains("Start Margin (-U)")
                    && !pass_joined.contains("End Margin (+U)")
                    && !pass_joined.contains("Start Angle (°)")
                    && !pass_joined.contains("End Angle (°)"),
                "{kind:?} [Passes] must not duplicate rotary axial/angular boundary fields"
            );
        }

        // 3. Pocket2D & Adaptive2D -> Pocket Area Mode + Open Outer Boundary in [Geometry], NOT in [Passes]
        for is_adaptive in [false, true] {
            let mut proj = ProjectDocument::new("test_pkt");
            proj.tools = tools.tools.clone();
            let mut op = Operation::new("op_pkt", "Pocket", OperationKind::Pocket2D);
            op.tool_id = Some(tool_id.clone());
            if is_adaptive && let StrategyParams::Pocket2D(p) = op.strategy_mut() {
                p.strategy = PocketStrategyKind::Adaptive(AdaptiveClearingParams::default());
            }
            proj.setups[0].operations = vec![op];

            let geo_joined =
                render_inspector_texts(&mut proj, &rs_box, &tools, OperationTab::Geometry)
                    .join("\n");
            assert!(
                geo_joined.contains("Pocket Area Mode")
                    && geo_joined.contains("Open Outer Boundary"),
                "Pocket2D (adaptive={is_adaptive}) [Geometry] must show Pocket Area Mode and Open Outer Boundary"
            );

            let pass_joined =
                render_inspector_texts(&mut proj, &rs_box, &tools, OperationTab::Passes).join("\n");
            assert!(
                !pass_joined.contains("Boundary Mode")
                    && !pass_joined.contains("Treat Outer Boundary as Open"),
                "Pocket2D (adaptive={is_adaptive}) [Passes] must not duplicate pocket boundary controls"
            );
        }

        // 4. Contour2D -> Clip to Stock Bounds & Open Chain Extension in [Geometry], NOT in [Passes]
        {
            let mut proj = ProjectDocument::new("test_cnt");
            proj.tools = tools.tools.clone();
            let mut op = Operation::new("op_cnt", "Contour", OperationKind::Contour2D);
            op.tool_id = Some(tool_id);
            proj.setups[0].operations = vec![op];

            let geo_joined =
                render_inspector_texts(&mut proj, &rs_box, &tools, OperationTab::Geometry)
                    .join("\n");
            assert!(
                geo_joined.contains("Clip to Stock Bounds")
                    && geo_joined.contains("Open Chain Extension"),
                "Contour2D [Geometry] must show Stock Clip and Open Chain Extension"
            );

            let pass_joined =
                render_inspector_texts(&mut proj, &rs_box, &tools, OperationTab::Passes).join("\n");
            assert!(
                !pass_joined.contains("Clip to Stock XY Bounds")
                    && !pass_joined.contains("Open Extension"),
                "Contour2D [Passes] must not duplicate Stock Clip / Open Extension"
            );
        }
    }

    #[test]
    fn pick_card_default_open_collapses_on_automatic_boundaries() {
        // 3D surface & Facing collapse when StockBox or ModelSilhouette, open on SelectedRegions
        for kind in [
            OperationKind::Facing,
            OperationKind::Adaptive3D,
            OperationKind::Waterline3D,
            OperationKind::Parallel3D,
            OperationKind::Scallop3D,
            OperationKind::Pencil3D,
            OperationKind::Undercut3D,
        ] {
            let mut op = Operation::new("op", "Test", kind);
            op.machining_boundary_mut().unwrap().boundary_mode = MachiningBoundaryMode::StockBox;
            assert!(!geometry_tab::pick_card_default_open(&op));
            op.machining_boundary_mut().unwrap().boundary_mode =
                MachiningBoundaryMode::ModelBoundingBox;
            assert!(!geometry_tab::pick_card_default_open(&op));
            op.machining_boundary_mut().unwrap().boundary_mode =
                MachiningBoundaryMode::ModelSilhouette;
            assert!(!geometry_tab::pick_card_default_open(&op));
            op.machining_boundary_mut().unwrap().boundary_mode =
                MachiningBoundaryMode::SelectedRegions;
            assert!(geometry_tab::pick_card_default_open(&op));
        }

        // Rotary collapses unless SelectedGeometry is active
        for kind in [OperationKind::Rotary2Linear, OperationKind::Rotary4AxisSim] {
            let mut op = Operation::new("op_r", "Rotary", kind);
            op.rotary_axial_boundary_mut().unwrap().axial_boundary_mode =
                RotaryAxialBoundaryMode::ModelBounds;
            assert!(!geometry_tab::pick_card_default_open(&op));
            op.rotary_axial_boundary_mut().unwrap().axial_boundary_mode =
                RotaryAxialBoundaryMode::ExplicitRange;
            assert!(!geometry_tab::pick_card_default_open(&op));
            op.rotary_axial_boundary_mut().unwrap().axial_boundary_mode =
                RotaryAxialBoundaryMode::SelectedGeometry;
            assert!(geometry_tab::pick_card_default_open(&op));
        }
    }

    #[test]
    fn geometry_tab_mutation_records_undo_and_marks_operation_dirty() {
        let tools = default_starter_tool_library();
        let rs_rot = make_resolved_rotary_setup();
        let mut proj = ProjectDocument::new("undo_test");
        proj.tools = tools.tools.clone();
        let mut op = Operation::new("op_rot", "Rotary Finish", OperationKind::Rotary4AxisSim);
        op.tool_id = Some(tools.tools[0].id.clone());
        op.runtime_status = OperationStatus::Clean;
        proj.setups[0].operations = vec![op];

        let ctx = egui::Context::default();
        let mut history = UndoHistory::default();
        let mut tab = OperationTab::Geometry;
        let mut pick_mode = ViewportPickMode::InspectEntity;
        let mut pick_target = GeometryPickTarget::CuttingArea;
        let mut editing_shared_avoid_id: Option<String> = None;
        let mut lock_chipload = true;
        let mut show_z_planes = false;
        let mut auto_rotate_indexed_op = true;
        let mut open_tool_manager = false;

        // First frame: initialize layout, then probe exact screen center of the "0–180°" preset chip
        let mut chip_pos: Option<egui::Pos2> = None;
        let mut init_out = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(420.0, 900.0),
                )),
                ..Default::default()
            },
            |ui| {
                egui::CentralPanel::default().show(ui, |ui| {
                    show_operation_inspector(
                        ui,
                        OperationInspectorCtx {
                            project: &mut proj,
                            setup_idx: 0,
                            op_idx: 0,
                            global_tools: &tools,
                            history: &mut history,
                            cad_doc: None,
                            resolved_setup: Some(&rs_rot),
                            surface_mesh: None,
                            boundary_preview: None,
                            selected_face_normal_wcs: None,
                            extracted: None,
                            tab: &mut tab,
                            pick_mode: &mut pick_mode,
                            pick_target: &mut pick_target,
                            editing_shared_avoid_id: &mut editing_shared_avoid_id,
                            lock_chipload: &mut lock_chipload,
                            show_z_planes: &mut show_z_planes,
                            auto_rotate_indexed_op: &mut auto_rotate_indexed_op,
                            open_tool_manager: &mut open_tool_manager,
                            sim_engine: chip_sim::SimEngineKind::TriDexel,
                        },
                    );
                });
            },
        );
        init_out.textures_delta.clear();
        // Locate "0–180°" text position in painted shapes
        let mut probe = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(420.0, 900.0),
                )),
                ..Default::default()
            },
            |ui| {
                egui::CentralPanel::default().show(ui, |ui| {
                    show_operation_inspector(
                        ui,
                        OperationInspectorCtx {
                            project: &mut proj,
                            setup_idx: 0,
                            op_idx: 0,
                            global_tools: &tools,
                            history: &mut history,
                            cad_doc: None,
                            resolved_setup: Some(&rs_rot),
                            surface_mesh: None,
                            boundary_preview: None,
                            selected_face_normal_wcs: None,
                            extracted: None,
                            tab: &mut tab,
                            pick_mode: &mut pick_mode,
                            pick_target: &mut pick_target,
                            editing_shared_avoid_id: &mut editing_shared_avoid_id,
                            lock_chipload: &mut lock_chipload,
                            show_z_planes: &mut show_z_planes,
                            auto_rotate_indexed_op: &mut auto_rotate_indexed_op,
                            open_tool_manager: &mut open_tool_manager,
                            sim_engine: chip_sim::SimEngineKind::TriDexel,
                        },
                    );
                });
            },
        );
        probe.textures_delta.clear();
        fn find_text_center(shape: &egui::Shape, needle: &str) -> Option<egui::Pos2> {
            match shape {
                egui::Shape::Text(t) if t.galley.text() == needle => {
                    Some(t.pos + t.galley.size() * 0.5)
                }
                egui::Shape::Vec(v) => v.iter().find_map(|s| find_text_center(s, needle)),
                _ => None,
            }
        }
        for cs in &probe.shapes {
            if let Some(p) = find_text_center(&cs.shape, "0–180°") {
                chip_pos = Some(p);
                break;
            }
        }
        let click_pt = chip_pos.expect("0–180° preset chip must be rendered");

        // Second frame: click the "0–180°" preset chip
        let click_input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(420.0, 900.0),
            )),
            events: vec![
                egui::Event::PointerMoved(click_pt),
                egui::Event::PointerButton {
                    pos: click_pt,
                    button: egui::PointerButton::Primary,
                    pressed: true,
                    modifiers: egui::Modifiers::NONE,
                },
                egui::Event::PointerButton {
                    pos: click_pt,
                    button: egui::PointerButton::Primary,
                    pressed: false,
                    modifiers: egui::Modifiers::NONE,
                },
            ],
            ..Default::default()
        };
        let mut click_out = ctx.run_ui(click_input, |ui| {
            egui::CentralPanel::default().show(ui, |ui| {
                show_operation_inspector(
                    ui,
                    OperationInspectorCtx {
                        project: &mut proj,
                        setup_idx: 0,
                        op_idx: 0,
                        global_tools: &tools,
                        history: &mut history,
                        cad_doc: None,
                        resolved_setup: Some(&rs_rot),
                        surface_mesh: None,
                        boundary_preview: None,
                        selected_face_normal_wcs: None,
                        extracted: None,
                        tab: &mut tab,
                        pick_mode: &mut pick_mode,
                        pick_target: &mut pick_target,
                        editing_shared_avoid_id: &mut editing_shared_avoid_id,
                        lock_chipload: &mut lock_chipload,
                        show_z_planes: &mut show_z_planes,
                        auto_rotate_indexed_op: &mut auto_rotate_indexed_op,
                        open_tool_manager: &mut open_tool_manager,
                        sim_engine: chip_sim::SimEngineKind::TriDexel,
                    },
                );
            });
        });
        click_out.textures_delta.clear();

        let rb = proj.setups[0].operations[0]
            .rotary_axial_boundary()
            .unwrap();
        assert!((rb.start_angle_deg - 0.0).abs() < 1e-6);
        assert!((rb.end_angle_deg - 180.0).abs() < 1e-6);
        assert_eq!(
            proj.setups[0].operations[0].runtime_status,
            OperationStatus::Dirty,
            "Geometry tab boundary edit must mark operation Dirty"
        );
        assert!(
            history.can_undo(),
            "Geometry tab boundary edit must record an Undo step"
        );

        // Undo restores the original 360° sector
        let undone = history.undo(&mut proj);
        assert!(undone.is_some());
        let rb_restored = proj.setups[0].operations[0]
            .rotary_axial_boundary()
            .unwrap();
        assert!((rb_restored.end_angle_deg - 360.0).abs() < 1e-6);
    }

    #[test]
    fn m84_model_bounds_inputs_sketch_auto_expand_quick_select_and_indexed_toggle() {
        use chip_cam::{GeometrySelectionItem, OperationRotaryMode};
        use chip_sketch::{RectAnchorMode, SketchEntity, SketchEntityEntry, SketchLayer};

        let tools = default_starter_tool_library();
        let rs_rot = make_resolved_rotary_setup();
        let mut proj = ProjectDocument::new("m84_ui_test");
        proj.tools = tools.tools.clone();

        let mut op = Operation::new("op_a3d", "3D Adaptive", OperationKind::Adaptive3D);
        op.tool_id = Some(tools.tools[0].id.clone());
        op.rotary = OperationRotaryMode::Indexed3Plus1 { angle_deg: 90.0 };
        op.machining_boundary_mut().unwrap().boundary_mode =
            MachiningBoundaryMode::ModelBoundingBox;
        proj.setups[0].operations = vec![op];

        // 1. ModelBoundingBox renders all four per-side offset inputs in [Geometry]
        let mb_texts = render_inspector_texts(&mut proj, &rs_rot, &tools, OperationTab::Geometry);
        let mb_joined = mb_texts.join("\n");
        assert!(
            mb_joined.contains("-X Offset")
                && mb_joined.contains("+X Offset")
                && mb_joined.contains("-Y Offset")
                && mb_joined.contains("+Y Offset"),
            "ModelBoundingBox must render all 4 per-side offset inputs; got:\n{mb_joined}"
        );

        // 2. [Passes] shows inline auto-rotate toggle inside 4th-Axis Indexing / Wrapping
        let pass_texts = render_inspector_texts(&mut proj, &rs_rot, &tools, OperationTab::Passes);
        let pass_joined = pass_texts.join("\n");
        assert!(
            pass_joined.contains("Auto-rotate view to index angle"),
            "[Passes] must show inline auto-rotate checkbox for Indexed 3+1; got:\n{pass_joined}"
        );

        // 3. Switching from ModelSilhouette -> SelectedRegions auto-expands `2D Sketch Layers & Quick Select`
        //    and clicking a per-entity quick-add chip inserts `GeometrySelectionItem::SketchEntity`!
        let mut sk = SketchLayer::new("sk_1", "Boundary Sketch", "s1");
        sk.entities.push(SketchEntityEntry {
            id: "rect_1".to_string(),
            name: "Rect 1".to_string(),
            is_construction: false,
            sub_layer: None,
            entity: SketchEntity::Rectangle {
                anchor: RectAnchorMode::Center,
                x: ParamLength::from_mm(0.0),
                y: ParamLength::from_mm(0.0),
                width: ParamLength::from_mm(40.0),
                height: ParamLength::from_mm(20.0),
                corner_radius: ParamLength::from_mm(0.0),
                rotation_deg: 0.0,
            },
        });
        sk.entities.push(SketchEntityEntry {
            id: "const_line".to_string(),
            name: "Construction".to_string(),
            is_construction: true,
            sub_layer: None,
            entity: SketchEntity::new_circle(
                ParamLength::from_mm(0.0),
                ParamLength::from_mm(0.0),
                ParamLength::from_mm(10.0),
            ),
        });
        proj.sketches = vec![sk];
        proj.setups[0].operations[0]
            .machining_boundary_mut()
            .unwrap()
            .boundary_mode = MachiningBoundaryMode::ModelSilhouette;

        let ctx = egui::Context::default();
        let mut history = UndoHistory::default();
        let mut tab = OperationTab::Geometry;
        let mut pick_mode = ViewportPickMode::InspectEntity;
        let mut pick_target = GeometryPickTarget::CuttingArea;
        let mut editing_shared_avoid_id: Option<String> = None;
        let mut lock_chipload = true;
        let mut show_z_planes = false;
        let mut auto_rotate = true;
        let mut open_tool_manager = false;

        let mut run_frame = |ctx: &egui::Context,
                             proj: &mut ProjectDocument,
                             history: &mut UndoHistory,
                             events: Vec<egui::Event>|
         -> (
            Vec<egui::epaint::ClippedShape>,
            Option<OperationInspectorAction>,
        ) {
            let mut action = None;
            let mut out = ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(
                        egui::Pos2::ZERO,
                        egui::vec2(440.0, 1200.0),
                    )),
                    events,
                    ..Default::default()
                },
                |ui| {
                    egui::CentralPanel::default().show(ui, |ui| {
                        action = show_operation_inspector(
                            ui,
                            OperationInspectorCtx {
                                project: proj,
                                setup_idx: 0,
                                op_idx: 0,
                                global_tools: &tools,
                                history,
                                cad_doc: None,
                                resolved_setup: Some(&rs_rot),
                                surface_mesh: None,
                                boundary_preview: None,
                                selected_face_normal_wcs: None,
                                extracted: None,
                                tab: &mut tab,
                                pick_mode: &mut pick_mode,
                                pick_target: &mut pick_target,
                                editing_shared_avoid_id: &mut editing_shared_avoid_id,
                                lock_chipload: &mut lock_chipload,
                                show_z_planes: &mut show_z_planes,
                                auto_rotate_indexed_op: &mut auto_rotate,
                                open_tool_manager: &mut open_tool_manager,
                                sim_engine: chip_sim::SimEngineKind::TriDexel,
                            },
                        );
                    });
                },
            );
            out.textures_delta.clear();
            (out.shapes, action)
        };

        // Frame 1 (ModelSilhouette): sketch card starts collapsed (`+ Rect 1` not visible)
        let (shapes_collapsed, _) = run_frame(&ctx, &mut proj, &mut history, Vec::new());
        let mut t_col = Vec::new();
        collect_shape_texts(&shapes_collapsed, &mut t_col);
        assert!(
            !t_col.iter().any(|t| t == "+ Rect 1"),
            "sketch card should start collapsed under ModelSilhouette"
        );

        // Switch Boundary Source to SelectedRegions -> sketch card auto-expands immediately!
        proj.setups[0].operations[0]
            .machining_boundary_mut()
            .unwrap()
            .boundary_mode = MachiningBoundaryMode::SelectedRegions;
        let _ = run_frame(&ctx, &mut proj, &mut history, Vec::new());
        let (shapes_open, _) = run_frame(&ctx, &mut proj, &mut history, Vec::new());
        let mut t_open = Vec::new();
        collect_shape_texts(&shapes_open, &mut t_open);
        assert!(
            t_open.iter().any(|t| t == "+ Rect 1"),
            "switching to SelectedRegions must auto-expand 2D Sketch Layers & Quick Select and show '+ Rect 1'; got: {t_open:?}"
        );
        assert!(
            !t_open.iter().any(|t| t.contains("Construction")),
            "construction entities must be excluded from quick-add chips"
        );

        // Click `+ Rect 1` -> inserts `GeometrySelectionItem::SketchEntity`
        fn find_text_center(shape: &egui::Shape, needle: &str) -> Option<egui::Pos2> {
            match shape {
                egui::Shape::Text(t) if t.galley.text() == needle => {
                    Some(t.pos + t.galley.size() * 0.5)
                }
                egui::Shape::Vec(v) => v.iter().find_map(|s| find_text_center(s, needle)),
                _ => None,
            }
        }
        let chip_pt = shapes_open
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "+ Rect 1"))
            .expect("+ Rect 1 chip must be painted");
        let _ = run_frame(
            &ctx,
            &mut proj,
            &mut history,
            vec![
                egui::Event::PointerMoved(chip_pt),
                egui::Event::PointerButton {
                    pos: chip_pt,
                    button: egui::PointerButton::Primary,
                    pressed: true,
                    modifiers: egui::Modifiers::NONE,
                },
                egui::Event::PointerButton {
                    pos: chip_pt,
                    button: egui::PointerButton::Primary,
                    pressed: false,
                    modifiers: egui::Modifiers::NONE,
                },
            ],
        );
        assert!(
            proj.setups[0].operations[0].selections.iter().any(|s| {
                matches!(
                    s,
                    GeometrySelectionItem::SketchEntity {
                        sketch_id,
                        entity_id,
                        ..
                    } if sketch_id == "sk_1" && entity_id == "rect_1"
                )
            }),
            "clicking '+ Rect 1' must insert GeometrySelectionItem::SketchEntity; selections: {:?}",
            proj.setups[0].operations[0].selections
        );

        // 4. Empty sketches state under SelectedRegions shows `+ New Sketch` and emits CreateSketch action
        proj.sketches.clear();
        let _ = run_frame(&ctx, &mut proj, &mut history, Vec::new());
        let (shapes_empty, _) = run_frame(&ctx, &mut proj, &mut history, Vec::new());
        let new_sk_pt = shapes_empty
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "+ New Sketch"))
            .expect("empty sketch state must render '+ New Sketch' button");
        let (_, create_act) = run_frame(
            &ctx,
            &mut proj,
            &mut history,
            vec![
                egui::Event::PointerMoved(new_sk_pt),
                egui::Event::PointerButton {
                    pos: new_sk_pt,
                    button: egui::PointerButton::Primary,
                    pressed: true,
                    modifiers: egui::Modifiers::NONE,
                },
                egui::Event::PointerButton {
                    pos: new_sk_pt,
                    button: egui::PointerButton::Primary,
                    pressed: false,
                    modifiers: egui::Modifiers::NONE,
                },
            ],
        );
        assert_eq!(create_act, Some(OperationInspectorAction::CreateSketch));
    }

    #[test]
    fn m86_avoid_keepout_ui_shared_regions_fixtures_pick_target_and_undo() {
        use chip_cam::{FixtureBody, GeometrySelectionItem, SharedAvoidRegion};
        use chip_math::DVec2;
        use chip_sketch::{RectAnchorMode, SketchEntity, SketchEntityEntry, SketchLayer};

        use crate::panels::browser::{CamBrowserCtx, show_cam_browser};
        use crate::panels::{InspectorFocus, SetupTab};

        let tools = default_starter_tool_library();
        let rs_box = make_resolved_box_setup();
        let mut proj = ProjectDocument::new("m86_avoid_ui_test");
        proj.tools = tools.tools.clone();

        // Add a Setup fixture clamp and a 2D keep-out sketch
        proj.setups[0].fixtures.push(FixtureBody::default_toe_clamp(
            "clamp_1",
            "Toe Clamp 1",
            DVec2::new(-35.0, 0.0),
            180.0,
        ));
        let mut sk = SketchLayer::new("sk_ko", "Keep-Out Sketch", "setup_1");
        sk.entities.push(SketchEntityEntry {
            id: "ko_rect".to_string(),
            name: "Clamp Zone".to_string(),
            sub_layer: None,
            is_construction: false,
            entity: SketchEntity::Rectangle {
                anchor: RectAnchorMode::Center,
                x: ParamLength::from_mm(0.0),
                y: ParamLength::from_mm(0.0),
                width: ParamLength::from_mm(20.0),
                height: ParamLength::from_mm(12.0),
                corner_radius: ParamLength::from_mm(0.0),
                rotation_deg: 0.0,
            },
        });
        sk.rebuild_regions(None);
        proj.sketches = vec![sk];

        let mut op1 = Operation::new("op_1", "Rough Adaptive", OperationKind::Adaptive3D);
        op1.tool_id = Some(tools.tools[0].id.clone());
        op1.runtime_status = OperationStatus::Clean;
        op1.avoid
            .selections
            .push(GeometrySelectionItem::SketchEntity {
                sketch_id: "sk_ko".to_string(),
                entity_id: "ko_rect".to_string(),
                reverse: false,
                side_flip: false,
            });
        let mut op2 = Operation::new("op_2", "Finish Waterline", OperationKind::Waterline3D);
        op2.tool_id = Some(tools.tools[0].id.clone());
        op2.runtime_status = OperationStatus::Clean;
        proj.setups[0].operations = vec![op1, op2];

        fn find_text_center(shape: &egui::Shape, needle: &str) -> Option<egui::Pos2> {
            match shape {
                egui::Shape::Text(t) if t.galley.text() == needle => {
                    Some(t.pos + t.galley.size() * 0.5)
                }
                egui::Shape::Vec(v) => v.iter().find_map(|s| find_text_center(s, needle)),
                _ => None,
            }
        }
        fn click_events(pt: egui::Pos2) -> Vec<egui::Event> {
            vec![
                egui::Event::PointerMoved(pt),
                egui::Event::PointerButton {
                    pos: pt,
                    button: egui::PointerButton::Primary,
                    pressed: true,
                    modifiers: egui::Modifiers::NONE,
                },
                egui::Event::PointerButton {
                    pos: pt,
                    button: egui::PointerButton::Primary,
                    pressed: false,
                    modifiers: egui::Modifiers::NONE,
                },
            ]
        }

        let ctx = egui::Context::default();
        let mut history = UndoHistory::default();
        let mut tab = OperationTab::Geometry;
        let mut pick_mode = ViewportPickMode::InspectEntity;
        let mut pick_target = GeometryPickTarget::CuttingArea;
        let mut editing_shared_avoid_id: Option<String> = None;
        let mut lock_chipload = true;
        let mut show_z_planes = false;
        let mut auto_rotate = true;
        let mut open_tool_manager = false;

        let mut run_op = |ctx: &egui::Context,
                          proj: &mut ProjectDocument,
                          op_idx: usize,
                          history: &mut UndoHistory,
                          pick_mode: &mut ViewportPickMode,
                          pick_target: &mut GeometryPickTarget,
                          editing_shared_avoid_id: &mut Option<String>,
                          events: Vec<egui::Event>|
         -> Vec<egui::epaint::ClippedShape> {
            let mut out = ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(
                        egui::Pos2::ZERO,
                        egui::vec2(460.0, 1800.0),
                    )),
                    events,
                    ..Default::default()
                },
                |ui| {
                    egui::CentralPanel::default().show(ui, |ui| {
                        show_operation_inspector(
                            ui,
                            OperationInspectorCtx {
                                project: proj,
                                setup_idx: 0,
                                op_idx,
                                global_tools: &tools,
                                history,
                                cad_doc: None,
                                resolved_setup: Some(&rs_box),
                                surface_mesh: None,
                                boundary_preview: None,
                                selected_face_normal_wcs: None,
                                extracted: None,
                                tab: &mut tab,
                                pick_mode,
                                pick_target,
                                editing_shared_avoid_id,
                                lock_chipload: &mut lock_chipload,
                                show_z_planes: &mut show_z_planes,
                                auto_rotate_indexed_op: &mut auto_rotate,
                                open_tool_manager: &mut open_tool_manager,
                                sim_engine: chip_sim::SimEngineKind::TriDexel,
                            },
                        );
                    });
                },
            );
            out.textures_delta.clear();
            out.shapes
        };

        // 1. Auto-expanded Avoid / Keep-Out Regions card renders fixtures, clearances, and Save Local as Shared…
        let _ = run_op(
            &ctx,
            &mut proj,
            0,
            &mut history,
            &mut pick_mode,
            &mut pick_target,
            &mut editing_shared_avoid_id,
            Vec::new(),
        );
        let shapes1 = run_op(
            &ctx,
            &mut proj,
            0,
            &mut history,
            &mut pick_mode,
            &mut pick_target,
            &mut editing_shared_avoid_id,
            Vec::new(),
        );
        let mut texts1 = Vec::new();
        collect_shape_texts(&shapes1, &mut texts1);
        let joined1 = texts1.join("\n");
        assert!(
            joined1.contains("Avoid / Keep-Out Regions")
                && joined1.contains("Save Local as Shared…")
                && joined1.contains("Avoid All Setup Fixtures")
                && joined1.contains("[ ] Toe Clamp 1")
                && joined1.contains("Radial Clearance")
                && joined1.contains("Axial Clearance"),
            "expected Avoid / Keep-Out controls to be visible; got:\n{joined1}"
        );

        // 2. Toggle individual fixture chip `[ ] Toe Clamp 1` -> adds `clamp_1`, records Undo, marks Dirty
        let clamp_pt = shapes1
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "[ ] Toe Clamp 1"))
            .expect("[ ] Toe Clamp 1 chip must be painted");
        let _ = run_op(
            &ctx,
            &mut proj,
            0,
            &mut history,
            &mut pick_mode,
            &mut pick_target,
            &mut editing_shared_avoid_id,
            click_events(clamp_pt),
        );
        assert_eq!(
            proj.setups[0].operations[0].avoid.fixture_ids,
            vec!["clamp_1"]
        );
        assert_eq!(
            proj.setups[0].operations[0].runtime_status,
            OperationStatus::Dirty
        );
        assert!(history.can_undo());

        // 3. Toggle `Avoid All Setup Fixtures` checkbox -> sets avoid_all_fixtures = true
        let shapes2 = run_op(
            &ctx,
            &mut proj,
            0,
            &mut history,
            &mut pick_mode,
            &mut pick_target,
            &mut editing_shared_avoid_id,
            Vec::new(),
        );
        let all_fix_pt = shapes2
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Avoid All Setup Fixtures"))
            .expect("Avoid All Setup Fixtures checkbox must be painted");
        let _ = run_op(
            &ctx,
            &mut proj,
            0,
            &mut history,
            &mut pick_mode,
            &mut pick_target,
            &mut editing_shared_avoid_id,
            click_events(all_fix_pt),
        );
        assert!(proj.setups[0].operations[0].avoid.avoid_all_fixtures);

        // Undo reverts `avoid_all_fixtures` back to false while preserving `clamp_1`
        assert!(history.undo(&mut proj).is_some());
        assert!(!proj.setups[0].operations[0].avoid.avoid_all_fixtures);
        assert_eq!(
            proj.setups[0].operations[0].avoid.fixture_ids,
            vec!["clamp_1"]
        );

        // 4. Promote local keep-out via `Save Local as Shared…` -> `Save` in one atomic Undo step
        let shapes3 = run_op(
            &ctx,
            &mut proj,
            0,
            &mut history,
            &mut pick_mode,
            &mut pick_target,
            &mut editing_shared_avoid_id,
            Vec::new(),
        );
        let save_local_pt = shapes3
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Save Local as Shared…"))
            .expect("Save Local as Shared… button must be painted");
        let _ = run_op(
            &ctx,
            &mut proj,
            0,
            &mut history,
            &mut pick_mode,
            &mut pick_target,
            &mut editing_shared_avoid_id,
            click_events(save_local_pt),
        );
        let shapes_prompt = run_op(
            &ctx,
            &mut proj,
            0,
            &mut history,
            &mut pick_mode,
            &mut pick_target,
            &mut editing_shared_avoid_id,
            Vec::new(),
        );
        let confirm_pt = shapes_prompt
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Save"))
            .expect("Save confirm button must be painted");
        let _ = run_op(
            &ctx,
            &mut proj,
            0,
            &mut history,
            &mut pick_mode,
            &mut pick_target,
            &mut editing_shared_avoid_id,
            click_events(confirm_pt),
        );
        assert_eq!(proj.avoid_regions.len(), 1);
        let promoted_id = proj.avoid_regions[0].id.clone();
        assert_eq!(proj.avoid_regions[0].selections.len(), 1);
        assert_eq!(proj.avoid_regions[0].fixture_ids, vec!["clamp_1"]);
        assert!(proj.setups[0].operations[0].avoid.selections.is_empty());
        assert!(proj.setups[0].operations[0].avoid.fixture_ids.is_empty());
        assert_eq!(
            proj.setups[0].operations[0].avoid.shared_region_ids,
            vec![promoted_id.clone()]
        );

        // 5. Link shared region on Op 2 as well, then enter Edit mode on Op 1 and verify banner!
        assert!(avoid_card::toggle_shared_avoid_link(
            &mut proj,
            &mut history,
            0,
            1,
            &promoted_id
        ));
        assert_eq!(proj.avoid_region_usage_count(&promoted_id), 2);
        assert_eq!(
            proj.setups[0].operations[1].runtime_status,
            OperationStatus::Dirty
        );

        let shapes_linked = run_op(
            &ctx,
            &mut proj,
            0,
            &mut history,
            &mut pick_mode,
            &mut pick_target,
            &mut editing_shared_avoid_id,
            Vec::new(),
        );
        let edit_pt = shapes_linked
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Edit"))
            .expect("Edit button on linked shared region must be painted");
        let _ = run_op(
            &ctx,
            &mut proj,
            0,
            &mut history,
            &mut pick_mode,
            &mut pick_target,
            &mut editing_shared_avoid_id,
            click_events(edit_pt),
        );
        assert_eq!(
            editing_shared_avoid_id.as_deref(),
            Some(promoted_id.as_str())
        );
        assert_eq!(pick_target, GeometryPickTarget::AvoidKeepOut);
        assert!(matches!(pick_mode, ViewportPickMode::OpPlanarFaces { .. }));

        let shapes_editing = run_op(
            &ctx,
            &mut proj,
            0,
            &mut history,
            &mut pick_mode,
            &mut pick_target,
            &mut editing_shared_avoid_id,
            Vec::new(),
        );
        let mut edit_texts = Vec::new();
        collect_shape_texts(&shapes_editing, &mut edit_texts);
        let edit_joined = edit_texts.join("\n");
        assert!(
            edit_joined.contains("Editing Shared Region:")
                && edit_joined.contains("linked to 2 operation(s)"),
            "expected shared-region edit banner; got:\n{edit_joined}"
        );

        // 6. Switch Pick Target back to `Cutting Area` via segmented toggle -> exits shared edit
        let cut_pt = shapes_editing
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Cutting Area"))
            .expect("Cutting Area pick target button must be painted");
        let _ = run_op(
            &ctx,
            &mut proj,
            0,
            &mut history,
            &mut pick_mode,
            &mut pick_target,
            &mut editing_shared_avoid_id,
            click_events(cut_pt),
        );
        assert_eq!(pick_target, GeometryPickTarget::CuttingArea);
        assert!(editing_shared_avoid_id.is_none());

        // 7. Detach to Local on Op 2 copies selections & fixtures and unlinks shared ID
        assert!(avoid_card::detach_shared_avoid_to_local(
            &mut proj,
            &mut history,
            0,
            1,
            &promoted_id
        ));
        assert!(
            proj.setups[0].operations[1]
                .avoid
                .shared_region_ids
                .is_empty()
        );
        assert_eq!(proj.setups[0].operations[1].avoid.selections.len(), 1);
        assert_eq!(
            proj.setups[0].operations[1].avoid.fixture_ids,
            vec!["clamp_1"]
        );
        assert_eq!(proj.avoid_region_usage_count(&promoted_id), 1);

        // 8. CAM Browser renders `Keep-Out Regions (1)` with usage badge `used by 1 op`
        let browser_ctx = egui::Context::default();
        let tp_cache = chip_project::ToolpathCache::new(8);
        let mut active_setup_idx = 0;
        let mut focus = InspectorFocus::Operation {
            setup_idx: 0,
            op_idx: 0,
        };
        let mut setup_tab = SetupTab::Wcs;
        let mut hidden_ops = std::collections::HashSet::new();
        let mut selected_sketch_entity_idx = None;
        let mut selected_fixture_idx = None;
        let mut browser_out = browser_ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(320.0, 1200.0),
                )),
                ..Default::default()
            },
            |ui| {
                egui::CentralPanel::default().show(ui, |ui| {
                    let _ = show_cam_browser(
                        ui,
                        CamBrowserCtx {
                            project: &mut proj,
                            global_tools: &tools,
                            history: &mut history,
                            toolpath_cache: &tp_cache,
                            cad_doc: None,
                            cad_revision: 0,
                            active_setup_idx: &mut active_setup_idx,
                            focus: &mut focus,
                            setup_tab: &mut setup_tab,
                            op_tab: &mut tab,
                            pick_mode: &mut pick_mode,
                            hidden_operation_ids: &mut hidden_ops,
                            editing_shared_avoid_id: &mut editing_shared_avoid_id,
                            selected_sketch_entity_idx: &mut selected_sketch_entity_idx,
                            selected_fixture_idx: &mut selected_fixture_idx,
                        },
                    );
                });
            },
        );
        browser_out.textures_delta.clear();
        let mut b_texts = Vec::new();
        collect_shape_texts(&browser_out.shapes, &mut b_texts);
        let b_joined = b_texts.join("\n");
        assert!(
            b_joined.contains("Keep-Out Regions (1)") && b_joined.contains("used by 1 op"),
            "CAM Browser must display Keep-Out Regions (1) and usage count; got:\n{b_joined}"
        );

        // Verify safe delete via ProjectDocument::remove_avoid_region cleans Op 1's shared link
        let _unused = SharedAvoidRegion::new("tmp", "Tmp");
        assert!(proj.remove_avoid_region(&promoted_id));
        assert!(proj.avoid_regions.is_empty());
        assert!(
            proj.setups[0].operations[0]
                .avoid
                .shared_region_ids
                .is_empty()
        );
    }

    fn collect_text_rects(shapes: &[egui::epaint::ClippedShape]) -> Vec<(String, egui::Rect)> {
        fn visit(shape: &egui::Shape, out: &mut Vec<(String, egui::Rect)>) {
            match shape {
                egui::Shape::Text(t) => {
                    out.push((t.galley.text().to_owned(), t.visual_bounding_rect()));
                }
                egui::Shape::Vec(v) => {
                    for s in v {
                        visit(s, out);
                    }
                }
                _ => {}
            }
        }
        let mut out = Vec::new();
        for cs in shapes {
            visit(&cs.shape, &mut out);
        }
        out
    }

    fn click_pos_events(pos: egui::Pos2) -> Vec<egui::Event> {
        vec![
            egui::Event::PointerMoved(pos),
            egui::Event::PointerButton {
                pos,
                button: egui::PointerButton::Primary,
                pressed: true,
                modifiers: egui::Modifiers::NONE,
            },
            egui::Event::PointerButton {
                pos,
                button: egui::PointerButton::Primary,
                pressed: false,
                modifiers: egui::Modifiers::NONE,
            },
        ]
    }

    #[allow(clippy::too_many_arguments)]
    fn drag_inspector_widget(
        ctx: &egui::Context,
        proj: &mut ProjectDocument,
        rs: &ResolvedSetup,
        tools: &ToolLibrary,
        history: &mut UndoHistory,
        tab: OperationTab,
        pos: egui::Pos2,
        dx: f32,
        time: &mut f64,
    ) -> Vec<(String, egui::Rect)> {
        let end = egui::pos2(pos.x + dx, pos.y);
        let _ = run_inspector_frame(
            ctx,
            proj,
            rs,
            tools,
            history,
            tab,
            vec![
                egui::Event::PointerMoved(pos),
                egui::Event::PointerButton {
                    pos,
                    button: egui::PointerButton::Primary,
                    pressed: true,
                    modifiers: egui::Modifiers::NONE,
                },
            ],
            time,
        );
        let _ = run_inspector_frame(
            ctx,
            proj,
            rs,
            tools,
            history,
            tab,
            vec![egui::Event::PointerMoved(end)],
            time,
        );
        run_inspector_frame(
            ctx,
            proj,
            rs,
            tools,
            history,
            tab,
            vec![egui::Event::PointerButton {
                pos: end,
                button: egui::PointerButton::Primary,
                pressed: false,
                modifiers: egui::Modifiers::NONE,
            }],
            time,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn open_inspector_combo(
        ctx: &egui::Context,
        proj: &mut ProjectDocument,
        rs: &ResolvedSetup,
        tools: &ToolLibrary,
        history: &mut UndoHistory,
        tab: OperationTab,
        pos: egui::Pos2,
        time: &mut f64,
    ) -> Vec<(String, egui::Rect)> {
        let _ = run_inspector_frame(
            ctx,
            proj,
            rs,
            tools,
            history,
            tab,
            click_pos_events(pos),
            time,
        );
        run_inspector_frame(ctx, proj, rs, tools, history, tab, Vec::new(), time)
    }

    #[allow(clippy::too_many_arguments)]
    fn run_inspector_frame(
        ctx: &egui::Context,
        proj: &mut ProjectDocument,
        rs: &ResolvedSetup,
        tools: &ToolLibrary,
        history: &mut UndoHistory,
        tab: OperationTab,
        events: Vec<egui::Event>,
        time: &mut f64,
    ) -> Vec<(String, egui::Rect)> {
        *time += 0.25;
        let mut cur_tab = tab;
        let mut pick_mode = ViewportPickMode::InspectEntity;
        let mut pick_target = GeometryPickTarget::CuttingArea;
        let mut editing_shared_avoid_id: Option<String> = None;
        let mut lock_chipload = true;
        let mut show_z_planes = false;
        let mut auto_rotate_indexed_op = true;
        let mut open_tool_manager = false;
        let mut out = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(480.0, 2200.0),
                )),
                time: Some(*time),
                events,
                ..Default::default()
            },
            |ui| {
                egui::CentralPanel::default().show(ui, |ui| {
                    show_operation_inspector(
                        ui,
                        OperationInspectorCtx {
                            project: proj,
                            setup_idx: 0,
                            op_idx: 0,
                            global_tools: tools,
                            history,
                            cad_doc: None,
                            resolved_setup: Some(rs),
                            surface_mesh: None,
                            boundary_preview: None,
                            selected_face_normal_wcs: None,
                            extracted: None,
                            tab: &mut cur_tab,
                            pick_mode: &mut pick_mode,
                            pick_target: &mut pick_target,
                            editing_shared_avoid_id: &mut editing_shared_avoid_id,
                            lock_chipload: &mut lock_chipload,
                            show_z_planes: &mut show_z_planes,
                            auto_rotate_indexed_op: &mut auto_rotate_indexed_op,
                            open_tool_manager: &mut open_tool_manager,
                            sim_engine: chip_sim::SimEngineKind::TriDexel,
                        },
                    );
                });
            },
        );
        out.textures_delta.clear();
        collect_text_rects(&out.shapes)
    }

    fn find_widget_on_row(
        rects: &[(String, egui::Rect)],
        row_label: &str,
        predicate: impl Fn(&str) -> bool,
    ) -> egui::Pos2 {
        let lbl_rect = rects
            .iter()
            .find(|(t, _)| t == row_label)
            .unwrap_or_else(|| panic!("expected row label '{row_label}' to render"))
            .1;
        rects
            .iter()
            .find(|(t, r)| {
                r.min.x > lbl_rect.max.x - 4.0
                    && (r.center().y - lbl_rect.center().y).abs() < 12.0
                    && predicate(t)
            })
            .unwrap_or_else(|| panic!("expected control matching predicate on row '{row_label}'"))
            .1
            .center()
    }

    #[test]
    fn m87_tool_tab_feed_overrides_and_reset_with_undo() {
        let tools = default_starter_tool_library();
        let rs = make_resolved_box_setup();
        let mut proj = ProjectDocument::new("m87_tool_tab");
        proj.display_units = DisplayUnit::Inch;
        proj.tools = tools.tools.clone();
        let mut op = Operation::new("op1", "Pocket", OperationKind::Pocket2D);
        op.tool_id = Some(tools.tools[0].id.clone());
        proj.setups[0].operations = vec![op];

        let ctx = egui::Context::default();
        let mut history = UndoHistory::new(32);
        let mut time = 1.0;

        let _ = run_inspector_frame(
            &ctx,
            &mut proj,
            &rs,
            &tools,
            &mut history,
            OperationTab::Tool,
            Vec::new(),
            &mut time,
        );

        // Mutate Lead Feed, Ramp Feed, Plunge Feed (in Inch / ipm), and Axial Stepdown (in).
        for (row, suffix, expected_undo, expected_reset) in [
            (
                "Lead Feed",
                " ipm",
                "Adjust Lead Feedrate",
                "Reset Lead Feedrate",
            ),
            (
                "Ramp Feed",
                " ipm",
                "Adjust Ramp Feedrate",
                "Reset Ramp Feedrate",
            ),
            (
                "Plunge Feed",
                " ipm",
                "Adjust Plunge Feedrate",
                "Reset Plunge Feedrate",
            ),
            (
                "Axial Stepdown",
                " in",
                "Adjust Stepdown Override",
                "Reset Stepdown Override",
            ),
        ] {
            let rects = run_inspector_frame(
                &ctx,
                &mut proj,
                &rs,
                &tools,
                &mut history,
                OperationTab::Tool,
                Vec::new(),
                &mut time,
            );
            let drag_pt = find_widget_on_row(&rects, row, |t| t.ends_with(suffix));
            let after_drag = drag_inspector_widget(
                &ctx,
                &mut proj,
                &rs,
                &tools,
                &mut history,
                OperationTab::Tool,
                drag_pt,
                35.0,
                &mut time,
            );
            assert_eq!(
                history.peek_undo_label(),
                Some(expected_undo),
                "dragging {row} must record undo label '{expected_undo}'"
            );
            let reset_pt = find_widget_on_row(&after_drag, row, |t| t == "Reset");
            let _ = run_inspector_frame(
                &ctx,
                &mut proj,
                &rs,
                &tools,
                &mut history,
                OperationTab::Tool,
                click_pos_events(reset_pt),
                &mut time,
            );
            assert_eq!(
                history.peek_undo_label(),
                Some(expected_reset),
                "clicking Reset on {row} must record undo label '{expected_reset}'"
            );
        }

        let ov = &proj.setups[0].operations[0].feed_overrides;
        assert!(ov.lead_feed_mm_min.is_none());
        assert!(ov.ramp_feed_mm_min.is_none());
        assert!(ov.plunge_feed_mm_min.is_none());
        assert!(ov.stepdown_mm.is_none());
    }

    #[test]
    fn m87_linking_tab_independent_lead_out_and_pre_drilled_entry_with_undo() {
        use chip_cam::EntryStrategy;

        let tools = default_starter_tool_library();
        let rs = make_resolved_box_setup();
        let mut proj = ProjectDocument::new("m87_linking");
        proj.display_units = DisplayUnit::Millimeter;
        proj.tools = tools.tools.clone();
        let mut op = Operation::new("op_lnk", "Contour", OperationKind::Contour2D);
        op.tool_id = Some(tools.tools[0].id.clone());
        // Disable lead-in to prove Lead-Out / Mirror Lead-In is decoupled from lead_in.enabled!
        op.linking.lead_in.enabled = false;
        op.linking.lead_out_same_as_in = true;
        op.linking.lead_out.enabled = true;
        proj.setups[0].operations = vec![op];

        let ctx = egui::Context::default();
        let mut history = UndoHistory::new(32);
        let mut time = 1.0;

        let rects = run_inspector_frame(
            &ctx,
            &mut proj,
            &rs,
            &tools,
            &mut history,
            OperationTab::Linking,
            Vec::new(),
            &mut time,
        );
        // Mirror Lead-In must render even when lead_in.enabled == false.
        let mirror_pt = find_widget_on_row(&rects, "Lead-Out", |t| t == "Mirror Lead-In");
        let after_unmirror = run_inspector_frame(
            &ctx,
            &mut proj,
            &rs,
            &tools,
            &mut history,
            OperationTab::Linking,
            click_pos_events(mirror_pt),
            &mut time,
        );
        assert!(!proj.setups[0].operations[0].linking.lead_out_same_as_in);
        assert_eq!(history.peek_undo_label(), Some("Toggle Mirror Lead-Out"));

        // Verify all 5 independent Lead-Out controls render.
        for lbl in [
            "Tangent Lead-Out",
            "Lead-Out Radius",
            "Lead-Out Sweep",
            "Lead-Out Linear",
            "Lead-Out Vert. R",
        ] {
            assert!(
                after_unmirror.iter().any(|(t, _)| t == lbl),
                "expected independent lead-out control '{lbl}' to render"
            );
        }

        // Mutate Lead-Out Sweep DragValue and verify undo label.
        let sweep_pt = find_widget_on_row(&after_unmirror, "Lead-Out Sweep", |t| t.ends_with('°'));
        let orig_sweep = proj.setups[0].operations[0]
            .linking
            .lead_out
            .horizontal_sweep_deg;
        let _ = drag_inspector_widget(
            &ctx,
            &mut proj,
            &rs,
            &tools,
            &mut history,
            OperationTab::Linking,
            sweep_pt,
            25.0,
            &mut time,
        );
        assert_ne!(
            proj.setups[0].operations[0]
                .linking
                .lead_out
                .horizontal_sweep_deg,
            orig_sweep
        );
        assert_eq!(history.peek_undo_label(), Some("Edit Lead-Out Sweep"));

        // Mutate Lead-Out Radius ParamLength via Enter text edit.
        let r_rects = run_inspector_frame(
            &ctx,
            &mut proj,
            &rs,
            &tools,
            &mut history,
            OperationTab::Linking,
            Vec::new(),
            &mut time,
        );
        let lo_rad_pt = find_widget_on_row(&r_rects, "Lead-Out Radius", |t| !t.is_empty());
        let _ = run_inspector_frame(
            &ctx,
            &mut proj,
            &rs,
            &tools,
            &mut history,
            OperationTab::Linking,
            click_pos_events(lo_rad_pt),
            &mut time,
        );
        let _ = run_inspector_frame(
            &ctx,
            &mut proj,
            &rs,
            &tools,
            &mut history,
            OperationTab::Linking,
            vec![
                egui::Event::Key {
                    key: egui::Key::A,
                    physical_key: None,
                    pressed: true,
                    repeat: false,
                    modifiers: egui::Modifiers::COMMAND,
                },
                egui::Event::Text("2.75".to_string()),
                egui::Event::Key {
                    key: egui::Key::Enter,
                    physical_key: None,
                    pressed: true,
                    repeat: false,
                    modifiers: egui::Modifiers::NONE,
                },
            ],
            &mut time,
        );
        assert!(
            (proj.setups[0].operations[0]
                .linking
                .lead_out
                .horizontal_radius
                .value_mm
                - 2.75)
                .abs()
                < 1e-6
        );
        assert_eq!(history.peek_undo_label(), Some("Edit Lead-Out Radius"));

        // Toggle Tangent Lead-Out off and verify Undo label.
        let lo_en_pt = find_widget_on_row(&r_rects, "Tangent Lead-Out", |t| t == "Enabled");
        let _ = run_inspector_frame(
            &ctx,
            &mut proj,
            &rs,
            &tools,
            &mut history,
            OperationTab::Linking,
            click_pos_events(lo_en_pt),
            &mut time,
        );
        assert!(!proj.setups[0].operations[0].linking.lead_out.enabled);
        assert_eq!(history.peek_undo_label(), Some("Toggle Lead-Out"));

        // Select Pre-Drilled Point from Entry Mode ComboBox and mutate X & Y in Inch units.
        proj.display_units = DisplayUnit::Inch;
        let ent_rects = run_inspector_frame(
            &ctx,
            &mut proj,
            &rs,
            &tools,
            &mut history,
            OperationTab::Linking,
            Vec::new(),
            &mut time,
        );
        let ent_combo_pt = find_widget_on_row(&ent_rects, "Entry Mode", |t| t == "Helical Ramp");
        let ent_popup = open_inspector_combo(
            &ctx,
            &mut proj,
            &rs,
            &tools,
            &mut history,
            OperationTab::Linking,
            ent_combo_pt,
            &mut time,
        );
        let pre_opt = ent_popup
            .iter()
            .find(|(t, _)| t == "Pre-Drilled Point")
            .expect("Pre-Drilled Point must appear in Entry Mode dropdown")
            .1
            .center();
        let _ = run_inspector_frame(
            &ctx,
            &mut proj,
            &rs,
            &tools,
            &mut history,
            OperationTab::Linking,
            click_pos_events(pre_opt),
            &mut time,
        );
        assert!(matches!(
            proj.setups[0].operations[0].linking.entry_mode,
            EntryStrategy::PreDrilled { .. }
        ));
        assert_eq!(history.peek_undo_label(), Some("Change Entry Strategy"));

        let after_pre = run_inspector_frame(
            &ctx,
            &mut proj,
            &rs,
            &tools,
            &mut history,
            OperationTab::Linking,
            Vec::new(),
            &mut time,
        );
        let x_pt = find_widget_on_row(&after_pre, "Pre-Drill X", |t| t.ends_with(" in"));
        let _ = drag_inspector_widget(
            &ctx,
            &mut proj,
            &rs,
            &tools,
            &mut history,
            OperationTab::Linking,
            x_pt,
            25.0,
            &mut time,
        );
        assert_eq!(
            history.peek_undo_label(),
            Some("Edit Pre-Drilled Entry Point")
        );
        let y_pt = find_widget_on_row(&after_pre, "Pre-Drill Y", |t| t.ends_with(" in"));
        let _ = drag_inspector_widget(
            &ctx,
            &mut proj,
            &rs,
            &tools,
            &mut history,
            OperationTab::Linking,
            y_pt,
            25.0,
            &mut time,
        );
        assert_eq!(
            history.peek_undo_label(),
            Some("Edit Pre-Drilled Entry Point")
        );
        if let EntryStrategy::PreDrilled { point_wcs } =
            proj.setups[0].operations[0].linking.entry_mode
        {
            assert!(point_wcs.x.abs() > 0.1 && point_wcs.y.abs() > 0.1);
        } else {
            panic!("expected PreDrilled entry mode");
        }
    }

    #[test]
    fn m87_passes_tab_missing_strategy_params_and_undo() {
        use chip_cam::DrillCycleKind;
        use chip_tools::{CuttingPreset, SurfaceSpeedPresetSpec};

        let mut tools = default_starter_tool_library();
        // Add a second preset to tool 0 (flat endmill) for VCarve Clearing Preset testing.
        let flat_tool = tools
            .tools
            .iter_mut()
            .find(|t| {
                t.geometry
                    .supports_operation(ToolOperationClass::VCarveFlatClearance)
            })
            .expect("starter library must include a flat endmill");
        let flat_id = flat_tool.id.clone();
        let flat_d = flat_tool.geometry.cutting_diameter_mm();
        let flat_fl = flat_tool.geometry.flutes();
        flat_tool
            .presets
            .push(CuttingPreset::from_surface_speed(SurfaceSpeedPresetSpec {
                id: "hardwood_clear",
                name: "Hardwood Clearance",
                material: "Hardwood",
                diameter_mm: flat_d,
                flutes: flat_fl,
                surface_speed_m_min: 300.0,
                chipload_mm_tooth: 0.06,
                default_stepdown_mm: 2.0,
                default_stepover_mm: 2.0,
                coolant_override: Some(chip_tools::CoolantMode::AirBlast),
            }));

        let rs = make_resolved_box_setup();
        let ctx = egui::Context::default();
        let mut history = UndoHistory::new(64);
        let mut time = 1.0;

        // 1. DrillBore: Dwell, Chip-Break Retract, Decreasing Peck + Min Peck Depth, Fixed Bore Pitch, Bore Finish Step.
        {
            let mut proj = ProjectDocument::new("m87_drill");
            proj.tools = tools.tools.clone();
            let mut op = Operation::new("op_dr", "Drill", OperationKind::DrillBore);
            op.tool_id = Some(flat_id.clone());
            if let StrategyParams::DrillBore(p) = op.strategy_mut() {
                p.cycle_kind = DrillCycleKind::ChipBreak;
                p.bore_finish_pass = true;
            }
            proj.setups[0].operations = vec![op];

            let rects = run_inspector_frame(
                &ctx,
                &mut proj,
                &rs,
                &tools,
                &mut history,
                OperationTab::Passes,
                Vec::new(),
                &mut time,
            );
            for expected in [
                "Dwell (s)",
                "Chip-Break Retract",
                "Decreasing Peck",
                "Fixed Bore Pitch",
                "Bore Finish Step",
            ] {
                assert!(
                    rects.iter().any(|(t, _)| t == expected),
                    "DrillBore (ChipBreak) must render '{expected}'"
                );
            }

            // Edit Dwell (s)
            let dwell_pt = find_widget_on_row(&rects, "Dwell (s)", |t| t.ends_with(" s"));
            let _ = drag_inspector_widget(
                &ctx,
                &mut proj,
                &rs,
                &tools,
                &mut history,
                OperationTab::Passes,
                dwell_pt,
                25.0,
                &mut time,
            );
            assert_eq!(history.peek_undo_label(), Some("Edit Drill Dwell"));

            // Enable Decreasing Peck -> reveals Peck Decay Factor & Min Peck Depth
            let dec_pt = find_widget_on_row(&rects, "Decreasing Peck", |t| t == "Enable");
            let after_dec = run_inspector_frame(
                &ctx,
                &mut proj,
                &rs,
                &tools,
                &mut history,
                OperationTab::Passes,
                click_pos_events(dec_pt),
                &mut time,
            );
            assert_eq!(history.peek_undo_label(), Some("Toggle Decreasing Peck"));
            assert!(after_dec.iter().any(|(t, _)| t == "Peck Decay Factor"));
            assert!(after_dec.iter().any(|(t, _)| t == "Min Peck Depth"));

            let decay_pt =
                find_widget_on_row(&after_dec, "Peck Decay Factor", |t| t.ends_with('x'));
            let _ = drag_inspector_widget(
                &ctx,
                &mut proj,
                &rs,
                &tools,
                &mut history,
                OperationTab::Passes,
                decay_pt,
                -20.0,
                &mut time,
            );
            assert_eq!(
                history.peek_undo_label(),
                Some("Edit Decreasing Peck Factor")
            );

            // Enable Fixed Bore Pitch -> reveals Pitch / Rev
            let pitch_toggle = find_widget_on_row(&after_dec, "Fixed Bore Pitch", |t| {
                t == "Override Pitch/Rev"
            });
            let after_pitch = run_inspector_frame(
                &ctx,
                &mut proj,
                &rs,
                &tools,
                &mut history,
                OperationTab::Passes,
                click_pos_events(pitch_toggle),
                &mut time,
            );
            assert_eq!(
                history.peek_undo_label(),
                Some("Toggle Bore Pitch Override")
            );
            assert!(after_pitch.iter().any(|(t, _)| t == "Pitch / Rev"));

            // Verify Spot (G82) also shows Dwell (s)
            if let StrategyParams::DrillBore(p) = proj.setups[0].operations[0].strategy_mut() {
                p.cycle_kind = DrillCycleKind::Spot;
            }
            let spot_rects = run_inspector_frame(
                &ctx,
                &mut proj,
                &rs,
                &tools,
                &mut history,
                OperationTab::Passes,
                Vec::new(),
                &mut time,
            );
            assert!(spot_rects.iter().any(|(t, _)| t == "Dwell (s)"));
        }

        // 2. Contour2D: optional Finish Pass Feed when finish_passes > 0
        {
            let mut proj = ProjectDocument::new("m87_contour");
            proj.display_units = DisplayUnit::Inch;
            proj.tools = tools.tools.clone();
            let mut op = Operation::new("op_cnt", "Contour", OperationKind::Contour2D);
            op.tool_id = Some(flat_id.clone());
            if let StrategyParams::Contour2D(p) = op.strategy_mut() {
                p.finish_passes = 1;
            }
            proj.setups[0].operations = vec![op];

            let rects = run_inspector_frame(
                &ctx,
                &mut proj,
                &rs,
                &tools,
                &mut history,
                OperationTab::Passes,
                Vec::new(),
                &mut time,
            );
            let ovr_pt = find_widget_on_row(&rects, "Finish Pass Feed", |t| t == "Override");
            let after_ovr = run_inspector_frame(
                &ctx,
                &mut proj,
                &rs,
                &tools,
                &mut history,
                OperationTab::Passes,
                click_pos_events(ovr_pt),
                &mut time,
            );
            assert_eq!(
                history.peek_undo_label(),
                Some("Toggle Contour Finish Feed")
            );
            let ff_pt = find_widget_on_row(&after_ovr, "Finish Pass Feed", |t| t.ends_with(" ipm"));
            let _ = drag_inspector_widget(
                &ctx,
                &mut proj,
                &rs,
                &tools,
                &mut history,
                OperationTab::Passes,
                ff_pt,
                30.0,
                &mut time,
            );
            assert_eq!(history.peek_undo_label(), Some("Edit Contour Finish Feed"));
            if let StrategyParams::Contour2D(p) = proj.setups[0].operations[0].resolved_strategy() {
                assert!(p.finish_feed_mm_min.is_some());
            }
        }

        // 3. Adaptive2D & Adaptive3D: Min Trochoid R + Adaptive3D Stay-Down Feed, Max RCT Boost, Min Flat Area
        {
            let mut proj = ProjectDocument::new("m87_adp2d");
            proj.tools = tools.tools.clone();
            let mut op = Operation::new("op_a2", "Adaptive 2D", OperationKind::Pocket2D);
            op.tool_id = Some(flat_id.clone());
            if let StrategyParams::Pocket2D(p) = op.strategy_mut() {
                p.strategy = PocketStrategyKind::Adaptive(AdaptiveClearingParams::default());
            }
            proj.setups[0].operations = vec![op];
            let rects = run_inspector_frame(
                &ctx,
                &mut proj,
                &rs,
                &tools,
                &mut history,
                OperationTab::Passes,
                Vec::new(),
                &mut time,
            );
            assert!(
                rects.iter().any(|(t, _)| t == "Min Trochoid R"),
                "2D Adaptive must render Min Trochoid R"
            );
        }
        {
            let mut proj = ProjectDocument::new("m87_adp3d");
            proj.display_units = DisplayUnit::Millimeter;
            proj.tools = tools.tools.clone();
            let mut op = Operation::new("op_a3", "Adaptive 3D", OperationKind::Adaptive3D);
            op.tool_id = Some(flat_id.clone());
            proj.setups[0].operations = vec![op];

            let rects = run_inspector_frame(
                &ctx,
                &mut proj,
                &rs,
                &tools,
                &mut history,
                OperationTab::Passes,
                Vec::new(),
                &mut time,
            );
            for expected in [
                "Min Trochoid R",
                "Min Flat Area",
                "Stay-Down Feed",
                "Max RCT Boost",
            ] {
                assert!(
                    rects.iter().any(|(t, _)| t == expected),
                    "3D Adaptive must render '{expected}'"
                );
            }
            let flat_pt = find_widget_on_row(&rects, "Min Flat Area", |t| t.ends_with(" mm2"));
            let _ = drag_inspector_widget(
                &ctx,
                &mut proj,
                &rs,
                &tools,
                &mut history,
                OperationTab::Passes,
                flat_pt,
                25.0,
                &mut time,
            );
            assert_eq!(
                history.peek_undo_label(),
                Some("Edit 3D Adaptive Min Flat Area")
            );

            let stay_pt = find_widget_on_row(&rects, "Stay-Down Feed", |t| t.ends_with('x'));
            let _ = drag_inspector_widget(
                &ctx,
                &mut proj,
                &rs,
                &tools,
                &mut history,
                OperationTab::Passes,
                stay_pt,
                20.0,
                &mut time,
            );
            assert_eq!(
                history.peek_undo_label(),
                Some("Edit 3D Adaptive Stay-Down Feed Multiplier")
            );

            let rct_pt = find_widget_on_row(&rects, "Max RCT Boost", |t| t.ends_with('x'));
            let _ = drag_inspector_widget(
                &ctx,
                &mut proj,
                &rs,
                &tools,
                &mut history,
                OperationTab::Passes,
                rct_pt,
                20.0,
                &mut time,
            );
            assert_eq!(
                history.peek_undo_label(),
                Some("Edit 3D Adaptive Max Chip-Thinning Multiplier")
            );
        }

        // 4. VCarve: Clearing Preset ComboBox populated from Stage 1 flat-clearing tool's presets.
        {
            let mut proj = ProjectDocument::new("m87_vcarve");
            proj.tools = tools.tools.clone();
            let mut op = Operation::new("op_vc", "V-Carve", OperationKind::VCarve);
            op.tool_id = Some(flat_id.clone());
            if let StrategyParams::VCarve(p) = op.strategy_mut() {
                p.flat_depth = Some(ParamLength::from_mm(3.0));
                p.clearing_tool_id = Some(flat_id.clone());
                p.clearing_preset_id = None;
            }
            proj.setups[0].operations = vec![op];

            let rects = run_inspector_frame(
                &ctx,
                &mut proj,
                &rs,
                &tools,
                &mut history,
                OperationTab::Passes,
                Vec::new(),
                &mut time,
            );
            let preset_cb =
                find_widget_on_row(&rects, "Clearing Preset", |t| t == "Auto (First Preset)");
            let popup = open_inspector_combo(
                &ctx,
                &mut proj,
                &rs,
                &tools,
                &mut history,
                OperationTab::Passes,
                preset_cb,
                &mut time,
            );
            let hw_opt = popup
                .iter()
                .find(|(t, _)| t == "Hardwood Clearance")
                .expect("Hardwood Clearance preset must appear in Clearing Preset ComboBox")
                .1
                .center();
            let _ = run_inspector_frame(
                &ctx,
                &mut proj,
                &rs,
                &tools,
                &mut history,
                OperationTab::Passes,
                click_pos_events(hw_opt),
                &mut time,
            );
            assert_eq!(
                history.peek_undo_label(),
                Some("Change V-Carve Clearing Preset")
            );
            if let StrategyParams::VCarve(p) = proj.setups[0].operations[0].resolved_strategy() {
                assert_eq!(p.clearing_preset_id.as_deref(), Some("hardwood_clear"));
            } else {
                panic!("expected VCarve strategy");
            }
        }
    }

    #[test]
    fn tool_tab_shows_readonly_project_material_pill_and_automatch_on_tool_switch() {
        use chip_tools::{ALUMINUM_6063_T5_ID, ProjectMaterialSelection, builtin_carvera_machine};

        let ctx = egui::Context::default();
        let carvera = builtin_carvera_machine();
        let carvera_lib = carvera.to_tool_library();
        let t1 = carvera.tools.iter().find(|t| t.tool_number == 1).unwrap();
        let t2 = carvera.tools.iter().find(|t| t.tool_number == 2).unwrap();

        let mut proj = ProjectDocument::new("M93 Tool Tab Test");
        proj.workpiece_material = Some(ProjectMaterialSelection::builtin(ALUMINUM_6063_T5_ID));
        proj.ensure_tool_embedded(t1);
        let mut op = Operation::new("op1", "Pocket 1", OperationKind::Pocket2D);
        op.tool_id = Some(t1.id.clone());
        op.preset_id = Some("carvera_atc_al6063".to_string());
        proj.setups[0].operations = vec![op];

        let rs = make_resolved_box_setup();
        let mut history = UndoHistory::default();
        let mut time = 1.0;

        let run_tool_tab = |events: Vec<egui::Event>,
                            proj: &mut ProjectDocument,
                            history: &mut UndoHistory,
                            t: &mut f64| {
            *t += 0.2;
            let mut cur_tab = OperationTab::Tool;
            let mut pick_mode = ViewportPickMode::InspectEntity;
            let mut pick_target = GeometryPickTarget::CuttingArea;
            let mut editing_shared_avoid_id: Option<String> = None;
            let mut lock_chipload = true;
            let mut show_z_planes = false;
            let mut auto_rotate_indexed_op = true;
            let mut open_tool_manager = false;
            let mut action = None;

            let mut out = ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(
                        egui::Pos2::ZERO,
                        egui::vec2(420.0, 1100.0),
                    )),
                    time: Some(*t),
                    events,
                    ..Default::default()
                },
                |ui| {
                    egui::CentralPanel::default().show(ui, |ui| {
                        action = show_operation_inspector(
                            ui,
                            OperationInspectorCtx {
                                project: proj,
                                setup_idx: 0,
                                op_idx: 0,
                                global_tools: &carvera_lib,
                                history,
                                cad_doc: None,
                                resolved_setup: Some(&rs),
                                surface_mesh: None,
                                boundary_preview: None,
                                selected_face_normal_wcs: None,
                                extracted: None,
                                tab: &mut cur_tab,
                                pick_mode: &mut pick_mode,
                                pick_target: &mut pick_target,
                                editing_shared_avoid_id: &mut editing_shared_avoid_id,
                                lock_chipload: &mut lock_chipload,
                                show_z_planes: &mut show_z_planes,
                                auto_rotate_indexed_op: &mut auto_rotate_indexed_op,
                                open_tool_manager: &mut open_tool_manager,
                                sim_engine: chip_sim::SimEngineKind::TriDexel,
                            },
                        );
                    });
                },
            );
            out.textures_delta.clear();
            fn visit_rects(shape: &egui::Shape, out: &mut Vec<(String, egui::Rect)>) {
                match shape {
                    egui::Shape::Text(ts) => {
                        out.push((ts.galley.text().to_string(), ts.visual_bounding_rect()));
                    }
                    egui::Shape::Vec(v) => {
                        for s in v {
                            visit_rects(s, out);
                        }
                    }
                    _ => {}
                }
            }
            let mut rects = Vec::new();
            for cs in &out.shapes {
                visit_rects(&cs.shape, &mut rects);
            }
            (rects, action)
        };

        let (rects, _) = run_tool_tab(Vec::new(), &mut proj, &mut history, &mut time);
        let pill_label = "Project Material: Aluminum 6063-T5 Extrusion";
        let pill_pos = rects
            .iter()
            .find(|(txt, _)| txt == pill_label)
            .expect("Operation Inspector [Tool] must render read-only Project Material pill")
            .1
            .center();
        assert!(
            !rects.iter().any(|(txt, _)| txt == "Stock Material"),
            "Operation Inspector [Tool] must never render a per-operation Stock Material picker"
        );

        // Clicking the read-only pill emits OpenProjectMaterial
        let (_, act) = run_tool_tab(
            click_pos_events(pill_pos),
            &mut proj,
            &mut history,
            &mut time,
        );
        assert_eq!(act, Some(OperationInspectorAction::OpenProjectMaterial));

        // Switching tool from T1 to T2 (3.175x25 1FL) must auto-select 6063-T5 Deep preset!
        let (rects2, _) = run_tool_tab(Vec::new(), &mut proj, &mut history, &mut time);
        let tool_cb_pos = rects2
            .iter()
            .find(|(txt, _)| txt.contains("T1"))
            .expect("Active Cutter ComboBox must render T1")
            .1
            .center();
        let _ = run_tool_tab(
            click_pos_events(tool_cb_pos),
            &mut proj,
            &mut history,
            &mut time,
        );
        let (popup_rects, _) = run_tool_tab(Vec::new(), &mut proj, &mut history, &mut time);
        let t2_opt = popup_rects
            .iter()
            .find(|(txt, _)| txt.contains("25mm 1FL"))
            .expect("T2 25mm 1FL O-flute must appear in Active Cutter popup")
            .1
            .center();
        let _ = run_tool_tab(click_pos_events(t2_opt), &mut proj, &mut history, &mut time);
        assert_eq!(proj.setups[0].operations[0].tool_id.as_ref(), Some(&t2.id));
        assert_eq!(
            proj.setups[0].operations[0].preset_id.as_deref(),
            Some("carvera_atc_al6063_deep"),
            "Tool switch must auto-select matching 6063-T5 preset for project workpiece material"
        );
    }
}
