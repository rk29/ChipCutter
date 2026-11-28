pub mod avoid_card;
mod cutting_area_cards;
pub mod geometry_tab;
pub mod heights_tab;
pub mod linking_tab;
pub mod machinability_card;
pub mod passes_tab;
pub mod tool_tab;

use chip_cad::CadDocument;
use chip_cam::{
    BoundaryPreviewOverlay, ExtractedSelectionSummary, MachinabilityFix, OperationStatus,
    ResolvedHeights, ResolvedSetup, SharedAvoidRegion, SurfaceMesh3D, ToolOperationClass,
    apply_machinability_fix, cutter_supports_operation,
};
use chip_machines::MachineDefinition;
use chip_math::{DVec3, EvalContext};
use chip_project::{ProjectDocument, UndoHistory};
use chip_tools::ToolLibrary;

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
    pub active_machine: Option<&'a MachineDefinition>,
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
    pub show_reach_heatmap: &'a mut bool,
    pub force_expand_machinability: bool,
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
        active_machine,
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
        show_reach_heatmap,
        force_expand_machinability,
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
    let mut applied_machinability_fix: Option<MachinabilityFix> = None;
    let mut switch_to_tool_tab = false;
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

    let machinability_report = if matches!(*tab, OperationTab::Tool | OperationTab::Passes) {
        machinability_card::evaluate_operation_machinability(
            project,
            setup_idx,
            op_idx,
            global_tools,
            active_machine,
            resolved_setup,
            cad_doc,
        )
    } else {
        None
    };
    let machine_name = active_machine
        .map(|m| m.name.as_str())
        .unwrap_or("Generic 3/4-Axis CNC Mill");
    let primary_hole_diameter_mm = extracted
        .and_then(|res| res.ok())
        .and_then(|sum| sum.primary_hole_diameter_mm);

    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            match *tab {
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
                        machinability_report: machinability_report.as_ref(),
                        machine_name,
                        show_reach_heatmap,
                        force_expand_machinability,
                        applied_fix: &mut applied_machinability_fix,
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
                                    sum.inferred_top_z_wcs
                                        - (sum.inferred_bottom_z_wcs + axial_stock)
                                })
                            })
                        });
                    let mut clearing_tool_candidates: Vec<passes_tab::ClearingToolCandidate> =
                        Vec::new();
                    for t in project.tools.iter().chain(global_tools.tools.iter()) {
                        if cutter_supports_operation(
                            &t.geometry,
                            ToolOperationClass::VCarveFlatClearance,
                        ) && !clearing_tool_candidates.iter().any(|cand| cand.id == t.id)
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
                            primary_hole_diameter_mm,
                            resolved_thread_depth_mm,
                            machinability_report: machinability_report.as_ref(),
                            sim_engine,
                            display_unit,
                            setup_idx,
                            op_idx,
                            eval_ctx: &eval_ctx,
                            resolved_rotary: resolved_setup.and_then(|rs| rs.rotary),
                            selected_face_normal_wcs,
                            auto_rotate_indexed_op,
                            pick_mode,
                            applied_fix: &mut applied_machinability_fix,
                            switch_to_tool_tab: &mut switch_to_tool_tab,
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
            }
            if force_expand_machinability
                && matches!(*tab, OperationTab::Tool | OperationTab::Passes)
            {
                let (anchor, _) =
                    ui.allocate_exact_size(egui::vec2(1.0, 2.0), egui::Sense::hover());
                ui.scroll_to_rect_animation(
                    anchor,
                    Some(egui::Align::BOTTOM),
                    egui::style::ScrollAnimation::none(),
                );
            }
        });

    if let Some(fix) = applied_machinability_fix
        && apply_machinability_fix(&mut project.setups[setup_idx].operations[op_idx], &fix)
    {
        mutated_label = Some("Apply Safe Machinability Fix");
    }
    if switch_to_tool_tab {
        *tab = OperationTab::Tool;
    }

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
mod tests;
