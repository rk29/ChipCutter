pub mod geometry_tab;
pub mod heights_tab;
pub mod linking_tab;
pub mod passes_tab;
pub mod tool_tab;

use chip_cad::CadDocument;
use chip_cam::{ExtractedSelectionSummary, OperationStatus, ResolvedSetup};
use chip_math::{DVec3, EvalContext};
use chip_project::{ProjectDocument, UndoHistory};
use chip_tools::{ToolId, ToolLibrary, ToolOperationClass};

use crate::{
    panels::{OperationTab, ViewportPickMode, browser::default_pick_mode_for_op},
    widgets::{BannerKind, inline_banner, static_card},
};

/// Context passed to [`show_operation_inspector`].
pub struct OperationInspectorCtx<'a> {
    pub project: &'a mut ProjectDocument,
    pub setup_idx: usize,
    pub op_idx: usize,
    pub global_tools: &'a ToolLibrary,
    pub history: &'a mut UndoHistory,
    pub cad_doc: Option<&'a CadDocument>,
    pub resolved_setup: Option<&'a ResolvedSetup>,
    pub selected_face_normal_wcs: Option<DVec3>,
    pub extracted: Option<Result<&'a ExtractedSelectionSummary, &'a str>>,
    pub tab: &'a mut OperationTab,
    pub pick_mode: &'a mut ViewportPickMode,
    pub lock_chipload: &'a mut bool,
    pub show_z_planes: &'a mut bool,
    pub open_tool_manager: &'a mut bool,
}

/// Renders the standardized 5-tab **Operation Inspector** (`[Tool]`, `[Geometry]`, `[Heights]`, `[Passes]`, `[Linking]`).
pub fn show_operation_inspector(ui: &mut egui::Ui, ctx: OperationInspectorCtx<'_>) {
    let OperationInspectorCtx {
        project,
        setup_idx,
        op_idx,
        global_tools,
        history,
        cad_doc,
        resolved_setup,
        selected_face_normal_wcs,
        extracted,
        tab,
        pick_mode,
        lock_chipload,
        show_z_planes,
        open_tool_manager,
    } = ctx;

    if setup_idx >= project.setups.len() || op_idx >= project.setups[setup_idx].operations.len() {
        inline_banner(ui, BannerKind::Info, "No Operation selected.");
        return;
    }

    let display_unit = project.display_units;
    // Snapshot only the single active Operation (never cloning `project.tools` or other Setups/Operations on idle frames!)
    let pre_op = project.setups[setup_idx].operations[op_idx].clone();
    let pre_tools_len = project.tools.len();
    let mut mutated_label: Option<&'static str> = None;

    let op_kind = project.setups[setup_idx].operations[op_idx].kind;
    static_card(ui, None, |ui| {
        ui.horizontal(|ui| {
            ui.strong(op_kind.label());
            let name_ref = &mut project.setups[setup_idx].operations[op_idx].name;
            let resp = ui.add(
                egui::TextEdit::singleline(name_ref).desired_width(ui.available_width().max(80.0)),
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
                .clicked()
            {
                *pick_mode = ViewportPickMode::InspectEntity;
            }
            if ui
                .selectable_value(tab, OperationTab::Geometry, "Geometry")
                .clicked()
            {
                *pick_mode = default_pick_mode_for_op(setup_idx, op_idx, op_kind, *tab);
            }
            if ui
                .selectable_value(tab, OperationTab::Heights, "Heights")
                .clicked()
            {
                *pick_mode = ViewportPickMode::InspectEntity;
            }
            if ui
                .selectable_value(tab, OperationTab::Passes, "Passes")
                .clicked()
            {
                *pick_mode = ViewportPickMode::InspectEntity;
            }
            if ui
                .selectable_value(tab, OperationTab::Linking, "Linking")
                .clicked()
            {
                *pick_mode = ViewportPickMode::InspectEntity;
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
                    mutated_label: &mut mutated_label,
                },
            ),
            OperationTab::Geometry => geometry_tab::show_geometry_tab(
                ui,
                geometry_tab::GeometryTabCtx {
                    op: &mut project.setups[setup_idx].operations[op_idx],
                    sketches: &project.sketches,
                    setup_idx,
                    op_idx,
                    display_unit,
                    cad_doc,
                    extracted,
                    pick_mode,
                    mutated_label: &mut mutated_label,
                },
            ),
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
                let tool_feed_preview = project.setups[setup_idx].operations[op_idx]
                    .tool_id
                    .as_ref()
                    .and_then(|tid| project.tool(tid).or_else(|| global_tools.tool(tid)))
                    .map(|t| {
                        let eff = project.setups[setup_idx].operations[op_idx]
                            .effective_cutting_params(t);
                        (t.geometry.cutting_diameter_mm(), eff.cutting_feed_mm_min)
                    });
                let mut clearing_tool_candidates: Vec<(ToolId, String, Option<String>)> =
                    Vec::new();
                for t in project.tools.iter().chain(global_tools.tools.iter()) {
                    if t.geometry
                        .supports_operation(ToolOperationClass::VCarveFlatClearance)
                        && !clearing_tool_candidates
                            .iter()
                            .any(|(id, _, _)| *id == t.id)
                    {
                        let preset_id = t.presets.first().map(|p| p.id.clone());
                        clearing_tool_candidates.push((
                            t.id.clone(),
                            format!(
                                "T{} — {} (Ø{:.2} mm)",
                                t.tool_number,
                                t.name,
                                t.geometry.cutting_diameter_mm()
                            ),
                            preset_id,
                        ));
                    }
                }
                passes_tab::show_passes_tab(
                    ui,
                    passes_tab::PassesTabCtx {
                        op: &mut project.setups[setup_idx].operations[op_idx],
                        preceding_ops: &preceding_ops,
                        clearing_tool_candidates: &clearing_tool_candidates,
                        tool_feed_preview,
                        display_unit,
                        setup_idx,
                        op_idx,
                        eval_ctx: &eval_ctx,
                        resolved_rotary: resolved_setup.and_then(|rs| rs.rotary),
                        selected_face_normal_wcs,
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
        history.record_before_edit(action, None, &pre_doc);
        project.setups[setup_idx].operations[op_idx].mark_dirty();
    }
}
