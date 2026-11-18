pub mod geometry_tab;
pub mod heights_tab;
pub mod linking_tab;
pub mod passes_tab;
pub mod tool_tab;

use chip_cad::CadDocument;
use chip_cam::{ExtractedSelectionSummary, OperationStatus, ResolvedSetup};
use chip_math::EvalContext;
use chip_project::{ProjectDocument, UndoHistory};
use chip_tools::ToolLibrary;

use crate::panels::{OperationTab, ViewportPickMode, browser::default_pick_mode_for_op};

/// Context passed to [`show_operation_inspector`].
pub struct OperationInspectorCtx<'a> {
    pub project: &'a mut ProjectDocument,
    pub setup_idx: usize,
    pub op_idx: usize,
    pub global_tools: &'a ToolLibrary,
    pub history: &'a mut UndoHistory,
    pub cad_doc: Option<&'a CadDocument>,
    pub resolved_setup: Option<&'a ResolvedSetup>,
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
        extracted,
        tab,
        pick_mode,
        lock_chipload,
        show_z_planes,
        open_tool_manager,
    } = ctx;

    if setup_idx >= project.setups.len() || op_idx >= project.setups[setup_idx].operations.len() {
        ui.weak("No Operation selected.");
        return;
    }

    let display_unit = project.display_units;
    // Snapshot only the single active Operation (never cloning `project.tools` or other Setups/Operations on idle frames!)
    let pre_op = project.setups[setup_idx].operations[op_idx].clone();
    let pre_tools_len = project.tools.len();
    let mut mutated_label: Option<&'static str> = None;

    let op_kind = project.setups[setup_idx].operations[op_idx].kind;
    ui.horizontal(|ui| {
        ui.strong(op_kind.label());
        let name_ref = &mut project.setups[setup_idx].operations[op_idx].name;
        let resp = ui.text_edit_singleline(name_ref);
        if resp.lost_focus() && project.setups[setup_idx].operations[op_idx].name != pre_op.name {
            mutated_label = Some("Rename Operation");
        }
    });

    ui.separator();
    ui.horizontal_wrapped(|ui| {
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
    ui.separator();

    if let OperationStatus::NeedsRepick { message } = project.setups[setup_idx].operations[op_idx]
        .runtime_status
        .clone()
    {
        egui::Frame::NONE
            .fill(egui::Color32::from_rgb(56, 30, 24))
            .stroke(egui::Stroke::new(
                1.0,
                egui::Color32::from_rgb(235, 118, 68),
            ))
            .inner_margin(8.0)
            .corner_radius(4.0)
            .show(ui, |ui| {
                ui.colored_label(
                    egui::Color32::from_rgb(255, 168, 96),
                    "[!] CAD Feature Needs Re-Pick (Associative Reload)",
                );
                ui.add_space(2.0);
                ui.small(&message);
                ui.add_space(4.0);
                ui.horizontal_wrapped(|ui| {
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
            });
        ui.add_space(6.0);
    }

    let mut eval_ctx = resolved_setup
        .map(|rs| rs.eval_context(display_unit))
        .unwrap_or_else(|| EvalContext::new(display_unit));

    if let Some(tid) = project.setups[setup_idx].operations[op_idx]
        .tool_id
        .as_ref()
        && let Some(tool) = project.tool(tid).or_else(|| global_tools.tool(tid))
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
            OperationTab::Passes => passes_tab::show_passes_tab(
                ui,
                &mut project.setups[setup_idx].operations[op_idx],
                setup_idx,
                op_idx,
                &eval_ctx,
                pick_mode,
                &mut mutated_label,
            ),
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
        let mut pre_doc = project.clone();
        pre_doc.tools.truncate(pre_tools_len);
        pre_doc.setups[setup_idx].operations[op_idx] = pre_op;
        history.record_before_edit(action, None, &pre_doc);
        project.setups[setup_idx].operations[op_idx].mark_dirty();
    }
}
