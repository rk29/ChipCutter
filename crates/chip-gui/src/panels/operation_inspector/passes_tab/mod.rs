pub mod milling_2d;
pub mod vcarve;

use std::ops::RangeInclusive;

use chip_cam::{Operation, OperationKind, RestMachiningConfig, RestMachiningMode, StrategyParams};
use chip_math::{DisplayUnit, EvalContext, ParamLength};
use chip_tools::ToolId;

use crate::{
    panels::ViewportPickMode,
    widgets::{
        BannerKind, INPUT_COL_WIDTH, edit_param_length, grid_row, inline_banner, inspector_grid,
        section_card,
    },
};

/// Standard ComboBox width inside two-column Inspector grids (aligns with `edit_param_length` + pills).
pub(super) const COMBO_COL_WIDTH: f32 = INPUT_COL_WIDTH + 54.0;

/// Shared per-operation UI state threaded into strategy section cards.
pub(super) struct PassItemCtx<'a> {
    pub setup_idx: usize,
    pub op_idx: usize,
    pub eval_ctx: &'a EvalContext,
    pub mutated_label: &'a mut Option<&'static str>,
}

impl PassItemCtx<'_> {
    #[inline]
    pub(super) const fn ids(&self) -> (usize, usize) {
        (self.setup_idx, self.op_idx)
    }

    pub(super) fn grid(
        &mut self,
        ui: &mut egui::Ui,
        id_tag: &'static str,
        f: impl FnOnce(&mut GridForm<'_, '_>),
    ) {
        let id = (id_tag, self.setup_idx, self.op_idx);
        inspector_grid(ui, id, |ui| {
            let mut form = GridForm {
                ui,
                id_tag,
                setup_idx: self.setup_idx,
                op_idx: self.op_idx,
                eval_ctx: self.eval_ctx,
                mutated_label: self.mutated_label,
                next_tip: None,
            };
            f(&mut form);
        });
    }

    pub(super) fn card_grid(
        &mut self,
        ui: &mut egui::Ui,
        id_tag: &'static str,
        title: &str,
        f: impl FnOnce(&mut GridForm<'_, '_>),
    ) {
        let card_id = (id_tag, self.setup_idx, self.op_idx);
        section_card(ui, card_id, title, true, |ui| {
            self.grid(ui, id_tag, f);
        });
    }
}

/// Ergonomic two-column row helper inside an [`inspector_grid`].
pub(super) struct GridForm<'a, 'u> {
    pub ui: &'u mut egui::Ui,
    pub id_tag: &'static str,
    pub setup_idx: usize,
    pub op_idx: usize,
    pub eval_ctx: &'a EvalContext,
    pub mutated_label: &'a mut Option<&'static str>,
    next_tip: Option<&'static str>,
}

impl GridForm<'_, '_> {
    #[inline]
    pub(super) fn set_tip(&mut self, tooltip: &'static str) {
        self.next_tip = Some(tooltip);
    }

    pub(super) fn len(&mut self, label: &str, param: &mut ParamLength, undo_label: &'static str) {
        let tip = self.next_tip.take();
        let id = (self.id_tag, label, self.setup_idx, self.op_idx);
        let ctx = self.eval_ctx;
        let mutated: &mut Option<&'static str> = self.mutated_label;
        grid_row(self.ui, label, tip, |ui| {
            if edit_param_length(ui, id, param, ctx, INPUT_COL_WIDTH) {
                *mutated = Some(undo_label);
            }
        });
    }

    pub(super) fn check(
        &mut self,
        label: &str,
        flag: &mut bool,
        checkbox_text: &str,
        undo_label: &'static str,
    ) {
        let tip = self.next_tip.take();
        let mutated: &mut Option<&'static str> = self.mutated_label;
        grid_row(self.ui, label, tip, |ui| {
            if ui.checkbox(flag, checkbox_text).changed() {
                *mutated = Some(undo_label);
            }
        });
    }

    pub(super) fn toggle(&mut self, label: &str, flag: &mut bool, undo_label: &'static str) {
        self.check(label, flag, "Enabled", undo_label);
    }

    pub(super) fn drag<N: egui::emath::Numeric>(
        &mut self,
        label: &str,
        value: &mut N,
        range: RangeInclusive<N>,
        speed_suffix: (f64, &'static str),
        undo_label: &'static str,
    ) {
        let tip = self.next_tip.take();
        let mutated: &mut Option<&'static str> = self.mutated_label;
        grid_row(self.ui, label, tip, |ui| {
            let mut dv = egui::DragValue::new(value)
                .range(range)
                .speed(speed_suffix.0);
            if !speed_suffix.1.is_empty() {
                dv = dv.suffix(speed_suffix.1);
            }
            if ui.add(dv).changed() {
                *mutated = Some(undo_label);
            }
        });
    }

    pub(super) fn combo<T: Copy + PartialEq>(
        &mut self,
        label: &str,
        value: &mut T,
        options: (&[T], impl Fn(T) -> &'static str),
        undo_label: &'static str,
    ) {
        let tip = self.next_tip.take();
        let id = (self.id_tag, label, self.setup_idx, self.op_idx);
        let mutated: &mut Option<&'static str> = self.mutated_label;
        let (items, fmt_fn) = options;
        grid_row(self.ui, label, tip, |ui| {
            egui::ComboBox::from_id_salt(id)
                .width(COMBO_COL_WIDTH)
                .selected_text(fmt_fn(*value))
                .show_ui(ui, |ui| {
                    for &cand in items {
                        if ui.selectable_value(value, cand, fmt_fn(cand)).changed() {
                            *mutated = Some(undo_label);
                        }
                    }
                });
        });
    }

    pub(super) fn row(
        &mut self,
        label: &str,
        add_control: impl FnOnce(&mut egui::Ui, &mut Option<&'static str>),
    ) {
        let tip = self.next_tip.take();
        let mutated: &mut Option<&'static str> = self.mutated_label;
        grid_row(self.ui, label, tip, |ui| {
            add_control(ui, mutated);
        });
    }
}

pub struct PassesTabCtx<'a> {
    pub op: &'a mut Operation,
    pub preceding_ops: &'a [(String, String)],
    pub clearing_tool_candidates: &'a [(ToolId, String, Option<String>)],
    pub tool_feed_preview: Option<(f64, f64)>,
    pub display_unit: DisplayUnit,
    pub setup_idx: usize,
    pub op_idx: usize,
    pub eval_ctx: &'a EvalContext,
    pub pick_mode: &'a mut ViewportPickMode,
    pub mutated_label: &'a mut Option<&'static str>,
}

pub fn show_passes_tab(ui: &mut egui::Ui, ctx: PassesTabCtx<'_>) {
    let PassesTabCtx {
        op,
        preceding_ops,
        clearing_tool_candidates,
        tool_feed_preview,
        display_unit,
        setup_idx,
        op_idx,
        eval_ctx,
        pick_mode,
        mutated_label,
    } = ctx;

    let mut item = PassItemCtx {
        setup_idx,
        op_idx,
        eval_ctx,
        mutated_label,
    };

    item.card_grid(
        ui,
        "passes_depth_card",
        "Depth Passes & Stock to Leave",
        |f| {
            f.set_tip("Split total axial cut depth into stepped Z levels");
            f.check(
                "Multiple Depths",
                &mut op.passes.multiple_depths,
                "Enable Z Stepdowns",
                "Toggle Multiple Depths",
            );
            if op.passes.multiple_depths {
                f.set_tip("Maximum axial depth of cut (ap) per Z level");
                f.len(
                    "Maximum Stepdown",
                    &mut op.passes.max_stepdown,
                    "Edit Max Stepdown",
                );
                f.set_tip("Divide total cut depth into equal Z stepdowns up to Maximum Stepdown");
                f.check(
                    "Even Stepdowns",
                    &mut op.passes.even_stepdowns,
                    "Equalize Z Levels",
                    "Toggle Even Stepdowns",
                );
            }
            f.set_tip("Radial stock allowance left on vertical walls for a finishing pass");
            f.len(
                "Radial Stock",
                &mut op.passes.stock_to_leave_radial,
                "Edit Radial Stock to Leave",
            );
            f.set_tip("Axial stock allowance left on horizontal floors for a finishing pass");
            f.len(
                "Axial Stock",
                &mut op.passes.stock_to_leave_axial,
                "Edit Axial Stock to Leave",
            );
        },
    );

    if matches!(op.kind, OperationKind::Pocket2D | OperationKind::Contour2D) {
        show_rest_machining_card(ui, &mut op.passes.rest_machining, preceding_ops, &mut item);
    }

    match op.strategy_mut() {
        StrategyParams::Facing(p) => milling_2d::show_facing_params(ui, p, item),
        StrategyParams::Contour2D(p) => milling_2d::show_contour2d_params(ui, p, item, pick_mode),
        StrategyParams::Pocket2D(p) => milling_2d::show_pocket2d_params(
            ui,
            milling_2d::PocketParamsUiCtx {
                p,
                tool_feed_preview,
                display_unit,
                item,
            },
        ),
        StrategyParams::DrillBore(p) => milling_2d::show_drill_bore_params(ui, p, item),
        StrategyParams::ChamferTrace(p) => milling_2d::show_chamfer_trace_params(ui, p, item),
        StrategyParams::VCarve(p) => vcarve::show_vcarve_params(
            ui,
            vcarve::VCarveParamsUiCtx {
                p,
                clearing_tool_candidates,
                item,
            },
        ),
    }
}

fn show_rest_machining_card(
    ui: &mut egui::Ui,
    rest: &mut RestMachiningConfig,
    preceding_ops: &[(String, String)],
    item: &mut PassItemCtx<'_>,
) {
    let (setup_idx, op_idx) = item.ids();
    let eval_ctx = item.eval_ctx;
    section_card(
        ui,
        ("passes_rest_card", setup_idx, op_idx),
        "2.5D Rest Machining",
        true,
        |ui| {
            let cur_mode = match &rest.mode {
                RestMachiningMode::Disabled => 0_u8,
                RestMachiningMode::FromPreviousTool { .. } => 1_u8,
                RestMachiningMode::FromPrecedingOperations { .. } => 2_u8,
            };
            let mut sel_mode = cur_mode;

            item.grid(ui, "passes_rest_mode_grid", |f| {
                f.set_tip("Machine only uncut corners/regions left behind by larger cutters or earlier operations");
                f.row("Rest Mode", |ui, mutated| {
                    if ui.selectable_value(&mut sel_mode, 0, "Off").clicked() && cur_mode != 0 {
                        rest.mode = RestMachiningMode::Disabled;
                        *mutated = Some("Disable Rest Machining");
                    }
                    if ui
                        .selectable_value(&mut sel_mode, 1, "Prev Tool")
                        .on_hover_text("Compute uncut corners analytically from a larger reference cutter diameter")
                        .clicked()
                        && cur_mode != 1
                    {
                        rest.mode = RestMachiningMode::FromPreviousTool {
                            prev_tool_diameter: ParamLength {
                                expr: Some("tool_d * 2.0".to_string()),
                                value_mm: 12.0,
                            },
                            prev_stock_to_leave: ParamLength::from_mm(0.0),
                        };
                        rest.reevaluate(eval_ctx);
                        *mutated = Some("Set Rest Mode: Previous Tool");
                    }
                    if ui
                        .selectable_value(&mut sel_mode, 2, "Preceding")
                        .on_hover_text("Subtract exact 2D swept footprints of earlier operations in this Setup")
                        .clicked()
                        && cur_mode != 2
                    {
                        rest.mode = RestMachiningMode::FromPrecedingOperations {
                            source_op_ids: Vec::new(),
                        };
                        *mutated = Some("Set Rest Mode: Preceding Operations");
                    }
                });

                if let RestMachiningMode::FromPreviousTool {
                    prev_tool_diameter,
                    prev_stock_to_leave,
                } = &mut rest.mode
                {
                    f.set_tip("Reference diameter of the larger roughing cutter used earlier");
                    f.len("Prev. Tool Ø", prev_tool_diameter, "Edit Rest Previous Tool Diameter");
                    f.set_tip("Radial stock allowance left on walls by the previous roughing tool");
                    f.len("Prev. Radial Stock", prev_stock_to_leave, "Edit Rest Previous Stock to Leave");
                }

                if !rest.is_disabled() {
                    f.set_tip("Extra overlap into already-cleared stock so rest passes blend without cusps");
                    f.len("Blend Overlap", &mut rest.rest_overlap, "Edit Rest Overlap");
                    f.set_tip("Ignore uncut slivers thinner than this threshold to avoid air cutting");
                    f.len("Min Thickness", &mut rest.min_rest_thickness, "Edit Minimum Rest Thickness");
                }
            });

            if let RestMachiningMode::FromPrecedingOperations { source_op_ids } = &mut rest.mode {
                ui.add_space(4.0);
                if preceding_ops.is_empty() {
                    inline_banner(
                        ui,
                        BannerKind::Warn,
                        "No enabled operations precede this operation in the Setup.",
                    );
                } else {
                    let mut all_preceding = source_op_ids.is_empty();
                    if ui
                        .checkbox(&mut all_preceding, "All Enabled Preceding Operations")
                        .changed()
                    {
                        if all_preceding {
                            source_op_ids.clear();
                        } else if let Some((first_id, _)) = preceding_ops.first() {
                            source_op_ids.push(first_id.clone());
                        }
                        *item.mutated_label = Some("Toggle All Preceding Rest Ops");
                    }
                    if !all_preceding {
                        for (uid, uname) in preceding_ops {
                            let mut checked = source_op_ids.iter().any(|x| x == uid);
                            if ui.checkbox(&mut checked, uname).changed() {
                                if checked {
                                    source_op_ids.push(uid.clone());
                                } else {
                                    source_op_ids.retain(|x| x != uid);
                                }
                                *item.mutated_label = Some("Update Rest Source Operations");
                            }
                        }
                    }
                }
            }
        },
    );
}
