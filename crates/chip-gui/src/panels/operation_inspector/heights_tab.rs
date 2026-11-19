use chip_cam::{HeightReference, Operation, ResolvedHeights, ResolvedSetup};
use chip_math::{DisplayUnit, EvalContext};

use crate::widgets::edit_param_length;

pub struct HeightsTabCtx<'a> {
    pub op: &'a mut Operation,
    pub setup_idx: usize,
    pub op_idx: usize,
    pub display_unit: DisplayUnit,
    pub eval_ctx: &'a EvalContext,
    pub resolved_setup: Option<&'a ResolvedSetup>,
    pub inf_top: f64,
    pub inf_bot: f64,
    pub show_z_planes: &'a mut bool,
    pub mutated_label: &'a mut Option<&'static str>,
}

pub fn show_heights_tab(ui: &mut egui::Ui, ctx: HeightsTabCtx<'_>) {
    let HeightsTabCtx {
        op,
        setup_idx,
        op_idx,
        display_unit,
        eval_ctx,
        resolved_setup,
        inf_top,
        inf_bot,
        show_z_planes,
        mutated_label,
    } = ctx;

    ui.checkbox(
        show_z_planes,
        "Preview translucent Z-height planes in 3D viewport",
    );
    ui.separator();

    let rows = [
        ("Clearance Height", 0usize),
        ("Retract Height", 1usize),
        ("Feed Height", 2usize),
        ("Top Height", 3usize),
        ("Bottom Height", 4usize),
    ];

    for (title, which) in rows {
        ui.strong(title);
        let spec = match which {
            0 => &mut op.heights.clearance,
            1 => &mut op.heights.retract,
            2 => &mut op.heights.feed,
            3 => &mut op.heights.top,
            _ => &mut op.heights.bottom,
        };
        egui::ComboBox::from_id_salt(("op_h_ref", setup_idx, op_idx, which))
            .selected_text(spec.reference.label())
            .show_ui(ui, |ui| {
                for r in HeightReference::OPERATION_REFS {
                    if ui
                        .selectable_value(&mut spec.reference, r, r.label())
                        .changed()
                    {
                        *mutated_label = Some("Change Operation Height Reference");
                    }
                }
            });
        if edit_param_length(
            ui,
            ("op_h_off", setup_idx, op_idx, which),
            &mut spec.offset,
            eval_ctx,
            120.0,
        ) {
            *mutated_label = Some("Edit Operation Height Offset");
        }
        ui.add_space(4.0);
    }

    if let Some(rs) = resolved_setup {
        ui.separator();
        let mut h_eval = op.heights.clone();
        h_eval.reevaluate(eval_ctx);
        let tok = crate::theme::ThemeTokens::from_ui(ui);
        match ResolvedHeights::resolve(
            &h_eval,
            &rs.model_aabb_wcs,
            &rs.stock.aabb_wcs(),
            rs.clearance_z_wcs,
            rs.retract_z_wcs,
            inf_top,
            inf_bot,
        ) {
            Ok(rh) => {
                ui.horizontal(|ui| {
                    crate::widgets::status_badge_icon(
                        ui,
                        crate::widgets::StatusBadgeKind::Clean,
                        None,
                    );
                    ui.colored_label(tok.clean_fg, "Z-Height Hierarchy Valid");
                });
                ui.label(format!(
                    "Clearance: {}  |  Retract: {}",
                    display_unit.format_length(rh.clearance_z_wcs),
                    display_unit.format_length(rh.retract_z_wcs),
                ));
                ui.label(format!(
                    "Feed:      {}  |  Top:     {}",
                    display_unit.format_length(rh.feed_z_wcs),
                    display_unit.format_length(rh.top_z_wcs),
                ));
                ui.label(format!(
                    "Bottom:    {}",
                    display_unit.format_length(rh.bottom_z_wcs),
                ));
            }
            Err(err) => {
                ui.horizontal(|ui| {
                    crate::widgets::status_badge_icon(
                        ui,
                        crate::widgets::StatusBadgeKind::Error,
                        None,
                    );
                    ui.colored_label(tok.error_fg, err.to_string());
                });
            }
        }
    }
}
