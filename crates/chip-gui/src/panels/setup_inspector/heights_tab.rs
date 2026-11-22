use chip_cam::{HeightReference, ResolvedSetup, Setup};
use chip_math::{DisplayUnit, EvalContext};

use crate::widgets::{
    BannerKind, INPUT_COL_WIDTH, grid_row, inline_banner, inspector_grid, section_card,
};

use super::SetupGrid;

pub(super) struct HeightsTabCtx<'a> {
    pub(super) setup: &'a mut Setup,
    pub(super) setup_idx: usize,
    pub(super) eval_ctx: &'a EvalContext,
    pub(super) display_unit: DisplayUnit,
    pub(super) resolved: Option<Result<&'a ResolvedSetup, &'a str>>,
    pub(super) show_z_planes: &'a mut bool,
    pub(super) mutated_label: &'a mut Option<&'static str>,
}

pub(super) fn show_heights_tab(ui: &mut egui::Ui, ctx: HeightsTabCtx<'_>) {
    let HeightsTabCtx {
        setup,
        setup_idx,
        eval_ctx,
        display_unit,
        resolved,
        show_z_planes,
        mutated_label,
    } = ctx;

    section_card(
        ui,
        ("setup_safe_z_card", setup_idx),
        "Safe Z Clearance & Retract",
        true,
        |ui| {
            inspector_grid(ui, ("setup_heights_grid", setup_idx), |ui| {
                grid_row(
                    ui,
                    "3D Z Planes",
                    Some("Render translucent Z planes in 3D viewport"),
                    |ui| {
                        ui.checkbox(show_z_planes, "Preview in viewport")
                            .on_hover_text("Overlay translucent Clearance and Retract Z planes inside the 3D viewport");
                    },
                );

                render_setup_height_ref_row(
                    ui,
                    "Clearance Datum",
                    ("setup_clr_ref", setup_idx),
                    &mut setup.heights.clearance.reference,
                    "Change Setup Clearance Reference",
                    mutated_label,
                );
                let mut g = SetupGrid {
                    ui,
                    setup_idx,
                    eval_ctx,
                    mutated: mutated_label,
                };
                g.len(
                    "Clearance Offset",
                    "setup_clr_off",
                    &mut setup.heights.clearance.offset,
                    "Edit Setup Clearance Offset",
                );
                render_setup_height_ref_row(
                    g.ui,
                    "Retract Datum",
                    ("setup_ret_ref", setup_idx),
                    &mut setup.heights.retract.reference,
                    "Change Setup Retract Reference",
                    g.mutated,
                );
                g.len(
                    "Retract Offset",
                    "setup_ret_off",
                    &mut setup.heights.retract.offset,
                    "Edit Setup Retract Offset",
                );
            });

            if let Some(Ok(rs)) = resolved {
                let fmt = |v| display_unit.format_length(v);
                ui.add_space(5.0);
                inline_banner(
                    ui,
                    BannerKind::Info,
                    format!(
                        "Clearance Z: {}  ·  Retract Z: {}\nStock Top: {}  ·  Model Top: {}  ·  Stock Bot: {}",
                        fmt(rs.clearance_z_wcs),
                        fmt(rs.retract_z_wcs),
                        fmt(rs.stock.aabb_wcs().max.z),
                        fmt(rs.model_aabb_wcs.max.z),
                        fmt(rs.stock.aabb_wcs().min.z),
                    ),
                );
            }
        },
    );
}

fn render_setup_height_ref_row(
    ui: &mut egui::Ui,
    label: &str,
    salt: (&str, usize),
    val: &mut HeightReference,
    undo: &'static str,
    mutated: &mut Option<&'static str>,
) {
    grid_row(
        ui,
        label,
        Some("Reference datum plane for Setup Z rapid moves"),
        |ui| {
            egui::ComboBox::from_id_salt(salt)
                .width(INPUT_COL_WIDTH + 30.0)
                .selected_text(val.label())
                .show_ui(ui, |ui| {
                    for r in HeightReference::SETUP_REFS {
                        if ui
                            .selectable_value(val, r, r.label())
                            .on_hover_text(format!("Measure height offset from {}", r.label()))
                            .changed()
                        {
                            *mutated = Some(undo);
                        }
                    }
                })
                .response
                .on_hover_text("Select reference datum plane for this Setup Z height");
        },
    );
}
