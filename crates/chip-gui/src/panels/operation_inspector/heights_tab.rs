use chip_cam::{HeightReference, Operation, ResolvedHeights, ResolvedSetup};
use chip_math::{DisplayUnit, EvalContext};

use crate::widgets::{
    BannerKind, INPUT_COL_WIDTH, edit_param_length, grid_row, inline_banner, inspector_grid,
    section_card, static_card,
};

const COMBO_COL_WIDTH: f32 = INPUT_COL_WIDTH + 54.0;

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

    let planes = [
        (
            "Clearance",
            "Highest safe Z plane used for initial/final XY rapids above clamps and stock",
            0usize,
        ),
        (
            "Retract",
            "Intermediate rapid retract Z plane between cutting regions",
            1usize,
        ),
        (
            "Feed",
            "Z height where tool transitions from rapid G0 plunge to controlled feedrate",
            2usize,
        ),
        (
            "Top",
            "Upper Z surface where material cutting begins",
            3usize,
        ),
        (
            "Bottom",
            "Final target cut floor Z height (must be <= Top Height)",
            4usize,
        ),
    ];

    section_card(
        ui,
        ("heights_planes_card", setup_idx, op_idx),
        "Z-Height Planes & Offsets",
        true,
        |ui| {
            inspector_grid(ui, ("heights_planes_grid", setup_idx, op_idx), |ui| {
                grid_row(
                    ui,
                    "3D Preview",
                    Some("Render translucent Z-height reference planes in the 3D viewport"),
                    |ui| {
                        ui.checkbox(show_z_planes, "Show Z-Planes")
                            .on_hover_text("Render translucent color-coded Clearance, Retract, Feed, Top, and Bottom Z planes in the 3D viewport");
                    },
                );

                for (name, tip, which) in planes {
                    let spec = match which {
                        0 => &mut op.heights.clearance,
                        1 => &mut op.heights.retract,
                        2 => &mut op.heights.feed,
                        3 => &mut op.heights.top,
                        _ => &mut op.heights.bottom,
                    };
                    let ref_lbl = format!("{name} Ref");
                    let off_lbl = format!("{name} Offset");
                    grid_row(ui, &ref_lbl, Some(tip), |ui| {
                        egui::ComboBox::from_id_salt(("op_h_ref", setup_idx, op_idx, which))
                            .width(COMBO_COL_WIDTH)
                            .truncate()
                            .selected_text(spec.reference.label())
                            .show_ui(ui, |ui| {
                                for r in HeightReference::OPERATION_REFS {
                                    if ui
                                        .selectable_value(&mut spec.reference, r, r.label())
                                        .on_hover_text(format!(
                                            "Measure {name} height relative to {}",
                                            r.label()
                                        ))
                                        .changed()
                                    {
                                        *mutated_label = Some("Change Operation Height Reference");
                                    }
                                }
                            })
                            .response
                            .on_hover_text(tip);
                    });
                    grid_row(
                        ui,
                        &off_lbl,
                        Some("Signed Z offset added to the selected height reference datum"),
                        |ui| {
                            if edit_param_length(
                                ui,
                                ("op_h_off", setup_idx, op_idx, which),
                                &mut spec.offset,
                                eval_ctx,
                                INPUT_COL_WIDTH,
                            ) {
                                *mutated_label = Some("Edit Operation Height Offset");
                            }
                        },
                    );
                }
            });
        },
    );

    if let Some(rs) = resolved_setup {
        let mut h_eval = op.heights.clone();
        h_eval.reevaluate(eval_ctx);
        static_card(
            ui,
            Some("Resolved WCS Z Hierarchy"),
            |ui| match ResolvedHeights::resolve(
                &h_eval,
                &rs.model_aabb_wcs,
                &rs.stock.aabb_wcs(),
                rs.clearance_z_wcs,
                rs.retract_z_wcs,
                inf_top,
                inf_bot,
            ) {
                Ok(rh) => {
                    inline_banner(
                        ui,
                        BannerKind::Clean,
                        "Z-Height Hierarchy Valid (Clearance ≥ Retract ≥ Feed ≥ Top ≥ Bottom)",
                    );
                    ui.add_space(4.0);
                    inspector_grid(ui, ("heights_resolved_grid", setup_idx, op_idx), |ui| {
                        for (lbl, val) in [
                            ("Clearance Z", rh.clearance_z_wcs),
                            ("Retract Z", rh.retract_z_wcs),
                            ("Feed Z", rh.feed_z_wcs),
                            ("Top Z", rh.top_z_wcs),
                            ("Bottom Z", rh.bottom_z_wcs),
                        ] {
                            grid_row(ui, lbl, None, |ui| {
                                ui.strong(display_unit.format_length(val));
                            });
                        }
                    });
                }
                Err(err) => {
                    inline_banner(ui, BannerKind::Error, err.to_string());
                }
            },
        );
    }
}
