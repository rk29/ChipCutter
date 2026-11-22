use std::fmt::Debug;
use std::hash::Hash;

use chip_cam::{FixedBoxXyAlign, FixedBoxZAlign, ResolvedSetup, Setup, StockAxis, StockConfig};
use chip_math::{DisplayUnit, EvalContext, ParamLength};

use crate::widgets::{
    BannerKind, INPUT_COL_WIDTH, grid_row, inline_banner, inspector_grid, section_card,
};

use super::SetupGrid;

#[derive(Clone, Copy)]
pub(super) struct StockTabView<'a> {
    pub(super) setup_idx: usize,
    pub(super) is_pure_2d: bool,
    pub(super) eval_ctx: &'a EvalContext,
    pub(super) display_unit: DisplayUnit,
    pub(super) resolved: Option<Result<&'a ResolvedSetup, &'a str>>,
    pub(super) preceding_setups: &'a [(String, String, bool)],
}

pub(super) fn show_stock_tab(
    ui: &mut egui::Ui,
    setup: &mut Setup,
    view: StockTabView<'_>,
    mutated_label: &mut Option<&'static str>,
) {
    let idx = view.setup_idx;
    if idx > 0 && !view.preceding_setups.is_empty() {
        section_card(
            ui,
            ("stock_carryover_card", idx),
            "Stock Carryover",
            true,
            |ui| {
                render_stock_carryover_body(ui, setup, view, mutated_label);
            },
        );
    }
    section_card(
        ui,
        ("stock_geom_card", idx),
        "Stock Geometry & Dimensions",
        true,
        |ui| {
            render_stock_geom_body(ui, setup, view, mutated_label);
        },
    );
    section_card(
        ui,
        ("stock_offsets_card", idx),
        "Stock Offsets",
        true,
        |ui| {
            render_stock_offsets_body(ui, setup, view, mutated_label);
        },
    );
}

fn render_stock_carryover_body(
    ui: &mut egui::Ui,
    setup: &mut Setup,
    view: StockTabView<'_>,
    mutated_label: &mut Option<&'static str>,
) {
    let selected_label = match setup.carryover_from_setup_id.as_deref() {
        None => "Independent Raw Stock".to_string(),
        Some(src_id) => view
            .preceding_setups
            .iter()
            .find(|(id, _, _)| id == src_id)
            .map_or_else(
                || format!("Carry Over From: {src_id} (missing)"),
                |(_, name, _)| format!("Carry Over From: {name}"),
            ),
    };

    inspector_grid(ui, ("stock_carryover_grid", view.setup_idx), |ui| {
        grid_row(
            ui,
            "Stock Source",
            Some(
                "Choose whether this Setup starts from an independent raw blank or carries over the exact machined 3D Tri-Dexel stock from a preceding Setup",
            ),
            |ui| {
                egui::ComboBox::from_id_salt(("stock_carryover_cb", view.setup_idx))
                    .width(INPUT_COL_WIDTH + 54.0)
                    .truncate()
                    .selected_text(selected_label)
                    .show_ui(ui, |ui| {
                        if ui
                            .selectable_value(
                                &mut setup.carryover_from_setup_id,
                                None,
                                "Independent Raw Stock",
                            )
                            .on_hover_text(
                                "Start this Setup from a fresh raw stock blank defined below",
                            )
                            .changed()
                        {
                            *mutated_label = Some("Disable Stock Carryover");
                        }
                        for (sid, sname, _) in view.preceding_setups {
                            if ui
                                .selectable_value(
                                    &mut setup.carryover_from_setup_id,
                                    Some(sid.clone()),
                                    format!("Carry Over From: {sname}"),
                                )
                                .on_hover_text(format!(
                                    "Transfer the exact machined volumetric Tri-Dexel solid from '{sname}' into this Setup's WCS for rest-aware Op-2 roughing and continuous simulation"
                                ))
                                .changed()
                            {
                                *mutated_label = Some("Change Stock Carryover Source");
                            }
                        }
                    })
                    .response
                    .on_hover_text(
                        "Carry over remaining machined stock from an earlier Setup for multi-sided / flip-part machining (G54 -> G55)",
                    );
            },
        );
    });

    if let Some(src_id) = setup.carryover_from_setup_id.as_deref() {
        ui.add_space(4.0);
        if let Some((_, sname, all_clean)) =
            view.preceding_setups.iter().find(|(id, _, _)| id == src_id)
        {
            if *all_clean {
                inline_banner(
                    ui,
                    BannerKind::Info,
                    format!(
                        "Carryover active from '{sname}' (up-to-date). Rest-aware Op-2 toolpaths & simulation start from '{sname}' remaining 3D stock; Tri-Dexel preserves underside cavities while Heightmap projects top Z."
                    ),
                );
            } else {
                inline_banner(
                    ui,
                    BannerKind::Warn,
                    format!(
                        "Upstream setup '{sname}' has ungenerated or dirty toolpaths — press Ctrl+G to regenerate before simulating."
                    ),
                );
            }
        } else {
            inline_banner(
                ui,
                BannerKind::Error,
                format!("Carryover source setup '{src_id}' was not found among preceding setups."),
            );
        }
    }
}

fn render_stock_geom_body(
    ui: &mut egui::Ui,
    setup: &mut Setup,
    view: StockTabView<'_>,
    mutated_label: &mut Option<&'static str>,
) {
    let StockTabView {
        setup_idx,
        is_pure_2d,
        eval_ctx,
        display_unit,
        resolved,
        ..
    } = view;

    inspector_grid(ui, ("stock_geom_grid", setup_idx), |ui| {
        grid_row(
            ui,
            "Stock Mode",
            Some("Raw stock blank shape and sizing mode"),
            |ui| {
                render_stock_mode_combo(ui, setup_idx, &mut setup.stock, mutated_label);
            },
        );

        let mut g = SetupGrid {
            ui,
            setup_idx,
            eval_ctx,
            mutated: mutated_label,
        };
        match &mut setup.stock {
            StockConfig::RelativeBox {
                sheet_thickness_z,
                round_increment_mm,
                ..
            } => {
                if is_pure_2d {
                    g.len(
                        "Sheet Thickness (Z)",
                        "stk_sheet_z",
                        sheet_thickness_z,
                        "Edit Pure-2D Sheet Thickness",
                    );
                }
                let mut round_on = round_increment_mm.is_some();
                if grid_row(
                    g.ui,
                    "Round Saw Blank",
                    Some("Round total stock up to 5 mm step"),
                    |ui| {
                        ui.checkbox(&mut round_on, "Up to 5 mm step")
                            .on_hover_text("Round computed stock blank dimensions up to the nearest 5 mm increment")
                            .changed()
                    },
                ) {
                    *round_increment_mm = round_on.then_some(5.0);
                    *g.mutated = Some("Toggle Stock Rounding");
                }
            }
            StockConfig::FixedBox {
                width_x,
                depth_y,
                height_z,
                ..
            } => {
                g.len("Width (WCS X)", "fbox_x", width_x, "Edit Fixed Stock Width");
                g.len("Depth (WCS Y)", "fbox_y", depth_y, "Edit Fixed Stock Depth");
                g.len(
                    "Height (WCS Z)",
                    "fbox_z",
                    height_z,
                    "Edit Fixed Stock Height",
                );
            }
            StockConfig::RelativeCylinder { axis, .. } => {
                render_cyl_axis_row(g.ui, ("rcyl_axis", setup_idx), axis, g.mutated);
            }
            StockConfig::FixedCylinder {
                axis,
                diameter,
                length,
                ..
            } => {
                render_cyl_axis_row(g.ui, ("fcyl_axis", setup_idx), axis, g.mutated);
                g.len("Bar Diameter", "fcyl_d", diameter, "Edit Cylinder Diameter");
                g.len("Bar Length", "fcyl_l", length, "Edit Cylinder Length");
            }
        }
    });

    if let Some(Ok(rs)) = resolved {
        let s = rs.stock.aabb_wcs().size();
        let m = rs.model_aabb_wcs.size();
        let fmt = |v| display_unit.format_length(v);
        ui.add_space(5.0);
        inline_banner(
            ui,
            BannerKind::Info,
            format!(
                "Model: {} × {} × {}\nStock: {} × {} × {}",
                fmt(m.x),
                fmt(m.y),
                fmt(m.z),
                fmt(s.x),
                fmt(s.y),
                fmt(s.z)
            ),
        );
    }
}

fn render_stock_mode_combo(
    ui: &mut egui::Ui,
    setup_idx: usize,
    stock: &mut StockConfig,
    mutated_label: &mut Option<&'static str>,
) {
    let mm = ParamLength::from_mm;
    egui::ComboBox::from_id_salt(("stock_mode_cb", setup_idx))
        .width(INPUT_COL_WIDTH + 44.0)
        .truncate()
        .selected_text(stock.kind_label())
        .show_ui(ui, |ui| {
            let is_rbox = matches!(stock, StockConfig::RelativeBox { .. });
            if ui
                .selectable_label(is_rbox, "Relative Box Stock")
                .on_hover_text("Size rectangular stock automatically from the model bounding box plus margins")
                .clicked()
                && !is_rbox
            {
                *stock = StockConfig::default();
                *mutated_label = Some("Change Stock Mode to Relative Box");
            }
            let is_fbox = matches!(stock, StockConfig::FixedBox { .. });
            if ui
                .selectable_label(is_fbox, "Fixed-Size Box Stock")
                .on_hover_text("Specify exact rectangular raw stock width, depth, and height")
                .clicked()
                && !is_fbox
            {
                *stock = StockConfig::FixedBox {
                    width_x: mm(100.0),
                    depth_y: mm(80.0),
                    height_z: mm(25.0),
                    xy_align: FixedBoxXyAlign::CenterXy,
                    z_align: FixedBoxZAlign::default(),
                };
                *mutated_label = Some("Change Stock Mode to Fixed Box");
            }
            let is_rcyl = matches!(stock, StockConfig::RelativeCylinder { .. });
            if ui
                .selectable_label(is_rcyl, "Relative Cylindrical Stock")
                .on_hover_text("Size round bar stock automatically from the model diameter/length plus margins")
                .clicked()
                && !is_rcyl
            {
                *stock = StockConfig::RelativeCylinder {
                    axis: StockAxis::Z,
                    radial_offset: mm(3.0),
                    top_offset: mm(1.0),
                    bottom_offset: mm(5.0),
                };
                *mutated_label = Some("Change Stock Mode to Relative Cylinder");
            }
            let is_fcyl = matches!(stock, StockConfig::FixedCylinder { .. });
            if ui
                .selectable_label(is_fcyl, "Fixed Cylindrical Stock")
                .on_hover_text("Specify exact round bar stock diameter and length")
                .clicked()
                && !is_fcyl
            {
                *stock = StockConfig::FixedCylinder {
                    axis: StockAxis::Z,
                    diameter: mm(130.0),
                    length: mm(30.0),
                    axial_top_offset: mm(1.0),
                };
                *mutated_label = Some("Change Stock Mode to Fixed Cylinder");
            }
        })
        .response
        .on_hover_text("Choose raw stock blank shape and sizing mode (relative or fixed box/cylinder)");
}

fn render_cyl_axis_row(
    ui: &mut egui::Ui,
    id_salt: impl Hash + Debug,
    axis: &mut StockAxis,
    mutated_label: &mut Option<&'static str>,
) {
    grid_row(
        ui,
        "Bar Axis",
        Some("Principal WCS axis of round bar stock"),
        |ui| {
            egui::ComboBox::from_id_salt(id_salt)
                .width(INPUT_COL_WIDTH + 44.0)
                .selected_text(axis.label())
                .show_ui(ui, |ui| {
                    for a in StockAxis::ALL {
                        if ui
                            .selectable_value(axis, a, a.label())
                            .on_hover_text(format!(
                                "Orient cylindrical bar stock along {}",
                                a.label()
                            ))
                            .changed()
                        {
                            *mutated_label = Some("Change Cylinder Stock Axis");
                        }
                    }
                })
                .response
                .on_hover_text("Select principal WCS axis along which the round bar stock runs");
        },
    );
}

fn render_stock_offsets_body(
    ui: &mut egui::Ui,
    setup: &mut Setup,
    view: StockTabView<'_>,
    mutated_label: &mut Option<&'static str>,
) {
    let setup_idx = view.setup_idx;
    inspector_grid(ui, ("stock_offsets_grid", setup_idx), |ui| {
        let mut g = SetupGrid {
            ui,
            setup_idx,
            eval_ctx: view.eval_ctx,
            mutated: mutated_label,
        };
        match &mut setup.stock {
            StockConfig::RelativeBox {
                side_pos_x,
                side_neg_x,
                side_pos_y,
                side_neg_y,
                symmetric_xy,
                top_offset_z,
                bottom_offset_z,
                ..
            } => {
                if grid_row(
                    g.ui,
                    "Symmetric XY",
                    Some("Apply identical side stock on ±X and ±Y"),
                    |ui| {
                        ui.checkbox(symmetric_xy, "Mirror ±X / ±Y")
                            .on_hover_text("Keep +X/-X and +Y/-Y stock side allowances symmetric")
                            .changed()
                    },
                ) {
                    if *symmetric_xy {
                        *side_neg_x = side_pos_x.clone();
                        *side_neg_y = side_pos_y.clone();
                    }
                    *g.mutated = Some("Toggle Symmetric Stock Margins");
                }
                let px_lbl = if *symmetric_xy {
                    "Side X Margin (±X)"
                } else {
                    "Side +X Margin"
                };
                if g.len(px_lbl, "stk_px", side_pos_x, "Edit Stock Side Margin") && *symmetric_xy {
                    *side_neg_x = side_pos_x.clone();
                }
                if !*symmetric_xy {
                    g.len(
                        "Side -X Margin",
                        "stk_nx",
                        side_neg_x,
                        "Edit Stock -X Margin",
                    );
                }
                let py_lbl = if *symmetric_xy {
                    "Side Y Margin (±Y)"
                } else {
                    "Side +Y Margin"
                };
                if g.len(py_lbl, "stk_py", side_pos_y, "Edit Stock Y Margin") && *symmetric_xy {
                    *side_neg_y = side_pos_y.clone();
                }
                if !*symmetric_xy {
                    g.len(
                        "Side -Y Margin",
                        "stk_ny",
                        side_neg_y,
                        "Edit Stock -Y Margin",
                    );
                }
                g.len(
                    "Top Allowance (+Z)",
                    "stk_tz",
                    top_offset_z,
                    "Edit Stock Top Offset",
                );
                g.len(
                    "Bottom Holding (-Z)",
                    "stk_bz",
                    bottom_offset_z,
                    "Edit Stock Bottom Offset",
                );
            }
            StockConfig::FixedBox {
                xy_align, z_align, ..
            } => {
                match xy_align {
                    FixedBoxXyAlign::CenterXy => {
                        grid_row(
                            g.ui,
                            "XY Alignment",
                            Some("Part XY placement inside fixed stock"),
                            |ui| {
                                ui.label("Centered in XY");
                            },
                        );
                    }
                    FixedBoxXyAlign::OffsetFromFrontLeft { dx, dy } => {
                        g.len("Front-Left ΔX", "fbox_dx", dx, "Edit Fixed Stock XY Offset");
                        g.len("Front-Left ΔY", "fbox_dy", dy, "Edit Fixed Stock XY Offset");
                    }
                }
                match z_align {
                    FixedBoxZAlign::OffsetFromTop { top_allowance } => {
                        g.len(
                            "Top Allowance (+Z)",
                            "fbox_ta",
                            top_allowance,
                            "Edit Fixed Stock Top Allowance",
                        );
                    }
                    FixedBoxZAlign::OffsetFromBottom { bottom_allowance } => {
                        g.len(
                            "Bottom Allowance (-Z)",
                            "fbox_ba",
                            bottom_allowance,
                            "Edit Fixed Stock Bottom Allowance",
                        );
                    }
                    FixedBoxZAlign::CenterZ => {
                        grid_row(g.ui, "Z Alignment", None, |ui| {
                            ui.label("Centered in Z");
                        });
                    }
                }
            }
            StockConfig::RelativeCylinder {
                radial_offset,
                top_offset,
                bottom_offset,
                ..
            } => {
                g.len(
                    "Radial Margin",
                    "rcyl_r",
                    radial_offset,
                    "Edit Cylinder Radial Offset",
                );
                g.len(
                    "Top Margin (+Z)",
                    "rcyl_t",
                    top_offset,
                    "Edit Cylinder Top Offset",
                );
                g.len(
                    "Bottom Chuck (-Z)",
                    "rcyl_b",
                    bottom_offset,
                    "Edit Cylinder Bottom Offset",
                );
            }
            StockConfig::FixedCylinder {
                axial_top_offset, ..
            } => {
                g.len(
                    "Top Facing Offset",
                    "fcyl_to",
                    axial_top_offset,
                    "Edit Cylinder Top Offset",
                );
            }
        }
    });
}
