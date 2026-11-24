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

    let is_rotary_enabled = !setup.rotary.is_disabled();
    inspector_grid(ui, ("stock_geom_grid", setup_idx), |ui| {
        grid_row(
            ui,
            "Stock Mode",
            Some("Raw stock blank shape and sizing mode"),
            |ui| {
                render_stock_mode_combo(
                    ui,
                    setup_idx,
                    is_rotary_enabled,
                    &mut setup.stock,
                    mutated_label,
                );
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
                grid_row(
                    g.ui,
                    "Round Saw Blank",
                    Some(
                        "Round computed stock blank dimensions up to a standard saw-cut increment",
                    ),
                    |ui| {
                        ui.horizontal(|ui| {
                            if ui
                                .checkbox(&mut round_on, "Round up")
                                .on_hover_text(
                                    "Round computed stock blank dimensions up to the nearest saw-cut step",
                                )
                                .changed()
                            {
                                *round_increment_mm = round_on.then_some(5.0);
                                *g.mutated = Some("Toggle Stock Rounding");
                            }
                            if let Some(inc_mm) = round_increment_mm {
                                let mut disp_val = display_unit.from_mm(*inc_mm);
                                let min_disp = display_unit.from_mm(0.1);
                                let max_disp = display_unit.from_mm(250.0);
                                let speed = if display_unit == DisplayUnit::Inch {
                                    0.05
                                } else {
                                    0.5
                                };
                                let max_dec = if display_unit == DisplayUnit::Inch { 4 } else { 2 };
                                let resp = ui
                                    .add(
                                        egui::DragValue::new(&mut disp_val)
                                            .speed(speed)
                                            .range(min_disp..=max_disp)
                                            .max_decimals(max_dec)
                                            .suffix(format!(" {}", display_unit.suffix())),
                                    )
                                    .on_hover_text(
                                        "Stock blank rounding increment (e.g. 5 mm, 10 mm, 0.25 in / 6.35 mm, 0.5 in / 12.7 mm)",
                                    );
                                if resp.changed() {
                                    let new_mm = display_unit.to_mm(disp_val).clamp(0.1, 250.0);
                                    if new_mm.is_finite() {
                                        *inc_mm = new_mm;
                                        *g.mutated = Some("Edit Stock Rounding Increment");
                                    }
                                }
                            }
                        });
                    },
                );
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
    is_rotary_enabled: bool,
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
                let default_z_align = if is_rotary_enabled {
                    FixedBoxZAlign::CenterZ
                } else {
                    FixedBoxZAlign::default()
                };
                *stock = StockConfig::FixedBox {
                    width_x: mm(100.0),
                    depth_y: mm(80.0),
                    height_z: mm(25.0),
                    xy_align: FixedBoxXyAlign::CenterXy,
                    z_align: default_z_align,
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
                let mm = ParamLength::from_mm;
                let xy_label = match xy_align {
                    FixedBoxXyAlign::CenterXy => "Centered in XY",
                    FixedBoxXyAlign::OffsetFromFrontLeft { .. } => "Offset from Front-Left",
                };
                let mut xy_changed = false;
                grid_row(
                    g.ui,
                    "XY Alignment",
                    Some("Control how the model is placed in XY inside the fixed-size box stock"),
                    |ui| {
                        egui::ComboBox::from_id_salt(("fbox_xy_align_cb", setup_idx))
                            .width(INPUT_COL_WIDTH + 44.0)
                            .truncate()
                            .selected_text(xy_label)
                            .show_ui(ui, |ui| {
                                let is_center = matches!(xy_align, FixedBoxXyAlign::CenterXy);
                                if ui
                                    .selectable_label(is_center, "Centered in XY")
                                    .on_hover_text(
                                        "Center the model symmetrically along both X and Y inside the fixed stock blank",
                                    )
                                    .clicked()
                                    && !is_center
                                {
                                    *xy_align = FixedBoxXyAlign::CenterXy;
                                    xy_changed = true;
                                }
                                let is_fl =
                                    matches!(xy_align, FixedBoxXyAlign::OffsetFromFrontLeft { .. });
                                if ui
                                    .selectable_label(is_fl, "Offset from Front-Left")
                                    .on_hover_text(
                                        "Position the model at explicit ΔX / ΔY offsets from the stock front-left (-X, -Y) corner",
                                    )
                                    .clicked()
                                    && !is_fl
                                {
                                    *xy_align = FixedBoxXyAlign::OffsetFromFrontLeft {
                                        dx: mm(5.0),
                                        dy: mm(5.0),
                                    };
                                    xy_changed = true;
                                }
                            })
                            .response
                            .on_hover_text("Select XY alignment of model inside fixed-size box stock");
                    },
                );
                if xy_changed {
                    *g.mutated = Some("Change Fixed Stock XY Alignment");
                }
                if let FixedBoxXyAlign::OffsetFromFrontLeft { dx, dy } = xy_align {
                    g.len("Front-Left ΔX", "fbox_dx", dx, "Edit Fixed Stock XY Offset");
                    g.len("Front-Left ΔY", "fbox_dy", dy, "Edit Fixed Stock XY Offset");
                }

                let z_label = match z_align {
                    FixedBoxZAlign::OffsetFromTop { .. } => "Offset from Top (+Z)",
                    FixedBoxZAlign::OffsetFromBottom { .. } => "Offset from Bottom (-Z)",
                    FixedBoxZAlign::CenterZ => "Centered in Z",
                };
                let mut z_changed = false;
                grid_row(
                    g.ui,
                    "Z Alignment",
                    Some(
                        "Control how the model is placed vertically in Z inside the fixed-size box stock (use Centered in Z for rectangular rotary bar stock)",
                    ),
                    |ui| {
                        egui::ComboBox::from_id_salt(("fbox_z_align_cb", setup_idx))
                            .width(INPUT_COL_WIDTH + 44.0)
                            .truncate()
                            .selected_text(z_label)
                            .show_ui(ui, |ui| {
                                let is_top =
                                    matches!(z_align, FixedBoxZAlign::OffsetFromTop { .. });
                                if ui
                                    .selectable_label(is_top, "Offset from Top (+Z)")
                                    .on_hover_text(
                                        "Place stock top above model top by the specified Top Allowance",
                                    )
                                    .clicked()
                                    && !is_top
                                {
                                    *z_align = FixedBoxZAlign::OffsetFromTop {
                                        top_allowance: mm(1.0),
                                    };
                                    z_changed = true;
                                }
                                let is_bot =
                                    matches!(z_align, FixedBoxZAlign::OffsetFromBottom { .. });
                                if ui
                                    .selectable_label(is_bot, "Offset from Bottom (-Z)")
                                    .on_hover_text(
                                        "Place stock bottom below model bottom by the specified Bottom Allowance",
                                    )
                                    .clicked()
                                    && !is_bot
                                {
                                    *z_align = FixedBoxZAlign::OffsetFromBottom {
                                        bottom_allowance: mm(1.0),
                                    };
                                    z_changed = true;
                                }
                                let is_center = matches!(z_align, FixedBoxZAlign::CenterZ);
                                if ui
                                    .selectable_label(is_center, "Centered in Z")
                                    .on_hover_text(
                                        "Center the model symmetrically along Z inside the fixed box stock (recommended for 4th-axis rectangular bar stock)",
                                    )
                                    .clicked()
                                    && !is_center
                                {
                                    *z_align = FixedBoxZAlign::CenterZ;
                                    z_changed = true;
                                }
                            })
                            .response
                            .on_hover_text("Select Z alignment of model inside fixed-size box stock");
                    },
                );
                if z_changed {
                    *g.mutated = Some("Change Fixed Stock Z Alignment");
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
                    FixedBoxZAlign::CenterZ => {}
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

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cam::{
        FixedBoxXyAlign, FixedBoxZAlign, Operation, OperationKind, OperationStatus,
        RotaryDriveMode, StockConfig,
    };
    use chip_math::ParamLength;
    use chip_project::{ProjectDocument, UndoHistory};

    use crate::panels::{
        SetupTab, ViewportPickMode,
        setup_inspector::{FixtureSafetySummary, SetupInspectorCtx, show_setup_inspector},
    };

    fn collect_texts(shapes: &[egui::epaint::ClippedShape], out: &mut Vec<String>) {
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

    fn find_text_center(shape: &egui::Shape, needle: &str) -> Option<egui::Pos2> {
        match shape {
            egui::Shape::Text(t) if t.galley.text() == needle => {
                Some(t.pos + t.galley.size() * 0.5)
            }
            egui::Shape::Vec(v) => v.iter().find_map(|s| find_text_center(s, needle)),
            _ => None,
        }
    }

    fn click_at(pt: egui::Pos2) -> Vec<egui::Event> {
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

    fn run_stock_tab_frame(
        ctx: &egui::Context,
        project: &mut ProjectDocument,
        history: &mut UndoHistory,
        events: Vec<egui::Event>,
    ) -> Vec<egui::epaint::ClippedShape> {
        let mut tab = SetupTab::Stock;
        let mut pick_mode = ViewportPickMode::InspectEntity;
        let mut sel_fix = None;
        let mut show_fix = true;
        let mut show_27 = false;
        let mut show_z = false;
        let mut open_mm = false;
        let mut m_changed = false;
        let mut sw_mid = None;
        let mut imp_fix = false;
        let mut open_fj = false;
        let mut open_sj = false;
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
                    let _ = show_setup_inspector(
                        ui,
                        SetupInspectorCtx {
                            project,
                            setup_idx: 0,
                            history,
                            project_dir: None,
                            cad_doc: None,
                            resolved: None,
                            resolved_fixtures: &[],
                            fixture_safety: FixtureSafetySummary::default(),
                            tab: &mut tab,
                            pick_mode: &mut pick_mode,
                            selected_fixture_idx: &mut sel_fix,
                            show_fixtures: &mut show_fix,
                            show_27pt_dots: &mut show_27,
                            show_z_planes: &mut show_z,
                            machines: &[],
                            open_machine_manager: &mut open_mm,
                            machine_changed: &mut m_changed,
                            switch_machine_id: &mut sw_mid,
                            import_fixture_requested: &mut imp_fix,
                            open_flip_jig_wizard: &mut open_fj,
                            open_soft_jaw_wizard: &mut open_sj,
                        },
                    );
                });
            },
        );
        out.textures_delta.clear();
        out.shapes
    }

    #[test]
    fn m86_fixed_box_xy_z_alignment_controls_and_rotary_center_z_default() {
        // 1. Standard 3-axis Setup: switching Stock Mode to Fixed-Size Box Stock defaults z_align to OffsetFromTop
        let ctx3 = egui::Context::default();
        let mut proj3 = ProjectDocument::new("fixed_box_3axis");
        let mut op = Operation::new("op1", "Face", OperationKind::Facing);
        op.runtime_status = OperationStatus::Clean;
        proj3.setups[0].operations = vec![op];
        let mut hist3 = UndoHistory::default();

        let _ = run_stock_tab_frame(&ctx3, &mut proj3, &mut hist3, Vec::new());
        let s1 = run_stock_tab_frame(&ctx3, &mut proj3, &mut hist3, Vec::new());
        let mode_pt = s1
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Relative Box Stock"))
            .expect("Stock Mode combo must show Relative Box Stock");
        let _ = run_stock_tab_frame(&ctx3, &mut proj3, &mut hist3, click_at(mode_pt));
        let s_open = run_stock_tab_frame(&ctx3, &mut proj3, &mut hist3, Vec::new());
        let fbox_pt = s_open
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Fixed-Size Box Stock"))
            .expect("Fixed-Size Box Stock option must be painted");
        let _ = run_stock_tab_frame(&ctx3, &mut proj3, &mut hist3, click_at(fbox_pt));
        assert!(matches!(
            proj3.setups[0].stock,
            StockConfig::FixedBox {
                xy_align: FixedBoxXyAlign::CenterXy,
                z_align: FixedBoxZAlign::OffsetFromTop { .. },
                ..
            }
        ));
        assert_eq!(
            proj3.setups[0].operations[0].runtime_status,
            OperationStatus::Dirty
        );
        assert!(hist3.can_undo());

        // 2. Switch XY Alignment from Centered in XY -> Offset from Front-Left and verify Front-Left ΔX/ΔY render
        let _ = run_stock_tab_frame(&ctx3, &mut proj3, &mut hist3, Vec::new());
        let s_fb = run_stock_tab_frame(&ctx3, &mut proj3, &mut hist3, Vec::new());
        let xy_pt = s_fb
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Centered in XY"))
            .expect("XY Alignment combo must render Centered in XY");
        let _ = run_stock_tab_frame(&ctx3, &mut proj3, &mut hist3, click_at(xy_pt));
        let s_xy_open = run_stock_tab_frame(&ctx3, &mut proj3, &mut hist3, Vec::new());
        let fl_pt = s_xy_open
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Offset from Front-Left"))
            .expect("Offset from Front-Left option must be painted");
        let _ = run_stock_tab_frame(&ctx3, &mut proj3, &mut hist3, click_at(fl_pt));
        assert!(matches!(
            proj3.setups[0].stock,
            StockConfig::FixedBox {
                xy_align: FixedBoxXyAlign::OffsetFromFrontLeft { .. },
                ..
            }
        ));
        let _ = run_stock_tab_frame(&ctx3, &mut proj3, &mut hist3, Vec::new());
        let s_after_xy = run_stock_tab_frame(&ctx3, &mut proj3, &mut hist3, Vec::new());
        let mut t_xy = Vec::new();
        collect_texts(&s_after_xy, &mut t_xy);
        let j_xy = t_xy.join("\n");
        assert!(
            j_xy.contains("Front-Left ΔX") && j_xy.contains("Front-Left ΔY"),
            "OffsetFromFrontLeft must display Front-Left ΔX / ΔY inputs; got:\n{j_xy}"
        );

        // 3. Switch Z Alignment from Offset from Top (+Z) -> Offset from Bottom (-Z) -> Centered in Z
        let z_pt = s_after_xy
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Offset from Top (+Z)"))
            .expect("Z Alignment combo must render Offset from Top (+Z)");
        let _ = run_stock_tab_frame(&ctx3, &mut proj3, &mut hist3, click_at(z_pt));
        let s_z_open = run_stock_tab_frame(&ctx3, &mut proj3, &mut hist3, Vec::new());
        let bot_pt = s_z_open
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Offset from Bottom (-Z)"))
            .expect("Offset from Bottom (-Z) option must be painted");
        let _ = run_stock_tab_frame(&ctx3, &mut proj3, &mut hist3, click_at(bot_pt));
        assert!(matches!(
            proj3.setups[0].stock,
            StockConfig::FixedBox {
                z_align: FixedBoxZAlign::OffsetFromBottom { .. },
                ..
            }
        ));
        let _ = run_stock_tab_frame(&ctx3, &mut proj3, &mut hist3, Vec::new());
        let s_bot = run_stock_tab_frame(&ctx3, &mut proj3, &mut hist3, Vec::new());
        let mut t_bot = Vec::new();
        collect_texts(&s_bot, &mut t_bot);
        assert!(
            t_bot.join("\n").contains("Bottom Allowance (-Z)"),
            "OffsetFromBottom must display Bottom Allowance (-Z)"
        );

        let z_pt2 = s_bot
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Offset from Bottom (-Z)"))
            .expect("Z Alignment combo must render Offset from Bottom (-Z)");
        let _ = run_stock_tab_frame(&ctx3, &mut proj3, &mut hist3, click_at(z_pt2));
        let s_z_open2 = run_stock_tab_frame(&ctx3, &mut proj3, &mut hist3, Vec::new());
        let cz_pt = s_z_open2
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Centered in Z"))
            .expect("Centered in Z option must be painted");
        let _ = run_stock_tab_frame(&ctx3, &mut proj3, &mut hist3, click_at(cz_pt));
        assert!(matches!(
            proj3.setups[0].stock,
            StockConfig::FixedBox {
                z_align: FixedBoxZAlign::CenterZ,
                ..
            }
        ));

        // Undo reverts Z Alignment back to OffsetFromBottom
        assert!(hist3.undo(&mut proj3).is_some());
        assert!(matches!(
            proj3.setups[0].stock,
            StockConfig::FixedBox {
                z_align: FixedBoxZAlign::OffsetFromBottom { .. },
                ..
            }
        ));

        // 4. Rotary-enabled Setup: switching Stock Mode to Fixed-Size Box Stock auto-defaults z_align to CenterZ!
        let ctx_rot = egui::Context::default();
        let mut proj_rot = ProjectDocument::new("fixed_box_rotary");
        proj_rot.setups[0].rotary.mode = RotaryDriveMode::Indexed3Plus1;
        proj_rot.setups[0].stock = StockConfig::RelativeBox {
            side_pos_x: ParamLength::from_mm(2.0),
            side_neg_x: ParamLength::from_mm(2.0),
            side_pos_y: ParamLength::from_mm(2.0),
            side_neg_y: ParamLength::from_mm(2.0),
            symmetric_xy: true,
            top_offset_z: ParamLength::from_mm(1.0),
            bottom_offset_z: ParamLength::from_mm(1.0),
            sheet_thickness_z: ParamLength::from_mm(12.0),
            round_increment_mm: None,
        };
        let mut hist_rot = UndoHistory::default();
        let _ = run_stock_tab_frame(&ctx_rot, &mut proj_rot, &mut hist_rot, Vec::new());
        let sr1 = run_stock_tab_frame(&ctx_rot, &mut proj_rot, &mut hist_rot, Vec::new());
        let rmode_pt = sr1
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Relative Box Stock"))
            .expect("Rotary Stock Mode combo must show Relative Box Stock");
        let _ = run_stock_tab_frame(&ctx_rot, &mut proj_rot, &mut hist_rot, click_at(rmode_pt));
        let sr_open = run_stock_tab_frame(&ctx_rot, &mut proj_rot, &mut hist_rot, Vec::new());
        let rfbox_pt = sr_open
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Fixed-Size Box Stock"))
            .expect("Fixed-Size Box Stock option must be painted");
        let _ = run_stock_tab_frame(&ctx_rot, &mut proj_rot, &mut hist_rot, click_at(rfbox_pt));
        assert!(
            matches!(
                proj_rot.setups[0].stock,
                StockConfig::FixedBox {
                    z_align: FixedBoxZAlign::CenterZ,
                    ..
                }
            ),
            "rotary-enabled Setup switching to FixedBox must auto-default z_align to CenterZ; got {:?}",
            proj_rot.setups[0].stock
        );
    }

    #[test]
    fn m88_stock_rounding_increment_preserves_custom_and_converts_units() {
        let ctx = egui::Context::default();
        let mut proj = ProjectDocument::new("round_inc_test");
        proj.display_units = chip_math::DisplayUnit::Inch;
        if let StockConfig::RelativeBox {
            round_increment_mm, ..
        } = &mut proj.setups[0].stock
        {
            *round_increment_mm = Some(6.35);
        }
        let mut hist = UndoHistory::default();

        // 1. Rendering in Inch mode preserves custom 6.35 mm (0.25 in) without overwriting to 5.0 mm
        let _ = run_stock_tab_frame(&ctx, &mut proj, &mut hist, Vec::new());
        let s1 = run_stock_tab_frame(&ctx, &mut proj, &mut hist, Vec::new());
        if let StockConfig::RelativeBox {
            round_increment_mm, ..
        } = &proj.setups[0].stock
        {
            assert_eq!(*round_increment_mm, Some(6.35));
        } else {
            panic!("expected RelativeBox");
        }
        let mut texts = Vec::new();
        collect_texts(&s1, &mut texts);
        let joined = texts.join("\n");
        assert!(
            texts.iter().any(|t| t == "0.25") && texts.iter().any(|t| t.trim() == "in"),
            "Inch mode must display 6.35 mm as 0.25 in; got:\n{joined}"
        );

        // 2. Edit the DragValue in Inch mode: click to enter edit mode, then type "0.5" + Enter -> 12.7 mm
        let dv_pt = s1
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "0.25"))
            .expect("Rounding increment DragValue must display '0.25'");
        let _ = run_stock_tab_frame(&ctx, &mut proj, &mut hist, click_at(dv_pt));
        let edit_events = vec![
            egui::Event::Key {
                key: egui::Key::A,
                physical_key: None,
                pressed: true,
                repeat: false,
                modifiers: egui::Modifiers::COMMAND,
            },
            egui::Event::Text("0.5".to_string()),
            egui::Event::Key {
                key: egui::Key::Enter,
                physical_key: None,
                pressed: true,
                repeat: false,
                modifiers: egui::Modifiers::NONE,
            },
        ];
        let _ = run_stock_tab_frame(&ctx, &mut proj, &mut hist, edit_events);
        let _ = run_stock_tab_frame(&ctx, &mut proj, &mut hist, Vec::new());
        if let StockConfig::RelativeBox {
            round_increment_mm, ..
        } = &proj.setups[0].stock
        {
            let v = round_increment_mm.expect("round increment must remain Some");
            assert!(
                (v - 12.7).abs() < 1e-6,
                "0.5 in must convert to 12.7 mm; got {v}"
            );
        } else {
            panic!("expected RelativeBox");
        }

        // 3. Uncheck "Round up" -> None, re-check -> defaults to 5.0 mm, Undo reverts cleanly
        let s2 = run_stock_tab_frame(&ctx, &mut proj, &mut hist, Vec::new());
        let cb_pt = s2
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Round up"))
            .expect("Round Saw Blank row must render 'Round up' checkbox");
        let _ = run_stock_tab_frame(&ctx, &mut proj, &mut hist, click_at(cb_pt));
        assert!(matches!(
            proj.setups[0].stock,
            StockConfig::RelativeBox {
                round_increment_mm: None,
                ..
            }
        ));
        let _ = run_stock_tab_frame(&ctx, &mut proj, &mut hist, click_at(cb_pt));
        assert!(matches!(
            proj.setups[0].stock,
            StockConfig::RelativeBox {
                round_increment_mm: Some(v),
                ..
            } if (v - 5.0).abs() < 1e-6
        ));
        assert!(hist.undo(&mut proj).is_some());
        assert!(matches!(
            proj.setups[0].stock,
            StockConfig::RelativeBox {
                round_increment_mm: None,
                ..
            }
        ));
    }
}
