use chip_cam::{
    AxisAnchor, BoxAnchor27, ResolvedSetup, RotaryAxisKind, RotaryDirection, RotaryDriveMode,
    RotaryUnwindMode, Setup, StockConfig, WcsOriginMode,
};
use chip_math::{DisplayUnit, EvalContext, ParamLength};

use crate::widgets::{
    BannerKind, INPUT_COL_WIDTH, grid_row, inline_banner, inspector_grid, section_card, static_card,
};

use super::SetupGrid;

pub(super) fn show_rotary_tab(
    ui: &mut egui::Ui,
    setup: &mut Setup,
    setup_idx: usize,
    eval_ctx: &EvalContext,
    display_unit: DisplayUnit,
    resolved: Option<Result<&ResolvedSetup, &str>>,
    mutated_label: &mut Option<&'static str>,
) {
    section_card(
        ui,
        ("setup_rotary_kinematics", setup_idx),
        "4th-Axis Rotary Kinematics",
        true,
        |ui| {
            inspector_grid(ui, ("setup_rotary_grid", setup_idx), |ui| {
                grid_row(
                    ui,
                    "Rotary Mode",
                    Some(
                        "Select 3-axis (disabled), 2-linear + rotary (Z+X/Z+Y), indexed 3+1, or full simultaneous 4-axis",
                    ),
                    |ui| {
                        let prev = setup.rotary.mode;
                        egui::ComboBox::from_id_salt(("rot_mode_cb", setup_idx))
                            .width(INPUT_COL_WIDTH + 54.0)
                            .truncate()
                            .selected_text(setup.rotary.mode.label())
                            .show_ui(ui, |ui| {
                                for m in RotaryDriveMode::ALL {
                                    ui.selectable_value(&mut setup.rotary.mode, m, m.label())
                                        .on_hover_text(format!("Set 4th-axis rotary mode to {}", m.label()));
                                }
                            })
                            .response
                            .on_hover_text("Select 3-axis (disabled), 2-linear + rotary, indexed 3+1, or simultaneous 4-axis mode");
                        if setup.rotary.mode != prev {
                            *mutated_label = Some("Change Rotary Drive Mode");
                        }
                    },
                );

                if setup.rotary.is_enabled() {
                    grid_row(
                        ui,
                        "Rotary Axis",
                        Some("Principal Setup WCS axis aligned with the rotary chuck centerline"),
                        |ui| {
                            let prev = setup.rotary.axis;
                            egui::ComboBox::from_id_salt(("rot_axis_cb", setup_idx))
                                .width(INPUT_COL_WIDTH + 54.0)
                                .truncate()
                                .selected_text(setup.rotary.axis.label())
                                .show_ui(ui, |ui| {
                                    for a in RotaryAxisKind::ALL {
                                        ui.selectable_value(&mut setup.rotary.axis, a, a.label())
                                            .on_hover_text(format!("Align rotary centerline with {}", a.label()));
                                    }
                                })
                                .response
                                .on_hover_text("Choose which Setup WCS axis aligns with the 4th-axis rotary chuck");
                            if setup.rotary.axis != prev {
                                *mutated_label = Some("Change Rotary Axis");
                            }
                        },
                    );

                    grid_row(
                        ui,
                        "Direction",
                        Some("Positive rotation polarity around the rotary axis"),
                        |ui| {
                            let prev = setup.rotary.direction;
                            egui::ComboBox::from_id_salt(("rot_dir_cb", setup_idx))
                                .width(INPUT_COL_WIDTH + 54.0)
                                .truncate()
                                .selected_text(setup.rotary.direction.label())
                                .show_ui(ui, |ui| {
                                    for d in RotaryDirection::ALL {
                                        ui.selectable_value(
                                            &mut setup.rotary.direction,
                                            d,
                                            d.label(),
                                        )
                                        .on_hover_text(
                                            format!("Use {} rotation polarity", d.label()),
                                        );
                                    }
                                })
                                .response
                                .on_hover_text(
                                    "Choose positive rotation polarity (right-hand or inverted)",
                                );
                            if setup.rotary.direction != prev {
                                *mutated_label = Some("Change Rotary Polarity");
                            }
                        },
                    );

                    grid_row(
                        ui,
                        "G-Code Letter",
                        Some("Rotary axis address word emitted by 4-axis post-processors"),
                        |ui| {
                            let cur_letter = setup.rotary.effective_axis_letter();
                            let prev = setup.rotary.axis_letter_override;
                            egui::ComboBox::from_id_salt(("rot_letter_cb", setup_idx))
                                .width(92.0)
                                .selected_text(format!("{cur_letter}-Axis"))
                                .show_ui(ui, |ui| {
                                    let def = setup.rotary.axis.default_letter();
                                    ui.selectable_value(
                                        &mut setup.rotary.axis_letter_override,
                                        None,
                                        format!("Auto ({def})"),
                                    )
                                    .on_hover_text("Use the standard ISO axis letter for the chosen rotary axis");
                                    for c in ['A', 'B', 'C'] {
                                        ui.selectable_value(
                                            &mut setup.rotary.axis_letter_override,
                                            Some(c),
                                            format!("{c}-Axis"),
                                        )
                                        .on_hover_text(format!("Always emit {c} rotary coordinates in G-code"));
                                    }
                                })
                                .response
                                .on_hover_text("Select rotary axis word (A, B, or C) emitted in posted G-code");
                            if setup.rotary.axis_letter_override != prev {
                                *mutated_label = Some("Change Rotary Axis Letter");
                            }
                        },
                    );

                    grid_row(
                        ui,
                        "Unwind Mode",
                        Some(
                            "Angular rollover policy for multi-turn spirals vs. shortest-path indexing",
                        ),
                        |ui| {
                            let prev = setup.rotary.unwind_mode;
                            egui::ComboBox::from_id_salt(("rot_unwind_cb", setup_idx))
                                .width(INPUT_COL_WIDTH + 54.0)
                                .truncate()
                                .selected_text(setup.rotary.unwind_mode.label())
                                .show_ui(ui, |ui| {
                                    for u in RotaryUnwindMode::ALL {
                                        ui.selectable_value(
                                            &mut setup.rotary.unwind_mode,
                                            u,
                                            u.label(),
                                        )
                                        .on_hover_text(format!("Use {} angular rollover policy", u.label()));
                                    }
                                })
                                .response
                                .on_hover_text("Choose continuous multi-turn angles vs. 0..360 shortest-path indexing");
                            if setup.rotary.unwind_mode != prev {
                                *mutated_label = Some("Change Rotary Unwind Mode");
                            }
                        },
                    );
                }
            });
        },
    );

    if !setup.rotary.is_enabled() {
        ui.add_space(4.0);
        inline_banner(
            ui,
            BannerKind::Info,
            "Enable 4th-Axis Rotary above to configure A/B/C centerline kinematics, chuck clearance, polar stock simulation, and 4-axis G-code output.",
        );
        return;
    }

    ui.add_space(5.0);
    let (off0_lbl, off1_lbl) = setup.rotary.axis.offset_axis_labels();
    section_card(
        ui,
        ("setup_rotary_chuck_card", setup_idx),
        "Centerline, Clearance & Chuck",
        true,
        |ui| {
            inspector_grid(ui, ("setup_rotary_chuck_grid", setup_idx), |ui| {
                let mut sg = SetupGrid {
                    ui,
                    setup_idx,
                    eval_ctx,
                    mutated: mutated_label,
                };
                sg.len(
                    off0_lbl,
                    "rot_off0",
                    &mut setup.rotary.center_offset_wcs[0],
                    "Edit Rotary Center Offset",
                );
                sg.len(
                    off1_lbl,
                    "rot_off1",
                    &mut setup.rotary.center_offset_wcs[1],
                    "Edit Rotary Center Offset",
                );
                sg.len(
                    "Clearance Margin",
                    "rot_clr_off",
                    &mut setup.rotary.clearance_radial_offset,
                    "Edit Rotary Clearance Offset",
                );
                sg.len(
                    "Chuck Diameter",
                    "rot_chuck_d",
                    &mut setup.rotary.chuck_diameter,
                    "Edit Rotary Chuck Diameter",
                );
                sg.len(
                    "Chuck Length",
                    "rot_chuck_l",
                    &mut setup.rotary.chuck_length,
                    "Edit Rotary Chuck Length",
                );
                grid_row(
                    sg.ui,
                    "Chuck Side",
                    Some("Place rotary chuck at the negative or positive end of the stock axis"),
                    |ui| {
                        let prev = setup.rotary.chuck_end_positive;
                        ui.horizontal(|ui| {
                            ui.selectable_value(
                                &mut setup.rotary.chuck_end_positive,
                                false,
                                "-End",
                            )
                            .on_hover_text(
                                "Place the rotary chuck at the negative end of the bar stock",
                            );
                            ui.selectable_value(&mut setup.rotary.chuck_end_positive, true, "+End")
                                .on_hover_text(
                                    "Place the rotary chuck at the positive end of the bar stock",
                                );
                        });
                        if setup.rotary.chuck_end_positive != prev {
                            *sg.mutated = Some("Flip Rotary Chuck End");
                        }
                    },
                );
            });

            ui.add_space(4.0);
            if ui
                .button("Align Cylindrical Stock to Rotary Axis")
                .on_hover_text(
                    "Switch Setup stock to a Relative Cylinder concentric with the configured rotary axis and WCS center",
                )
                .clicked()
            {
                setup.stock = StockConfig::RelativeCylinder {
                    axis: setup.rotary.axis.to_stock_axis(),
                    radial_offset: ParamLength::from_mm(2.0),
                    top_offset: ParamLength::from_mm(2.0),
                    bottom_offset: ParamLength::from_mm(2.0),
                };
                setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
                    anchor: BoxAnchor27 {
                        x: AxisAnchor::Center,
                        y: AxisAnchor::Center,
                        z: AxisAnchor::Center,
                    },
                };
                *mutated_label = Some("Align Cylindrical Stock to Rotary Axis");
            }
        },
    );

    if let Some(Ok(rs)) = resolved
        && let Some(rot) = rs.rotary
    {
        ui.add_space(5.0);
        static_card(ui, Some("Resolved Rotary Envelope"), |ui| {
            ui.monospace(format!(
                "Axis:      {}-Axis ({})",
                rot.axis_letter,
                rot.axis.label()
            ));
            ui.monospace(format!(
                "Stock R:   {}   Clearance R: {}",
                display_unit.format_length(rot.stock_outer_radius_mm),
                display_unit.format_length(rot.clearance_radius_mm)
            ));
            ui.monospace(format!(
                "Axial U:   {} .. {}",
                display_unit.format_length(rot.stock_u_min),
                display_unit.format_length(rot.stock_u_max)
            ));
            ui.monospace(format!(
                "Chuck:     {} .. {} (Ø{})",
                display_unit.format_length(rot.chuck_u_min),
                display_unit.format_length(rot.chuck_u_max),
                display_unit.format_length(2.0 * rot.chuck_radius_mm)
            ));
        });
    }
}
