use chip_cam::{EntryStrategy, Operation, RetractPolicy};
use chip_math::EvalContext;

use crate::widgets::{INPUT_COL_WIDTH, edit_param_length, grid_row, inspector_grid, section_card};

const COMBO_COL_WIDTH: f32 = INPUT_COL_WIDTH + 54.0;

pub fn show_linking_tab(
    ui: &mut egui::Ui,
    op: &mut Operation,
    setup_idx: usize,
    op_idx: usize,
    eval_ctx: &EvalContext,
    mutated_label: &mut Option<&'static str>,
) {
    section_card(
        ui,
        ("lnk_retract_card", setup_idx, op_idx),
        "Retract & Stay-Down Policy",
        true,
        |ui| {
            inspector_grid(ui, ("lnk_retract_grid", setup_idx, op_idx), |ui| {
                grid_row(
                    ui,
                    "Retract Policy",
                    Some("Control Z clearance level used when linking between cutting passes"),
                    |ui| {
                        egui::ComboBox::from_id_salt(("lnk_ret", setup_idx, op_idx))
                            .width(COMBO_COL_WIDTH)
                            .truncate()
                            .selected_text(op.linking.retract_policy.label())
                            .show_ui(ui, |ui| {
                                for r in RetractPolicy::ALL {
                                    if ui
                                        .selectable_value(
                                            &mut op.linking.retract_policy,
                                            r,
                                            r.label(),
                                        )
                                        .changed()
                                    {
                                        *mutated_label = Some("Change Retract Policy");
                                    }
                                }
                            });
                    },
                );
                grid_row(
                    ui,
                    "Max Stay-Down",
                    Some("Maximum link distance kept near the floor before performing a Z retract"),
                    |ui| {
                        if edit_param_length(
                            ui,
                            ("lnk_stay", setup_idx, op_idx),
                            &mut op.linking.safe_stay_down_distance,
                            eval_ctx,
                            INPUT_COL_WIDTH,
                        ) {
                            *mutated_label = Some("Edit Stay-Down Distance");
                        }
                    },
                );
            });
        },
    );

    section_card(
        ui,
        ("lnk_entry_card", setup_idx, op_idx),
        "Ramp & Cavity Entry",
        true,
        |ui| {
            inspector_grid(ui, ("lnk_entry_grid", setup_idx, op_idx), |ui| {
                let cur_lbl = op.linking.entry_mode.label();
                grid_row(
                    ui,
                    "Entry Mode",
                    Some(
                        "Closed-cavity plunge strategy (helical ramp, linear ramp, or direct plunge)",
                    ),
                    |ui| {
                        egui::ComboBox::from_id_salt(("lnk_ent", setup_idx, op_idx))
                            .width(COMBO_COL_WIDTH)
                            .truncate()
                            .selected_text(cur_lbl)
                            .show_ui(ui, |ui| {
                                for (lbl, mode) in [
                                    ("Helical Ramp", EntryStrategy::HelicalRamp),
                                    ("Linear Profile Ramp", EntryStrategy::LinearRamp),
                                    ("Direct Plunge", EntryStrategy::Plunge),
                                ] {
                                    if ui
                                        .selectable_label(op.linking.entry_mode == mode, lbl)
                                        .clicked()
                                    {
                                        op.linking.entry_mode = mode;
                                        *mutated_label = Some("Change Entry Strategy");
                                    }
                                }
                            });
                    },
                );
                grid_row(
                    ui,
                    "Ramp Angle",
                    Some("Descent slope angle (°) for helical and linear cavity entry ramps"),
                    |ui| {
                        if ui
                            .add(
                                egui::DragValue::new(&mut op.linking.ramp_angle_deg)
                                    .range(0.5..=30.0)
                                    .speed(0.25)
                                    .suffix("°"),
                            )
                            .changed()
                        {
                            *mutated_label = Some("Edit Ramp Angle");
                        }
                    },
                );
                grid_row(
                    ui,
                    "Min Helix Ø",
                    Some("Minimum center-path helix diameter before falling back to linear ramp"),
                    |ui| {
                        if edit_param_length(
                            ui,
                            ("lnk_min_hd", setup_idx, op_idx),
                            &mut op.linking.min_ramp_diameter,
                            eval_ctx,
                            INPUT_COL_WIDTH,
                        ) {
                            *mutated_label = Some("Edit Min Helix Diameter");
                        }
                    },
                );
            });
        },
    );

    section_card(
        ui,
        ("lnk_lead_card", setup_idx, op_idx),
        "Lead-In / Lead-Out & Seam",
        true,
        |ui| {
            inspector_grid(ui, ("lnk_lead_grid", setup_idx, op_idx), |ui| {
                grid_row(
                    ui,
                    "Tangent Lead-In",
                    Some(
                        "Smooth arc/tangent entry onto contour walls to prevent plunge witness marks",
                    ),
                    |ui| {
                        if ui
                            .checkbox(&mut op.linking.lead_in.enabled, "Enabled")
                            .changed()
                        {
                            *mutated_label = Some("Toggle Lead-In");
                        }
                    },
                );
                if op.linking.lead_in.enabled {
                    grid_row(
                        ui,
                        "Horiz. Radius",
                        Some("XY arc radius of the tangent lead-in move"),
                        |ui| {
                            if edit_param_length(
                                ui,
                                ("lnk_lhr", setup_idx, op_idx),
                                &mut op.linking.lead_in.horizontal_radius,
                                eval_ctx,
                                INPUT_COL_WIDTH,
                            ) {
                                *mutated_label = Some("Edit Lead-In Radius");
                            }
                        },
                    );
                    grid_row(
                        ui,
                        "Horiz. Sweep",
                        Some("Included XY arc sweep angle (°) of the tangent lead-in move"),
                        |ui| {
                            if ui
                                .add(
                                    egui::DragValue::new(
                                        &mut op.linking.lead_in.horizontal_sweep_deg,
                                    )
                                    .range(0.0..=180.0)
                                    .suffix("°"),
                                )
                                .changed()
                            {
                                *mutated_label = Some("Edit Lead-In Sweep");
                            }
                        },
                    );
                    grid_row(
                        ui,
                        "Linear Tangent",
                        Some(
                            "Straight tangent segment length between horizontal lead arc and wall",
                        ),
                        |ui| {
                            if edit_param_length(
                                ui,
                                ("lnk_llin", setup_idx, op_idx),
                                &mut op.linking.lead_in.linear_length,
                                eval_ctx,
                                INPUT_COL_WIDTH,
                            ) {
                                *mutated_label = Some("Edit Lead-In Linear");
                            }
                        },
                    );
                    grid_row(
                        ui,
                        "Vert. Radius",
                        Some("Vertical Z arc radius blending plunge into horizontal lead move"),
                        |ui| {
                            if edit_param_length(
                                ui,
                                ("lnk_lvr", setup_idx, op_idx),
                                &mut op.linking.lead_in.vertical_radius,
                                eval_ctx,
                                INPUT_COL_WIDTH,
                            ) {
                                *mutated_label = Some("Edit Vertical Lead Radius");
                            }
                        },
                    );
                    grid_row(
                        ui,
                        "Lead-Out",
                        Some(
                            "Mirror Lead-In arc/tangent geometry symmetrically when exiting contour wall",
                        ),
                        |ui| {
                            if ui
                                .checkbox(&mut op.linking.lead_out_same_as_in, "Mirror Lead-In")
                                .changed()
                            {
                                *mutated_label = Some("Toggle Mirror Lead-Out");
                            }
                        },
                    );
                }
                grid_row(
                    ui,
                    "Seam Overlap",
                    Some(
                        "Extra travel past closed contour start point before lead-out to remove seam cusp",
                    ),
                    |ui| {
                        if edit_param_length(
                            ui,
                            ("lnk_ovl", setup_idx, op_idx),
                            &mut op.linking.seam_overlap,
                            eval_ctx,
                            INPUT_COL_WIDTH,
                        ) {
                            *mutated_label = Some("Edit Seam Overlap");
                        }
                    },
                );
            });
        },
    );
}
