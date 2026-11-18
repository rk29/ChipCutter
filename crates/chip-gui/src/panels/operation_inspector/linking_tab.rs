use chip_cam::{EntryStrategy, Operation, RetractPolicy};
use chip_math::EvalContext;

use crate::widgets::edit_param_length;

pub fn show_linking_tab(
    ui: &mut egui::Ui,
    op: &mut Operation,
    setup_idx: usize,
    op_idx: usize,
    eval_ctx: &EvalContext,
    mutated_label: &mut Option<&'static str>,
) {
    ui.strong("Retract & Stay-Down Policy");
    egui::ComboBox::from_id_salt(("lnk_ret", setup_idx, op_idx))
        .selected_text(op.linking.retract_policy.label())
        .show_ui(ui, |ui| {
            for r in RetractPolicy::ALL {
                if ui
                    .selectable_value(&mut op.linking.retract_policy, r, r.label())
                    .changed()
                {
                    *mutated_label = Some("Change Retract Policy");
                }
            }
        });
    ui.label("Max Stay-Down Link Distance:");
    if edit_param_length(
        ui,
        ("lnk_stay", setup_idx, op_idx),
        &mut op.linking.safe_stay_down_distance,
        eval_ctx,
        120.0,
    ) {
        *mutated_label = Some("Edit Stay-Down Distance");
    }

    ui.separator();
    ui.strong("Ramp & Cavity Entry");
    let cur_lbl = op.linking.entry_mode.label();
    egui::ComboBox::from_id_salt(("lnk_ent", setup_idx, op_idx))
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
    ui.horizontal(|ui| {
        ui.label("Ramp Angle (°):");
        if ui
            .add(
                egui::DragValue::new(&mut op.linking.ramp_angle_deg)
                    .range(0.5..=30.0)
                    .speed(0.25),
            )
            .changed()
        {
            *mutated_label = Some("Edit Ramp Angle");
        }
    });
    ui.label("Minimum Helix Path Diameter:");
    if edit_param_length(
        ui,
        ("lnk_min_hd", setup_idx, op_idx),
        &mut op.linking.min_ramp_diameter,
        eval_ctx,
        120.0,
    ) {
        *mutated_label = Some("Edit Min Helix Diameter");
    }

    ui.separator();
    ui.strong("Lead-In / Lead-Out & Seam Overlap");
    if ui
        .checkbox(&mut op.linking.lead_in.enabled, "Enable Tangent Lead-In")
        .changed()
    {
        *mutated_label = Some("Toggle Lead-In");
    }
    if op.linking.lead_in.enabled {
        ui.label("Horizontal Lead Arc Radius:");
        if edit_param_length(
            ui,
            ("lnk_lhr", setup_idx, op_idx),
            &mut op.linking.lead_in.horizontal_radius,
            eval_ctx,
            110.0,
        ) {
            *mutated_label = Some("Edit Lead-In Radius");
        }
        ui.horizontal(|ui| {
            ui.label("Horizontal Sweep (°):");
            if ui
                .add(
                    egui::DragValue::new(&mut op.linking.lead_in.horizontal_sweep_deg)
                        .range(0.0..=180.0),
                )
                .changed()
            {
                *mutated_label = Some("Edit Lead-In Sweep");
            }
        });
        ui.label("Linear Tangent Lead Length:");
        if edit_param_length(
            ui,
            ("lnk_llin", setup_idx, op_idx),
            &mut op.linking.lead_in.linear_length,
            eval_ctx,
            110.0,
        ) {
            *mutated_label = Some("Edit Lead-In Linear");
        }
        ui.label("Vertical Lead Arc Radius:");
        if edit_param_length(
            ui,
            ("lnk_lvr", setup_idx, op_idx),
            &mut op.linking.lead_in.vertical_radius,
            eval_ctx,
            110.0,
        ) {
            *mutated_label = Some("Edit Vertical Lead Radius");
        }
        if ui
            .checkbox(
                &mut op.linking.lead_out_same_as_in,
                "Lead-Out Mirrors Lead-In",
            )
            .changed()
        {
            *mutated_label = Some("Toggle Mirror Lead-Out");
        }
    }
    ui.label("Closed Contour Seam Overlap:");
    if edit_param_length(
        ui,
        ("lnk_ovl", setup_idx, op_idx),
        &mut op.linking.seam_overlap,
        eval_ctx,
        110.0,
    ) {
        *mutated_label = Some("Edit Seam Overlap");
    }
}
