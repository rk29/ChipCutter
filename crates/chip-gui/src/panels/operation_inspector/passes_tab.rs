use chip_cam::{
    ChamferSourceMode, ChamferTraceMode, ChamferTraceParams, Contour2DParams, ContourCompensation,
    DrillBoreParams, DrillCycleKind, FacingCutMode, FacingParams, HoleMachiningMode, Operation,
    Pocket2DParams, PocketPassOrder, PocketStrategyKind, StrategyParams, TabPlacementMode,
    TabProfileKind,
};
use chip_math::{EvalContext, ParamLength};

use crate::{panels::ViewportPickMode, widgets::edit_param_length};

pub fn show_passes_tab(
    ui: &mut egui::Ui,
    op: &mut Operation,
    setup_idx: usize,
    op_idx: usize,
    eval_ctx: &EvalContext,
    pick_mode: &mut ViewportPickMode,
    mutated_label: &mut Option<&'static str>,
) {
    ui.strong("Depth Passes & Stock to Leave");
    if ui
        .checkbox(&mut op.passes.multiple_depths, "Multiple Z Stepdowns")
        .changed()
    {
        *mutated_label = Some("Toggle Multiple Depths");
    }
    if op.passes.multiple_depths {
        ui.label("Maximum Stepdown (`ap`):");
        if edit_param_length(
            ui,
            ("op_max_ap", setup_idx, op_idx),
            &mut op.passes.max_stepdown,
            eval_ctx,
            120.0,
        ) {
            *mutated_label = Some("Edit Max Stepdown");
        }
        if ui
            .checkbox(&mut op.passes.even_stepdowns, "Use Even Z Stepdowns")
            .changed()
        {
            *mutated_label = Some("Toggle Even Stepdowns");
        }
    }

    ui.add_space(4.0);
    ui.label("Radial Stock to Leave:");
    if edit_param_length(
        ui,
        ("op_rad_leave", setup_idx, op_idx),
        &mut op.passes.stock_to_leave_radial,
        eval_ctx,
        120.0,
    ) {
        *mutated_label = Some("Edit Radial Stock to Leave");
    }
    ui.label("Axial Stock to Leave:");
    if edit_param_length(
        ui,
        ("op_ax_leave", setup_idx, op_idx),
        &mut op.passes.stock_to_leave_axial,
        eval_ctx,
        120.0,
    ) {
        *mutated_label = Some("Edit Axial Stock to Leave");
    }

    ui.separator();
    ui.strong("Strategy Parameters");

    match op.strategy_mut() {
        StrategyParams::Facing(p) => {
            show_facing_params(ui, p, setup_idx, op_idx, eval_ctx, mutated_label);
        }
        StrategyParams::Contour2D(p) => {
            show_contour2d_params(ui, p, setup_idx, op_idx, eval_ctx, pick_mode, mutated_label);
        }
        StrategyParams::Pocket2D(p) => {
            show_pocket2d_params(ui, p, setup_idx, op_idx, eval_ctx, mutated_label);
        }
        StrategyParams::DrillBore(p) => {
            show_drill_bore_params(ui, p, setup_idx, op_idx, eval_ctx, mutated_label);
        }
        StrategyParams::ChamferTrace(p) => {
            show_chamfer_trace_params(ui, p, setup_idx, op_idx, eval_ctx, mutated_label);
        }
    }
}

fn show_facing_params(
    ui: &mut egui::Ui,
    p: &mut FacingParams,
    setup_idx: usize,
    op_idx: usize,
    eval_ctx: &EvalContext,
    mutated_label: &mut Option<&'static str>,
) {
    egui::ComboBox::from_id_salt(("face_cut_mode", setup_idx, op_idx))
        .selected_text(p.cut_mode.label())
        .show_ui(ui, |ui| {
            for m in FacingCutMode::ALL {
                if ui.selectable_value(&mut p.cut_mode, m, m.label()).changed() {
                    *mutated_label = Some("Change Facing Cut Mode");
                }
            }
        });
    ui.label("Stepover (`ae`):");
    if edit_param_length(
        ui,
        ("face_ae", setup_idx, op_idx),
        &mut p.stepover,
        eval_ctx,
        120.0,
    ) {
        *mutated_label = Some("Edit Facing Stepover");
    }
    ui.label("Pass Extension Outside Stock:");
    if edit_param_length(
        ui,
        ("face_ext", setup_idx, op_idx),
        &mut p.pass_extension,
        eval_ctx,
        120.0,
    ) {
        *mutated_label = Some("Edit Facing Extension");
    }
    if ui
        .checkbox(
            &mut p.auto_longest_axis,
            "Auto-align along longest XY stock axis",
        )
        .changed()
    {
        *mutated_label = Some("Toggle Facing Auto Axis");
    }
    if !p.auto_longest_axis {
        ui.horizontal(|ui| {
            ui.label("Pass Angle (°):");
            if ui
                .add(egui::DragValue::new(&mut p.pass_direction_deg).speed(1.0))
                .changed()
            {
                *mutated_label = Some("Edit Facing Angle");
            }
        });
    }
}

fn show_contour2d_params(
    ui: &mut egui::Ui,
    p: &mut Contour2DParams,
    setup_idx: usize,
    op_idx: usize,
    eval_ctx: &EvalContext,
    pick_mode: &mut ViewportPickMode,
    mutated_label: &mut Option<&'static str>,
) {
    ui.label("Cutter Compensation:");
    egui::ComboBox::from_id_salt(("cnt_comp", setup_idx, op_idx))
        .selected_text(p.compensation.label())
        .show_ui(ui, |ui| {
            for c in ContourCompensation::ALL {
                if ui
                    .selectable_value(&mut p.compensation, c, c.label())
                    .changed()
                {
                    *mutated_label = Some("Change Contour Compensation");
                }
            }
        });
    if ui
        .checkbox(&mut p.wear_comp_metadata, "Emit G41/G42 Wear Comp Metadata")
        .changed()
    {
        *mutated_label = Some("Toggle Wear Comp");
    }
    if ui
        .checkbox(&mut p.continuous_spiral_ramp, "Continuous Spiral Ramp")
        .changed()
    {
        *mutated_label = Some("Toggle Spiral Ramp");
    }

    ui.horizontal(|ui| {
        ui.label("Roughing Passes:");
        if ui
            .add(egui::DragValue::new(&mut p.roughing_passes).range(0..=16))
            .changed()
        {
            *mutated_label = Some("Edit Roughing Passes");
        }
        ui.label("Finish Passes:");
        if ui
            .add(egui::DragValue::new(&mut p.finish_passes).range(0..=8))
            .changed()
        {
            *mutated_label = Some("Edit Finish Passes");
        }
    });
    if p.roughing_passes > 1 {
        ui.label("Roughing Stepover:");
        if edit_param_length(
            ui,
            ("cnt_r_step", setup_idx, op_idx),
            &mut p.roughing_stepover,
            eval_ctx,
            120.0,
        ) {
            *mutated_label = Some("Edit Roughing Stepover");
        }
    }
    if p.finish_passes > 0 {
        ui.label("Finishing Stepover:");
        if edit_param_length(
            ui,
            ("cnt_f_step", setup_idx, op_idx),
            &mut p.finish_stepover,
            eval_ctx,
            120.0,
        ) {
            *mutated_label = Some("Edit Finish Stepover");
        }
        if ui
            .checkbox(&mut p.repeat_finish_pass, "Repeat Final Pass (Spring Pass)")
            .changed()
        {
            *mutated_label = Some("Toggle Spring Pass");
        }
    }

    ui.separator();
    if ui
        .checkbox(&mut p.tabs.enabled, "Enable Holding Tabs (Bridges)")
        .changed()
    {
        *mutated_label = Some("Toggle Holding Tabs");
    }
    if p.tabs.enabled {
        let mut is_tri = matches!(p.tabs.profile, TabProfileKind::Triangular { .. });
        ui.horizontal(|ui| {
            ui.label("Shape:");
            if ui
                .selectable_value(&mut is_tri, true, "Ramped Triangular")
                .clicked()
            {
                p.tabs.profile = TabProfileKind::Triangular {
                    ramp_angle_deg: 25.0,
                };
                *mutated_label = Some("Change Tab Profile");
            }
            if ui
                .selectable_value(&mut is_tri, false, "Rectangular")
                .clicked()
            {
                p.tabs.profile = TabProfileKind::Rectangular;
                *mutated_label = Some("Change Tab Profile");
            }
        });
        if let TabProfileKind::Triangular { ramp_angle_deg } = &mut p.tabs.profile {
            ui.horizontal(|ui| {
                ui.label("Tab Ramp Angle (°):");
                if ui
                    .add(egui::DragValue::new(ramp_angle_deg).range(5.0..=75.0))
                    .changed()
                {
                    *mutated_label = Some("Edit Tab Ramp Angle");
                }
            });
        }

        ui.label("Tab Width:");
        if edit_param_length(
            ui,
            ("tab_w", setup_idx, op_idx),
            &mut p.tabs.width,
            eval_ctx,
            110.0,
        ) {
            *mutated_label = Some("Edit Tab Width");
        }
        ui.label("Tab Height (from Bottom Z):");
        if edit_param_length(
            ui,
            ("tab_h", setup_idx, op_idx),
            &mut p.tabs.height,
            eval_ctx,
            110.0,
        ) {
            *mutated_label = Some("Edit Tab Height");
        }

        let mut is_manual = matches!(p.tabs.placement, TabPlacementMode::Manual);
        ui.horizontal(|ui| {
            ui.label("Placement:");
            if ui
                .selectable_value(&mut is_manual, false, "Automatic")
                .clicked()
            {
                p.tabs.placement = TabPlacementMode::default();
                if matches!(pick_mode, ViewportPickMode::PlaceContourTab { .. }) {
                    *pick_mode = ViewportPickMode::InspectEntity;
                }
                *mutated_label = Some("Set Automatic Tabs");
            }
            if ui
                .selectable_value(&mut is_manual, true, "Manual Interactive")
                .clicked()
            {
                p.tabs.placement = TabPlacementMode::Manual;
                *pick_mode = ViewportPickMode::PlaceContourTab { setup_idx, op_idx };
                *mutated_label = Some("Set Manual Tabs");
            }
        });

        match &mut p.tabs.placement {
            TabPlacementMode::Automatic {
                count,
                by_spacing,
                target_spacing,
            } => {
                if ui
                    .checkbox(by_spacing, "Place by target perimeter spacing")
                    .changed()
                {
                    *mutated_label = Some("Toggle Tab Spacing Mode");
                }
                if *by_spacing {
                    if edit_param_length(
                        ui,
                        ("tab_sp", setup_idx, op_idx),
                        target_spacing,
                        eval_ctx,
                        110.0,
                    ) {
                        *mutated_label = Some("Edit Tab Spacing");
                    }
                } else {
                    ui.horizontal(|ui| {
                        ui.label("Tab Count:");
                        if ui.add(egui::DragValue::new(count).range(1..=32)).changed() {
                            *mutated_label = Some("Edit Tab Count");
                        }
                    });
                }
            }
            TabPlacementMode::Manual => {
                let picking_tabs =
                    *pick_mode == ViewportPickMode::PlaceContourTab { setup_idx, op_idx };
                let btn_txt = if picking_tabs {
                    "[*] Click Contour in 3D Viewport (Active)"
                } else {
                    "Click Contour in 3D Viewport to Add Tabs"
                };
                if ui.button(btn_txt).clicked() {
                    *pick_mode = if picking_tabs {
                        ViewportPickMode::InspectEntity
                    } else {
                        ViewportPickMode::PlaceContourTab { setup_idx, op_idx }
                    };
                }
                let mut del_anchor = None;
                for (a_idx, anchor) in p.tabs.manual_anchors.iter().enumerate() {
                    ui.horizontal(|ui| {
                        ui.weak(format!(
                            "Tab #{} (C{}, s={:.1}%)",
                            a_idx + 1,
                            anchor.contour_index + 1,
                            anchor.s_norm * 100.0
                        ));
                        if ui.small_button("×").clicked() {
                            del_anchor = Some(a_idx);
                        }
                    });
                }
                if let Some(rm) = del_anchor {
                    p.tabs.manual_anchors.remove(rm);
                    *mutated_label = Some("Remove Manual Tab");
                }
            }
        }
    }
}

fn show_pocket2d_params(
    ui: &mut egui::Ui,
    p: &mut Pocket2DParams,
    setup_idx: usize,
    op_idx: usize,
    eval_ctx: &EvalContext,
    mutated_label: &mut Option<&'static str>,
) {
    let mut is_zigzag = matches!(p.strategy, PocketStrategyKind::ZigZag { .. });
    ui.horizontal(|ui| {
        ui.label("Clearing:");
        if ui
            .selectable_value(&mut is_zigzag, false, "Contour-Parallel")
            .clicked()
        {
            p.strategy = PocketStrategyKind::ContourParallel;
            *mutated_label = Some("Select Contour-Parallel Pocket");
        }
        if ui
            .selectable_value(&mut is_zigzag, true, "Zig-Zag Raster")
            .clicked()
        {
            p.strategy = PocketStrategyKind::ZigZag {
                pass_angle_deg: 0.0,
            };
            *mutated_label = Some("Select Zig-Zag Pocket");
        }
    });
    if let PocketStrategyKind::ZigZag { pass_angle_deg } = &mut p.strategy {
        ui.horizontal(|ui| {
            ui.label("Raster Angle (°):");
            if ui
                .add(egui::DragValue::new(pass_angle_deg).speed(1.0))
                .changed()
            {
                *mutated_label = Some("Edit Pocket Raster Angle");
            }
        });
    }

    ui.label("Stepover (`ae`):");
    if edit_param_length(
        ui,
        ("pkt_ae", setup_idx, op_idx),
        &mut p.stepover,
        eval_ctx,
        120.0,
    ) {
        *mutated_label = Some("Edit Pocket Stepover");
    }

    ui.label("Pass Ordering:");
    egui::ComboBox::from_id_salt(("pkt_ord", setup_idx, op_idx))
        .selected_text(p.pass_order.label())
        .show_ui(ui, |ui| {
            for o in PocketPassOrder::ALL {
                if ui
                    .selectable_value(&mut p.pass_order, o, o.label())
                    .changed()
                {
                    *mutated_label = Some("Change Pocket Pass Order");
                }
            }
        });

    ui.label("Open-Edge Overhang:");
    if edit_param_length(
        ui,
        ("pkt_ovh", setup_idx, op_idx),
        &mut p.open_edge_overhang,
        eval_ctx,
        120.0,
    ) {
        *mutated_label = Some("Edit Open-Edge Overhang");
    }

    if ui
        .checkbox(&mut p.finish_wall_pass, "Dedicated Wall Finishing Pass")
        .changed()
    {
        *mutated_label = Some("Toggle Pocket Finish Wall Pass");
    }
    if p.finish_wall_pass {
        ui.label("Finish Wall Allowance:");
        if edit_param_length(
            ui,
            ("pkt_fin_ae", setup_idx, op_idx),
            &mut p.finish_wall_stepover,
            eval_ctx,
            120.0,
        ) {
            *mutated_label = Some("Edit Pocket Finish Allowance");
        }
    }
}

fn show_drill_bore_params(
    ui: &mut egui::Ui,
    p: &mut DrillBoreParams,
    setup_idx: usize,
    op_idx: usize,
    eval_ctx: &EvalContext,
    mutated_label: &mut Option<&'static str>,
) {
    ui.label("Machining Mode:");
    egui::ComboBox::from_id_salt(("db_mode", setup_idx, op_idx))
        .selected_text(p.mode.label())
        .show_ui(ui, |ui| {
            for m in HoleMachiningMode::ALL {
                if ui.selectable_value(&mut p.mode, m, m.label()).changed() {
                    *mutated_label = Some("Change Hole Machining Mode");
                }
            }
        });
    if ui
        .checkbox(
            &mut p.optimize_hole_order,
            "Nearest-Neighbor + 2-Opt Hole Sorting",
        )
        .changed()
    {
        *mutated_label = Some("Toggle Hole Order Optimization");
    }

    ui.separator();
    ui.strong("Drilling Cycle Settings");
    egui::ComboBox::from_id_salt(("db_cyc", setup_idx, op_idx))
        .selected_text(p.cycle_kind.label())
        .show_ui(ui, |ui| {
            for ck in DrillCycleKind::ALL {
                if ui
                    .selectable_value(&mut p.cycle_kind, ck, ck.label())
                    .changed()
                {
                    *mutated_label = Some("Change Drill Cycle Kind");
                }
            }
        });
    if matches!(
        p.cycle_kind,
        DrillCycleKind::ChipBreak | DrillCycleKind::DeepPeck
    ) {
        ui.label("Peck Depth (`Q`):");
        if edit_param_length(
            ui,
            ("db_peck", setup_idx, op_idx),
            &mut p.peck_depth,
            eval_ctx,
            110.0,
        ) {
            *mutated_label = Some("Edit Peck Depth");
        }
    }
    if ui
        .checkbox(
            &mut p.tip_breakthrough,
            "Drill Tip Breakthrough Compensation",
        )
        .changed()
    {
        *mutated_label = Some("Toggle Drill Breakthrough");
    }
    if p.tip_breakthrough {
        ui.label("Extra Breakthrough Past Bottom:");
        if edit_param_length(
            ui,
            ("db_bt", setup_idx, op_idx),
            &mut p.breakthrough_extra,
            eval_ctx,
            110.0,
        ) {
            *mutated_label = Some("Edit Breakthrough Extra");
        }
    }

    ui.separator();
    ui.strong("Helical Bore Settings (when D_tool < D_hole)");
    ui.horizontal(|ui| {
        ui.label("Helix Ramp Angle (°):");
        if ui
            .add(
                egui::DragValue::new(&mut p.bore_ramp_angle_deg)
                    .range(0.5..=15.0)
                    .speed(0.2),
            )
            .changed()
        {
            *mutated_label = Some("Edit Bore Ramp Angle");
        }
    });
    if ui
        .checkbox(&mut p.bore_finish_pass, "Radial Finish Pass")
        .changed()
    {
        *mutated_label = Some("Toggle Bore Finish Pass");
    }
    if ui
        .checkbox(&mut p.bore_spring_pass, "Repeat Finish Spring Pass")
        .changed()
    {
        *mutated_label = Some("Toggle Bore Spring Pass");
    }
}

fn show_chamfer_trace_params(
    ui: &mut egui::Ui,
    p: &mut ChamferTraceParams,
    setup_idx: usize,
    op_idx: usize,
    eval_ctx: &EvalContext,
    mutated_label: &mut Option<&'static str>,
) {
    egui::ComboBox::from_id_salt(("ct_mode", setup_idx, op_idx))
        .selected_text(p.mode.label())
        .show_ui(ui, |ui| {
            for m in ChamferTraceMode::ALL {
                if ui.selectable_value(&mut p.mode, m, m.label()).changed() {
                    *mutated_label = Some("Change Chamfer/Trace Mode");
                }
            }
        });

    match p.mode {
        ChamferTraceMode::EdgeChamfer => {
            let mut is_modeled = matches!(p.chamfer_source, ChamferSourceMode::ModeledFace);
            ui.horizontal(|ui| {
                ui.label("CAD Source:");
                if ui
                    .selectable_value(&mut is_modeled, false, "Sharp 90° Edge")
                    .clicked()
                {
                    p.chamfer_source = ChamferSourceMode::SharpEdge {
                        width: ParamLength::from_mm(0.5),
                    };
                    *mutated_label = Some("Select Sharp Edge Chamfer");
                }
                if ui
                    .selectable_value(&mut is_modeled, true, "Modeled Bevel Face")
                    .clicked()
                {
                    p.chamfer_source = ChamferSourceMode::ModeledFace;
                    *mutated_label = Some("Select Modeled Chamfer Face");
                }
            });
            if let ChamferSourceMode::SharpEdge { width } = &mut p.chamfer_source {
                ui.label("Chamfer Width (`w`):");
                if edit_param_length(ui, ("chamf_w", setup_idx, op_idx), width, eval_ctx, 110.0) {
                    *mutated_label = Some("Edit Chamfer Width");
                }
            }
            ui.label("Tip Offset Past Lower Edge (`d_tip`):");
            if edit_param_length(
                ui,
                ("chamf_tip", setup_idx, op_idx),
                &mut p.tip_offset,
                eval_ctx,
                110.0,
            ) {
                *mutated_label = Some("Edit Chamfer Tip Offset");
            }
            if ui
                .checkbox(
                    &mut p.check_adjacent_walls,
                    "Guard against adjacent lower floor gouges",
                )
                .changed()
            {
                *mutated_label = Some("Toggle Chamfer Floor Guard");
            }
        }
        ChamferTraceMode::CurveTrace => {
            egui::ComboBox::from_id_salt(("tr_comp", setup_idx, op_idx))
                .selected_text(p.trace_compensation.label())
                .show_ui(ui, |ui| {
                    for c in ContourCompensation::ALL {
                        if ui
                            .selectable_value(&mut p.trace_compensation, c, c.label())
                            .changed()
                        {
                            *mutated_label = Some("Change Trace Compensation");
                        }
                    }
                });
            ui.label("Axial Depth Below Curve:");
            if edit_param_length(
                ui,
                ("tr_ax", setup_idx, op_idx),
                &mut p.trace_axial_offset,
                eval_ctx,
                110.0,
            ) {
                *mutated_label = Some("Edit Trace Axial Depth");
            }
        }
    }
}
