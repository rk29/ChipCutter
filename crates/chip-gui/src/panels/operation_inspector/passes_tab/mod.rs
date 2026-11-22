pub mod deburr3d_card;
pub mod milling_2d;
pub mod surface_3d;
pub mod thread_mill;
pub mod vcarve;

use std::ops::RangeInclusive;

use chip_cam::{
    FeedOptimizationConfig, Operation, OperationKind, OperationRotaryMode, ResolvedRotaryConfig,
    RestMachiningConfig, RestMachiningMode, StrategyParams, compute_index_angle_for_normal,
};
use chip_math::{DVec3, DisplayUnit, EvalContext, ParamLength};
use chip_tools::{CutterGeometry, ToolId};

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
            let mut resp = ui.checkbox(flag, checkbox_text);
            if let Some(t) = tip {
                resp = resp.on_hover_text(t);
            }
            if resp.changed() {
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
            let mut resp = ui.add(dv);
            if let Some(t) = tip {
                resp = resp.on_hover_text(t);
            }
            if resp.changed() {
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
            let resp = egui::ComboBox::from_id_salt(id)
                .width(COMBO_COL_WIDTH)
                .truncate()
                .selected_text(fmt_fn(*value))
                .show_ui(ui, |ui| {
                    for &cand in items {
                        if ui.selectable_value(value, cand, fmt_fn(cand)).changed() {
                            *mutated = Some(undo_label);
                        }
                    }
                })
                .response;
            if let Some(t) = tip {
                resp.on_hover_text(t);
            }
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
    pub assigned_tool_geom: Option<&'a CutterGeometry>,
    pub tool_feed_preview: Option<(f64, f64)>,
    pub resolved_thread_depth_mm: Option<f64>,
    pub display_unit: DisplayUnit,
    pub setup_idx: usize,
    pub op_idx: usize,
    pub eval_ctx: &'a EvalContext,
    pub resolved_rotary: Option<ResolvedRotaryConfig>,
    pub selected_face_normal_wcs: Option<DVec3>,
    pub pick_mode: &'a mut ViewportPickMode,
    pub mutated_label: &'a mut Option<&'static str>,
}

pub fn show_passes_tab(ui: &mut egui::Ui, ctx: PassesTabCtx<'_>) {
    let PassesTabCtx {
        op,
        preceding_ops,
        clearing_tool_candidates,
        assigned_tool_geom,
        tool_feed_preview,
        resolved_thread_depth_mm,
        display_unit,
        setup_idx,
        op_idx,
        eval_ctx,
        resolved_rotary,
        selected_face_normal_wcs,
        pick_mode,
        mutated_label,
    } = ctx;

    let mut item = PassItemCtx {
        setup_idx,
        op_idx,
        eval_ctx,
        mutated_label,
    };

    if op.kind != OperationKind::Deburr3D {
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
                    f.set_tip(
                        "Divide total cut depth into equal Z stepdowns up to Maximum Stepdown",
                    );
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
    }

    if matches!(
        op.kind,
        OperationKind::Pocket2D
            | OperationKind::Contour2D
            | OperationKind::Adaptive3D
            | OperationKind::Parallel3D
            | OperationKind::Scallop3D
    ) {
        let rest_title = if op.kind.is_3d_surface() {
            "3D Rest Machining"
        } else {
            "2.5D Rest Machining"
        };
        show_rest_machining_card(
            ui,
            rest_title,
            &mut op.passes.rest_machining,
            preceding_ops,
            &mut item,
        );
    }

    if let Some(rot) = resolved_rotary
        && !op.kind.is_rotary_native()
    {
        show_operation_rotary_card(
            ui,
            &mut op.rotary,
            op.kind,
            rot,
            selected_face_normal_wcs,
            &mut item,
        );
    }

    if !matches!(
        op.kind,
        OperationKind::DrillBore | OperationKind::ThreadMill
    ) {
        show_feed_optimization_card(ui, &mut op.passes.feed_optimization, &mut item);
    }

    let has_explicit_selections = !op.selections.is_empty();
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
        StrategyParams::Waterline3D(p) => surface_3d::show_waterline3d_params(ui, p, item),
        StrategyParams::Adaptive3D(p) => surface_3d::show_adaptive3d_params(ui, p, item),
        StrategyParams::Parallel3D(p) => surface_3d::show_parallel3d_params(ui, p, item),
        StrategyParams::Scallop3D(p) => surface_3d::show_scallop3d_params(ui, p, item),
        StrategyParams::Pencil3D(p) => surface_3d::show_pencil3d_params(ui, p, item),
        StrategyParams::Deburr3D(p) => deburr3d_card::show_deburr3d_params(
            ui,
            deburr3d_card::Deburr3DParamsUiCtx {
                p,
                assigned_tool_geom,
                has_explicit_selections,
                display_unit,
                item,
            },
        ),
        StrategyParams::Rotary2Linear(p) => surface_3d::show_rotary2linear_params(ui, p, item),
        StrategyParams::Rotary4AxisSim(p) => surface_3d::show_rotary4axis_sim_params(ui, p, item),
        StrategyParams::ThreadMill(p) => thread_mill::show_thread_mill_params(
            ui,
            thread_mill::ThreadMillParamsUiCtx {
                p,
                assigned_tool_geom,
                tool_feed_preview,
                resolved_thread_depth_mm,
                display_unit,
                item,
            },
        ),
    }
}

fn show_operation_rotary_card(
    ui: &mut egui::Ui,
    rotary_mode: &mut OperationRotaryMode,
    op_kind: OperationKind,
    rot: ResolvedRotaryConfig,
    selected_face_normal_wcs: Option<DVec3>,
    item: &mut PassItemCtx<'_>,
) {
    let (setup_idx, op_idx) = item.ids();
    let eval_ctx = item.eval_ctx;
    let supports_wrap = matches!(
        op_kind,
        OperationKind::Contour2D
            | OperationKind::Pocket2D
            | OperationKind::ChamferTrace
            | OperationKind::VCarve
            | OperationKind::DrillBore
    );

    section_card(
        ui,
        ("passes_rotary_card", setup_idx, op_idx),
        "4th-Axis Indexing / Wrapping",
        true,
        |ui| {
            let cur_mode = match rotary_mode {
                OperationRotaryMode::None => 0_u8,
                OperationRotaryMode::Indexed3Plus1 { .. } => 1_u8,
                OperationRotaryMode::WrapCylinder { .. } => 2_u8,
            };
            let mut sel_mode = cur_mode;

            item.grid(ui, "passes_rotary_grid", |f| {
                f.set_tip("Position the rotary chuck at a fixed 3+1 index angle or wrap a 2D sketch onto a cylinder");
                f.row("Rotary Mode", |ui, mutated| {
                    if ui.selectable_value(&mut sel_mode, 0, "None").clicked() && cur_mode != 0 {
                        *rotary_mode = OperationRotaryMode::None;
                        *mutated = Some("Set Operation Rotary Mode: None");
                    }
                    if ui
                        .selectable_value(&mut sel_mode, 1, "Indexed 3+1")
                        .on_hover_text(
                            "Index rotary axis at safe clearance radius, then mill along +Z",
                        )
                        .clicked()
                        && cur_mode != 1
                    {
                        *rotary_mode = OperationRotaryMode::Indexed3Plus1 { angle_deg: 0.0 };
                        *mutated = Some("Set Operation Rotary Mode: Indexed 3+1");
                    }
                    if supports_wrap
                        && ui
                            .selectable_value(&mut sel_mode, 2, "Wrap 2D")
                            .on_hover_text(
                                "Wrap flat 2D XY sketch toolpaths onto a cylinder (Z+X+A / Z+Y+B)",
                            )
                            .clicked()
                        && cur_mode != 2
                    {
                        let mut m = OperationRotaryMode::WrapCylinder {
                            wrap_radius: ParamLength {
                                expr: Some("rotary_radius".to_string()),
                                value_mm: rot.stock_outer_radius_mm.max(1.0),
                            },
                            axial_offset: ParamLength::from_mm(0.0),
                            angle_offset_deg: 0.0,
                        };
                        m.reevaluate(eval_ctx);
                        *rotary_mode = m;
                        *mutated = Some("Set Operation Rotary Mode: Wrap 2D on Cylinder");
                    }
                });

                match rotary_mode {
                    OperationRotaryMode::None => {}
                    OperationRotaryMode::Indexed3Plus1 { angle_deg } => {
                        let axis_lbl = format!("{}-Axis Index", rot.axis_letter);
                        f.set_tip("Target rotary index angle rotated at safe clearance radius before cutting");
                        f.row(&axis_lbl, |ui, mutated| {
                            if ui
                                .add(
                                    egui::DragValue::new(angle_deg)
                                        .speed(1.0)
                                        .range(-360.0..=360.0)
                                        .suffix("°"),
                                )
                                .on_hover_text("Target rotary index angle in degrees before executing this 3+1 operation")
                                .changed()
                            {
                                *mutated = Some("Edit Indexed 3+1 Angle");
                            }
                        });
                        f.set_tip("Snap rotary index angle to standard 90° cardinal faces");
                        f.row("Quick Index", |ui, mutated| {
                            ui.horizontal(|ui| {
                                ui.spacing_mut().item_spacing.x = 4.0;
                                for quick in [0.0_f64, 90.0, 180.0, 270.0] {
                                    let active = (*angle_deg - quick).abs() < 0.05;
                                    if ui
                                        .selectable_label(active, format!("{quick:.0}°"))
                                        .on_hover_text(format!("Index rotary axis to {quick:.0} degrees"))
                                        .clicked()
                                    {
                                        *angle_deg = quick;
                                        *mutated = Some("Set Cardinal 3+1 Index Angle");
                                    }
                                }
                            });
                        });
                        let face_angle = selected_face_normal_wcs
                            .and_then(|n| compute_index_angle_for_normal(n, &rot));
                        f.set_tip("Automatically compute the index angle that aims the selected 3D planar face straight up at the +Z spindle");
                        f.row("3D Face Align", |ui, mutated| {
                            let label = if let Some(a) = face_angle {
                                format!("Align to Face ({a:.1}°)")
                            } else {
                                "Align to Selected 3D Face".to_string()
                            };
                            let btn = ui
                                .add_enabled(face_angle.is_some(), egui::Button::new(label))
                                .on_disabled_hover_text(
                                    "Click any planar 3D face in the viewport first to align the 4th axis to its normal.",
                                );
                            if btn.clicked()
                                && let Some(a) = face_angle
                            {
                                *angle_deg = a;
                                *mutated = Some("Align 3+1 Index to Selected 3D Face");
                            }
                        });
                    }
                    OperationRotaryMode::WrapCylinder {
                        wrap_radius,
                        axial_offset,
                        angle_offset_deg,
                    } => {
                        f.set_tip("Target cylinder outer radius onto which the 2D sketch (Z=0) is wrapped");
                        f.len("Wrap Radius", wrap_radius, "Edit Cylinder Wrap Radius");
                        f.set_tip("Axial shift along the rotary axis applied to the wrapped 2D sketch");
                        f.len("Axial Offset", axial_offset, "Edit Cylinder Axial Offset");
                        f.set_tip("Starting angular orientation around the cylinder");
                        f.drag(
                            "Angle Offset",
                            angle_offset_deg,
                            -360.0..=360.0,
                            (1.0, "°"),
                            "Edit Cylinder Angle Offset",
                        );
                    }
                }
            });
        },
    );
}

fn show_feed_optimization_card(
    ui: &mut egui::Ui,
    feed_opt: &mut FeedOptimizationConfig,
    item: &mut PassItemCtx<'_>,
) {
    let (setup_idx, op_idx) = item.ids();
    let eval_ctx = item.eval_ctx;
    section_card(
        ui,
        ("passes_feed_opt_card", setup_idx, op_idx),
        "Feed Optimization",
        feed_opt.enabled,
        |ui| {
            item.grid(ui, "passes_feed_opt_grid", |f| {
                let was_enabled = feed_opt.enabled;
                f.set_tip(
                    "Modulate cutting feedrates around sharp corners, concave internal arcs, and light-engagement passes without altering rapids or plunges",
                );
                f.check(
                    "Feed Optimization",
                    &mut feed_opt.enabled,
                    "Enable Engagement-Aware Feed",
                    "Toggle Feed Optimization",
                );
                if feed_opt.enabled && !was_enabled {
                    feed_opt.reevaluate(eval_ctx);
                }

                if feed_opt.enabled {
                    f.set_tip(
                        "Split segments approaching sharp linear corners to decelerate smoothly into the apex and recover after exit",
                    );
                    f.check(
                        "Corner Slowdown",
                        &mut feed_opt.corner_slowdown,
                        "Lookahead Deceleration",
                        "Toggle Corner Slowdown",
                    );

                    if feed_opt.corner_slowdown || feed_opt.internal_arc_scaling {
                        let mut min_pct = (feed_opt.min_feed_fraction.clamp(0.15, 1.0)) * 100.0;
                        f.set_tip(
                            "Minimum allowed feedrate percentage in tight corners and concave arcs (prevents tool rubbing and melting)",
                        );
                        f.row("Min Feed %", |ui, mutated| {
                            if ui
                                .add(
                                    egui::DragValue::new(&mut min_pct)
                                        .range(15.0..=100.0)
                                        .speed(1.0)
                                        .fixed_decimals(0)
                                        .suffix("%"),
                                )
                                .on_hover_text(
                                    "Minimum feedrate floor as a percentage of nominal cutting feed (15%–100%)",
                                )
                                .changed()
                            {
                                feed_opt.min_feed_fraction = (min_pct / 100.0).clamp(0.15, 1.0);
                                *mutated = Some("Edit Min Feed Fraction");
                            }
                        });
                    }

                    if feed_opt.corner_slowdown {
                        f.set_tip(
                            "Minimum linear direction deflection angle triggering corner lookahead slowdown",
                        );
                        f.drag(
                            "Corner Threshold",
                            &mut feed_opt.corner_angle_threshold_deg,
                            5.0..=120.0,
                            (1.0, "°"),
                            "Edit Corner Angle Threshold",
                        );
                        f.set_tip(
                            "Lookahead deceleration and exit recovery ramp distance around sharp corners (supports formulas like tool_d * 0.5)",
                        );
                        f.len(
                            "Slowdown Distance",
                            &mut feed_opt.slowdown_distance,
                            "Edit Corner Slowdown Distance",
                        );
                    }

                    f.set_tip(
                        "Scale centerline feedrate by (R_arc - R_tool) / R_arc on concave internal arcs so cutting-edge chipload stays constant",
                    );
                    f.check(
                        "Internal Arcs",
                        &mut feed_opt.internal_arc_scaling,
                        "Perimeter-to-Center Scaling",
                        "Toggle Internal Arc Scaling",
                    );

                    f.set_tip(
                        "Boost straight/low-curvature light-engagement passes to maintain constant chipload while still throttling inside corners and arcs",
                    );
                    f.check(
                        "Chip-Thinning Boost",
                        &mut feed_opt.chip_thinning_boost,
                        "Boost Straight Passes",
                        "Toggle Chip-Thinning Boost",
                    );
                    if feed_opt.chip_thinning_boost {
                        f.set_tip(
                            "Upper cap on chip-thinning feedrate multiplier relative to nominal cutting feed (1.0x–2.5x)",
                        );
                        f.drag(
                            "Max Multiplier",
                            &mut feed_opt.max_thinning_multiplier,
                            1.0..=2.5,
                            (0.05, "x"),
                            "Edit Max Chip-Thinning Multiplier",
                        );
                    }
                }
            });
        },
    );
}

fn show_rest_machining_card(
    ui: &mut egui::Ui,
    title: &str,
    rest: &mut RestMachiningConfig,
    preceding_ops: &[(String, String)],
    item: &mut PassItemCtx<'_>,
) {
    let (setup_idx, op_idx) = item.ids();
    let eval_ctx = item.eval_ctx;
    section_card(
        ui,
        ("passes_rest_card", setup_idx, op_idx),
        title,
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
                    if ui
                        .selectable_value(&mut sel_mode, 0, "Off")
                        .on_hover_text("Disable rest machining and clear the entire selected region")
                        .clicked()
                        && cur_mode != 0
                    {
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
                        .on_hover_text("Automatically subtract material removed by every enabled operation earlier in this Setup")
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
                            if ui
                                .checkbox(&mut checked, uname)
                                .on_hover_text(format!("Include '{uname}' in the preceding stock subtraction for rest machining"))
                                .changed()
                            {
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
