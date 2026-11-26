use chip_cam::{
    AdaptiveClearingParams, AdaptiveCutDirection, ChamferSourceMode, ChamferTraceMode,
    ChamferTraceParams, Contour2DParams, ContourCompensation, DrillBoreParams, DrillCycleKind,
    FacingCutMode, FacingParams, HoleMachiningMode, Pocket2DParams, PocketPassOrder,
    PocketStrategyKind, TabPlacementMode, TabProfileKind,
};
use chip_math::{DisplayUnit, ParamLength};
use chip_tools::compute_adaptive_thinning_feed;

use super::PassItemCtx;
use crate::{
    panels::ViewportPickMode,
    widgets::{BannerKind, IconKind, inline_banner, section_card, small_icon_button},
};

pub(super) fn show_facing_params(
    ui: &mut egui::Ui,
    p: &mut FacingParams,
    mut item: PassItemCtx<'_>,
) {
    item.card_grid(ui, "passes_facing_card", "Facing Strategy", |f| {
        f.set_tip("Zig-Zag bi-directional vs. One-Way facing raster direction");
        f.combo(
            "Cut Mode",
            &mut p.cut_mode,
            (&FacingCutMode::ALL, |m| m.label()),
            "Change Facing Cut Mode",
        );
        f.set_tip("Radial width of cut (ae) between parallel facing passes");
        f.len("Stepover", &mut p.stepover, "Edit Facing Stepover");
        f.set_tip("Distance cutter center overruns stock boundary so corners clear cleanly");
        f.len("Extension", &mut p.pass_extension, "Edit Facing Extension");
        f.set_tip("Automatically align facing passes along the longest XY stock axis");
        f.toggle(
            "Auto Longest XY",
            &mut p.auto_longest_axis,
            "Toggle Facing Auto",
        );
        if !p.auto_longest_axis {
            f.set_tip("Custom XY facing raster pass angle in degrees");
            f.drag(
                "Pass Angle",
                &mut p.pass_direction_deg,
                -360.0..=360.0,
                (1.0, "°"),
                "Edit Facing Angle",
            );
        }
        let mut has_finish = p.finishing_stepdown.is_some();
        f.set_tip("Reserve a dedicated light final axial skim pass at target Bottom Z");
        f.check(
            "Finish Pass",
            &mut has_finish,
            "Dedicated Finishing Stepdown",
            "Toggle Facing Finishing Stepdown",
        );
        if has_finish && p.finishing_stepdown.is_none() {
            p.finishing_stepdown = Some(ParamLength::from_mm(0.25));
        } else if !has_finish {
            p.finishing_stepdown = None;
        }
        if let Some(fs) = &mut p.finishing_stepdown {
            f.set_tip("Axial depth of cut reserved for the final facing finishing pass");
            f.len("Finish Stepdown", fs, "Edit Facing Finishing Stepdown");
        }
    });
}

pub(super) fn show_contour2d_params(
    ui: &mut egui::Ui,
    p: &mut Contour2DParams,
    tool_feed_preview: Option<(f64, f64)>,
    display_unit: DisplayUnit,
    mut item: PassItemCtx<'_>,
    pick_mode: &mut ViewportPickMode,
) {
    let ids = item.ids();
    item.card_grid(ui, "passes_contour_card", "2D Contour Strategy", |f| {
        f.set_tip("Offset cutter Left (climb), Right (conventional), or Center (on-curve)");
        f.combo(
            "Compensation",
            &mut p.compensation,
            (&ContourCompensation::ALL, |c| c.label()),
            "Change Contour Compensation",
        );
        f.set_tip("Emit controller G41/G42 cutter wear compensation metadata");
        f.toggle(
            "G41/G42 Wear",
            &mut p.wear_comp_metadata,
            "Toggle Wear Comp",
        );
        f.set_tip("Helically ramp continuously along closed contours instead of stepped Z levels");
        f.toggle(
            "Spiral Ramp",
            &mut p.continuous_spiral_ramp,
            "Toggle Spiral",
        );
        f.set_tip("Number of lateral XY offset roughing passes per Z depth");
        f.drag(
            "Rough Passes",
            &mut p.roughing_passes,
            0..=16,
            (0.25, ""),
            "Edit Roughing Passes",
        );
        if p.roughing_passes > 1 {
            f.set_tip("Lateral XY stepover between contour roughing passes");
            f.len(
                "Rough Step",
                &mut p.roughing_stepover,
                "Edit Roughing Stepover",
            );
        }
        f.set_tip("Number of dedicated light-engagement XY finishing passes");
        f.drag(
            "Finish Passes",
            &mut p.finish_passes,
            0..=8,
            (0.25, ""),
            "Edit Finish Passes",
        );
        if p.finish_passes > 0 {
            f.set_tip("Lateral XY allowance taken by each contour finishing pass");
            f.len(
                "Finish Step",
                &mut p.finish_stepover,
                "Edit Finish Stepover",
            );
            let mut has_finish_feed = p.finish_feed_mm_min.is_some();
            let default_finish_feed_mm = tool_feed_preview
                .map(|(_, nom_feed)| (0.75 * nom_feed).max(10.0))
                .unwrap_or(600.0);
            let feed_min = display_unit.feed_from_mm_min(1.0);
            let feed_max = display_unit.feed_from_mm_min(25_000.0);
            let feed_speed = display_unit.feed_from_mm_min(5.0);
            let feed_decimals = if display_unit == DisplayUnit::Inch {
                2
            } else {
                1
            };
            f.set_tip(
                "Optional dedicated XY feedrate for contour finishing passes (defaults to nominal cutting feed)",
            );
            f.row("Finish Pass Feed", |ui, mutated| {
                if ui
                    .checkbox(&mut has_finish_feed, "Override")
                    .on_hover_text(
                        "Use a custom feedrate on contour finishing passes instead of the nominal cutting feedrate",
                    )
                    .changed()
                {
                    p.finish_feed_mm_min = if has_finish_feed {
                        Some(default_finish_feed_mm)
                    } else {
                        None
                    };
                    *mutated = Some("Toggle Contour Finish Feed");
                }
                if let Some(ff_mm) = &mut p.finish_feed_mm_min {
                    let mut disp = display_unit.feed_from_mm_min(*ff_mm);
                    if ui
                        .add(
                            egui::DragValue::new(&mut disp)
                                .speed(feed_speed)
                                .max_decimals(feed_decimals)
                                .range(feed_min..=feed_max)
                                .suffix(format!(" {}", display_unit.feed_suffix())),
                        )
                        .on_hover_text("Dedicated linear XY feedrate for contour finishing passes")
                        .changed()
                    {
                        *ff_mm = display_unit.feed_to_mm_min(disp);
                        *mutated = Some("Edit Contour Finish Feed");
                    }
                } else {
                    ui.weak("(cutting feed)");
                }
            });
            f.set_tip("Repeat final contour pass at zero extra offset to eliminate deflection");
            f.toggle(
                "Spring Pass",
                &mut p.repeat_finish_pass,
                "Toggle Spring Pass",
            );
        }
    });

    section_card(
        ui,
        ("passes_tabs_card", ids.0, ids.1),
        "Holding Tabs",
        true,
        |ui| {
            item.grid(ui, "passes_tabs_grid", |f| {
                f.set_tip("Leave uncut bridges along the bottom of the contour to retain the part");
                f.toggle("Holding Tabs", &mut p.tabs.enabled, "Toggle Holding Tabs");
                if p.tabs.enabled {
                    let mut is_tri = matches!(p.tabs.profile, TabProfileKind::Triangular { .. });
                    f.set_tip("Ramped 3D triangular tabs maintain smooth XY feed motion");
                    f.row("Tab Profile", |ui, mutated| {
                        if ui
                            .selectable_value(&mut is_tri, true, "Triangular")
                            .clicked()
                        {
                            p.tabs.profile = TabProfileKind::Triangular {
                                ramp_angle_deg: 25.0,
                            };
                            *mutated = Some("Change Tab Profile");
                        }
                        if ui
                            .selectable_value(&mut is_tri, false, "Rectangular")
                            .clicked()
                        {
                            p.tabs.profile = TabProfileKind::Rectangular;
                            *mutated = Some("Change Tab Profile");
                        }
                    });
                    if let TabProfileKind::Triangular { ramp_angle_deg } = &mut p.tabs.profile {
                        f.set_tip("Ramp slope angle for 3D triangular holding tabs");
                        f.drag(
                            "Ramp Angle",
                            ramp_angle_deg,
                            5.0..=75.0,
                            (0.5, "°"),
                            "Edit Tab Ramp Angle",
                        );
                    }
                    f.set_tip("Bridge width measured along the contour perimeter");
                    f.len("Tab Width", &mut p.tabs.width, "Edit Tab Width");
                    f.set_tip("Bridge thickness measured upward from Bottom Z");
                    f.len("Tab Height", &mut p.tabs.height, "Edit Tab Height");

                    let mut is_manual = matches!(p.tabs.placement, TabPlacementMode::Manual);
                    f.set_tip("Distribute tabs evenly or click anchor points directly in 3D");
                    f.row("Placement", |ui, mutated| {
                        if ui
                            .selectable_value(&mut is_manual, false, "Automatic")
                            .clicked()
                        {
                            p.tabs.placement = TabPlacementMode::default();
                            if matches!(pick_mode, ViewportPickMode::PlaceContourTab { .. }) {
                                *pick_mode = ViewportPickMode::InspectEntity;
                            }
                            *mutated = Some("Set Automatic Tabs");
                        }
                        if ui
                            .selectable_value(&mut is_manual, true, "Manual")
                            .clicked()
                        {
                            p.tabs.placement = TabPlacementMode::Manual;
                            *pick_mode = ViewportPickMode::PlaceContourTab {
                                setup_idx: ids.0,
                                op_idx: ids.1,
                            };
                            *mutated = Some("Set Manual Tabs");
                        }
                    });

                    if let TabPlacementMode::Automatic {
                        count,
                        by_spacing,
                        target_spacing,
                    } = &mut p.tabs.placement
                    {
                        f.set_tip("Place tabs by target perimeter interval instead of fixed count");
                        f.toggle("By Spacing", by_spacing, "Toggle Tab Spacing Mode");
                        if *by_spacing {
                            f.set_tip("Target arc-length distance between holding tabs");
                            f.len("Tab Spacing", target_spacing, "Edit Tab Spacing");
                        } else {
                            f.set_tip("Number of evenly spaced holding tabs per closed contour");
                            f.drag("Tab Count", count, 1..=32, (0.25, ""), "Edit Tab Count");
                        }
                    }
                }
            });

            if p.tabs.enabled && matches!(p.tabs.placement, TabPlacementMode::Manual) {
                show_manual_tabs_footer(ui, p, ids, pick_mode, item.mutated_label);
            }
        },
    );
}

fn show_manual_tabs_footer(
    ui: &mut egui::Ui,
    p: &mut Contour2DParams,
    ids: (usize, usize),
    pick_mode: &mut ViewportPickMode,
    mutated_label: &mut Option<&'static str>,
) {
    ui.add_space(4.0);
    let target_mode = ViewportPickMode::PlaceContourTab {
        setup_idx: ids.0,
        op_idx: ids.1,
    };
    let picking_tabs = *pick_mode == target_mode;
    let btn_txt = if picking_tabs {
        "Stop 3D Tab Picking"
    } else {
        "Pick Tab Points in 3D Viewport"
    };
    if ui
        .button(btn_txt)
        .on_hover_text(if picking_tabs {
            "Exit interactive 3D holding-tab placement mode"
        } else {
            "Click along the selected contour in the 3D viewport to place manual holding tabs"
        })
        .clicked()
    {
        *pick_mode = if picking_tabs {
            ViewportPickMode::InspectEntity
        } else {
            target_mode
        };
    }
    ui.add_space(3.0);
    if picking_tabs {
        inline_banner(
            ui,
            BannerKind::Info,
            "Click along the selected contour in the 3D viewport to place holding tabs.",
        );
    } else if p.tabs.manual_anchors.is_empty() {
        inline_banner(ui, BannerKind::Warn, "No manual tab anchors placed yet.");
    }
    let mut del_anchor = None;
    for (a_idx, anchor) in p.tabs.manual_anchors.iter().enumerate() {
        ui.horizontal(|ui| {
            ui.weak(format!(
                "Tab #{} (Loop {}, s={:.1}%)",
                a_idx + 1,
                anchor.contour_index + 1,
                anchor.s_norm * 100.0
            ));
            if small_icon_button(ui, IconKind::Trash, "Remove this manual holding tab anchor")
                .clicked()
            {
                del_anchor = Some(a_idx);
            }
        });
    }
    if let Some(rm) = del_anchor {
        p.tabs.manual_anchors.remove(rm);
        *mutated_label = Some("Remove Manual Tab");
    }
}

pub(super) struct PocketParamsUiCtx<'a> {
    pub p: &'a mut Pocket2DParams,
    pub tool_feed_preview: Option<(f64, f64)>,
    pub display_unit: DisplayUnit,
    pub item: PassItemCtx<'a>,
}

pub(super) fn show_pocket2d_params(ui: &mut egui::Ui, ctx: PocketParamsUiCtx<'_>) {
    let PocketParamsUiCtx {
        p,
        tool_feed_preview,
        display_unit,
        mut item,
    } = ctx;
    let eval_ctx = item.eval_ctx;

    item.card_grid(ui, "passes_pocket_card", "2D Pocket Strategy", |f| {
        let cur_strat = match &p.strategy {
            PocketStrategyKind::ContourParallel => 0_u8,
            PocketStrategyKind::ZigZag { .. } => 1_u8,
            PocketStrategyKind::Adaptive(_) => 2_u8,
        };
        let mut sel_strat = cur_strat;
        f.set_tip("Offset rings, parallel raster, or constant-engagement 2D Adaptive");
        f.row("Clearing Mode", |ui, mutated| {
            if ui
                .selectable_value(&mut sel_strat, 0, "Offset")
                .on_hover_text(
                    "Contour-parallel inward/outward offset rings following pocket walls",
                )
                .clicked()
                && cur_strat != 0
            {
                p.strategy = PocketStrategyKind::ContourParallel;
                *mutated = Some("Select Contour-Parallel Pocket");
            }
            if ui
                .selectable_value(&mut sel_strat, 1, "Zig-Zag")
                .on_hover_text("Parallel linear back-and-forth raster clearing passes")
                .clicked()
                && cur_strat != 1
            {
                p.strategy = PocketStrategyKind::ZigZag {
                    pass_angle_deg: 0.0,
                };
                *mutated = Some("Select Zig-Zag Pocket");
            }
            if ui
                .selectable_value(&mut sel_strat, 2, "Adaptive")
                .on_hover_text(
                    "Constant radial engagement trochoidal clearing with micro-lift linking",
                )
                .clicked()
                && cur_strat != 2
            {
                let mut adaptive = AdaptiveClearingParams::default();
                adaptive.reevaluate(eval_ctx);
                p.strategy = PocketStrategyKind::Adaptive(adaptive);
                *mutated = Some("Select 2D Adaptive Clearing");
            }
        });

        if let PocketStrategyKind::ZigZag { pass_angle_deg } = &mut p.strategy {
            f.set_tip("XY orientation angle for Zig-Zag pocket clearing passes");
            f.drag(
                "Raster Angle",
                pass_angle_deg,
                -360.0..=360.0,
                (1.0, "°"),
                "Edit Pocket Raster Angle",
            );
        }

        if !matches!(p.strategy, PocketStrategyKind::Adaptive(_)) {
            f.set_tip("Radial width of cut (ae) between pocket clearing passes");
            f.len("Stepover", &mut p.stepover, "Edit Pocket Stepover");
            f.set_tip("Order offset passes Inside-Out (from center) or Outside-In");
            f.combo(
                "Pass Order",
                &mut p.pass_order,
                (&PocketPassOrder::ALL, |o| o.label()),
                "Change Pocket Pass Order",
            );
        }

        f.set_tip("Cutter extension past open pocket edges to eliminate corner cusps");
        f.len(
            "Overhang",
            &mut p.open_edge_overhang,
            "Edit Open-Edge Overhang",
        );
        f.set_tip("Run a dedicated final contour pass along pocket walls and islands");
        f.toggle("Wall Finish", &mut p.finish_wall_pass, "Toggle Finish Wall");
        if p.finish_wall_pass {
            f.set_tip("Radial allowance removed by the wall finishing pass");
            f.len(
                "Wall Allow",
                &mut p.finish_wall_stepover,
                "Edit Pocket Finish Allowance",
            );
        }
    });

    if let PocketStrategyKind::Adaptive(adaptive) = &mut p.strategy {
        show_adaptive_card(ui, adaptive, tool_feed_preview, display_unit, &mut item);
    }
}

fn show_adaptive_card(
    ui: &mut egui::Ui,
    adp: &mut AdaptiveClearingParams,
    tool_feed_preview: Option<(f64, f64)>,
    display_unit: DisplayUnit,
    item: &mut PassItemCtx<'_>,
) {
    let ids = item.ids();
    section_card(
        ui,
        ("adp_card", ids.0, ids.1),
        "2D Adaptive Clearing",
        true,
        |ui| {
            item.grid(ui, "passes_adaptive_grid", |f| {
                f.set_tip("Target maximum radial engagement (ae) maintained via trochoidal arcs");
                f.len("Optimal Load", &mut adp.optimal_load, "Edit Optimal Load");
                f.set_tip("Minimum trochoidal loop radius permitted in tight pockets and corners");
                f.len(
                    "Min Trochoid R",
                    &mut adp.min_trochoid_radius,
                    "Edit Min Trochoid Radius",
                );
                f.set_tip("Climb, Conventional, or Bi-Directional adaptive engagement");
                f.combo(
                    "Cut Direction",
                    &mut adp.cut_direction,
                    (&AdaptiveCutDirection::ALL, |d| d.label()),
                    "Change Adaptive Cut Direction",
                );
                f.set_tip("Small axial Z lift above floor during stay-down back-linking moves");
                f.len("Micro-Lift Z", &mut adp.micro_lift_z, "Edit Micro-Lift Z");
                f.set_tip("Feedrate multiplier applied during non-cutting stay-down links");
                f.drag(
                    "Stay-Down",
                    &mut adp.no_engagement_feed_multiplier,
                    1.0..=5.0,
                    (0.1, "x"),
                    "Edit Stay-Down Feed Multiplier",
                );
                f.set_tip("Boost cutting feedrate automatically to maintain target hex chipload");
                f.toggle("Chip Thinning", &mut adp.auto_chip_thinning, "Toggle RCT");
                if adp.auto_chip_thinning {
                    f.set_tip("Upper cap on radial chip-thinning feedrate multiplier");
                    f.drag(
                        "Max Boost",
                        &mut adp.max_thinning_feed_multiplier,
                        1.0..=3.5,
                        (0.05, "x"),
                        "Edit Max Chip-Thinning Multiplier",
                    );
                }
            });

            if let (true, Some((tool_d, nom_feed))) = (adp.auto_chip_thinning, tool_feed_preview) {
                let comp_feed = compute_adaptive_thinning_feed(
                    tool_d,
                    adp.optimal_load.value_mm,
                    nom_feed,
                    adp.max_thinning_feed_multiplier,
                );
                ui.add_space(4.0);
                inline_banner(
                    ui,
                    BannerKind::Info,
                    format!(
                        "Effective RCT-Compensated Feed: {:.0} {}",
                        display_unit.feed_from_mm_min(comp_feed),
                        display_unit.feed_suffix()
                    ),
                );
            }
        },
    );
}

pub(super) struct DrillBoreUiCtx<'a> {
    pub p: &'a mut DrillBoreParams,
    pub tool_diameter_mm: Option<f64>,
    pub radial_stock_mm: f64,
    pub primary_hole_diameter_mm: Option<f64>,
    pub item: PassItemCtx<'a>,
}

pub(super) fn show_drill_bore_params(ui: &mut egui::Ui, ctx: DrillBoreUiCtx<'_>) {
    let DrillBoreUiCtx {
        p,
        tool_diameter_mm,
        radial_stock_mm,
        primary_hole_diameter_mm,
        mut item,
    } = ctx;
    let tok = crate::theme::ThemeTokens::from_ui(ui);
    item.card_grid(ui, "passes_drill_card", "Drill & Helical Bore", |f| {
        f.set_tip("Auto-dispatch between axial drilling and helical boring by cutter diameter");
        f.combo(
            "Hole Mode",
            &mut p.mode,
            (&HoleMachiningMode::ALL, |m| m.label()),
            "Change Hole Machining Mode",
        );
        f.set_tip("Sort holes via Nearest-Neighbor + 2-Opt to minimize rapid travel");
        f.toggle(
            "2-Opt Sort",
            &mut p.optimize_hole_order,
            "Toggle Hole Order",
        );
        f.set_tip("Canned/expanded drilling cycle (G81 rapid out, G73 chip-break, G83 deep peck)");
        f.combo(
            "Drill Cycle",
            &mut p.cycle_kind,
            (&DrillCycleKind::ALL, |ck| ck.label()),
            "Change Drill Cycle Kind",
        );
        if matches!(
            p.cycle_kind,
            DrillCycleKind::Spot | DrillCycleKind::ChipBreak | DrillCycleKind::DeepPeck
        ) {
            f.set_tip("Bottom-of-hole dwell duration in seconds (G82 P dwell or chip-clearing pause)");
            f.drag(
                "Dwell (s)",
                &mut p.dwell_seconds,
                0.0..=30.0,
                (0.1, " s"),
                "Edit Drill Dwell",
            );
        }
        if matches!(
            p.cycle_kind,
            DrillCycleKind::ChipBreak | DrillCycleKind::DeepPeck
        ) {
            f.set_tip("Incremental axial peck step (Q) before chip-break or full retract");
            f.len("Peck Depth", &mut p.peck_depth, "Edit Peck Depth");
            if p.cycle_kind == DrillCycleKind::ChipBreak {
                f.set_tip("Partial Z retract distance between G73 chip-breaking pecks to snap stringy chips");
                f.len(
                    "Chip-Break Retract",
                    &mut p.chip_break_retract,
                    "Edit Chip-Break Retract",
                );
            }
            let mut use_dec = p.decreasing_peck_factor.is_some();
            f.set_tip("Progressively reduce peck depth with each step to improve chip evacuation in deep holes");
            f.row("Decreasing Peck", |ui, mutated| {
                if ui
                    .checkbox(&mut use_dec, "Enable")
                    .on_hover_text("Multiply each subsequent peck depth by a decay factor down to Min Peck Depth")
                    .changed()
                {
                    p.decreasing_peck_factor = if use_dec { Some(0.8) } else { None };
                    *mutated = Some("Toggle Decreasing Peck");
                }
            });
            if let Some(factor) = &mut p.decreasing_peck_factor {
                f.set_tip("Per-peck depth multiplier (0.25x to 1.0x) applied after each peck");
                f.drag(
                    "Peck Decay Factor",
                    factor,
                    0.25..=1.0,
                    (0.02, "x"),
                    "Edit Decreasing Peck Factor",
                );
                f.set_tip("Minimum axial peck depth floor when decreasing peck reduction is enabled");
                f.len("Min Peck Depth", &mut p.min_peck_depth, "Edit Min Peck Depth");
            }
        }
        f.set_tip("Extend drill depth by drill cone height so full diameter clears bottom");
        f.toggle(
            "Breakthrough",
            &mut p.tip_breakthrough,
            "Toggle Breakthrough",
        );
        if p.tip_breakthrough {
            f.set_tip("Additional breakthrough clearance past full drill diameter exit");
            f.len(
                "Extra Depth",
                &mut p.breakthrough_extra,
                "Edit Breakthrough Extra",
            );
        }
        let mut use_fixed_pitch = p.bore_pitch_per_rev.is_some();
        f.set_tip("Override helical ramp angle with an explicit axial Z drop per full 360-degree revolution");
        f.row("Fixed Bore Pitch", |ui, mutated| {
            if ui
                .checkbox(&mut use_fixed_pitch, "Override Pitch/Rev")
                .on_hover_text("Specify an exact Z drop per helical revolution instead of computing from ramp angle")
                .changed()
            {
                p.bore_pitch_per_rev = if use_fixed_pitch {
                    Some(ParamLength::from_mm(1.0))
                } else {
                    None
                };
                *mutated = Some("Toggle Bore Pitch Override");
            }
        });
        if let Some(pitch) = &mut p.bore_pitch_per_rev {
            f.set_tip("Axial Z descent per full 360-degree helical revolution (overrides Helical Pitch angle)");
            f.len("Pitch / Rev", pitch, "Edit Bore Pitch Per Rev");
        } else {
            f.set_tip("Helical ramp angle (°) when interpolating holes larger than cutter diameter");
            f.drag(
                "Helical Pitch",
                &mut p.bore_ramp_angle_deg,
                0.5..=15.0,
                (0.2, "°"),
                "Edit Bore Ramp Angle",
            );
        }
        f.set_tip("Add a full-circle radial finishing pass at hole bottom");
        f.toggle("Bore Finish", &mut p.bore_finish_pass, "Toggle Bore Finish");
        if p.bore_finish_pass {
            f.set_tip("Radial stock allowance left for the final full-circle helical bore finishing pass");
            f.len(
                "Bore Finish Step",
                &mut p.bore_finish_stepover,
                "Edit Bore Finish Stepover",
            );
        }
        f.set_tip("Repeat circular finishing pass to eliminate radial cutter deflection");
        f.toggle("Bore Spring", &mut p.bore_spring_pass, "Toggle Bore Spring");

        if p.mode != HoleMachiningMode::Drill {
            let amp_opt = match (primary_hole_diameter_mm, tool_diameter_mm) {
                (Some(hole_d), Some(tool_d))
                    if hole_d.is_finite() && tool_d.is_finite() && hole_d > tool_d + 0.04 =>
                {
                    let r_hole = 0.5 * hole_d;
                    let r_orbit = (r_hole - 0.5 * tool_d - radial_stock_mm.max(0.0)).max(0.02);
                    Some((r_hole / r_orbit, (r_orbit / r_hole).clamp(0.20, 1.0)))
                }
                _ => None,
            };
            f.set_tip(
                "Scale helical bore tool-center feedrate by (r_orbit / R_hole) so cutting-edge wall chipload matches programmed feed",
            );
            f.row("Inner-Arc Feed Comp", |ui, mutated| {
                if ui
                    .checkbox(&mut p.bore_inner_arc_feed_comp, "Enabled")
                    .on_hover_text(
                        "Prevent concave internal-bore wall chipload overload by scaling centerline feed by r_orbit / R_hole",
                    )
                    .changed()
                {
                    *mutated = Some("Toggle Helical Inner-Arc Feed Comp");
                }
                if let Some((amp, scale)) = amp_opt {
                    let (txt, col) = if p.bore_inner_arc_feed_comp {
                        (
                            format!("R_hole/r_orbit {amp:.2}x (F x{scale:.2})"),
                            tok.clean_fg,
                        )
                    } else if amp > 1.35 {
                        (format!("R_hole/r_orbit {amp:.2}x!"), tok.warn_fg)
                    } else {
                        (format!("R_hole/r_orbit {amp:.2}x"), tok.text_secondary)
                    };
                    ui.label(egui::RichText::new(txt).small().monospace().color(col));
                }
            });

            let mut use_chip_clear = p.bore_chip_clear_depth.is_some();
            f.set_tip(
                "Periodically retract helical bore out to R-plane to evacuate chips in deep holes",
            );
            f.row("Chip-Clear Retract", |ui, mutated| {
                if ui
                    .checkbox(&mut use_chip_clear, "Enable")
                    .on_hover_text(
                        "Insert full Z retracts to R-plane every Clear Depth interval during deep helical boring",
                    )
                    .changed()
                {
                    p.bore_chip_clear_depth = if use_chip_clear {
                        let def_mm = primary_hole_diameter_mm
                            .map(|hd| (2.0 * hd).clamp(3.0, 10.0))
                            .unwrap_or(6.0);
                        Some(ParamLength::from_mm(def_mm))
                    } else {
                        None
                    };
                    *mutated = Some("Toggle Bore Chip-Clear Retract");
                }
            });
            if let Some(clear_depth) = &mut p.bore_chip_clear_depth {
                f.set_tip("Axial depth interval between full chip-evacuation retracts to R-plane");
                f.len(
                    "Clear Depth",
                    clear_depth,
                    "Edit Bore Chip-Clear Retract Depth",
                );
            }
        }
    });
}

pub(super) fn show_chamfer_trace_params(
    ui: &mut egui::Ui,
    p: &mut ChamferTraceParams,
    mut item: PassItemCtx<'_>,
) {
    item.card_grid(ui, "passes_chamfer_card", "Chamfer & 2D Curve Trace", |f| {
        f.set_tip("Deburr/bevel 2.5D edges with a tapered cutter or trace 2D curves axially");
        f.combo("Mode", &mut p.mode, (&ChamferTraceMode::ALL, |m| m.label()), "Change Chamfer/Trace Mode");

        match p.mode {
            ChamferTraceMode::EdgeChamfer => {
                let mut is_modeled = matches!(p.chamfer_source, ChamferSourceMode::ModeledFace);
                f.set_tip("Cut bevel on a sharp 90° CAD edge or follow a modeled chamfer face");
                f.row("CAD Source", |ui, mutated| {
                    if ui
                        .selectable_value(&mut is_modeled, false, "Sharp Edge")
                        .on_hover_text("Cut a specified chamfer width into an un-beveled 90-degree CAD edge")
                        .clicked()
                    {
                        p.chamfer_source = ChamferSourceMode::SharpEdge {
                            width: ParamLength::from_mm(0.5),
                        };
                        *mutated = Some("Select Sharp Edge Chamfer");
                    }
                    if ui
                        .selectable_value(&mut is_modeled, true, "Modeled Bevel")
                        .on_hover_text("Follow the bevel width already modeled on the 3D CAD chamfer face")
                        .clicked()
                    {
                        p.chamfer_source = ChamferSourceMode::ModeledFace;
                        *mutated = Some("Select Modeled Chamfer Face");
                    }
                });
                if let ChamferSourceMode::SharpEdge { width } = &mut p.chamfer_source {
                    f.set_tip("Horizontal bevel width (w) cut into a sharp 90° edge");
                    f.len("Chamfer Width", width, "Edit Chamfer Width");
                }
                f.set_tip("Distance cutter tip extends past lower chamfer edge so angled flutes cut cleanly");
                f.len("Tip Offset", &mut p.tip_offset, "Edit Chamfer Tip Offset");
                f.set_tip("Automatically check and prevent chamfer tip from gouging adjacent lower floors");
                f.toggle("Floor Guard", &mut p.check_adjacent_walls, "Toggle Floor Guard");
            }
            ChamferTraceMode::CurveTrace => {
                f.set_tip("Cutter compensation relative to traced 2D curve (Center for engraving)");
                f.combo("Compensation", &mut p.trace_compensation, (&ContourCompensation::ALL, |c| c.label()), "Change Trace Compensation");
                f.set_tip("Engraving or slotting depth below the traced curve");
                f.len("Axial Depth", &mut p.trace_axial_offset, "Edit Trace Axial Depth");
            }
        }
    });
}
