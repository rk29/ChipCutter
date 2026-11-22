use std::sync::Arc;

use chip_math::{DisplayUnit, MM_PER_INCH};
use chip_sim::{ScrubSnapshot, SimEngineKind, SimResolution, format_cycle_time_hms};

use super::{
    SimPanelCtx, SimTriggerArgs, SimulationPanelState, SimulationUiAction,
    metrology_ui::show_error_metrics_dashboard,
};

impl SimulationPanelState {
    /// Renders the bottom `[Simulation Timeline]` workspace tab (`06-simulation.md` §1–4).
    pub fn show_ui(
        &mut self,
        ui: &mut egui::Ui,
        ctx: SimPanelCtx<'_>,
    ) -> Option<SimulationUiAction> {
        let SimPanelCtx {
            project,
            setup_idx,
            cad_doc,
            cad_revision,
            cache,
            project_dir,
            logs,
        } = ctx;
        self.repaint_ctx = Some(ui.ctx().clone());
        let mut action: Option<SimulationUiAction> = None;
        let display_unit = project.display_units;
        let prev_time_sec = self.current_time_sec;
        let prev_vis_flags = (
            self.show_machined_stock,
            self.color_by_operation,
            self.show_error_heatmap,
            self.heatmap_tolerance_mm.to_bits(),
            self.show_cutter_assembly,
            self.trail_up_to_cutter,
        );

        // Top Control Bar: Run Simulation, Engine, Resolution, Visual Toggles, Verification Status
        let mut engine_changed = false;
        ui.horizontal_wrapped(|ui| {
            let busy = self.is_busy();
            let btn_lbl = if busy {
                "Simulating…"
            } else if self.active_result.is_some() {
                "Refresh Simulation"
            } else {
                "Run Simulation"
            };
            if ui
                .add_enabled(!busy && cad_doc.is_some(), egui::Button::new(btn_lbl))
                .on_hover_text(
                    "Run background stock removal simulation, 5-rule crash/gouge checks, and error-volume/RMS metrology",
                )
                .clicked()
                && let Some(doc) = cad_doc
            {
                self.trigger_simulation(SimTriggerArgs {
                    project,
                    setup_idx,
                    cad_doc: Arc::clone(doc),
                    cad_revision,
                    cache,
                    project_dir,
                    egui_ctx: ui.ctx().clone(),
                    logs,
                });
            }
            if busy {
                ui.spinner();
                if ui
                    .small_button("Cancel")
                    .on_hover_text("Cancel the in-flight background simulation")
                    .clicked()
                {
                    self.cancel_in_flight();
                }
            }

            ui.separator();
            let prev_engine = self.engine;
            egui::ComboBox::from_id_salt("sim_engine_combo")
                .selected_text(self.engine.label())
                .width(172.0)
                .show_ui(ui, |ui| {
                    for eng in SimEngineKind::ALL {
                        ui.selectable_value(&mut self.engine, eng, eng.label())
                            .on_hover_text(eng.description());
                    }
                })
                .response
                .on_hover_text(format!(
                    "Simulation engine (persisted in prefs.toml).\n\nActive: {} -- {}",
                    self.engine.label(),
                    self.engine.description()
                ));
            if self.engine != prev_engine {
                engine_changed = true;
            }

            let res_tooltip = match self.engine {
                SimEngineKind::Heightmap => format!(
                    "Heightmap XY grid resolution (cells along longest XY stock axis). Higher values give sharper detail and more accurate error metrics at the cost of CPU/RAM.\n\nActive: {} -- {}",
                    self.resolution.label_for_engine(self.engine),
                    self.resolution.description_for_engine(self.engine)
                ),
                SimEngineKind::TriDexel => format!(
                    "Tri-Dexel 3D lattice resolution (isotropic cells along longest X/Y/Z stock axis across all 3 orthogonal ray planes). Higher values give sharper 3D edges and tighter error-volume/RMS metrology at the cost of CPU/RAM.\n\nActive: {} -- {}",
                    self.resolution.label_for_engine(self.engine),
                    self.resolution.description_for_engine(self.engine)
                ),
            };
            egui::ComboBox::from_id_salt("sim_res_combo")
                .selected_text(self.resolution.label_for_engine(self.engine))
                .width(124.0)
                .show_ui(ui, |ui| {
                    for r in SimResolution::ALL {
                        ui.selectable_value(
                            &mut self.resolution,
                            r,
                            r.label_for_engine(self.engine),
                        )
                        .on_hover_text(r.description_for_engine(self.engine));
                    }
                })
                .response
                .on_hover_text(res_tooltip);

            ui.checkbox(&mut self.show_machined_stock, "3D Stock")
                .on_hover_text("Show or hide the simulated 3D machined stock solid in the viewport");
            ui.add_enabled(
                !self.show_error_heatmap,
                egui::Checkbox::new(&mut self.color_by_operation, "Color by Op"),
            )
            .on_hover_text(
                "Color machined stock faces by which operation cut them (disabled while Error Heatmap is active)",
            )
            .on_disabled_hover_text(
                "Uncheck Error Heatmap to color cuts by operation instead of signed deviation",
            );
            ui.checkbox(&mut self.show_error_heatmap, "Error Heatmap")
                .on_hover_text(
                    "Toggle signed stock-vs-target deviation heatmap on the 3D stock (Red = Gouge, Green = Within Tolerance, Blue = Uncut Excess). Uncheck for fastest interactive display.",
                );
            ui.checkbox(&mut self.show_cutter_assembly, "3D Cutter")
                .on_hover_text("Show or hide the animated 3D tool, shank, and holder assembly");
            ui.checkbox(&mut self.trail_up_to_cutter, "Progressive Trail")
                .on_hover_text(
                    "Show only the toolpath segments executed up to the current timeline position",
                );

            ui.separator();
            let tok = crate::theme::ThemeTokens::from_ui(ui);
            let up_to_date = self.is_verified_up_to_date(project, setup_idx, cache, cad_revision);
            if let Some(res) = &self.active_result
                && self.active_setup_idx == setup_idx
            {
                let crit = res.critical_error_count();
                let warn = res.warning_count();
                if crit > 0 {
                    crate::widgets::status_badge_icon(
                        ui,
                        crate::widgets::StatusBadgeKind::Error,
                        None,
                    );
                    if ui
                        .button(
                            egui::RichText::new(format!("{crit} CRASH/GOUGE(S) — Inspect"))
                                .color(tok.error_fg)
                                .strong(),
                        )
                        .clicked()
                    {
                        action = Some(SimulationUiAction::FocusDiagnosticsTab);
                    }
                } else if !up_to_date {
                    crate::widgets::status_badge_icon(
                        ui,
                        crate::widgets::StatusBadgeKind::Dirty,
                        None,
                    );
                    ui.colored_label(tok.dirty_fg, "Toolpaths changed — Refresh Simulation");
                } else if warn > 0 {
                    crate::widgets::status_badge_icon(
                        ui,
                        crate::widgets::StatusBadgeKind::NeedsRepick,
                        None,
                    );
                    if ui
                        .button(
                            egui::RichText::new(format!("0 Crashes ({warn} Warning)"))
                                .color(tok.warn_fg),
                        )
                        .clicked()
                    {
                        action = Some(SimulationUiAction::FocusDiagnosticsTab);
                    }
                } else {
                    crate::widgets::status_badge_icon(
                        ui,
                        crate::widgets::StatusBadgeKind::Clean,
                        None,
                    );
                    ui.colored_label(tok.clean_fg, "Verified Crash-Free");
                }
            } else {
                ui.weak("Click 'Run Simulation' to compute 3D material removal & crash checks.");
            }
        });

        if engine_changed
            && !self.is_busy()
            && self.active_result.is_some()
            && let Some(doc) = cad_doc
        {
            self.trigger_simulation(SimTriggerArgs {
                project,
                setup_idx,
                cad_doc: Arc::clone(doc),
                cad_revision,
                cache,
                project_dir,
                egui_ctx: ui.ctx().clone(),
                logs,
            });
        }

        ui.separator();

        let Some(res) = self.active_result.clone() else {
            ui.weak("No stock simulation active for this Setup yet.");
            return action;
        };
        if self.active_setup_idx != setup_idx || res.timeline.is_empty() {
            ui.weak("Click 'Run Simulation' to simulate the currently selected Setup.");
            return action;
        }

        let snap: Option<ScrubSnapshot> = res.timeline.sample_at_time(self.current_time_sec);
        let cur_seg = snap.as_ref().map_or(0, |s| s.global_seg_idx);
        let active_err_report = self.active_error_report().cloned();

        egui::ScrollArea::vertical()
            .id_salt("sim_timeline_scroll")
            .auto_shrink([false, false])
            .show(ui, |ui| {
                // Row 2: Vector Transport Controls + Speed + Continuous Scrubber Slider on one sleek bar
                ui.horizontal(|ui| {
                    use crate::widgets::{IconKind, icon_button};
                    if icon_button(ui, IconKind::SkipStart, None, "Jump to Start").clicked() {
                        self.is_playing = false;
                        self.current_time_sec = 0.0;
                    }
                    if icon_button(
                        ui,
                        IconKind::StepBack,
                        None,
                        "Step Backward 1 Segment (Left Arrow)",
                    )
                    .clicked()
                    {
                        self.is_playing = false;
                        self.current_time_sec = res.timeline.step_segment_time(cur_seg, -1);
                    }
                    let (play_icon, play_txt) = if self.is_playing {
                        (IconKind::Pause, "Pause")
                    } else {
                        (IconKind::Play, "Play")
                    };
                    if icon_button(
                        ui,
                        play_icon,
                        Some(play_txt),
                        "Play / Pause Scrubbing (Space)",
                    )
                    .clicked()
                    {
                        if !self.is_playing
                            && self.current_time_sec >= res.timeline.total_time_sec - 1e-4
                        {
                            self.current_time_sec = 0.0;
                        }
                        self.is_playing = !self.is_playing;
                    }
                    if icon_button(
                        ui,
                        IconKind::StepForward,
                        None,
                        "Step Forward 1 Segment (Right Arrow)",
                    )
                    .clicked()
                    {
                        self.is_playing = false;
                        self.current_time_sec = res.timeline.step_segment_time(cur_seg, 1);
                    }
                    if icon_button(
                        ui,
                        IconKind::SkipNextOp,
                        Some("Next Op"),
                        "Jump to Next Operation",
                    )
                    .clicked()
                    {
                        self.is_playing = false;
                        self.current_time_sec =
                            res.timeline.next_operation_time(self.current_time_sec);
                    }
                    if icon_button(
                        ui,
                        IconKind::Tool,
                        Some("Next Tool"),
                        "Jump to Next Tool Change",
                    )
                    .clicked()
                    {
                        self.is_playing = false;
                        self.current_time_sec =
                            res.timeline.next_tool_change_time(self.current_time_sec);
                    }
                    if icon_button(
                        ui,
                        IconKind::SkipEnd,
                        Some("End"),
                        "Instant Jump to Final Machined Stock",
                    )
                    .clicked()
                    {
                        self.is_playing = false;
                        self.current_time_sec = res.timeline.total_time_sec;
                    }

                    ui.separator();
                    egui::ComboBox::from_id_salt("sim_speed_combo")
                        .selected_text(format!("{}x", self.speed_multiplier))
                        .width(52.0)
                        .show_ui(ui, |ui| {
                            for spd in [0.1, 0.5, 1.0, 5.0, 10.0, 25.0, 50.0] {
                                ui.selectable_value(
                                    &mut self.speed_multiplier,
                                    spd,
                                    format!("{spd}x"),
                                )
                                .on_hover_text(format!(
                                    "Play simulation at {spd}x real machining speed"
                                ));
                            }
                        })
                        .response
                        .on_hover_text("Simulation playback speed multiplier");

                    ui.selectable_value(&mut self.scrub_by_distance, false, "Time")
                        .on_hover_text("Scrub simulation timeline by elapsed machining time");
                    ui.selectable_value(&mut self.scrub_by_distance, true, "Dist")
                        .on_hover_text("Scrub simulation timeline by linear toolpath distance");
                    ui.separator();

                    ui.spacing_mut().slider_width = (ui.available_width() - 165.0).max(96.0);
                    if self.scrub_by_distance {
                        let mut cur_d = snap.as_ref().map_or(0.0, |s| s.distance_mm);
                        let max_d = res.timeline.total_distance_mm.max(0.001);
                        let resp = ui
                            .add(
                                egui::Slider::new(&mut cur_d, 0.0..=max_d)
                                    .show_value(false)
                                    .trailing_fill(true),
                            )
                            .on_hover_text(
                                "Scrub simulation progress by cumulative tool travel distance",
                            );
                        if resp.changed() {
                            self.is_playing = false;
                            if let Some(ds) = res.timeline.sample_at_distance(cur_d) {
                                self.current_time_sec = ds.time_sec;
                            }
                        }
                        ui.monospace(format!(
                            "{} / {}",
                            display_unit.format_length(cur_d),
                            display_unit.format_length(max_d)
                        ));
                    } else {
                        let max_t = res.timeline.total_time_sec.max(0.001);
                        let resp = ui
                            .add(
                                egui::Slider::new(&mut self.current_time_sec, 0.0..=max_t)
                                    .show_value(false)
                                    .trailing_fill(true),
                            )
                            .on_hover_text("Scrub simulation progress by elapsed cycle time");
                        if resp.changed() {
                            self.is_playing = false;
                        }
                        ui.monospace(format!(
                            "{} / {}",
                            format_cycle_time_hms(self.current_time_sec),
                            format_cycle_time_hms(res.timeline.total_time_sec)
                        ));
                    }
                });

                // Bottom Live Controller HUD Readout (`Op`, `Tool`, `WCS X/Y/Z`, `G0..G3`, `Feed`, `RPM`, `Seg`)
                if let Some(s) = &snap {
                    if let Some(tc) = &s.tool_change {
                        let tok = crate::theme::ThemeTokens::from_ui(ui);
                        ui.horizontal_wrapped(|ui| {
                            ui.colored_label(
                                tok.warn_fg,
                                egui::RichText::new(format!(
                                    "M6 Tool Change: T{} -> T{} ({}) · {:.1}s left",
                                    tc.from_tool_number,
                                    tc.to_tool_number,
                                    tc.to_tool_name,
                                    tc.remaining_sec
                                ))
                                .strong(),
                            );
                            ui.separator();
                            ui.label(format!(
                                "Next: Op {}: {}",
                                tc.next_op_idx + 1,
                                tc.next_op_name
                            ));
                            ui.separator();
                            if let Some(cyl) = res.as_cylindrical() {
                                ui.monospace(format!(
                                    "WCS X={} Y={} Z={} {}={:.1}°",
                                    display_unit.format_length(s.tip_pos_wcs.x),
                                    display_unit.format_length(s.tip_pos_wcs.y),
                                    display_unit.format_length(s.tip_pos_wcs.z),
                                    cyl.rotary.axis_letter,
                                    s.rotary_deg,
                                ));
                            } else {
                                ui.monospace(format!(
                                    "WCS X={} Y={} Z={}",
                                    display_unit.format_length(s.tip_pos_wcs.x),
                                    display_unit.format_length(s.tip_pos_wcs.y),
                                    display_unit.format_length(s.tip_pos_wcs.z),
                                ));
                            }
                            if ui
                                .small_button("Skip Tool Change")
                                .on_hover_text(
                                    "Jump immediately to the first move of the next operation",
                                )
                                .clicked()
                            {
                                self.current_time_sec = tc.end_time_sec;
                            }
                        });
                    } else {
                        let feed_disp = match display_unit {
                            DisplayUnit::Millimeter => {
                                format!("{:.0} mm/min", s.commanded_feed_mm_min)
                            }
                            DisplayUnit::Inch => {
                                format!("{:.1} ipm", s.commanded_feed_mm_min / MM_PER_INCH)
                            }
                        };
                        ui.horizontal_wrapped(|ui| {
                            ui.strong(format!("Op {}: {}", s.op_idx + 1, s.op_name));
                            ui.separator();
                            ui.monospace(format!("T{}", s.tool_number))
                                .on_hover_text(&s.tool_name);
                            ui.separator();
                            if let Some(cyl) = res.as_cylindrical() {
                                ui.monospace(format!(
                                    "X={} Y={} Z={} {}={:.1}°",
                                    display_unit.format_length(s.tip_pos_wcs.x),
                                    display_unit.format_length(s.tip_pos_wcs.y),
                                    display_unit.format_length(s.tip_pos_wcs.z),
                                    cyl.rotary.axis_letter,
                                    s.rotary_deg,
                                ));
                            } else {
                                ui.monospace(format!(
                                    "X={} Y={} Z={}",
                                    display_unit.format_length(s.tip_pos_wcs.x),
                                    display_unit.format_length(s.tip_pos_wcs.y),
                                    display_unit.format_length(s.tip_pos_wcs.z),
                                ));
                            }
                            ui.separator();
                            ui.monospace(format!(
                                "{} | F={} | S={:.0} | Seg {}/{}",
                                s.motion_code,
                                feed_disp,
                                s.spindle_rpm,
                                s.global_seg_idx + 1,
                                res.timeline.motion_spans.len()
                            ));
                            if ui
                                .small_button("Center")
                                .on_hover_text("Center camera on cutter tip")
                                .clicked()
                            {
                                action = Some(SimulationUiAction::CenterCameraWcs(s.tip_pos_wcs));
                            }
                        });
                    }
                }

                // Quantitative Stock-vs-Target Error-Volume & RMS Metrics Dashboard (Phase 11 M40)
                if let Some(rep) = &active_err_report
                    && let Some(metric_act) = show_error_metrics_dashboard(
                        ui,
                        &res,
                        rep,
                        snap.as_ref(),
                        &mut self.heatmap_tolerance_mm,
                        self.heatmap_max_scale_mm,
                        &mut self.show_error_heatmap,
                    )
                {
                    action = Some(metric_act);
                }
            });

        if (self.current_time_sec - prev_time_sec).abs() > 1e-9
            || (
                self.show_machined_stock,
                self.color_by_operation,
                self.show_error_heatmap,
                self.heatmap_tolerance_mm.to_bits(),
                self.show_cutter_assembly,
                self.trail_up_to_cutter,
            ) != prev_vis_flags
        {
            ui.ctx().request_repaint();
        }

        action
    }
}
