use std::sync::Arc;

use chip_math::{DisplayUnit, MM_PER_INCH};
use chip_sim::{
    ScrubSnapshot, SimComputeBackendChoice, SimEngineKind, SimResolution, format_cycle_time_hms,
};

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

        // Top Control Bar: Run Simulation, Engine, Resolution, Backend, Adapter Pill, Visual Toggles, Verification Status
        let mut engine_changed = false;
        let mut backend_changed = false;
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
                    "Run background stock removal simulation, 6-rule crash/gouge/fixture checks, and error-volume/RMS metrology",
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

            let prev_backend = self.backend_choice;
            let backend_short_label = |choice: SimComputeBackendChoice| -> &'static str {
                match choice {
                    SimComputeBackendChoice::Auto => "Auto (GPU)",
                    SimComputeBackendChoice::Gpu => "GPU",
                    SimComputeBackendChoice::Cpu => "CPU",
                }
            };
            let backend_item_label = |choice: SimComputeBackendChoice| -> &'static str {
                match choice {
                    SimComputeBackendChoice::Auto => "Auto (GPU)",
                    SimComputeBackendChoice::Gpu => "GPU (Compute)",
                    SimComputeBackendChoice::Cpu => "CPU (Exact)",
                }
            };
            let backend_item_desc = |choice: SimComputeBackendChoice| -> &'static str {
                match choice {
                    SimComputeBackendChoice::Auto => {
                        "Automatically run simulation on GPU compute (wgpu) when available, falling back transparently to exact CPU + Rayon otherwise."
                    }
                    SimComputeBackendChoice::Gpu => {
                        "Prefer GPU compute (wgpu) simulation and zero-readback direct GPU rendering (falls back to CPU if no compute adapter is available)."
                    }
                    SimComputeBackendChoice::Cpu => {
                        "Force exact CPU + Rayon simulation and CPU-tessellated stock rendering (useful for cross-checking GPU results)."
                    }
                }
            };
            egui::ComboBox::from_id_salt("sim_backend_combo")
                .selected_text(backend_short_label(self.backend_choice))
                .width(88.0)
                .show_ui(ui, |ui| {
                    for choice in SimComputeBackendChoice::ALL {
                        ui.selectable_value(
                            &mut self.backend_choice,
                            choice,
                            backend_item_label(choice),
                        )
                        .on_hover_text(backend_item_desc(choice));
                    }
                })
                .response
                .on_hover_text(format!(
                    "Simulation compute backend (persisted in prefs.toml).\n\nActive setting: {} -- {}",
                    backend_item_label(self.backend_choice),
                    backend_item_desc(self.backend_choice)
                ));
            if self.backend_choice != prev_backend {
                backend_changed = true;
            }

            let tok = crate::theme::ThemeTokens::from_ui(ui);
            let (pill_text, pill_bg, pill_border, pill_fg, pill_tooltip) =
                if let Some(res) = &self.active_result
                    && self.active_setup_idx == setup_idx
                {
                    let t = res.timing;
                    let timing_line = format!(
                        "Last run: {:.1} ms (setup {:.1} ms · sweep {:.1} ms · metrics {:.1} ms) · {} segs ({:.0} seg/s)",
                        t.total_ms,
                        t.target_setup_ms,
                        t.sweep_ms,
                        t.metrics_ms,
                        t.total_segments,
                        t.segments_per_sec,
                    );
                    if res.active_backend.is_gpu {
                        let short_name = shorten_adapter_name(&res.active_backend.adapter_name);
                        (
                            format!("GPU · {short_name}"),
                            tok.info_bg,
                            tok.info_border,
                            tok.info_fg,
                            format!(
                                "Active simulation backend: GPU Compute ({})\nAdapter: {}\n{}\nZero-readback direct GPU stock rendering and instant Error Heatmap active.",
                                res.active_backend.backend_api,
                                res.active_backend.adapter_name,
                                timing_line
                            ),
                        )
                    } else if self.backend_choice == SimComputeBackendChoice::Cpu {
                        (
                            "CPU (Exact)".to_string(),
                            tok.card_bg,
                            tok.border_strong,
                            tok.text_secondary,
                            format!(
                                "Active simulation backend: CPU Exact (Rayon)\n{}\nExact CPU volumetric kernel and CPU-tessellated stock mesh active.",
                                timing_line
                            ),
                        )
                    } else {
                        (
                            "CPU Fallback".to_string(),
                            tok.warn_bg,
                            tok.warn_border,
                            tok.warn_fg,
                            format!(
                                "Active simulation backend: CPU Fallback (Rayon)\nGPU compute was unavailable or disabled; ran exact CPU simulation automatically.\n{}",
                                timing_line
                            ),
                        )
                    }
                } else if self.backend_choice == SimComputeBackendChoice::Cpu {
                    (
                        "CPU (Exact)".to_string(),
                        tok.card_bg,
                        tok.border_strong,
                        tok.text_secondary,
                        "Configured backend: CPU Exact (Rayon). Next simulation will execute on the exact CPU volumetric kernel."
                            .to_string(),
                    )
                } else if let Some(info) = self.gpu_adapter_info() {
                    let short_name = shorten_adapter_name(&info.adapter_name);
                    (
                        format!("GPU · {short_name}"),
                        tok.info_bg,
                        tok.info_border,
                        tok.info_fg,
                        format!(
                            "GPU Compute adapter ready ({})\nAdapter: {}\nNext simulation will execute on GPU with zero-readback rendering.",
                            info.backend_api, info.adapter_name
                        ),
                    )
                } else {
                    (
                        "CPU Fallback".to_string(),
                        tok.warn_bg,
                        tok.warn_border,
                        tok.warn_fg,
                        "No GPU compute adapter is active; simulation will fall back transparently to exact CPU + Rayon."
                            .to_string(),
                    )
                };
            egui::Frame::NONE
                .fill(pill_bg)
                .stroke(egui::Stroke::new(1.0, pill_border))
                .corner_radius(egui::CornerRadius::same(4))
                .inner_margin(egui::Margin::symmetric(6, 2))
                .show(ui, |ui| {
                    ui.label(
                        egui::RichText::new(pill_text)
                            .small()
                            .strong()
                            .color(pill_fg),
                    )
                    .on_hover_text(pill_tooltip);
                });

            if let Some(res) = &self.active_result
                && self.active_setup_idx == setup_idx
                && let Some(src_name) = &res.carried_over_from_setup
            {
                let carry_tip = if res.engine == SimEngineKind::TriDexel {
                    format!(
                        "Stock carried over from '{src_name}' via exact rigid-frame Tri-Dexel transfer.\nTimeline t = 0.0 starts from '{src_name}' remaining 3D solid (preserving underside cavities and cumulative error metrology)."
                    )
                } else {
                    format!(
                        "Stock carried over from '{src_name}' (projected top Z envelope in Heightmap mode).\nSwitch to Tri-Dexel (Volumetric 3D) to preserve underside cavities and flipped 3D pockets."
                    )
                };
                let short_src = if src_name.chars().count() > 18 {
                    let mut s: String = src_name.chars().take(17).collect();
                    s.push('…');
                    s
                } else {
                    src_name.clone()
                };
                egui::Frame::NONE
                    .fill(tok.info_bg)
                    .stroke(egui::Stroke::new(1.0, tok.info_border))
                    .corner_radius(egui::CornerRadius::same(4))
                    .inner_margin(egui::Margin::symmetric(6, 2))
                    .show(ui, |ui| {
                        ui.label(
                            egui::RichText::new(format!("Carried over from {short_src}"))
                                .small()
                                .strong()
                                .color(tok.info_fg),
                        )
                        .on_hover_text(carry_tip);
                    });
            }

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
                    ui.horizontal(|ui| {
                        crate::widgets::status_badge_icon(
                            ui,
                            crate::widgets::StatusBadgeKind::Clean,
                            None,
                        );
                        ui.colored_label(tok.clean_fg, "Verified Crash-Free");
                    });
                }
            } else {
                ui.weak("Click 'Run Simulation' to compute 3D material removal & crash checks.");
            }
        });

        if (engine_changed || backend_changed)
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
                            let feed_hud_tooltip = if res.timeline.max_cutting_feed_mm_min > 0.0 {
                                let (min_f, max_f, unit_lbl) = match display_unit {
                                    DisplayUnit::Millimeter => (
                                        format!("{:.0}", res.timeline.min_cutting_feed_mm_min),
                                        format!("{:.0}", res.timeline.max_cutting_feed_mm_min),
                                        "mm/min",
                                    ),
                                    DisplayUnit::Inch => (
                                        format!(
                                            "{:.1}",
                                            res.timeline.min_cutting_feed_mm_min / MM_PER_INCH
                                        ),
                                        format!(
                                            "{:.1}",
                                            res.timeline.max_cutting_feed_mm_min / MM_PER_INCH
                                        ),
                                        "ipm",
                                    ),
                                };
                                format!(
                                    "Active motion block, commanded feedrate, spindle RPM, and segment index.\nSetup cutting feed range: {min_f}–{max_f} {unit_lbl} ({} modulated cutting segments)",
                                    res.timeline.feed_modulated_segments
                                )
                            } else {
                                "Active motion block, commanded feedrate, spindle RPM, and segment index"
                                    .to_string()
                            };
                            ui.monospace(format!(
                                "{} | F={} | S={:.0} | Seg {}/{}",
                                s.motion_code,
                                feed_disp,
                                s.spindle_rpm,
                                s.global_seg_idx + 1,
                                res.timeline.motion_spans.len()
                            ))
                            .on_hover_text(feed_hud_tooltip);
                            if res.timeline.feed_modulated_segments > 0
                                && res.timeline.max_cutting_feed_mm_min
                                    > res.timeline.min_cutting_feed_mm_min + 0.5
                            {
                                let range_txt = match display_unit {
                                    DisplayUnit::Millimeter => format!(
                                        "Feed Opt: {:.0}–{:.0} mm/min ({} mod)",
                                        res.timeline.min_cutting_feed_mm_min,
                                        res.timeline.max_cutting_feed_mm_min,
                                        res.timeline.feed_modulated_segments
                                    ),
                                    DisplayUnit::Inch => format!(
                                        "Feed Opt: {:.1}–{:.1} ipm ({} mod)",
                                        res.timeline.min_cutting_feed_mm_min / MM_PER_INCH,
                                        res.timeline.max_cutting_feed_mm_min / MM_PER_INCH,
                                        res.timeline.feed_modulated_segments
                                    ),
                                };
                                ui.separator();
                                ui.colored_label(
                                    crate::theme::ThemeTokens::from_ui(ui).info_fg,
                                    egui::RichText::new(range_txt).monospace().small(),
                                )
                                .on_hover_text(
                                    "Engagement-aware feedrate modulation active: minimum to maximum cutting feedrate and count of slowed/modulated cutting segments",
                                );
                            }
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

/// Formats a verbose GPU adapter string (e.g. `"AMD Radeon 780M Graphics (RADV PHOENIX)"`)
/// into a compact toolbar badge label (`"Radeon 780M"`).
fn shorten_adapter_name(raw: &str) -> String {
    let trimmed = raw.split('(').next().unwrap_or(raw).trim();
    let stripped = trimmed
        .strip_prefix("AMD ")
        .or_else(|| trimmed.strip_prefix("NVIDIA "))
        .or_else(|| trimmed.strip_prefix("Intel(R) "))
        .or_else(|| trimmed.strip_prefix("Intel "))
        .or_else(|| trimmed.strip_prefix("Apple "))
        .unwrap_or(trimmed)
        .trim();
    let no_gfx = stripped
        .strip_suffix(" Graphics")
        .unwrap_or(stripped)
        .trim();
    let candidate = if no_gfx.is_empty() { trimmed } else { no_gfx };
    const MAX_CHARS: usize = 22;
    if candidate.chars().count() > MAX_CHARS {
        let mut s: String = candidate.chars().take(MAX_CHARS - 1).collect();
        s.push('…');
        s
    } else {
        candidate.to_string()
    }
}
