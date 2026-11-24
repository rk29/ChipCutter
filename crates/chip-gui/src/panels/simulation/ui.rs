use std::sync::Arc;

use chip_math::{DisplayUnit, MM_PER_INCH};
use chip_sim::{
    ScrubSnapshot, SimComputeBackendChoice, SimEngineKind, SimVerificationConfig,
    format_cycle_time_hms,
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
            self.heatmap_max_scale_mm.to_bits(),
            self.show_cutter_assembly,
            self.trail_up_to_cutter,
        );

        // Top Control Bar: Compact Settings Chip, Carryover/Undercut Pills, Visual Toggles, Verification Status
        let mut engine_changed = false;
        let mut simulate_all_clicked = false;
        let mut export_json_clicked = false;
        let busy = self.is_busy();
        ui.horizontal_wrapped(|ui| {
            let tok = crate::theme::ThemeTokens::from_ui(ui);
            let engine_short = match self.engine {
                SimEngineKind::Heightmap => "Heightmap",
                SimEngineKind::TriDexel => "Tri-Dexel",
            };
            let res_short = match self.resolution {
                chip_sim::SimResolution::Standard => "Standard",
                chip_sim::SimResolution::Fine => "Fine",
                chip_sim::SimResolution::Ultra => "Ultra",
            };
            let skin_short = if self.mesh_skinning {
                "Skinned"
            } else {
                "Crisp"
            };
            let (backend_short, pill_bg, pill_border, pill_fg, pill_tooltip) =
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
                            format!("GPU ({short_name})"),
                            tok.info_bg,
                            tok.info_border,
                            tok.info_fg,
                            format!(
                                "Engine: {} · Detail: {} · Surface: {}\nActive backend: GPU Compute ({}) — {}\n{}\n\nClick to configure Simulation Engine, Detail Level, Backend, and Mesh Skinning in Settings.",
                                self.engine.label(),
                                self.resolution.label_for_engine(self.engine),
                                skin_short,
                                res.active_backend.backend_api,
                                res.active_backend.adapter_name,
                                timing_line
                            ),
                        )
                    } else if self.backend_choice == SimComputeBackendChoice::Cpu {
                        (
                            "CPU".to_string(),
                            tok.card_bg,
                            tok.border_strong,
                            tok.text_secondary,
                            format!(
                                "Engine: {} · Detail: {} · Surface: {}\nActive backend: CPU Exact (Rayon)\n{}\n\nClick to configure Simulation settings.",
                                self.engine.label(),
                                self.resolution.label_for_engine(self.engine),
                                skin_short,
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
                                "Engine: {} · Detail: {} · Surface: {}\nActive backend: CPU Fallback (Rayon)\n{}\n\nClick to configure Simulation settings.",
                                self.engine.label(),
                                self.resolution.label_for_engine(self.engine),
                                skin_short,
                                timing_line
                            ),
                        )
                    }
                } else if self.backend_choice == SimComputeBackendChoice::Cpu {
                    (
                        "CPU".to_string(),
                        tok.card_bg,
                        tok.border_strong,
                        tok.text_secondary,
                        format!(
                            "Engine: {} · Detail: {} · Surface: {}\nConfigured backend: CPU Exact (Rayon)\n\nClick to configure Simulation settings.",
                            self.engine.label(),
                            self.resolution.label_for_engine(self.engine),
                            skin_short,
                        ),
                    )
                } else if let Some(info) = self.gpu_adapter_info() {
                    let short_name = shorten_adapter_name(&info.adapter_name);
                    (
                        format!("GPU ({short_name})"),
                        tok.info_bg,
                        tok.info_border,
                        tok.info_fg,
                        format!(
                            "Engine: {} · Detail: {} · Surface: {}\nGPU Compute adapter ready ({}) — {}\n\nClick to configure Simulation settings.",
                            self.engine.label(),
                            self.resolution.label_for_engine(self.engine),
                            skin_short,
                            info.backend_api,
                            info.adapter_name
                        ),
                    )
                } else {
                    (
                        "CPU Fallback".to_string(),
                        tok.warn_bg,
                        tok.warn_border,
                        tok.warn_fg,
                        format!(
                            "Engine: {} · Detail: {} · Surface: {}\nNo GPU compute adapter active; CPU + Rayon fallback.\n\nClick to configure Simulation settings.",
                            self.engine.label(),
                            self.resolution.label_for_engine(self.engine),
                            skin_short,
                        ),
                    )
                };

            let chip_label = format!("{engine_short} · {res_short} · {backend_short}");
            if ui
                .add(
                    egui::Button::new(egui::RichText::new(chip_label).small().strong().color(pill_fg))
                        .fill(pill_bg)
                        .stroke(egui::Stroke::new(1.0, pill_border))
                        .corner_radius(egui::CornerRadius::same(4)),
                )
                .on_hover_text(pill_tooltip)
                .clicked()
            {
                action = Some(SimulationUiAction::OpenSimulationSettings);
            }

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

            if self.engine == SimEngineKind::Heightmap
                && project.setups.get(setup_idx).is_some_and(|setup| {
                    setup.operations.iter().any(|op| {
                        !op.suppressed
                            && op.tool_id.as_ref().is_some_and(|tid| {
                                project
                                    .tools
                                    .iter()
                                    .find(|t| &t.id == tid)
                                    .is_some_and(|t| {
                                        matches!(
                                            t.geometry,
                                            chip_tools::CutterGeometry::LollipopEndmill { .. }
                                                | chip_tools::CutterGeometry::SlotMill { .. }
                                                | chip_tools::CutterGeometry::DovetailCutter { .. }
                                                | chip_tools::CutterGeometry::ThreadMill { .. }
                                        )
                                    })
                            })
                    })
                })
            {
                egui::Frame::NONE
                    .fill(tok.warn_bg)
                    .stroke(egui::Stroke::new(1.0, tok.warn_border))
                    .corner_radius(egui::CornerRadius::same(4))
                    .inner_margin(egui::Margin::symmetric(6, 2))
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            ui.label(
                                egui::RichText::new("Undercut cutter active — Heightmap is 2.5D")
                                    .small()
                                    .strong()
                                    .color(tok.warn_fg),
                            )
                            .on_hover_text(
                                "This setup uses a relieved-neck undercut cutter (Lollipop, Slot/T-Slot, Dovetail, or Thread Mill). Switch to Tri-Dexel (Volumetric 3D) to preserve overhanging roofs and model true 3D undercuts.",
                            );
                            if ui
                                .small_button("Use Tri-Dexel")
                                .on_hover_text("Switch simulation engine to Tri-Dexel (Volumetric 3D)")
                                .clicked()
                            {
                                self.engine = SimEngineKind::TriDexel;
                                engine_changed = true;
                            }
                        });
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
            if project.setups.len() > 1
                && ui
                    .add_enabled(!busy && cad_doc.is_some(), egui::Button::new("Simulate All Setups"))
                    .on_hover_text(
                        "Simulate and verify every Setup sequentially on a background worker (carrying over volumetric stock and computing cumulative end-of-job metrology)",
                    )
                    .clicked()
            {
                simulate_all_clicked = true;
            }
            let can_export_json = self.active_result.is_some() || self.multi_setup_summary.is_some();
            if ui
                .add_enabled(can_export_json, egui::Button::new("Export Metrology JSON..."))
                .on_hover_text(
                    "Save structured JSON error-volume, RMS deviation, and crash verification report to disk",
                )
                .on_disabled_hover_text("Run a simulation first to export a JSON metrology report")
                .clicked()
            {
                export_json_clicked = true;
            }

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
                    ui.colored_label(tok.dirty_fg, "Toolpaths changed — Click 'Simulate' to refresh");
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
            } else if busy {
                ui.weak("Simulation running…");
            } else {
                ui.weak(
                    "Click 'Simulate' on the top toolbar to compute 3D material removal & crash checks.",
                );
            }
        });

        if simulate_all_clicked && let Some(doc) = cad_doc {
            self.trigger_simulate_all_setups(SimTriggerArgs {
                project,
                setup_idx,
                cad_doc: Arc::clone(doc),
                cad_revision,
                cache,
                project_dir,
                egui_ctx: ui.ctx().clone(),
                logs,
            });
        } else if engine_changed
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

        if export_json_clicked {
            self.export_metrology_json_dialog(project, logs);
        }

        // Collapsible Verification Tolerances (SimVerificationConfig overrides, M89)
        let default_vc = SimVerificationConfig::default();
        let is_custom_vc = (self.verify_config.gouge_tol_mm - default_vc.gouge_tol_mm).abs() > 1e-6
            || (self.verify_config.rapid_crash_tol_mm - default_vc.rapid_crash_tol_mm).abs() > 1e-6
            || (self.verify_config.radial_wall_skin_mm - default_vc.radial_wall_skin_mm).abs()
                > 1e-6;
        let vc_hdr = if is_custom_vc {
            "Verification Tolerances [Custom]"
        } else {
            "Verification Tolerances"
        };
        egui::CollapsingHeader::new(vc_hdr)
            .id_salt("sim_verification_tolerances_header")
            .default_open(false)
            .show(ui, |ui| {
                let (u_scale, u_sfx, dec, spd) = match display_unit {
                    DisplayUnit::Millimeter => (1.0, " mm", 3usize, 0.002),
                    DisplayUnit::Inch => (1.0 / MM_PER_INCH, " in", 4usize, 0.0001),
                };
                ui.horizontal_wrapped(|ui| {
                    ui.label("Gouge Tol:")
                        .on_hover_text("Allowed axial target-model penetration before flagging a Target Model Gouge error");
                    let mut g_disp = (self.verify_config.gouge_tol_mm * u_scale)
                        .clamp(0.001 * u_scale, 2.0 * u_scale);
                    if ui
                        .add(
                            egui::DragValue::new(&mut g_disp)
                                .speed(spd)
                                .range((0.001 * u_scale)..=(2.0 * u_scale))
                                .fixed_decimals(dec)
                                .suffix(u_sfx),
                        )
                        .on_hover_text("Target solid gouge verification threshold (default 0.020 mm)")
                        .changed()
                        && g_disp.is_finite()
                    {
                        self.verify_config.gouge_tol_mm = (g_disp / u_scale).clamp(0.001, 2.0);
                    }

                    ui.separator();
                    ui.label("Rapid Crash Tol:")
                        .on_hover_text("Allowed G0 rapid contact penetration into uncut stock before flagging a Rapid-into-Stock Crash");
                    let mut r_disp = (self.verify_config.rapid_crash_tol_mm * u_scale)
                        .clamp(0.001 * u_scale, 2.0 * u_scale);
                    if ui
                        .add(
                            egui::DragValue::new(&mut r_disp)
                                .speed(spd)
                                .range((0.001 * u_scale)..=(2.0 * u_scale))
                                .fixed_decimals(dec)
                                .suffix(u_sfx),
                        )
                        .on_hover_text("Rapid-into-stock crash threshold (default 0.020 mm)")
                        .changed()
                        && r_disp.is_finite()
                    {
                        self.verify_config.rapid_crash_tol_mm = (r_disp / u_scale).clamp(0.001, 2.0);
                    }

                    ui.separator();
                    ui.label("Radial Wall Skin:")
                        .on_hover_text("Radial cliff skin margin preventing tangential wall quantization false-positives");
                    let mut w_disp =
                        (self.verify_config.radial_wall_skin_mm * u_scale).clamp(0.0, 1.0 * u_scale);
                    if ui
                        .add(
                            egui::DragValue::new(&mut w_disp)
                                .speed(spd)
                                .range(0.0..=(1.0 * u_scale))
                                .fixed_decimals(dec)
                                .suffix(u_sfx),
                        )
                        .on_hover_text("Radial wall skin tolerance (default 0.018 mm)")
                        .changed()
                        && w_disp.is_finite()
                    {
                        self.verify_config.radial_wall_skin_mm = (w_disp / u_scale).clamp(0.0, 1.0);
                    }

                    ui.separator();
                    if ui
                        .add_enabled(is_custom_vc, egui::Button::new("Reset Defaults").small())
                        .on_hover_text("Restore default crash & gouge verification tolerances (0.020 / 0.020 / 0.018 mm)")
                        .clicked()
                    {
                        self.verify_config = SimVerificationConfig::default();
                    }
                });
            });

        ui.separator();

        let Some(res) = self.active_result.clone() else {
            if busy {
                ui.weak("Simulation running…");
            } else {
                ui.weak("No stock simulation active for this Setup yet.");
            }
            if let Some(summary) = self.multi_setup_summary.clone() {
                show_multi_setup_summary_card(ui, &summary);
            }
            return action;
        };
        if self.active_setup_idx != setup_idx || res.timeline.is_empty() {
            if busy {
                ui.weak("Simulation running…");
            } else {
                ui.weak(
                    "Click 'Simulate' on the top toolbar to simulate the currently selected Setup.",
                );
            }
            if let Some(summary) = self.multi_setup_summary.clone() {
                show_multi_setup_summary_card(ui, &summary);
            }
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

                    ui.spacing_mut().slider_width = (ui.available_width() - 220.0).max(96.0);
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
                        let pct = compute_playback_percent(cur_d, res.timeline.total_distance_mm);
                        ui.monospace(format!(
                            "{} / {} ({pct:.0}%)",
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
                        let pct = compute_playback_percent(
                            self.current_time_sec,
                            res.timeline.total_time_sec,
                        );
                        ui.monospace(format!(
                            "{} / {} ({pct:.0}%)",
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
                            if let Some(rot) = res.rotary_spec() {
                                ui.monospace(format!(
                                    "WCS X={} Y={} Z={} {}={:.1}°",
                                    display_unit.format_length(s.tip_pos_wcs.x),
                                    display_unit.format_length(s.tip_pos_wcs.y),
                                    display_unit.format_length(s.tip_pos_wcs.z),
                                    rot.axis_letter,
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
                            if let Some(rot) = res.rotary_spec() {
                                ui.monospace(format!(
                                    "X={} Y={} Z={} {}={:.1}°",
                                    display_unit.format_length(s.tip_pos_wcs.x),
                                    display_unit.format_length(s.tip_pos_wcs.y),
                                    display_unit.format_length(s.tip_pos_wcs.z),
                                    rot.axis_letter,
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

                // Quantitative Stock-vs-Target Error-Volume & RMS Metrics Dashboard (Phase 11 M40, M89)
                if let Some(rep) = &active_err_report
                    && let Some(metric_act) = show_error_metrics_dashboard(
                        ui,
                        &res,
                        rep,
                        snap.as_ref(),
                        display_unit,
                        &mut self.heatmap_tolerance_mm,
                        &mut self.heatmap_max_scale_mm,
                        &mut self.show_error_heatmap,
                    )
                {
                    action = Some(metric_act);
                }

                // Multi-Setup Verification Chain Summary (M89)
                if let Some(summary) = self.multi_setup_summary.clone() {
                    show_multi_setup_summary_card(ui, &summary);
                }
            });

        if (self.current_time_sec - prev_time_sec).abs() > 1e-9
            || (
                self.show_machined_stock,
                self.color_by_operation,
                self.show_error_heatmap,
                self.heatmap_tolerance_mm.to_bits(),
                self.heatmap_max_scale_mm.to_bits(),
                self.show_cutter_assembly,
                self.trail_up_to_cutter,
            ) != prev_vis_flags
        {
            ui.ctx().request_repaint();
        }

        action
    }
}

fn show_multi_setup_summary_card(ui: &mut egui::Ui, summary: &super::MultiSetupSimSummary) {
    let tok = crate::theme::ThemeTokens::from_ui(ui);
    let total_crit: usize = summary
        .setups
        .iter()
        .map(|s| s.result.critical_error_count())
        .sum();
    let total_warn: usize = summary
        .setups
        .iter()
        .map(|s| s.result.warning_count())
        .sum();
    let total_time: f64 = summary
        .setups
        .iter()
        .map(|s| s.result.timeline.total_time_sec)
        .sum();

    ui.add_space(4.0);
    egui::Frame::group(ui.style())
        .inner_margin(egui::Margin::symmetric(6, 4))
        .show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.strong(format!(
                    "Multi-Setup Verification Summary ({} setups · {})",
                    summary.setups.len(),
                    summary.engine.label()
                ));
                ui.separator();
                ui.monospace(format!("Total Cycle: {}", format_cycle_time_hms(total_time)));
                ui.separator();
                if total_crit > 0 {
                    ui.colored_label(
                        tok.error_fg,
                        egui::RichText::new(format!("{total_crit} Critical Crash/Gouge(s)"))
                            .strong(),
                    );
                } else if total_warn > 0 {
                    ui.colored_label(
                        tok.warn_fg,
                        format!("PASS (0 Crashes, {total_warn} Warning(s))"),
                    );
                } else {
                    ui.colored_label(tok.clean_fg, "PASS (0 Crashes, 0 Warnings)");
                }
            });
            egui::Grid::new("sim_multi_setup_summary_grid")
                .striped(true)
                .spacing([12.0, 2.0])
                .show(ui, |ui| {
                    ui.strong("Setup");
                    ui.strong("Cycle");
                    ui.strong("Removed (mm^3)");
                    ui.strong("Crashes");
                    ui.strong("Warnings");
                    ui.strong("RMS (mm)");
                    ui.strong("In-Tol %");
                    ui.end_row();
                    for entry in &summary.setups {
                        let r = &entry.result;
                        ui.label(&entry.setup_name);
                        ui.monospace(format_cycle_time_hms(r.timeline.total_time_sec));
                        ui.monospace(format!("{:.1}", r.removed_volume_mm3));
                        let c = r.critical_error_count();
                        if c > 0 {
                            ui.colored_label(tok.error_fg, format!("{c}"));
                        } else {
                            ui.monospace("0");
                        }
                        ui.monospace(format!("{}", r.warning_count()));
                        if let Some(er) = &r.error_report {
                            ui.monospace(format!("{:.4}", er.rms_error_mm));
                            ui.monospace(format!("{:.1}%", er.in_tolerance_surface_pct));
                        } else {
                            ui.weak("--");
                            ui.weak("--");
                        }
                        ui.end_row();
                    }
                });
            if let Some(cum) = &summary.cumulative_error_report {
                ui.add_space(2.0);
                ui.monospace(format!(
                    "Cumulative End-of-Job (+/- {:.3} mm): Err Vol {:.1} mm^3 ({:.2}%) | Excess {:.1} | Gouge {:.1} | RMS {:.4} mm | In-Tol {:.1}%",
                    cum.tolerance_mm,
                    cum.total_error_volume_mm3,
                    cum.error_volume_pct_of_target,
                    cum.excess_volume_mm3,
                    cum.gouge_volume_mm3,
                    cum.rms_error_mm,
                    cum.in_tolerance_surface_pct,
                ));
            }
        });
}

/// Computes playback completion percentage in `[0.0, 100.0]` for the timeline scrubber.
pub(crate) fn compute_playback_percent(current: f64, total: f64) -> f64 {
    if !current.is_finite() || !total.is_finite() || total <= 1e-9 {
        100.0
    } else {
        (current / total * 100.0).clamp(0.0, 100.0)
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

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::{
        sync::Arc,
        time::{Duration, Instant},
    };

    use chip_cad::sample_sculpted_dome_stl_bytes;
    use chip_cam::{
        BoxAnchor27, MotionRole, Operation, OperationKind, Setup, StockConfig, ToolpathBuilder,
        WcsOriginMode, WorkOffsetSlot,
    };
    use chip_math::{DVec3, DisplayUnit, ParamLength};
    use chip_project::{ProjectDocument, ToolpathCache, compute_operation_input_hash};
    use chip_sim::{
        MachineKinematicsConfig, SetupSimulationInput, SimOperationInput, SimResolution,
        SimStockColorMode, SimVerificationConfig, run_setup_simulation,
    };
    use chip_tools::default_starter_tool_library;

    use super::compute_playback_percent;
    use crate::panels::simulation::{SimPanelCtx, SimTriggerArgs, SimulationPanelState};

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

    #[test]
    fn playback_percent_handles_endpoints_midpoints_and_edge_cases() {
        assert!((compute_playback_percent(0.0, 120.0) - 0.0).abs() < 1e-9);
        assert!((compute_playback_percent(30.0, 120.0) - 25.0).abs() < 1e-9);
        assert!((compute_playback_percent(120.0, 120.0) - 100.0).abs() < 1e-9);
        assert!((compute_playback_percent(150.0, 120.0) - 100.0).abs() < 1e-9);
        assert!((compute_playback_percent(-5.0, 120.0) - 0.0).abs() < 1e-9);
        assert!((compute_playback_percent(0.0, 0.0) - 100.0).abs() < 1e-9);
        assert!((compute_playback_percent(f64::NAN, 120.0) - 100.0).abs() < 1e-9);
    }

    #[test]
    fn simulation_panel_renders_heatmap_scale_verify_tolerances_multi_setup_and_exports_json() {
        let stl = sample_sculpted_dome_stl_bytes();
        let doc = Arc::new(
            chip_cad::load_mesh_cad_document(
                "dome.stl",
                &stl,
                &chip_cad::MeshImportOptions::default(),
            )
            .unwrap(),
        );
        let lib = default_starter_tool_library();
        let flat_6mm = lib.tools[0].clone();

        let mut project = ProjectDocument::new("M89 Sim Test");
        project.tools = lib.tools.clone();

        let make_setup = |id: &str, name: &str, slot: WorkOffsetSlot| -> Setup {
            let mut s = Setup::new(id, name, slot);
            s.wcs.origin = WcsOriginMode::StockBoxPoint {
                anchor: BoxAnchor27::TOP_CENTER,
            };
            s.stock = StockConfig::FixedBox {
                width_x: ParamLength::from_mm(140.0),
                depth_y: ParamLength::from_mm(140.0),
                height_z: ParamLength::from_mm(45.0),
                xy_align: chip_cam::FixedBoxXyAlign::CenterXy,
                z_align: chip_cam::FixedBoxZAlign::CenterZ,
            };
            let mut op = Operation::new(format!("{id}_op1"), "Slot Pass", OperationKind::Facing);
            op.tool_id = Some(flat_6mm.id.clone());
            s.operations.push(op);
            s
        };
        project.setups = vec![
            make_setup("setup_1", "Setup 1 — Op 1", WorkOffsetSlot::G54),
            make_setup("setup_2", "Setup 2 — Op 2", WorkOffsetSlot::G55),
        ];

        let mut cache = ToolpathCache::new(16);
        let mut b = ToolpathBuilder::new("slot");
        b.rapid_to(DVec3::new(-25.0, 0.0, 15.0));
        b.linear_to(DVec3::new(-25.0, 0.0, -2.0), 300.0, MotionRole::Plunge);
        b.linear_to(DVec3::new(25.0, 0.0, -2.0), 600.0, MotionRole::Cut);
        b.rapid_to(DVec3::new(25.0, 0.0, 15.0));
        let tp = Arc::new(b.finish());
        for s_idx in 0..2 {
            let hk = compute_operation_input_hash(&project, s_idx, 0, 1).unwrap();
            let op_id = project.setups[s_idx].operations[0].id.clone();
            cache.insert(op_id, hk, Arc::clone(&tp));
        }

        let resolved = project.setups[0]
            .resolve(&doc, DisplayUnit::Millimeter)
            .unwrap();
        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput {
                op_id: "setup_1_op1",
                op_name: "Slot Pass",
                tool: &flat_6mm,
                fallback_spindle_rpm: 12_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp,
            }],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
        })
        .unwrap();

        let mut panel = SimulationPanelState {
            show_error_heatmap: true,
            heatmap_tolerance_mm: 0.05,
            heatmap_max_scale_mm: 2.5,
            ..Default::default()
        };
        assert_eq!(
            panel.active_stock_color_mode(),
            SimStockColorMode::ErrorHeatmap {
                tolerance_mm: 0.05,
                max_scale_mm: 2.5,
            }
        );

        let sig_before = panel.compute_setup_sim_signature(&project, 0, &cache, 1);
        panel.verify_config.gouge_tol_mm = 0.04;
        let sig_after = panel.compute_setup_sim_signature(&project, 0, &cache, 1);
        assert_ne!(
            sig_before, sig_after,
            "changing SimVerificationConfig must invalidate cached simulation signature"
        );

        panel.active_run_id = 1;
        panel.active_setup_idx = 0;
        panel.active_signature = sig_after;
        panel.active_result = Some(Arc::new(sim));

        let ctx = egui::Context::default();
        let mut logs = Vec::new();
        let mut out = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(1280.0, 500.0),
                )),
                ..Default::default()
            },
            |ui| {
                egui::CentralPanel::default().show(ui, |ui| {
                    let _ = panel.show_ui(
                        ui,
                        SimPanelCtx {
                            project: &project,
                            setup_idx: 0,
                            cad_doc: Some(&doc),
                            cad_revision: 1,
                            cache: &cache,
                            project_dir: None,
                            logs: &mut logs,
                        },
                    );
                });
            },
        );
        out.textures_delta.clear();
        let mut texts = Vec::new();
        collect_texts(&out.shapes, &mut texts);
        assert!(
            texts.iter().any(|t| t.contains("Heatmap Max Scale")),
            "expected Heatmap Max Scale label in simulation UI, got: {texts:?}"
        );
        assert!(
            texts.iter().any(|t| t.contains("Verification Tolerances")),
            "expected Verification Tolerances section in simulation UI"
        );
        assert!(
            texts.iter().any(|t| t.contains("Simulate All Setups")),
            "expected Simulate All Setups button for multi-setup project"
        );
        assert!(
            texts.iter().any(|t| t.contains("Export Metrology JSON")),
            "expected Export Metrology JSON button"
        );

        // Single-setup JSON report serialization
        let json_str = panel.build_metrology_report_json(&project).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&json_str).unwrap();
        assert_eq!(parsed["project_name"], "M89 Sim Test");
        assert!((parsed["heatmap_max_scale_mm"].as_f64().unwrap() - 2.5).abs() < 1e-6);
        assert!((parsed["verify_config"]["gouge_tol_mm"].as_f64().unwrap() - 0.04).abs() < 1e-6);
        assert_eq!(parsed["setups"].as_array().unwrap().len(), 1);

        // Background multi-setup simulation chain
        panel.verify_config = SimVerificationConfig::default();
        panel.trigger_simulate_all_setups(SimTriggerArgs {
            project: &project,
            setup_idx: 0,
            cad_doc: Arc::clone(&doc),
            cad_revision: 1,
            cache: &cache,
            project_dir: None,
            egui_ctx: ctx.clone(),
            logs: &mut logs,
        });
        assert!(panel.is_busy());
        let deadline = Instant::now() + Duration::from_secs(15);
        while panel.is_busy() && Instant::now() < deadline {
            let _ = panel.poll_worker(&mut logs);
            std::thread::sleep(Duration::from_millis(5));
        }
        assert!(!panel.is_busy(), "multi-setup simulation timed out");
        let summary = panel
            .multi_setup_summary
            .as_ref()
            .expect("multi_setup_summary should be populated");
        assert_eq!(summary.setups.len(), 2);

        let multi_json = panel.build_metrology_report_json(&project).unwrap();
        let multi_val: serde_json::Value = serde_json::from_str(&multi_json).unwrap();
        assert_eq!(multi_val["setups"].as_array().unwrap().len(), 2);
        assert!(multi_val["cumulative_error_report"].is_object());
    }
}
