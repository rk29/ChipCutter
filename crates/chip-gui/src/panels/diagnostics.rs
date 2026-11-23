use std::time::{Duration, Instant};

use chip_cad::CadDocument;
use chip_cam::OperationStatus;
use chip_project::{ProjectDocument, ToolpathCache};
use chip_sim::{SetupSimulationResult, SimDiagnostic, SimSeverity};
use chip_tools::MachineDefinition;

use crate::{
    theme::ThemeTokens,
    widgets::{StatusBadgeKind, status_badge_icon},
};

/// Formats all Activity Log entries as a newline-delimited plain-text block for display and clipboard copy.
pub fn format_activity_log(logs: &[String]) -> String {
    logs.join("\n")
}

/// Collects fast in-memory machine envelope / spindle / feedrate / rotary warnings for the active
/// setup without touching disk or running full B-Rep extraction.
fn collect_active_setup_machine_warnings(
    project: &ProjectDocument,
    setup: &chip_cam::Setup,
    machine: &MachineDefinition,
    loaded_cad: Option<&CadDocument>,
) -> Vec<String> {
    let mut warnings = Vec::new();
    let empty_cad = CadDocument::empty();
    let cad = loaded_cad.unwrap_or(&empty_cad);
    if let Ok(resolved) = setup.resolve_with_sketches(cad, &project.sketches, project.display_units)
    {
        let sb = resolved.stock.aabb_wcs();
        let sx = sb.max.x - sb.min.x;
        let sy = sb.max.y - sb.min.y;
        let sz = sb.max.z - sb.min.z;
        let tr = &machine.travel_mm;
        if sx > tr.x_mm + 0.01 || sy > tr.y_mm + 0.01 || sz > tr.z_mm + 0.01 {
            warnings.push(format!(
                "Stock ({sx:.1} × {sy:.1} × {sz:.1} mm) exceeds '{}' travel envelope ({:.0} × {:.0} × {:.0} mm).",
                machine.name, tr.x_mm, tr.y_mm, tr.z_mm
            ));
        }
        if let Some(rot) = resolved.rotary {
            match &machine.rotary {
                None => warnings.push(format!(
                    "4th-axis rotary is active ({}-Axis), but '{}' is 3-axis only.",
                    rot.axis_letter, machine.name
                )),
                Some(mrot) => {
                    let dia = rot.stock_outer_radius_mm * 2.0;
                    let len = (rot.stock_u_max - rot.stock_u_min).abs();
                    if dia > mrot.max_stock_diameter_mm + 0.01
                        || len > mrot.max_stock_length_mm + 0.01
                    {
                        warnings.push(format!(
                            "Rotary stock (Ø{dia:.1} × {len:.1} mm) exceeds '{}' capacity (Ø{:.1} × {:.1} mm).",
                            machine.name, mrot.max_stock_diameter_mm, mrot.max_stock_length_mm
                        ));
                    }
                }
            }
        }
    }
    let sp = &machine.spindle;
    let kin = &machine.kinematics;
    for op in &setup.operations {
        if op.suppressed {
            continue;
        }
        let Some(tid) = &op.tool_id else {
            continue;
        };
        let Some(tool) = project.tool(tid) else {
            continue;
        };
        let eff = op.effective_cutting_params(tool);
        if eff.spindle_rpm < sp.min_rpm - 0.5 || eff.spindle_rpm > sp.max_rpm + 0.5 {
            warnings.push(format!(
                "{}: {:.0} RPM outside '{}' range ({:.0}–{:.0} RPM).",
                op.name, eff.spindle_rpm, machine.name, sp.min_rpm, sp.max_rpm
            ));
        }
        if eff.cutting_feed_mm_min > kin.max_cut_feed_xy_mm_min + 0.5 {
            warnings.push(format!(
                "{}: feed {:.0} mm/min exceeds '{}' XY limit ({:.0} mm/min).",
                op.name, eff.cutting_feed_mm_min, machine.name, kin.max_cut_feed_xy_mm_min
            ));
        }
        if eff.plunge_feed_mm_min > kin.max_cut_feed_z_mm_min + 0.5 {
            warnings.push(format!(
                "{}: plunge {:.0} mm/min exceeds '{}' Z limit ({:.0} mm/min).",
                op.name, eff.plunge_feed_mm_min, machine.name, kin.max_cut_feed_z_mm_min
            ));
        }
    }
    warnings
}

/// Formats estimated cycle seconds as `Xm YYs` or `YY.Ys`.
pub fn format_cycle_time(sec: f64) -> String {
    if !sec.is_finite() || sec <= 0.0 {
        return "0.0s".to_string();
    }
    if sec < 60.0 {
        format!("{sec:.1}s")
    } else {
        let mins = (sec / 60.0).floor() as u64;
        let rem = sec - (mins as f64) * 60.0;
        format!("{mins}m {rem:02.0}s")
    }
}

/// Renders the bottom **Diagnostics & Log** pane (`08-ui-viewport.md` & `06-simulation.md` §4),
/// returning `Some(SimDiagnostic)` when the user clicks `[Jump to Move]` on a crash/gouge/warning.
#[allow(clippy::too_many_arguments)]
pub fn show_diagnostics_panel(
    ui: &mut egui::Ui,
    project: &ProjectDocument,
    active_setup_idx: usize,
    cache: &ToolpathCache,
    cad_revision: u64,
    sim_result: Option<&SetupSimulationResult>,
    active_machine: Option<&MachineDefinition>,
    loaded_cad: Option<&CadDocument>,
    logs: &[String],
) -> Option<SimDiagnostic> {
    let tok = ThemeTokens::from_ui(ui);
    let u = project.display_units;
    let mut jump_target: Option<SimDiagnostic> = None;

    ui.columns(2, |cols| {
        // Left column: Simulation Crash/Gouge Diagnostics + Active Setup Toolpath Summary
        cols[0].strong("Simulation Verification & Toolpath Summary");
        cols[0].separator();

        if let Some(setup) = project.setups.get(active_setup_idx) {
            let mut total_sec = 0.0;
            let mut total_cut_mm = 0.0;
            let machine_warnings = active_machine
                .map(|m| collect_active_setup_machine_warnings(project, setup, m, loaded_cad))
                .unwrap_or_default();
            egui::ScrollArea::vertical()
                .id_salt("diag_tp_summary")
                .show(&mut cols[0], |ui| {
                    if !machine_warnings.is_empty() {
                        for w in &machine_warnings {
                            ui.horizontal_wrapped(|ui| {
                                status_badge_icon(ui, StatusBadgeKind::NeedsRepick, None);
                                ui.colored_label(tok.warn_fg, format!("[Machine Limit] {w}"));
                            });
                        }
                        ui.separator();
                    }
                    if let Some(sim) = sim_result {
                        if sim.diagnostics.is_empty() {
                            ui.horizontal(|ui| {
                                status_badge_icon(ui, StatusBadgeKind::Clean, None);
                                ui.colored_label(
                                    tok.clean_fg,
                                    format!(
                                        "Simulation Verified ({}): 0 crashes, 0 gouges, 0 warnings.",
                                        sim.engine.label()
                                    ),
                                );
                            });
                        } else {
                            for diag in &sim.diagnostics {
                                let (col, badge) = match diag.severity {
                                    SimSeverity::CriticalError => {
                                        (tok.error_fg, StatusBadgeKind::Error)
                                    }
                                    SimSeverity::Warning => {
                                        (tok.warn_fg, StatusBadgeKind::NeedsRepick)
                                    }
                                };
                                ui.horizontal_wrapped(|ui| {
                                    status_badge_icon(ui, badge, None);
                                    if ui
                                        .small_button("Jump to Move")
                                        .on_hover_text(
                                            "Snap 3D camera & simulation scrubber directly to this move",
                                        )
                                        .clicked()
                                    {
                                        jump_target = Some(diag.clone());
                                    }
                                    ui.colored_label(
                                        col,
                                        format!(
                                            "[{}] {} (Seg #{}, t={:.2}s): {}",
                                            diag.kind.label(),
                                            diag.op_name,
                                            diag.global_seg_idx + 1,
                                            diag.timeline_time_sec,
                                            diag.message
                                        ),
                                    );
                                });
                            }
                        }
                        if let Some(err) = &sim.error_report {
                            ui.monospace(format!(
                                "Error Vol: {:.1} mm^3 ({:.2}% of target) | Excess: {:.1} | Gouge: {:.1} | Eff: {:.1}%",
                                err.total_error_volume_mm3,
                                err.error_volume_pct_of_target,
                                err.excess_volume_mm3,
                                err.gouge_volume_mm3,
                                err.material_removal_efficiency_pct
                            ));
                            ui.monospace(format!(
                                "Deviation (+/-{:.3} mm): RMS {:.4} | MAE {:.4} | Bias {:+.4} | P95 {:.3} | P99 {:.3} | In-Tol {:.1}%",
                                err.tolerance_mm,
                                err.rms_error_mm,
                                err.mean_abs_error_mm,
                                err.mean_signed_error_mm,
                                err.p95_abs_error_mm,
                                err.p99_abs_error_mm,
                                err.in_tolerance_surface_pct
                            ));
                        }
                        ui.separator();
                    }

                    for (o_idx, op) in setup.operations.iter().enumerate() {
                        if op.suppressed {
                            ui.horizontal(|ui| {
                                status_badge_icon(ui, StatusBadgeKind::Suppressed, None);
                                ui.weak(format!("{}. {} (Suppressed)", o_idx + 1, op.name));
                            });
                            continue;
                        }
                        if let Some((_, tp)) =
                            cache.get_for_operation(project, active_setup_idx, o_idx, cad_revision)
                        {
                            total_sec += tp.stats.total_est_time_sec;
                            total_cut_mm += tp.stats.cutting_distance_mm;
                            ui.horizontal_wrapped(|ui| {
                                status_badge_icon(ui, StatusBadgeKind::Clean, None);
                                ui.colored_label(
                                    tok.clean_fg,
                                    format!("{}. {}", o_idx + 1, op.name),
                                );
                                ui.weak(format!(
                                    "— {} pass(es) · {} G1 · {} G2/G3 · Cut {} · Rapid {} · Est {}",
                                    tp.stats.pass_count,
                                    tp.stats.linear_move_count,
                                    tp.stats.arc_move_count,
                                    u.format_length(tp.stats.cutting_distance_mm),
                                    u.format_length(tp.stats.rapid_distance_mm),
                                    format_cycle_time(tp.stats.total_est_time_sec),
                                ));
                            });
                            for w in &tp.warnings {
                                ui.colored_label(tok.warn_fg, format!("   • {w}"));
                            }
                        } else {
                            match &op.runtime_status {
                                OperationStatus::Error { message }
                                | OperationStatus::NeedsRepick { message } => {
                                    ui.horizontal_wrapped(|ui| {
                                        status_badge_icon(ui, StatusBadgeKind::Error, None);
                                        ui.colored_label(
                                            tok.error_fg,
                                            format!("{}. {}: {message}", o_idx + 1, op.name),
                                        );
                                    });
                                }
                                OperationStatus::Generating => {
                                    ui.horizontal(|ui| {
                                        ui.spinner();
                                        ui.label(format!(
                                            "{}. {} — Generating…",
                                            o_idx + 1,
                                            op.name
                                        ));
                                    });
                                }
                                _ => {
                                    ui.horizontal_wrapped(|ui| {
                                        status_badge_icon(ui, StatusBadgeKind::Dirty, None);
                                        ui.colored_label(
                                            tok.dirty_fg,
                                            format!(
                                                "{}. {} — Dirty (press Ctrl+G to generate)",
                                                o_idx + 1,
                                                op.name
                                            ),
                                        );
                                    });
                                }
                            }
                        }
                    }

                    if total_sec > 0.0 {
                        ui.separator();
                        ui.strong(format!(
                            "Setup Total: Cut {} | Estimated Cycle Time: {}",
                            u.format_length(total_cut_mm),
                            format_cycle_time(total_sec)
                        ));
                    }
                });
        } else {
            cols[0].weak("No active Setup.");
        }

        // Right column: Event & Validation Log
        let copied_feedback_id = egui::Id::unique("diag_activity_log_copied_until");
        let copied_until = cols[1]
            .ctx()
            .data(|d| d.get_temp::<Instant>(copied_feedback_id));
        let show_copied = if let Some(until) = copied_until {
            if Instant::now() < until {
                cols[1]
                    .ctx()
                    .request_repaint_after(Duration::from_millis(250));
                true
            } else {
                cols[1]
                    .ctx()
                    .data_mut(|d| d.remove::<Instant>(copied_feedback_id));
                false
            }
        } else {
            false
        };

        cols[1].horizontal(|ui| {
            ui.strong("Activity Log");
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui
                    .add_enabled_ui(!logs.is_empty(), |ui| ui.small_button("Copy All"))
                    .inner
                    .on_hover_text("Copy all Activity Log entries to clipboard")
                    .on_disabled_hover_text("No activity log entries to copy")
                    .clicked()
                {
                    ui.ctx().copy_text(format_activity_log(logs));
                    ui.ctx().data_mut(|d| {
                        d.insert_temp(copied_feedback_id, Instant::now() + Duration::from_secs(2));
                    });
                }
                if show_copied {
                    ui.colored_label(tok.clean_fg, "Copied!");
                }
            });
        });
        cols[1].separator();
        if logs.is_empty() {
            cols[1].weak("No activity log entries yet.");
        } else {
            let log_text = format_activity_log(logs);
            egui::Frame::NONE
                .fill(tok.code_bg)
                .stroke(egui::Stroke::new(1.0, tok.border))
                .inner_margin(4.0)
                .corner_radius(4.0)
                .show(&mut cols[1], |ui| {
                    egui::ScrollArea::vertical()
                        .id_salt("diag_activity_log")
                        .auto_shrink([false, false])
                        .stick_to_bottom(true)
                        .show(ui, |ui| {
                            let mut read_only_text = log_text.as_str();
                            let output = egui::TextEdit::multiline(&mut read_only_text)
                                .id_salt("diag_activity_log_text")
                                .font(egui::TextStyle::Monospace)
                                .desired_width(f32::INFINITY)
                                .frame(egui::Frame::NONE)
                                .background_color(egui::Color32::TRANSPARENT)
                                .show(ui);

                            let selected_text = output
                                .cursor_range
                                .or_else(|| output.state.cursor.char_range())
                                .map(|range| range.slice_str(&log_text))
                                .filter(|s| !s.is_empty());

                            output.response.context_menu(|ui| {
                                if ui
                                    .add_enabled(
                                        selected_text.is_some(),
                                        egui::Button::new("Copy Selection"),
                                    )
                                    .on_hover_text(
                                        "Copy the selected Activity Log text to clipboard",
                                    )
                                    .on_disabled_hover_text(
                                        "Select text in the Activity Log to copy a selection",
                                    )
                                    .clicked()
                                {
                                    if let Some(sel) = selected_text {
                                        ui.ctx().copy_text(sel.to_owned());
                                        ui.ctx().data_mut(|d| {
                                            d.insert_temp(
                                                copied_feedback_id,
                                                Instant::now() + Duration::from_secs(2),
                                            );
                                        });
                                    }
                                    ui.close();
                                }
                                if ui
                                    .button("Copy All")
                                    .on_hover_text("Copy all Activity Log entries to clipboard")
                                    .clicked()
                                {
                                    ui.ctx().copy_text(log_text.clone());
                                    ui.ctx().data_mut(|d| {
                                        d.insert_temp(
                                            copied_feedback_id,
                                            Instant::now() + Duration::from_secs(2),
                                        );
                                    });
                                    ui.close();
                                }
                            });
                        });
                });
        }
    });

    jump_target
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    fn run_diag_frame(
        ctx: &egui::Context,
        input: egui::RawInput,
        project: &ProjectDocument,
        cache: &ToolpathCache,
        logs: &[String],
    ) -> egui::FullOutput {
        let mut out = ctx.run_ui(input, |ui| {
            egui::CentralPanel::default().show(ui, |ui| {
                show_diagnostics_panel(ui, project, 0, cache, 0, None, None, None, logs);
            });
        });
        out.textures_delta.clear();
        out
    }

    fn copied_texts(output: &egui::FullOutput) -> Vec<String> {
        output
            .platform_output
            .commands
            .iter()
            .filter_map(|cmd| match cmd {
                egui::OutputCommand::CopyText(text) => Some(text.clone()),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn format_activity_log_joins_entries_with_newlines() {
        assert_eq!(format_activity_log(&[]), "");
        assert_eq!(
            format_activity_log(&["ERROR loading project: missing file".to_string()]),
            "ERROR loading project: missing file"
        );
        assert_eq!(
            format_activity_log(&[
                "ChipCutter initialized.".to_string(),
                "ERROR loading project: missing file".to_string(),
            ]),
            "ChipCutter initialized.\nERROR loading project: missing file"
        );
    }

    #[test]
    fn activity_log_select_all_and_copy_shortcut_emits_copy_text_command() {
        let ctx = egui::Context::default();
        let project = ProjectDocument::new("Test");
        let cache = ToolpathCache::new(16);
        let logs = vec![
            "ChipCutter initialized.".to_string(),
            "ERROR loading project: invalid TOML syntax at line 12".to_string(),
        ];
        let screen = egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(800.0, 300.0));

        // Frame 1: render panel to compute widget layout.
        let _ = run_diag_frame(
            &ctx,
            egui::RawInput {
                screen_rect: Some(screen),
                ..Default::default()
            },
            &project,
            &cache,
            &logs,
        );

        // Frame 2: click inside the right-half Activity Log read-only TextEdit to focus it.
        let click_pos = egui::pos2(550.0, 70.0);
        let _ = run_diag_frame(
            &ctx,
            egui::RawInput {
                screen_rect: Some(screen),
                events: vec![
                    egui::Event::PointerMoved(click_pos),
                    egui::Event::PointerButton {
                        pos: click_pos,
                        button: egui::PointerButton::Primary,
                        pressed: true,
                        modifiers: egui::Modifiers::NONE,
                    },
                    egui::Event::PointerButton {
                        pos: click_pos,
                        button: egui::PointerButton::Primary,
                        pressed: false,
                        modifiers: egui::Modifiers::NONE,
                    },
                ],
                ..Default::default()
            },
            &project,
            &cache,
            &logs,
        );

        // Frame 3: attempt to type into the read-only log (must be ignored), then press
        // Ctrl+A (Select All) and Copy (Ctrl+C).
        let out = run_diag_frame(
            &ctx,
            egui::RawInput {
                screen_rect: Some(screen),
                events: vec![
                    egui::Event::Text("SHOULD_NOT_MODIFY".to_string()),
                    egui::Event::Key {
                        key: egui::Key::A,
                        physical_key: None,
                        pressed: true,
                        repeat: false,
                        modifiers: egui::Modifiers::COMMAND,
                    },
                    egui::Event::Copy,
                ],
                ..Default::default()
            },
            &project,
            &cache,
            &logs,
        );

        assert_eq!(copied_texts(&out), vec![format_activity_log(&logs)]);
    }

    #[test]
    fn activity_log_partial_selection_copy_emits_selected_substring() {
        let ctx = egui::Context::default();
        let project = ProjectDocument::new("Test");
        let cache = ToolpathCache::new(16);
        let logs = vec![
            "ChipCutter initialized.".to_string(),
            "ERROR loading project: missing file".to_string(),
        ];
        let screen = egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(800.0, 300.0));

        // Frame 1: initial layout.
        let _ = run_diag_frame(
            &ctx,
            egui::RawInput {
                screen_rect: Some(screen),
                ..Default::default()
            },
            &project,
            &cache,
            &logs,
        );

        // Frame 2: click inside the Activity Log to focus it.
        let click_pos = egui::pos2(430.0, 48.0);
        let _ = run_diag_frame(
            &ctx,
            egui::RawInput {
                screen_rect: Some(screen),
                events: vec![
                    egui::Event::PointerMoved(click_pos),
                    egui::Event::PointerButton {
                        pos: click_pos,
                        button: egui::PointerButton::Primary,
                        pressed: true,
                        modifiers: egui::Modifiers::NONE,
                    },
                    egui::Event::PointerButton {
                        pos: click_pos,
                        button: egui::PointerButton::Primary,
                        pressed: false,
                        modifiers: egui::Modifiers::NONE,
                    },
                ],
                ..Default::default()
            },
            &project,
            &cache,
            &logs,
        );

        // Frame 3: move to start of line 1, Shift+Right 10 times ("ChipCutter"), then Copy.
        let mut events = vec![egui::Event::Key {
            key: egui::Key::Home,
            physical_key: None,
            pressed: true,
            repeat: false,
            modifiers: egui::Modifiers::NONE,
        }];
        for _ in 0..10 {
            events.push(egui::Event::Key {
                key: egui::Key::ArrowRight,
                physical_key: None,
                pressed: true,
                repeat: false,
                modifiers: egui::Modifiers::SHIFT,
            });
        }
        events.push(egui::Event::Copy);

        let out = run_diag_frame(
            &ctx,
            egui::RawInput {
                screen_rect: Some(screen),
                events,
                ..Default::default()
            },
            &project,
            &cache,
            &logs,
        );

        assert_eq!(copied_texts(&out), vec!["ChipCutter".to_string()]);
    }

    #[test]
    fn activity_log_copy_all_button_emits_copy_text_command() {
        let ctx = egui::Context::default();
        let project = ProjectDocument::new("Test");
        let cache = ToolpathCache::new(16);
        let logs = vec![
            "Opening project foo.chip.toml...".to_string(),
            "ERROR loading project: file not found".to_string(),
        ];

        // Frame 1: render panel to compute layout.
        let screen = egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(800.0, 300.0));
        let _ = run_diag_frame(
            &ctx,
            egui::RawInput {
                screen_rect: Some(screen),
                ..Default::default()
            },
            &project,
            &cache,
            &logs,
        );

        // Frame 2: click the "Copy All" button in the top-right Activity Log header.
        let pos = egui::pos2(760.0, 16.0);
        let out = run_diag_frame(
            &ctx,
            egui::RawInput {
                screen_rect: Some(screen),
                events: vec![
                    egui::Event::PointerMoved(pos),
                    egui::Event::PointerButton {
                        pos,
                        button: egui::PointerButton::Primary,
                        pressed: true,
                        modifiers: egui::Modifiers::NONE,
                    },
                    egui::Event::PointerButton {
                        pos,
                        button: egui::PointerButton::Primary,
                        pressed: false,
                        modifiers: egui::Modifiers::NONE,
                    },
                ],
                ..Default::default()
            },
            &project,
            &cache,
            &logs,
        );

        assert_eq!(copied_texts(&out), vec![format_activity_log(&logs)]);
    }

    #[test]
    fn activity_log_empty_renders_cleanly_without_copy_commands() {
        let ctx = egui::Context::default();
        let project = ProjectDocument::new("Empty");
        let cache = ToolpathCache::new(16);
        let out = run_diag_frame(
            &ctx,
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::pos2(0.0, 0.0),
                    egui::vec2(800.0, 300.0),
                )),
                ..Default::default()
            },
            &project,
            &cache,
            &[],
        );
        assert!(copied_texts(&out).is_empty());
    }
}
