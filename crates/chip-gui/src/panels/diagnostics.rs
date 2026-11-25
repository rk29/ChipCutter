use std::time::{Duration, Instant};

use chip_cad::CadDocument;
use chip_cam::{MachinabilityFix, MachinabilitySeverity, OperationStatus};
use chip_project::{ProjectDocument, ToolpathCache};
use chip_sim::{SetupSimulationResult, SimDiagnostic, SimSeverity};
use chip_tools::{MachineDefinition, ToolLibrary};

use crate::{
    panels::operation_inspector::machinability_card::{
        evaluate_setup_machinability, primary_fixable_finding, sanitize_physics_text,
    },
    theme::ThemeTokens,
    widgets::{StatusBadgeKind, status_badge_icon},
};

/// Action emitted by [`show_diagnostics_panel`] back to the host workspace shell.
#[derive(Debug, Clone, PartialEq)]
pub enum DiagnosticsPanelAction {
    /// Jump 3D camera & simulation timeline to a crash/gouge diagnostic move.
    JumpToSimDiagnostic(SimDiagnostic),
    /// Focus an operation inside the right-hand Operation Inspector.
    FocusOperation { setup_idx: usize, op_idx: usize },
    /// Apply a deterministic 1-click safe machinability fix inside an atomic Undo snapshot.
    ApplyMachinabilityFix {
        setup_idx: usize,
        op_idx: usize,
        fix: MachinabilityFix,
    },
}

/// Formats all Activity Log entries as a newline-delimited plain-text block for display and clipboard copy.
pub fn format_activity_log(logs: &[String]) -> String {
    logs.join("\n")
}

fn truncate_cell_label(text: &str, max_chars: usize) -> String {
    let count = text.chars().count();
    if count <= max_chars || max_chars <= 3 {
        text.to_string()
    } else {
        let prefix: String = text.chars().take(max_chars - 3).collect();
        format!("{prefix}...")
    }
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
/// returning [`DiagnosticsPanelAction`] when the user clicks `[Jump to Move]`, clicks an operation
/// row in the Setup-wide **Machinability & Tool Safety** table, or triggers a 1-click safe fix.
#[allow(clippy::too_many_arguments)]
pub fn show_diagnostics_panel(
    ui: &mut egui::Ui,
    project: &ProjectDocument,
    active_setup_idx: usize,
    global_tools: &ToolLibrary,
    cache: &ToolpathCache,
    cad_revision: u64,
    sim_result: Option<&SetupSimulationResult>,
    active_machine: Option<&MachineDefinition>,
    loaded_cad: Option<&CadDocument>,
    logs: &[String],
) -> Option<DiagnosticsPanelAction> {
    let tok = ThemeTokens::from_ui(ui);
    let u = project.display_units;
    let mut panel_action: Option<DiagnosticsPanelAction> = None;

    ui.columns(2, |cols| {
        // Left column: Machinability & Tool Safety + Simulation Crash/Gouge Diagnostics + Summary
        cols[0].strong("Machinability, Simulation Verification & Toolpath Summary");
        cols[0].separator();

        if let Some(setup) = project.setups.get(active_setup_idx) {
            let mut total_sec = 0.0;
            let mut total_cut_mm = 0.0;
            let machine_warnings = active_machine
                .map(|m| collect_active_setup_machine_warnings(project, setup, m, loaded_cad))
                .unwrap_or_default();
            let setup_mach = evaluate_setup_machinability(
                project,
                active_setup_idx,
                global_tools,
                active_machine,
                loaded_cad,
            );
            egui::ScrollArea::vertical()
                .id_salt("diag_tp_summary")
                .show(&mut cols[0], |ui| {
                    if let Some(mach) = &setup_mach {
                        let active_ops: Vec<(usize, &chip_cam::OperationMachinabilityReport)> = mach
                            .operations
                            .iter()
                            .enumerate()
                            .filter(|(_, rep)| !rep.suppressed)
                            .collect();
                        let (hdr_col, hdr_badge) = match mach.severity {
                            MachinabilitySeverity::Critical => (tok.error_fg, "CRITICAL"),
                            MachinabilitySeverity::Warning => (tok.warn_fg, "WARN"),
                            MachinabilitySeverity::Info => (tok.info_fg, "INFO"),
                            MachinabilitySeverity::Ok => (tok.clean_fg, "OK"),
                        };
                        ui.horizontal_wrapped(|ui| {
                            ui.strong("Machinability & Tool Safety");
                            ui.colored_label(
                                hdr_col,
                                format!(
                                    "[{hdr_badge}] ({} crit, {} warn)",
                                    mach.critical_count, mach.warning_count
                                ),
                            );
                        });
                        if active_ops.is_empty() {
                            ui.weak("No active operations in this Setup.");
                        } else {
                            egui::Grid::new(("diag_machinability_table", active_setup_idx))
                                .num_columns(8)
                                .striped(true)
                                .spacing([6.0, 3.0])
                                .show(ui, |ui| {
                                    for col_hdr in [
                                        "Operation",
                                        "Tool",
                                        "L/D",
                                        "Spindle %",
                                        "Defl (um)",
                                        "h_max",
                                        "Status",
                                        "Action",
                                    ] {
                                        ui.label(
                                            egui::RichText::new(col_hdr)
                                                .small()
                                                .strong()
                                                .color(tok.text_secondary),
                                        );
                                    }
                                    ui.end_row();

                                    for (op_idx, rep) in active_ops {
                                        let op_full = sanitize_physics_text(&rep.op_name);
                                        let op_short = truncate_cell_label(&op_full, 24);
                                        let row_resp = ui
                                            .selectable_label(false, op_short)
                                            .on_hover_text(format!(
                                                "{op_full}\nClick to focus this Operation in the right Inspector"
                                            ));
                                        if row_resp.clicked() {
                                            panel_action =
                                                Some(DiagnosticsPanelAction::FocusOperation {
                                                    setup_idx: active_setup_idx,
                                                    op_idx,
                                                });
                                        }

                                        let tool_full = rep
                                            .tool_name
                                            .as_deref()
                                            .map(sanitize_physics_text)
                                            .unwrap_or_else(|| "(none)".to_string());
                                        let tool_short = truncate_cell_label(&tool_full, 20);
                                        ui.label(
                                            egui::RichText::new(tool_short)
                                                .small()
                                                .color(tok.text_secondary),
                                        )
                                        .on_hover_text(tool_full);

                                        if let Some(m) = &rep.metrics {
                                            let ld_col = if m.deflection.stickout_ld_ratio > 6.0 {
                                                tok.warn_fg
                                            } else {
                                                tok.text_primary
                                            };
                                            ui.label(
                                                egui::RichText::new(format!(
                                                    "{:.1}x",
                                                    m.deflection.stickout_ld_ratio
                                                ))
                                                .small()
                                                .monospace()
                                                .color(ld_col),
                                            );

                                            let (sp_txt, sp_col) = match m.spindle_load_pct {
                                                Some(pct) if pct > 100.0 => {
                                                    (format!("{pct:.0}%"), tok.error_fg)
                                                }
                                                Some(pct) if pct > 70.0 => {
                                                    (format!("{pct:.0}%"), tok.warn_fg)
                                                }
                                                Some(pct) => {
                                                    (format!("{pct:.0}%"), tok.text_primary)
                                                }
                                                None => ("-".to_string(), tok.text_muted),
                                            };
                                            ui.label(
                                                egui::RichText::new(sp_txt)
                                                    .small()
                                                    .monospace()
                                                    .color(sp_col),
                                            );

                                            let defl_col = if m.deflection.tip_deflection_mm > 0.060
                                            {
                                                tok.error_fg
                                            } else if m.deflection.tip_deflection_mm > 0.025 {
                                                tok.warn_fg
                                            } else {
                                                tok.text_primary
                                            };
                                            ui.label(
                                                egui::RichText::new(format!(
                                                    "{:.1}",
                                                    m.deflection.tip_deflection_um
                                                ))
                                                .small()
                                                .monospace()
                                                .color(defl_col),
                                            );

                                            ui.label(
                                                egui::RichText::new(format!(
                                                    "{:.3}",
                                                    m.effective_chip_thickness_mm
                                                ))
                                                .small()
                                                .monospace()
                                                .color(tok.text_primary),
                                            );
                                        } else {
                                            for _ in 0..4 {
                                                ui.label(
                                                    egui::RichText::new("-")
                                                        .small()
                                                        .color(tok.text_muted),
                                                );
                                            }
                                        }

                                        let (st_txt, st_col) = match rep.severity {
                                            MachinabilitySeverity::Critical => ("CRIT", tok.error_fg),
                                            MachinabilitySeverity::Warning => ("WARN", tok.warn_fg),
                                            MachinabilitySeverity::Info => ("INFO", tok.info_fg),
                                            MachinabilitySeverity::Ok => ("OK", tok.clean_fg),
                                        };
                                        ui.label(
                                            egui::RichText::new(st_txt)
                                                .small()
                                                .strong()
                                                .color(st_col),
                                        );

                                        if let Some(finding) = primary_fixable_finding(rep)
                                            && let Some(fix) = &finding.suggested_fix
                                        {
                                            let fix_tip = format!(
                                                "{}: {}",
                                                sanitize_physics_text(&finding.title),
                                                sanitize_physics_text(&fix.label())
                                            );
                                            if ui
                                                .push_id(("diag_mach_fix", active_setup_idx, op_idx), |ui| {
                                                    ui.small_button("Apply Fix")
                                                        .on_hover_text(fix_tip)
                                                        .clicked()
                                                })
                                                .inner
                                            {
                                                panel_action = Some(
                                                    DiagnosticsPanelAction::ApplyMachinabilityFix {
                                                        setup_idx: active_setup_idx,
                                                        op_idx,
                                                        fix: fix.clone(),
                                                    },
                                                );
                                            }
                                        } else {
                                            ui.label(
                                                egui::RichText::new("-")
                                                    .small()
                                                    .color(tok.text_muted),
                                            );
                                        }
                                        ui.end_row();
                                    }
                                });
                        }
                        ui.separator();
                    }

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
                                        panel_action = Some(
                                            DiagnosticsPanelAction::JumpToSimDiagnostic(
                                                diag.clone(),
                                            ),
                                        );
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

    panel_action
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
        let tools = ToolLibrary::default();
        let mut out = ctx.run_ui(input, |ui| {
            egui::CentralPanel::default().show(ui, |ui| {
                let _ = show_diagnostics_panel(
                    ui, project, 0, &tools, cache, 0, None, None, None, logs,
                );
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

    #[test]
    fn machinability_table_renders_and_routes_row_click_and_fix() {
        use chip_cam::{Operation, OperationKind, Setup, StockConfig, WorkOffsetSlot};
        use chip_math::ParamLength;
        use chip_tools::{
            ALUMINUM_6063_T5_ID, CutterGeometry, ProjectMaterialSelection, ToolId,
            builtin_carvera_air_machine,
        };

        let machine = builtin_carvera_air_machine();
        let mut project = ProjectDocument::new("DiagMach");
        project.workpiece_material = Some(ProjectMaterialSelection::builtin(ALUMINUM_6063_T5_ID));
        project.machine_id = Some(machine.id.clone());

        let mut t1 = machine.tools[0].clone();
        t1.id = ToolId("t_long".to_string());
        t1.name = "3.175mm 1FL Long".to_string();
        t1.geometry = CutterGeometry::FlatEndmill {
            diameter_mm: 3.175,
            flutes: 1,
            flute_length_mm: 25.0,
            shank_diameter_mm: 3.175,
            overall_length_mm: 55.0,
        };
        t1.stickout_length_mm = 30.0;
        t1.presets.clear();
        project.tools = vec![t1.clone()];

        let mut setup = Setup::new("s1", "Setup 1", WorkOffsetSlot::G54);
        setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(80.0),
            depth_y: ParamLength::from_mm(60.0),
            height_z: ParamLength::from_mm(25.0),
            xy_align: Default::default(),
            z_align: Default::default(),
        };
        let mut op = Operation::new("op_0", "Hazard Pocket", OperationKind::Pocket2D);
        op.tool_id = Some(t1.id);
        op.passes.multiple_depths = true;
        op.passes.max_stepdown = ParamLength::from_mm(6.0);
        op.feed_overrides.spindle_rpm = Some(12_000.0);
        op.feed_overrides.cutting_feed_mm_min = Some(900.0);
        op.feed_overrides.stepdown_mm = Some(6.0);
        op.feed_overrides.stepover_mm = Some(2.0);
        setup.operations = vec![op];
        project.setups = vec![setup];

        let tools = ToolLibrary::default();
        let cache = ToolpathCache::new(16);
        let ctx = egui::Context::default();
        let screen = egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(1280.0, 420.0));

        let run = |events: Vec<egui::Event>| -> (Vec<(String, egui::Rect)>, Option<DiagnosticsPanelAction>) {
            let mut act = None;
            let mut out = ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(screen),
                    events,
                    ..Default::default()
                },
                |ui| {
                    egui::CentralPanel::default().show(ui, |ui| {
                        act = show_diagnostics_panel(
                            ui,
                            &project,
                            0,
                            &tools,
                            &cache,
                            0,
                            None,
                            Some(&machine),
                            None,
                            &[],
                        );
                    });
                },
            );
            out.textures_delta.clear();
            let mut rects = Vec::new();
            fn visit(shape: &egui::Shape, out: &mut Vec<(String, egui::Rect)>) {
                match shape {
                    egui::Shape::Text(t) => {
                        out.push((t.galley.text().to_owned(), t.visual_bounding_rect()));
                    }
                    egui::Shape::Vec(v) => {
                        for s in v {
                            visit(s, out);
                        }
                    }
                    _ => {}
                }
            }
            for cs in &out.shapes {
                visit(&cs.shape, &mut rects);
            }
            (rects, act)
        };

        let _ = run(Vec::new());
        let (rects, _) = run(Vec::new());
        assert!(
            rects
                .iter()
                .any(|(t, _)| t == "Machinability & Tool Safety"),
            "Diagnostics panel must render Machinability & Tool Safety heading"
        );
        assert!(
            rects.iter().any(|(t, _)| t == "CRIT"),
            "Diagnostics machinability table must render CRIT status badge"
        );

        // Clicking operation name routes to FocusOperation
        let op_pos = rects
            .iter()
            .find(|(t, _)| t == "Hazard Pocket")
            .expect("Operation row must render 'Hazard Pocket'")
            .1
            .center();
        let (_, focus_act) = run(vec![
            egui::Event::PointerMoved(op_pos),
            egui::Event::PointerButton {
                pos: op_pos,
                button: egui::PointerButton::Primary,
                pressed: true,
                modifiers: egui::Modifiers::NONE,
            },
            egui::Event::PointerButton {
                pos: op_pos,
                button: egui::PointerButton::Primary,
                pressed: false,
                modifiers: egui::Modifiers::NONE,
            },
        ]);
        assert_eq!(
            focus_act,
            Some(DiagnosticsPanelAction::FocusOperation {
                setup_idx: 0,
                op_idx: 0
            })
        );

        // Clicking Apply Fix emits ApplyMachinabilityFix
        let fix_pos = rects
            .iter()
            .find(|(t, _)| t == "Apply Fix")
            .expect("Hazardous operation row must render 'Apply Fix' button")
            .1
            .center();
        let (_, fix_act) = run(vec![
            egui::Event::PointerMoved(fix_pos),
            egui::Event::PointerButton {
                pos: fix_pos,
                button: egui::PointerButton::Primary,
                pressed: true,
                modifiers: egui::Modifiers::NONE,
            },
            egui::Event::PointerButton {
                pos: fix_pos,
                button: egui::PointerButton::Primary,
                pressed: false,
                modifiers: egui::Modifiers::NONE,
            },
        ]);
        assert!(
            matches!(
                fix_act,
                Some(DiagnosticsPanelAction::ApplyMachinabilityFix {
                    setup_idx: 0,
                    op_idx: 0,
                    ..
                })
            ),
            "Clicking Apply Fix must emit ApplyMachinabilityFix; got {fix_act:?}"
        );
    }
}
