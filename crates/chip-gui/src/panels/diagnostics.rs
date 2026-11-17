use chip_cam::OperationStatus;
use chip_project::{ProjectDocument, ToolpathCache};

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

/// Renders the bottom **Diagnostics & Toolpath Summary** pane (`08-ui-viewport.md`).
pub fn show_diagnostics_panel(
    ui: &mut egui::Ui,
    project: &ProjectDocument,
    active_setup_idx: usize,
    cache: &ToolpathCache,
    cad_revision: u64,
    logs: &[String],
) {
    let u = project.display_units;

    ui.columns(2, |cols| {
        // Left column: Active Setup Toolpath Summary & Operation Warnings/Errors
        cols[0].strong("Active Setup Toolpath Summary");
        cols[0].separator();

        if let Some(setup) = project.setups.get(active_setup_idx) {
            let mut total_sec = 0.0;
            let mut total_cut_mm = 0.0;
            egui::ScrollArea::vertical()
                .id_salt("diag_tp_summary")
                .show(&mut cols[0], |ui| {
                    for (o_idx, op) in setup.operations.iter().enumerate() {
                        if op.suppressed {
                            ui.weak(format!("[-] {}. {} (Suppressed)", o_idx + 1, op.name));
                            continue;
                        }
                        if let Some((_, tp)) =
                            cache.get_for_operation(project, active_setup_idx, o_idx, cad_revision)
                        {
                            total_sec += tp.stats.total_est_time_sec;
                            total_cut_mm += tp.stats.cutting_distance_mm;
                            ui.horizontal_wrapped(|ui| {
                                ui.colored_label(
                                    egui::Color32::from_rgb(86, 224, 134),
                                    format!("[OK] {}. {}", o_idx + 1, op.name),
                                );
                                ui.weak(format!(
                                    "— {} pass(es) | {} G1 | {} G2/G3 | Cut {} | Rapid {} | Est {}",
                                    tp.stats.pass_count,
                                    tp.stats.linear_move_count,
                                    tp.stats.arc_move_count,
                                    u.format_length(tp.stats.cutting_distance_mm),
                                    u.format_length(tp.stats.rapid_distance_mm),
                                    format_cycle_time(tp.stats.total_est_time_sec),
                                ));
                            });
                            for w in &tp.warnings {
                                ui.colored_label(
                                    egui::Color32::from_rgb(248, 196, 76),
                                    format!("   [WARN] {w}"),
                                );
                            }
                        } else {
                            match &op.runtime_status {
                                OperationStatus::Error { message }
                                | OperationStatus::NeedsRepick { message } => {
                                    ui.colored_label(
                                        egui::Color32::from_rgb(248, 96, 96),
                                        format!("[!] {}. {}: {message}", o_idx + 1, op.name),
                                    );
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
                                    ui.colored_label(
                                        egui::Color32::from_rgb(244, 188, 76),
                                        format!(
                                            "[*] {}. {} — Dirty (press Ctrl+G to generate)",
                                            o_idx + 1,
                                            op.name
                                        ),
                                    );
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
        cols[1].strong("Activity Log");
        cols[1].separator();
        egui::ScrollArea::vertical()
            .id_salt("diag_activity_log")
            .stick_to_bottom(true)
            .show(&mut cols[1], |ui| {
                for entry in logs {
                    ui.monospace(entry);
                }
            });
    });
}
