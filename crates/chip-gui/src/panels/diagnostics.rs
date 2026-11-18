use chip_cam::OperationStatus;
use chip_project::{ProjectDocument, ToolpathCache};
use chip_sim::{SetupSimulationResult, SimDiagnostic, SimSeverity};

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
pub fn show_diagnostics_panel(
    ui: &mut egui::Ui,
    project: &ProjectDocument,
    active_setup_idx: usize,
    cache: &ToolpathCache,
    cad_revision: u64,
    sim_result: Option<&SetupSimulationResult>,
    logs: &[String],
) -> Option<SimDiagnostic> {
    let u = project.display_units;
    let mut jump_target: Option<SimDiagnostic> = None;

    ui.columns(2, |cols| {
        // Left column: Simulation Crash/Gouge Diagnostics + Active Setup Toolpath Summary
        cols[0].strong("Simulation Verification & Toolpath Summary");
        cols[0].separator();

        if let Some(setup) = project.setups.get(active_setup_idx) {
            let mut total_sec = 0.0;
            let mut total_cut_mm = 0.0;
            egui::ScrollArea::vertical()
                .id_salt("diag_tp_summary")
                .show(&mut cols[0], |ui| {
                    if let Some(sim) = sim_result {
                        if sim.diagnostics.is_empty() {
                            ui.colored_label(
                                egui::Color32::from_rgb(86, 224, 134),
                                "[OK] Simulation Verified: 0 crashes, 0 gouges, 0 warnings.",
                            );
                        } else {
                            for diag in &sim.diagnostics {
                                let col = match diag.severity {
                                    SimSeverity::CriticalError => {
                                        egui::Color32::from_rgb(252, 92, 92)
                                    }
                                    SimSeverity::Warning => egui::Color32::from_rgb(248, 198, 76),
                                };
                                ui.horizontal_wrapped(|ui| {
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
                                            "{} [{}] {} (Seg #{}, t={:.2}s): {}",
                                            diag.severity.badge(),
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
                        ui.separator();
                    }

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

    jump_target
}
