use chip_cad::CadDocument;
use chip_cam::OperationStatus;
use chip_project::{ProjectDocument, ToolpathCache};
use chip_sim::{SetupSimulationResult, SimDiagnostic, SimSeverity};
use chip_tools::MachineDefinition;

use crate::{
    theme::ThemeTokens,
    widgets::{StatusBadgeKind, status_badge_icon},
};

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
                                    "Simulation Verified: 0 crashes, 0 gouges, 0 warnings.",
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
