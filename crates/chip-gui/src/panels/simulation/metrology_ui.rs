use chip_math::DVec3;
use chip_sim::{ScrubSnapshot, SetupSimulationResult, StockErrorReport, deviation_to_heatmap_rgba};

use super::SimulationUiAction;

/// Renders the quantitative Stock-vs-Target Error-Volume & RMS Metrology dashboard inside the
/// Simulation Timeline pane (`06-simulation.md` / Phase 11 M40).
pub(super) fn show_error_metrics_dashboard(
    ui: &mut egui::Ui,
    res: &SetupSimulationResult,
    rep: &StockErrorReport,
    snap: Option<&ScrubSnapshot>,
    heatmap_tolerance_mm: &mut f64,
    heatmap_max_scale_mm: f64,
    show_error_heatmap: &mut bool,
) -> Option<SimulationUiAction> {
    let mut action = None;
    ui.add_space(1.0);
    let tok = crate::theme::ThemeTokens::from_ui(ui);
    let snap_rot_deg = snap.map_or(0.0, |s| s.rotary_deg);
    let locate_wcs = |loc: DVec3| -> DVec3 {
        if let Some(cyl) = res.as_cylindrical()
            && snap_rot_deg.abs() > 1e-6
        {
            cyl.rotary.rotate_point_wcs(loc, snap_rot_deg)
        } else {
            loc
        }
    };
    let engine_tag = if res.is_cylindrical() {
        "4-axis polar"
    } else {
        res.engine.short_slug()
    };

    egui::Frame::group(ui.style())
        .inner_margin(egui::Margin::symmetric(6, 3))
        .show(ui, |ui| {
            ui.spacing_mut().item_spacing.y = 2.0;
            ui.horizontal_wrapped(|ui| {
                ui.strong(format!("Error Metrics ({engine_tag})"));
                ui.separator();
                ui.monospace(format!(
                    "Err Vol: {:.1} mm^3 ({:.2}%) | Excess: {:.1} | Gouge: {:.1} | Eff: {:.1}%",
                    rep.total_error_volume_mm3,
                    rep.error_volume_pct_of_target,
                    rep.excess_volume_mm3,
                    rep.gouge_volume_mm3,
                    rep.material_removal_efficiency_pct
                ));
                ui.separator();
                ui.label("Tol:")
                    .on_hover_text("Tolerance band (+/- mm) shown in green on the Error Heatmap and used for In-Tol %");
                ui.add(
                    egui::DragValue::new(heatmap_tolerance_mm)
                        .speed(0.005)
                        .range(0.005..=5.0)
                        .fixed_decimals(3)
                        .prefix("+/- ")
                        .suffix(" mm"),
                )
                .on_hover_text(
                    "Adjust surface deviation pass tolerance (+/- mm); updates both Error Heatmap green band and In-Tol % live",
                );
                ui.separator();
                ui.checkbox(show_error_heatmap, "Heatmap")
                    .on_hover_text(
                        "Enable or disable the 3D Error Heatmap overlay (uncheck for faster stock display)",
                    );
                paint_heatmap_legend(ui, *heatmap_tolerance_mm, heatmap_max_scale_mm);
            });

            ui.horizontal_wrapped(|ui| {
                ui.monospace(format!(
                    "RMS: {:.4} mm | MAE: {:.4} | Bias: {:+.4} | P95: {:.3} | P99: {:.3}",
                    rep.rms_error_mm,
                    rep.mean_abs_error_mm,
                    rep.mean_signed_error_mm,
                    rep.p95_abs_error_mm,
                    rep.p99_abs_error_mm
                ));
                ui.separator();
                ui.colored_label(
                    if rep.in_tolerance_surface_pct >= 90.0 {
                        tok.clean_fg
                    } else {
                        tok.warn_fg
                    },
                    egui::RichText::new(format!("In-Tol: {:.1}%", rep.in_tolerance_surface_pct))
                        .monospace()
                        .strong(),
                );
                ui.separator();
                ui.monospace(format!("Max Excess: +{:.3} mm", rep.max_excess_mm));
                if let Some(loc) = rep.max_excess_location_wcs
                    && ui
                        .small_button("Locate")
                        .on_hover_text(format!(
                            "Center camera on worst uncut excess at WCS ({:.2}, {:.2}, {:.2})",
                            loc.x, loc.y, loc.z
                        ))
                        .clicked()
                {
                    action = Some(SimulationUiAction::CenterCameraWcs(locate_wcs(loc)));
                }
                ui.separator();
                ui.monospace(format!("Max Gouge: -{:.3} mm", rep.max_gouge_mm));
                if let Some(loc) = rep.max_gouge_location_wcs
                    && rep.max_gouge_mm > 1e-4
                    && ui
                        .small_button("Locate")
                        .on_hover_text(format!(
                            "Center camera on deepest gouge at WCS ({:.2}, {:.2}, {:.2})",
                            loc.x, loc.y, loc.z
                        ))
                        .clicked()
                {
                    action = Some(SimulationUiAction::CenterCameraWcs(locate_wcs(loc)));
                }
            });

            if !rep.per_operation.is_empty() {
                egui::CollapsingHeader::new(format!(
                    "Per-Operation Error Progression ({} ops)",
                    rep.per_operation.len()
                ))
                .id_salt("sim_per_op_error_table")
                .default_open(false)
                .show(ui, |ui| {
                    egui::Grid::new("sim_per_op_error_grid")
                        .striped(true)
                        .spacing([12.0, 2.0])
                        .show(ui, |ui| {
                            ui.strong("Operation");
                            ui.strong("Removed (mm^3)");
                            ui.strong("Excess (mm^3)");
                            ui.strong("Gouge (mm^3)");
                            ui.strong("Total Err (mm^3)");
                            ui.strong("RMS (mm)");
                            ui.end_row();
                            for (idx, snap_op) in rep.per_operation.iter().enumerate() {
                                ui.label(format!("{}. {}", idx + 1, snap_op.op_name));
                                ui.monospace(format!("{:.1}", snap_op.removed_volume_mm3));
                                ui.monospace(format!("{:.1}", snap_op.excess_volume_mm3));
                                ui.monospace(format!("{:.1}", snap_op.gouge_volume_mm3));
                                ui.monospace(format!("{:.1}", snap_op.total_error_volume_mm3));
                                ui.monospace(format!("{:.4}", snap_op.rms_error_mm));
                                ui.end_row();
                            }
                        });
                });
            }
        });

    action
}

fn paint_heatmap_legend(ui: &mut egui::Ui, tol_mm: f64, max_scale_mm: f64) {
    let scale = max_scale_mm.max(tol_mm * 2.0);
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 4.0;
        ui.small("Gouge (-)");
        let bar_w = 84.0;
        let bar_h = 10.0;
        let (rect, _resp) = ui.allocate_exact_size(egui::vec2(bar_w, bar_h), egui::Sense::hover());
        let painter = ui.painter();
        let steps = 21usize;
        let sw = rect.width() / steps as f32;
        for s in 0..steps {
            let u = (s as f64 + 0.5) / steps as f64;
            let dev = (u * 2.0 - 1.0) * scale;
            let rgba = deviation_to_heatmap_rgba(dev, tol_mm, scale);
            let col = egui::Color32::from_rgb(
                (rgba[0].clamp(0.0, 1.0) * 255.0) as u8,
                (rgba[1].clamp(0.0, 1.0) * 255.0) as u8,
                (rgba[2].clamp(0.0, 1.0) * 255.0) as u8,
            );
            let r = egui::Rect::from_min_max(
                egui::pos2(rect.left() + s as f32 * sw, rect.top()),
                egui::pos2(rect.left() + (s + 1) as f32 * sw + 0.5, rect.bottom()),
            );
            painter.rect_filled(r, 0.0, col);
        }
        painter.rect_stroke(
            rect,
            1.0,
            egui::Stroke::new(1.0, ui.visuals().widgets.noninteractive.bg_stroke.color),
            egui::StrokeKind::Outside,
        );
        ui.small("Excess (+)");
    });
}
