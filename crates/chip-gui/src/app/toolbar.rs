use chip_cam::OperationKind;

use crate::{
    app::ChipCutterApp,
    layout::{WorkspacePane, focus_workspace_pane},
    theme::ThemeTokens,
    widgets::{IconKind, StatusBadgeKind, icon_button, primary_icon_button, status_badge_icon},
};

impl ChipCutterApp {
    pub(crate) fn draw_top_bars(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        self.sync_window_title(&ctx);
        self.handle_global_shortcuts(&ctx);
        self.draw_menu_bar(ui, &ctx);
        self.draw_quick_toolbar(ui, &ctx);
        self.draw_status_banners(ui, &ctx);
    }

    fn draw_quick_toolbar(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        let tok = ThemeTokens::for_theme(self.theme);
        egui::Panel::top("chip_toolbar")
            .frame(
                egui::Frame::NONE
                    .fill(tok.toolbar_bg)
                    .stroke(egui::Stroke::new(1.0, tok.border))
                    .inner_margin(egui::Margin::symmetric(8, 5)),
            )
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 3.0;

                    if icon_button(
                        ui,
                        IconKind::FolderOpen,
                        Some("Open"),
                        "Open .chip.toml Project or .chipz Bundle (Ctrl+O)",
                    )
                    .clicked()
                    {
                        self.open_project_dialog(ctx);
                    }
                    if icon_button(
                        ui,
                        IconKind::Save,
                        Some("Save"),
                        "Save .chip.toml Project (Ctrl+S)",
                    )
                    .clicked()
                    {
                        self.save_project_dialog();
                    }
                    if icon_button(
                        ui,
                        IconKind::BundleZip,
                        Some(".chipz"),
                        "Export Self-Contained Portable Bundle (.chipz)",
                    )
                    .clicked()
                    {
                        self.export_chipz_dialog();
                    }
                    if icon_button(
                        ui,
                        IconKind::VectorImport,
                        Some("DXF/SVG"),
                        "Import 2D Vector Drawing (.dxf / .svg) or Start Pure-2D Job",
                    )
                    .clicked()
                    {
                        self.modals.vector_import.open = true;
                    }
                    if self.model.is_some()
                        && icon_button(
                            ui,
                            IconKind::Reload,
                            None,
                            "Associatively Reload Active STEP Model (Ctrl+R)",
                        )
                        .clicked()
                    {
                        self.reload_active_step_model(ctx);
                    }

                    ui.add_enabled_ui(self.history.can_undo(), |ui| {
                        if icon_button(ui, IconKind::Undo, None, "Undo (Ctrl+Z)").clicked() {
                            self.perform_undo();
                        }
                    });
                    ui.add_enabled_ui(self.history.can_redo(), |ui| {
                        if icon_button(ui, IconKind::Redo, None, "Redo (Ctrl+Y)").clicked() {
                            self.perform_redo();
                        }
                    });

                    ui.separator();

                    let active_setup_name = self
                        .project
                        .setups
                        .get(self.active_setup_idx)
                        .map_or("Active Setup", |s| s.name.as_str())
                        .to_string();
                    if icon_button(
                        ui,
                        IconKind::Sketch2D,
                        Some("+ Sketch"),
                        &format!("Create a New 2D Sketch Layer on {active_setup_name} XY"),
                    )
                    .clicked()
                    {
                        self.create_new_setup_xy_sketch();
                    }
                    if icon_button(
                        ui,
                        IconKind::Adaptive2D,
                        Some("+ Adaptive"),
                        &format!("Add 2D Adaptive / Trochoidal Clearing to {active_setup_name}"),
                    )
                    .clicked()
                    {
                        self.create_new_adaptive_operation();
                    }
                    if icon_button(
                        ui,
                        IconKind::Contour2D,
                        Some("+ Contour"),
                        &format!("Add 2D Contour / Profile operation to {active_setup_name}"),
                    )
                    .clicked()
                    {
                        self.add_standard_operation(OperationKind::Contour2D);
                    }
                    if icon_button(
                        ui,
                        IconKind::VCarve,
                        Some("+ V-Carve"),
                        &format!("Add 3D Medial-Axis V-Carve / Inlay operation to {active_setup_name}"),
                    )
                    .clicked()
                    {
                        self.create_new_vcarve_operation();
                    }
                    ui.menu_button("+ Op…", |ui| {
                        ui.label(
                            egui::RichText::new(format!("Add to {active_setup_name}"))
                                .small()
                                .strong()
                                .color(tok.text_muted),
                        );
                        ui.separator();
                        ui.label(egui::RichText::new("2D / 2.5D").small().strong());
                        for (lbl, kind) in [
                            ("Facing", OperationKind::Facing),
                            ("2D Pocket", OperationKind::Pocket2D),
                            ("Drill / Helical Bore", OperationKind::DrillBore),
                            ("Thread Mill", OperationKind::ThreadMill),
                            ("Slot / Keyway / T-Slot", OperationKind::SlotUndercut),
                            ("Dovetail Slot / Rail", OperationKind::Dovetail),
                            ("Edge Chamfer / Trace", OperationKind::ChamferTrace),
                        ] {
                            if ui.button(lbl).clicked() {
                                self.add_standard_operation(kind);
                                ui.close();
                            }
                        }
                        ui.separator();
                        ui.label(egui::RichText::new("3D Surface").small().strong());
                        for (lbl, kind) in [
                            ("3D Adaptive Roughing", OperationKind::Adaptive3D),
                            ("3D Waterline / Z-Level", OperationKind::Waterline3D),
                            ("3D Parallel Finishing", OperationKind::Parallel3D),
                            ("3D Constant-Scallop", OperationKind::Scallop3D),
                            ("3D Pencil Cleanup", OperationKind::Pencil3D),
                            ("3D Edge Deburr / Chamfer", OperationKind::Deburr3D),
                            ("3D Lollipop Undercut", OperationKind::Undercut3D),
                        ] {
                            if ui.button(lbl).clicked() {
                                self.add_standard_operation(kind);
                                ui.close();
                            }
                        }
                        ui.separator();
                        ui.label(egui::RichText::new("4th-Axis Rotary").small().strong());
                        for (lbl, kind) in [
                            ("4th-Axis Rotary Surfacing", OperationKind::Rotary2Linear),
                            ("Simultaneous 4-Axis (X+Y+Z+A)", OperationKind::Rotary4AxisSim),
                        ] {
                            if ui.button(lbl).clicked() {
                                self.add_standard_operation(kind);
                                ui.close();
                            }
                        }
                    })
                    .response
                    .on_hover_text(format!(
                        "Add another 2.5D, 3D surfacing, or 4th-axis operation to {active_setup_name}"
                    ));

                    ui.separator();

                    if self.dispatcher.is_busy() {
                        if icon_button(
                            ui,
                            IconKind::CancelStop,
                            Some("Cancel"),
                            "Cancel In-Flight Toolpath Generation (Esc)",
                        )
                        .clicked()
                        {
                            self.dispatcher.cancel_all();
                        }
                    } else if primary_icon_button(
                        ui,
                        IconKind::Generate,
                        "Generate",
                        "Generate Dirty Toolpaths in Active Setup (Ctrl+G)",
                    )
                    .clicked()
                    {
                        self.trigger_generate_dirty(Some(self.active_setup_idx), None);
                    }

                    if self.sim_panel.is_busy() {
                        if icon_button(
                            ui,
                            IconKind::CancelStop,
                            Some("Cancel"),
                            "Cancel In-Flight Simulation (Esc)",
                        )
                        .clicked()
                        {
                            self.sim_panel.cancel_in_flight();
                        }
                    } else if icon_button(
                        ui,
                        IconKind::Simulate,
                        Some("Simulate"),
                        "Run Stock Removal Simulation & 6-Rule Crash/Gouge/Fixture Verification",
                    )
                    .clicked()
                    {
                        focus_workspace_pane(&mut self.tree, WorkspacePane::SimulationTimeline);
                        self.trigger_active_setup_simulation(ctx);
                    }
                    if icon_button(
                        ui,
                        IconKind::PostGCode,
                        Some("Post"),
                        "Open G-Code Preview & Safety-Gated Export",
                    )
                    .clicked()
                    {
                        focus_workspace_pane(&mut self.tree, WorkspacePane::GCodePreview);
                    }

                    let job_progress = resolve_toolbar_job_progress(
                        self.dispatcher.batch_progress(),
                        self.sim_panel.in_flight_progress(),
                    );
                    if toolbar_progress_bar(ui, job_progress.as_ref()).clicked()
                        && self.sim_panel.is_busy()
                    {
                        focus_workspace_pane(&mut self.tree, WorkspacePane::SimulationTimeline);
                    }

                    ui.separator();

                    if icon_button(
                        ui,
                        IconKind::Tool,
                        Some("Tools"),
                        "Open Tool Library & Machine Tools Manager",
                    )
                    .clicked()
                    {
                        self.modals.tool_library.open = true;
                    }

                    // Right-aligned Utility controls (Command Palette & Settings)
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.spacing_mut().item_spacing.x = 4.0;
                        if icon_button(
                            ui,
                            IconKind::Settings,
                            None,
                            "Open Settings & Preferences (Ctrl+,)",
                        )
                        .clicked()
                        {
                            self.modals.settings.open = true;
                        }

                        ui.separator();

                        if icon_button(
                            ui,
                            IconKind::Search,
                            None,
                            "Open Fuzzy Command Palette (Ctrl+K)",
                        )
                        .clicked()
                        {
                            self.modals.command_palette.open_fresh();
                        }
                    });
                });
            });
    }

    fn draw_status_banners(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        let tok = ThemeTokens::for_theme(self.theme);
        if self.external_step_modified {
            egui::Panel::top("chip_external_step_banner")
                .frame(
                    egui::Frame::NONE
                        .fill(tok.dirty_bg)
                        .stroke(egui::Stroke::new(1.0, tok.dirty_border))
                        .inner_margin(egui::Margin::symmetric(10, 4)),
                )
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        status_badge_icon(ui, StatusBadgeKind::Dirty, None);
                        ui.colored_label(
                            tok.dirty_fg,
                            "External STEP file updated on disk — CAD revision change detected.",
                        );
                        if ui
                            .small_button("Reload & Re-Match (Ctrl+R)")
                            .on_hover_text("Reload the modified STEP model from disk and associatively re-bind CAM faces and edges")
                            .clicked()
                        {
                            self.reload_active_step_model(ctx);
                        }
                        if ui
                            .small_button("Dismiss")
                            .on_hover_text("Hide this file-change notification without reloading")
                            .clicked()
                        {
                            self.external_step_modified = false;
                        }
                    });
                });
        }

        if let Some((msg, has_warn)) = self.last_reload_banner.clone() {
            let (bg, border, fg, badge) = if has_warn {
                (
                    tok.warn_bg,
                    tok.warn_border,
                    tok.warn_fg,
                    StatusBadgeKind::NeedsRepick,
                )
            } else {
                (
                    tok.clean_bg,
                    tok.clean_border,
                    tok.clean_fg,
                    StatusBadgeKind::Clean,
                )
            };
            egui::Panel::top("chip_reload_summary_banner")
                .frame(
                    egui::Frame::NONE
                        .fill(bg)
                        .stroke(egui::Stroke::new(1.0, border))
                        .inner_margin(egui::Margin::symmetric(10, 4)),
                )
                .show(ui, |ui| {
                    ui.horizontal_wrapped(|ui| {
                        status_badge_icon(ui, badge, None);
                        ui.colored_label(fg, &msg);
                        if ui
                            .small_button("Regenerate Dirty (Ctrl+G)")
                            .on_hover_text("Recompute all toolpaths invalidated by the CAD reload and refresh simulation")
                            .clicked()
                        {
                            self.auto_sim_after_gen = true;
                            self.trigger_generate_dirty(None, None);
                        }
                        if ui
                            .small_button("Dismiss")
                            .on_hover_text("Close this reload summary banner")
                            .clicked()
                        {
                            self.last_reload_banner = None;
                        }
                    });
                });
        }
    }
}

/// Consolidated progress state rendered in the single small toolbar progress bar next to
/// `[Generate]`, `[Simulate]`, and `[Post]`.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct ToolbarJobProgress {
    pub fraction: f32,
    pub percent: u8,
    pub tooltip: String,
}

/// Selects the active progress snapshot to display in the single toolbar progress bar.
pub(crate) fn resolve_toolbar_job_progress(
    gen_prog: Option<chip_project::GenerationProgressSnapshot>,
    sim_prog: Option<chip_sim::SimProgressSnapshot>,
) -> Option<ToolbarJobProgress> {
    if let Some(prog) = sim_prog {
        let pct = prog.percent_rounded();
        Some(ToolbarJobProgress {
            fraction: prog.fraction.clamp(0.0, 1.0),
            percent: pct,
            tooltip: format!("Simulation running: {pct}% ({})", prog.stage.label()),
        })
    } else if let Some(prog) = gen_prog {
        let pct = prog.percent_rounded();
        let tooltip = if prog.total > 0 {
            format!(
                "Generating toolpaths: {}/{} ({pct}%)",
                prog.completed, prog.total
            )
        } else {
            format!("Generating toolpaths: {pct}%")
        };
        Some(ToolbarJobProgress {
            fraction: prog.fraction.clamp(0.0, 1.0),
            percent: pct,
            tooltip,
        })
    } else {
        None
    }
}

/// Renders a single compact progress bar next to `[Generate] [Simulate] [Post]` on the top bar.
fn toolbar_progress_bar(
    ui: &mut egui::Ui,
    progress: Option<&ToolbarJobProgress>,
) -> egui::Response {
    let tok = ThemeTokens::from_ui(ui);
    let size = egui::Vec2::new(56.0, 20.0);
    let sense = if progress.is_some() {
        egui::Sense::click()
    } else {
        egui::Sense::hover()
    };
    let (rect, resp) = ui.allocate_exact_size(size, sense);

    if ui.is_rect_visible(rect) {
        let painter = ui.painter();
        let radius = egui::CornerRadius::same(4);
        painter.rect(
            rect,
            radius,
            tok.card_bg,
            egui::Stroke::new(1.0, tok.border),
            egui::StrokeKind::Inside,
        );

        if let Some(prog) = progress {
            let frac = prog.fraction.clamp(0.0, 1.0);
            let inner = rect.shrink(1.0);
            let fill_w = (inner.width() * frac).clamp(0.0, inner.width());
            let fill_rect =
                egui::Rect::from_min_size(inner.min, egui::Vec2::new(fill_w, inner.height()));
            if fill_w > 0.5 {
                painter.rect_filled(fill_rect, egui::CornerRadius::same(3), tok.accent);
            }

            let label = format!("{}%", prog.percent);
            let font = egui::FontId::proportional(11.5);
            let unfilled_rect = egui::Rect::from_min_max(
                egui::Pos2::new(fill_rect.right(), inner.top()),
                inner.max,
            );
            if unfilled_rect.width() > 0.5 {
                painter.with_clip_rect(unfilled_rect).text(
                    rect.center(),
                    egui::Align2::CENTER_CENTER,
                    &label,
                    font.clone(),
                    tok.text_primary,
                );
            }
            if fill_rect.width() > 0.5 {
                painter.with_clip_rect(fill_rect).text(
                    rect.center(),
                    egui::Align2::CENTER_CENTER,
                    &label,
                    font,
                    tok.accent_on_fg,
                );
            }
        }
    }

    let tip = progress.map_or("Progress for Generate, Simulate, and Post", |p| {
        p.tooltip.as_str()
    });
    resp.on_hover_text(tip)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_project::GenerationProgressSnapshot;
    use chip_sim::{SimProgressSnapshot, SimProgressStage};

    use super::{resolve_toolbar_job_progress, toolbar_progress_bar};

    #[test]
    fn resolve_toolbar_job_progress_handles_idle_generation_and_simulation() {
        assert!(resolve_toolbar_job_progress(None, None).is_none());

        let gen_snap = GenerationProgressSnapshot {
            completed: 2,
            total: 4,
            fraction: 0.5,
        };
        let gen_prog = resolve_toolbar_job_progress(Some(gen_snap), None).expect("gen progress");
        assert_eq!(gen_prog.percent, 50);
        assert!((gen_prog.fraction - 0.5).abs() < 1e-6);
        assert!(gen_prog.tooltip.contains("2/4"));
        assert!(gen_prog.tooltip.contains("50%"));

        let sim_snap = SimProgressSnapshot {
            fraction: 0.73,
            percent: 73.0,
            stage: SimProgressStage::Sweeping,
        };
        let sim_prog = resolve_toolbar_job_progress(None, Some(sim_snap)).expect("sim progress");
        assert_eq!(sim_prog.percent, 73);
        assert!((sim_prog.fraction - 0.73).abs() < 1e-6);
        assert!(sim_prog.tooltip.contains("73%"));
        assert!(
            sim_prog
                .tooltip
                .contains(SimProgressStage::Sweeping.label())
        );
    }

    #[test]
    fn toolbar_progress_bar_renders_idle_and_active_states_without_panic() {
        let ctx = egui::Context::default();
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::Vec2::new(320.0, 100.0),
            )),
            ..Default::default()
        };
        let active = resolve_toolbar_job_progress(
            None,
            Some(SimProgressSnapshot {
                fraction: 0.42,
                percent: 42.0,
                stage: SimProgressStage::Sweeping,
            }),
        );
        let mut out = ctx.run_ui(input, |ui| {
            egui::CentralPanel::default().show(ui, |ui| {
                ui.horizontal(|ui| {
                    let _ = toolbar_progress_bar(ui, None);
                    let _ = toolbar_progress_bar(ui, active.as_ref());
                });
            });
        });
        out.textures_delta.clear();
        assert!(
            !out.shapes.is_empty(),
            "expected toolbar progress bar to paint track, fill, and percentage text"
        );
    }
}
