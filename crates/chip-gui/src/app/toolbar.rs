use chip_math::DisplayUnit;

use crate::{
    app::ChipCutterApp,
    layout::{WorkspacePane, focus_workspace_pane},
    theme::{AppTheme, ThemeTokens},
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
                    ui.spacing_mut().item_spacing.x = 5.0;

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

                    if icon_button(
                        ui,
                        IconKind::Sketch2D,
                        Some("+ Sketch"),
                        "Create a New 2D Sketch Layer on Active Setup XY",
                    )
                    .clicked()
                    {
                        self.create_new_setup_xy_sketch();
                    }
                    if icon_button(
                        ui,
                        IconKind::Adaptive2D,
                        Some("+ Adaptive"),
                        "Add a 2D Adaptive / Trochoidal Clearing Operation",
                    )
                    .clicked()
                    {
                        self.create_new_adaptive_operation();
                    }
                    if icon_button(
                        ui,
                        IconKind::VCarve,
                        Some("+ V-Carve"),
                        "Add a 3D Medial-Axis V-Carve / Flat Clearance / V-Inlay Operation",
                    )
                    .clicked()
                    {
                        self.create_new_vcarve_operation();
                    }

                    ui.separator();

                    if self.dispatcher.is_busy() {
                        ui.spinner();
                        let cancel_lbl = format!("Cancel ({})", self.dispatcher.in_flight_count());
                        if icon_button(
                            ui,
                            IconKind::CancelStop,
                            Some(&cancel_lbl),
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

                    if let Some(prog) = self.sim_panel.in_flight_progress() {
                        let pct = prog.percent_rounded();
                        let sim_lbl = format!("Sim {pct}%");
                        ui.spinner();
                        if icon_button(
                            ui,
                            IconKind::Simulate,
                            Some(&sim_lbl),
                            &format!(
                                "Simulation running: {pct}% ({}). Click to focus Simulation Timeline.",
                                prog.stage.label()
                            ),
                        )
                        .clicked()
                        {
                            focus_workspace_pane(&mut self.tree, WorkspacePane::SimulationTimeline);
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
                    if icon_button(
                        ui,
                        IconKind::Tool,
                        Some("Tools"),
                        "Open Tool Library Manager",
                    )
                    .clicked()
                    {
                        self.modals.tool_library.open = true;
                    }

                    // Right-aligned Theme & Display Units controls
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.spacing_mut().item_spacing.x = 5.0;
                        let (theme_icon, theme_tip) = match self.theme {
                            AppTheme::Dark => (
                                IconKind::Sun,
                                "Switch to Daylight High-Contrast Light Theme",
                            ),
                            AppTheme::Light => (IconKind::Moon, "Switch to Studio Dark Theme"),
                        };
                        if icon_button(ui, theme_icon, Some(self.theme.short_label()), theme_tip)
                            .clicked()
                        {
                            self.set_theme(ctx, self.theme.toggled());
                        }

                        ui.separator();

                        let mut u_sel = self.project.display_units;
                        ui.selectable_value(&mut u_sel, DisplayUnit::Inch, "in")
                            .on_hover_text("Display lengths and feed rates in Imperial inches (in, IPM, SFM)");
                        ui.selectable_value(&mut u_sel, DisplayUnit::Millimeter, "mm")
                            .on_hover_text("Display lengths and feed rates in Metric millimeters (mm, mm/min, m/min)");
                        if u_sel != self.project.display_units {
                            self.history.record_before_edit(
                                "Change Display Units",
                                None,
                                &self.project,
                            );
                            self.project.display_units = u_sel;
                            self.prefs.default_units = u_sel;
                            self.refresh_dependent_expressions();
                        }

                        ui.separator();

                        if icon_button(
                            ui,
                            IconKind::Search,
                            Some("Ctrl+K"),
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
