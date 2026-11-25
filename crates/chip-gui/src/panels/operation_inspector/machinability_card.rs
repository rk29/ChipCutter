//! Live **Machinability & Safety** advisor card for the Operation Inspector (`[Tool]` card +
//! `[Passes]` alert banner) and shared setup/browser evaluation helpers (`chip-gui`, Milestone 96).

use chip_cad::CadDocument;
use chip_cam::{
    MachinabilityFinding, MachinabilityFix, MachinabilitySeverity, OperationMachinabilityContext,
    OperationMachinabilityReport, ResolvedSetup, SetupMachinabilityReport,
    analyze_operation_machinability, analyze_setup_machinability,
};
use chip_project::ProjectDocument;
use chip_tools::{MachineDefinition, Tool, ToolLibrary, builtin_generic_machine};
use egui::{CornerRadius, Margin, RichText, Stroke, Ui};

use crate::{
    theme::ThemeTokens,
    widgets::{grid_row, inspector_grid, section_card},
};

/// Sanitizes physics advisory strings to plain ASCII so bundled `egui` fonts never render tofu
/// glyphs (`µ` -> `u`, `³` -> `3`, `²` -> `2`, `Ø` -> `D`, `°` -> ` deg`, etc.).
pub fn sanitize_physics_text(raw: &str) -> String {
    let mut out = String::with_capacity(raw.len());
    for ch in raw.chars() {
        match ch {
            '\u{00B5}' | '\u{03BC}' => out.push('u'),
            '\u{00B3}' => out.push('3'),
            '\u{00B2}' => out.push('2'),
            '\u{00D8}' => out.push('D'),
            '\u{00B0}' => out.push_str(" deg"),
            '\u{00D7}' => out.push('x'),
            '\u{2014}' | '\u{2013}' => out.push('-'),
            '\u{00B7}' | '\u{2022}' => out.push('|'),
            '\u{2026}' => out.push_str("..."),
            c if c.is_ascii() => out.push(c),
            _ => out.push(' '),
        }
    }
    out
}

/// Evaluates cutting-mechanics telemetry and advisory findings for a single operation.
pub fn evaluate_operation_machinability(
    project: &ProjectDocument,
    setup_idx: usize,
    op_idx: usize,
    global_tools: &ToolLibrary,
    active_machine: Option<&MachineDefinition>,
    resolved_setup: Option<&ResolvedSetup>,
    cad_doc: Option<&CadDocument>,
) -> Option<OperationMachinabilityReport> {
    let setup = project.setups.get(setup_idx)?;
    let op = setup.operations.get(op_idx)?;
    let tool = op
        .tool_id
        .as_ref()
        .and_then(|tid| project.tool(tid).or_else(|| global_tools.tool(tid)));
    let material = project.effective_workpiece_material();
    let fallback_machine;
    let machine = match active_machine {
        Some(m) => m,
        None => {
            fallback_machine = builtin_generic_machine();
            &fallback_machine
        }
    };
    let ctx = OperationMachinabilityContext {
        setup_id: &setup.id,
        setup_name: &setup.name,
        resolved_setup,
        cad_doc,
        sketches: &project.sketches,
        material: &material,
        machine,
        display_unit: project.display_units,
    };
    Some(analyze_operation_machinability(op, tool, &ctx))
}

/// Evaluates cutting-mechanics telemetry across all operations in a Setup.
pub fn evaluate_setup_machinability(
    project: &ProjectDocument,
    setup_idx: usize,
    global_tools: &ToolLibrary,
    active_machine: Option<&MachineDefinition>,
    cad_doc: Option<&CadDocument>,
) -> Option<SetupMachinabilityReport> {
    let setup = project.setups.get(setup_idx)?;
    let mut merged_tools: Vec<Tool> = project.tools.clone();
    for gt in &global_tools.tools {
        if !merged_tools.iter().any(|t| t.id == gt.id) {
            merged_tools.push(gt.clone());
        }
    }
    let material = project.effective_workpiece_material();
    let fallback_machine;
    let machine = match active_machine {
        Some(m) => m,
        None => {
            fallback_machine = builtin_generic_machine();
            &fallback_machine
        }
    };
    Some(analyze_setup_machinability(
        setup,
        &merged_tools,
        &material,
        machine,
        cad_doc,
        &project.sketches,
        project.display_units,
    ))
}

/// Returns a concise plain-ASCII warning summary for Project Tree hover tooltips when an
/// operation has at least one `Warning` or `Critical` machinability finding.
pub fn compact_browser_warning_summary(report: &OperationMachinabilityReport) -> Option<String> {
    if report.suppressed || report.severity < MachinabilitySeverity::Warning {
        return None;
    }
    let severe: Vec<&MachinabilityFinding> = report
        .findings
        .iter()
        .filter(|f| f.severity >= MachinabilitySeverity::Warning)
        .collect();
    if severe.is_empty() {
        return None;
    }
    let top_title = severe
        .iter()
        .max_by_key(|f| f.severity)
        .map(|f| sanitize_physics_text(&f.title))
        .unwrap_or_default();
    let tag = match report.severity {
        MachinabilitySeverity::Critical => "CRITICAL",
        MachinabilitySeverity::Warning => "WARN",
        MachinabilitySeverity::Info => "INFO",
        MachinabilitySeverity::Ok => "OK",
    };
    let extra = if severe.len() > 1 {
        format!(" (+{} more)", severe.len() - 1)
    } else {
        String::new()
    };
    Some(format!("Machinability [{tag}]: {top_title}{extra}"))
}

/// Returns the highest-severity finding that carries a deterministic 1-click safe fix.
pub fn primary_fixable_finding(
    report: &OperationMachinabilityReport,
) -> Option<&MachinabilityFinding> {
    report
        .findings
        .iter()
        .filter(|f| f.suggested_fix.is_some())
        .max_by_key(|f| f.severity)
}

fn severity_palette(
    tok: &ThemeTokens,
    severity: MachinabilitySeverity,
) -> (egui::Color32, egui::Color32, egui::Color32, &'static str) {
    match severity {
        MachinabilitySeverity::Ok => (tok.clean_bg, tok.clean_border, tok.clean_fg, "OK"),
        MachinabilitySeverity::Info => (tok.info_bg, tok.info_border, tok.info_fg, "INFO"),
        MachinabilitySeverity::Warning => (tok.warn_bg, tok.warn_border, tok.warn_fg, "WARN"),
        MachinabilitySeverity::Critical => {
            (tok.error_bg, tok.error_border, tok.error_fg, "CRITICAL")
        }
    }
}

/// Context passed into [`show_machinability_card`].
pub struct MachinabilityCardCtx<'a> {
    pub report: &'a OperationMachinabilityReport,
    pub material_name: &'a str,
    pub machine_name: &'a str,
    pub setup_idx: usize,
    pub op_idx: usize,
    pub show_reach_heatmap: &'a mut bool,
    pub force_open: bool,
}

/// Renders the collapsible **`Machinability & Safety`** card at the bottom of Operation Inspector
/// **`[Tool]`**. Returns `Some(MachinabilityFix)` when the operator clicks **`Apply Safe Fix`**.
pub fn show_machinability_card(
    ui: &mut Ui,
    ctx: MachinabilityCardCtx<'_>,
) -> Option<MachinabilityFix> {
    let MachinabilityCardCtx {
        report,
        material_name,
        machine_name,
        setup_idx,
        op_idx,
        show_reach_heatmap,
        force_open,
    } = ctx;

    let tok = ThemeTokens::from_ui(ui);
    let mut requested_fix: Option<MachinabilityFix> = None;

    let is_hazardous = report.severity >= MachinabilitySeverity::Warning;
    let card_salt = ("tool_machinability_card", setup_idx, op_idx);
    let card_state_id = ui.make_persistent_id(card_salt);
    let prev_sev_id = ui.make_persistent_id(("tool_machinability_prev_sev", setup_idx, op_idx));

    // Auto-expand when entering a Warning/Critical state or when forced open for capture, while
    // leaving Ok cards collapsed by default unless the operator opened them.
    let prev_sev = ui.data_mut(|d| d.get_temp::<MachinabilitySeverity>(prev_sev_id));
    if force_open || (is_hazardous && prev_sev.is_none_or(|p| p < MachinabilitySeverity::Warning)) {
        ui.data_mut(|d| d.insert_persisted(card_state_id, true));
    }
    ui.data_mut(|d| d.insert_temp(prev_sev_id, report.severity));

    let (_, _, _, sev_badge) = severity_palette(&tok, report.severity);
    let card_title = format!("Machinability & Safety [{sev_badge}]");

    section_card(
        ui,
        card_salt,
        &card_title,
        is_hazardous || force_open,
        |ui| {
            let visible_w =
                (ui.clip_rect().width() - 28.0).clamp(210.0, ui.available_width().max(210.0));
            let (bg, border, fg, tag) = severity_palette(&tok, report.severity);
            let status_summary = match report.severity {
                MachinabilitySeverity::Ok => {
                    "Within safe spindle, deflection, and chipload limits."
                }
                MachinabilitySeverity::Info => "Safe cut with an optional cycle-time optimization.",
                MachinabilitySeverity::Warning => {
                    "Elevated cutting load, deflection, or rubbing risk."
                }
                MachinabilitySeverity::Critical => {
                    "Tool-snap, stall, or severe overload hazard detected."
                }
            };

            egui::Frame::NONE
                .fill(bg)
                .stroke(Stroke::new(1.0, border))
                .corner_radius(CornerRadius::same(5))
                .inner_margin(Margin::symmetric(8, 5))
                .show(ui, |ui| {
                    ui.set_width((visible_w - 16.0).max(180.0));
                    ui.horizontal_wrapped(|ui| {
                        ui.label(RichText::new(format!("[{tag}]")).strong().color(fg));
                        ui.colored_label(fg, status_summary);
                    });
                    ui.label(
                        RichText::new(format!(
                            "{} | {}",
                            sanitize_physics_text(material_name),
                            sanitize_physics_text(machine_name)
                        ))
                        .small()
                        .color(tok.text_secondary),
                    );
                });

            ui.add_space(5.0);

            if let Some(m) = &report.metrics {
                let defl = m.deflection;
                inspector_grid(ui, ("mach_telemetry_grid", setup_idx, op_idx), |ui| {
                    grid_row(
                        ui,
                        "MRR",
                        Some("Material Removal Rate in cubic centimeters per minute"),
                        |ui| {
                            ui.monospace(format!("{:.2} cm3/min", m.mrr_cm3_min));
                        },
                    );

                    let spindle_color = match m.spindle_load_pct {
                        Some(pct) if pct > 100.0 => tok.error_fg,
                        Some(pct) if pct > 70.0 => tok.warn_fg,
                        _ => tok.text_primary,
                    };
                    grid_row(
                        ui,
                        "Spindle Load",
                        Some(
                            "Estimated cutting power vs available torque-derated spindle power at RPM",
                        ),
                        |ui| {
                            let txt = match (m.spindle_load_pct, m.available_spindle_power_watts) {
                                (Some(pct), Some(avail)) => {
                                    format!(
                                        "{:.1} W / {:.0} W ({:.0}%)",
                                        m.cutting_power_watts, avail, pct
                                    )
                                }
                                _ => format!("{:.1} W", m.cutting_power_watts),
                            };
                            ui.label(RichText::new(txt).monospace().color(spindle_color));
                        },
                    );

                    grid_row(
                        ui,
                        "Radial Force",
                        Some(
                            "Mean revolution-averaged radial force and instantaneous peak per-tooth impact force",
                        ),
                        |ui| {
                            ui.monospace(format!(
                                "{:.1} N (peak {:.1} N)",
                                m.radial_force_n, m.peak_radial_force_n
                            ));
                        },
                    );

                    let defl_color = if defl.tip_deflection_mm > 0.060
                        || defl.deflection_pct_diameter > 2.0
                    {
                        tok.error_fg
                    } else if defl.tip_deflection_mm > 0.025 || defl.deflection_pct_diameter > 0.8 {
                        tok.warn_fg
                    } else {
                        tok.text_primary
                    };
                    grid_row(
                        ui,
                        "Tip Deflection",
                        Some(
                            "Stepped cantilever lateral tip deflection in micrometers and % of cutter diameter",
                        ),
                        |ui| {
                            ui.label(
                                RichText::new(format!(
                                    "{:.1} um ({:.2}% D)",
                                    defl.tip_deflection_um, defl.deflection_pct_diameter
                                ))
                                .monospace()
                                .color(defl_color),
                            );
                        },
                    );

                    let sf_color = if defl.bending_safety_factor < 1.5 {
                        tok.error_fg
                    } else if defl.bending_safety_factor < 2.5 {
                        tok.warn_fg
                    } else {
                        tok.clean_fg
                    };
                    grid_row(
                        ui,
                        "Safety Factor",
                        Some(
                            "Extreme-fiber carbide/HSS bending fracture safety factor (allowable / peak stress)",
                        ),
                        |ui| {
                            let sf_txt = if defl.bending_safety_factor.is_finite() {
                                format!(
                                    "{:.2}x (core D{:.2} mm)",
                                    defl.bending_safety_factor, defl.effective_core_diameter_mm
                                )
                            } else {
                                format!("OK (core D{:.2} mm)", defl.effective_core_diameter_mm)
                            };
                            ui.label(RichText::new(sf_txt).monospace().color(sf_color));
                        },
                    );

                    grid_row(
                        ui,
                        "h_max",
                        Some("Effective maximum uncut chip thickness after radial chip thinning"),
                        |ui| {
                            ui.monospace(format!(
                                "{:.4} mm ({:.0}% RCT)",
                                m.effective_chip_thickness_mm,
                                m.chip_thinning_ratio * 100.0
                            ));
                        },
                    );

                    let ld_color = if defl.stickout_ld_ratio > 6.0 {
                        tok.warn_fg
                    } else {
                        tok.text_primary
                    };
                    grid_row(
                        ui,
                        "Stickout L/D",
                        Some(
                            "Tool stickout length divided by cutting diameter (deflection scales as (L/D)^3)",
                        ),
                        |ui| {
                            ui.label(
                                RichText::new(format!("{:.1}x", defl.stickout_ld_ratio))
                                    .monospace()
                                    .color(ld_color),
                            );
                        },
                    );
                });
            } else {
                ui.label(
                    RichText::new("Assign a tool to compute cutting physics telemetry.")
                        .small()
                        .color(tok.text_muted),
                );
            }

            if !report.findings.is_empty() {
                ui.add_space(6.0);
                for (idx, finding) in report.findings.iter().enumerate() {
                    let (f_bg, f_border, f_fg, f_tag) = severity_palette(&tok, finding.severity);
                    egui::Frame::NONE
                        .fill(f_bg)
                        .stroke(Stroke::new(1.0, f_border))
                        .corner_radius(CornerRadius::same(5))
                        .inner_margin(Margin::symmetric(8, 6))
                        .show(ui, |ui| {
                            ui.set_width((visible_w - 16.0).max(180.0));
                            ui.horizontal_wrapped(|ui| {
                                ui.label(
                                    RichText::new(format!("[{f_tag}]"))
                                        .small()
                                        .strong()
                                        .color(f_fg),
                                );
                                ui.label(
                                    RichText::new(sanitize_physics_text(&finding.title))
                                        .strong()
                                        .color(f_fg),
                                );
                            });
                            ui.label(
                                RichText::new(sanitize_physics_text(&finding.detail))
                                    .small()
                                    .color(tok.text_primary),
                            );
                            if let Some(fix) = &finding.suggested_fix {
                                let fix_lbl = sanitize_physics_text(&fix.label());
                                ui.add_space(4.0);
                                ui.horizontal_wrapped(|ui| {
                                    let btn = egui::Button::new(
                                        RichText::new("Apply Safe Fix")
                                            .small()
                                            .strong()
                                            .color(tok.accent_on_fg),
                                    )
                                    .fill(tok.accent)
                                    .stroke(Stroke::new(1.0, tok.accent_hover))
                                    .corner_radius(CornerRadius::same(4));
                                    if ui
                                        .push_id(("mach_fix_btn", setup_idx, op_idx, idx), |ui| {
                                            ui.add(btn)
                                                .on_hover_text(format!(
                                                    "1-click safe fix (Undoable): {fix_lbl}"
                                                ))
                                                .clicked()
                                        })
                                        .inner
                                    {
                                        requested_fix = Some(fix.clone());
                                    }
                                    ui.label(
                                        RichText::new(fix_lbl).small().color(tok.text_secondary),
                                    );
                                });
                            }
                        });
                    ui.add_space(4.0);
                }
            }

            ui.add_space(4.0);
            ui.separator();
            ui.add_space(4.0);
            ui.horizontal_wrapped(|ui| {
            ui.checkbox(show_reach_heatmap, "Show Reach / Min-R Overlay")
                .on_hover_text(
                    "Overlay tight internal corners (amber) and deep flute/stickout zones (amber/crimson) on the 3D CAD model",
                );
        });
        },
    );

    requested_fix
}

/// Action returned by [`show_passes_machinability_banner`].
#[derive(Debug, Clone, PartialEq)]
pub enum PassesMachinabilityBannerAction {
    None,
    ApplyFix(MachinabilityFix),
    InspectToolTab,
}

/// Renders a compact alert banner at the top of Operation Inspector **`[Passes]`** whenever
/// `report.severity` is `Warning` or `Critical`.
pub fn show_passes_machinability_banner(
    ui: &mut Ui,
    report: &OperationMachinabilityReport,
    setup_idx: usize,
    op_idx: usize,
) -> PassesMachinabilityBannerAction {
    if report.suppressed || report.severity < MachinabilitySeverity::Warning {
        return PassesMachinabilityBannerAction::None;
    }

    let tok = ThemeTokens::from_ui(ui);
    let (bg, border, fg, tag) = severity_palette(&tok, report.severity);
    let severe_count = report
        .findings
        .iter()
        .filter(|f| f.severity >= MachinabilitySeverity::Warning)
        .count();
    let top_finding = report
        .findings
        .iter()
        .filter(|f| f.severity >= MachinabilitySeverity::Warning)
        .max_by_key(|f| f.severity);
    let fix_candidate = primary_fixable_finding(report);

    let mut action = PassesMachinabilityBannerAction::None;

    let visible_w = (ui.clip_rect().width() - 28.0).clamp(210.0, ui.available_width().max(210.0));
    egui::Frame::NONE
        .fill(bg)
        .stroke(Stroke::new(1.0, border))
        .corner_radius(CornerRadius::same(5))
        .inner_margin(Margin::symmetric(8, 6))
        .show(ui, |ui| {
            ui.set_width((visible_w - 16.0).max(180.0));
            ui.horizontal_wrapped(|ui| {
                ui.label(RichText::new(format!("[{tag}]")).small().strong().color(fg));
                let title = top_finding
                    .map(|f| sanitize_physics_text(&f.title))
                    .unwrap_or_else(|| "Machinability hazard detected".to_string());
                let suffix = if severe_count > 1 {
                    format!(" (+{} more)", severe_count - 1)
                } else {
                    String::new()
                };
                ui.label(
                    RichText::new(format!("{title}{suffix}"))
                        .small()
                        .strong()
                        .color(fg),
                );
            });
            ui.add_space(3.0);
            ui.horizontal_wrapped(|ui| {
                if let Some(finding) = fix_candidate
                    && let Some(fix) = &finding.suggested_fix
                {
                    let fix_lbl = sanitize_physics_text(&fix.label());
                    let btn = egui::Button::new(
                        RichText::new("Apply Safe Fix")
                            .small()
                            .strong()
                            .color(tok.accent_on_fg),
                    )
                    .fill(tok.accent)
                    .stroke(Stroke::new(1.0, tok.accent_hover))
                    .corner_radius(CornerRadius::same(4));
                    if ui
                        .push_id(("passes_mach_fix_btn", setup_idx, op_idx), |ui| {
                            ui.add(btn)
                                .on_hover_text(format!("1-click safe fix (Undoable): {fix_lbl}"))
                                .clicked()
                        })
                        .inner
                    {
                        action = PassesMachinabilityBannerAction::ApplyFix(fix.clone());
                    }
                }
                if ui
                    .small_button("Inspect in [Tool]")
                    .on_hover_text("Open the full Machinability & Safety telemetry card in [Tool]")
                    .clicked()
                {
                    action = PassesMachinabilityBannerAction::InspectToolTab;
                }
            });
        });
    ui.add_space(5.0);

    action
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cam::{Operation, OperationKind, OperationStatus, Setup, StockConfig, WorkOffsetSlot};
    use chip_math::{DisplayUnit, ParamLength};
    use chip_project::{ProjectDocument, UndoHistory};
    use chip_tools::{
        ALUMINUM_6063_T5_ID, CutterGeometry, ProjectMaterialSelection, ToolLibrary,
        builtin_carvera_air_machine,
    };

    use super::*;
    use crate::{
        panels::{
            GeometryPickTarget, OperationTab, ViewportPickMode,
            operation_inspector::{OperationInspectorCtx, show_operation_inspector},
        },
        theme::{AppTheme, apply_theme},
    };

    fn make_hazardous_and_safe_project() -> (ProjectDocument, chip_tools::MachineDefinition) {
        let machine = builtin_carvera_air_machine();
        let mut proj = ProjectDocument::new("m96_machinability_ui");
        proj.workpiece_material = Some(ProjectMaterialSelection::builtin(ALUMINUM_6063_T5_ID));
        proj.machine_id = Some(machine.id.clone());

        // High-L/D 3.175mm x 25mm 1FL O-flute (L/D = 9.5x) based on Carvera Air starter tool
        let mut t1 = machine.tools[0].clone();
        t1.id = chip_tools::ToolId("t_long_1fl".to_string());
        t1.name = "3.175mm x 25mm 1FL O-Flute".to_string();
        t1.geometry = CutterGeometry::FlatEndmill {
            diameter_mm: 3.175,
            flutes: 1,
            flute_length_mm: 25.0,
            shank_diameter_mm: 3.175,
            overall_length_mm: 55.0,
        };
        t1.stickout_length_mm = 30.0;
        t1.presets.clear();
        proj.tools = vec![t1.clone()];

        let mut setup = Setup::new("s1", "Setup 1", WorkOffsetSlot::G54);
        setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(120.0),
            depth_y: ParamLength::from_mm(120.0),
            height_z: ParamLength::from_mm(25.0),
            xy_align: Default::default(),
            z_align: Default::default(),
        };

        let mut op = Operation::new("op_pocket", "Deep Pocket Roughing", OperationKind::Pocket2D);
        op.tool_id = Some(t1.id);
        op.passes.multiple_depths = true;
        op.passes.max_stepdown = ParamLength::from_mm(6.0);
        op.feed_overrides.stepdown_mm = Some(6.0);
        op.feed_overrides.stepover_mm = Some(2.0);
        op.feed_overrides.spindle_rpm = Some(12_000.0);
        op.feed_overrides.cutting_feed_mm_min = Some(300.0);
        op.runtime_status = OperationStatus::Clean;
        setup.operations = vec![op];
        proj.setups = vec![setup];

        (proj, machine)
    }

    fn collect_texts_and_rects(shapes: &[egui::epaint::ClippedShape]) -> Vec<(String, egui::Rect)> {
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
        let mut out = Vec::new();
        for cs in shapes {
            visit(&cs.shape, &mut out);
        }
        out
    }

    #[test]
    fn machinability_card_renders_ascii_and_stable_layout_in_dark_and_light() {
        let (proj, machine) = make_hazardous_and_safe_project();
        let tools = ToolLibrary::default();
        let rs = proj.setups[0]
            .resolve_with_sketches(&CadDocument::empty(), &[], DisplayUnit::Millimeter)
            .unwrap();
        let haz_report =
            evaluate_operation_machinability(&proj, 0, 0, &tools, Some(&machine), Some(&rs), None)
                .unwrap();
        assert_eq!(haz_report.severity, MachinabilitySeverity::Critical);

        let mut safe_proj = proj.clone();
        safe_proj.tools[0].stickout_length_mm = 10.0;
        safe_proj.setups[0].operations[0].passes.max_stepdown = ParamLength::from_mm(0.4);
        safe_proj.setups[0].operations[0].feed_overrides.stepdown_mm = Some(0.4);
        safe_proj.setups[0].operations[0].feed_overrides.stepover_mm = Some(1.6);
        safe_proj.setups[0].operations[0]
            .feed_overrides
            .cutting_feed_mm_min = Some(300.0);
        let safe_report = evaluate_operation_machinability(
            &safe_proj,
            0,
            0,
            &tools,
            Some(&machine),
            Some(&rs),
            None,
        )
        .unwrap();
        assert_eq!(safe_report.severity, MachinabilitySeverity::Ok);

        for theme in [AppTheme::Dark, AppTheme::Light] {
            for (label, report, force_open) in
                [("critical", &haz_report, false), ("ok", &safe_report, true)]
            {
                let ctx = egui::Context::default();
                apply_theme(&ctx, theme);
                let mut show_reach = false;
                let mut heights = Vec::new();
                for _frame in 0..4 {
                    let mut used_h = 0.0_f32;
                    let mut out = ctx.run_ui(
                        egui::RawInput {
                            screen_rect: Some(egui::Rect::from_min_size(
                                egui::Pos2::ZERO,
                                egui::vec2(380.0, 1200.0),
                            )),
                            ..Default::default()
                        },
                        |ui| {
                            egui::CentralPanel::default().show(ui, |ui| {
                                let start_y = ui.cursor().min.y;
                                let _ = show_machinability_card(
                                    ui,
                                    MachinabilityCardCtx {
                                        report,
                                        material_name: "Aluminum 6063-T5 Extrusion",
                                        machine_name: &machine.name,
                                        setup_idx: 0,
                                        op_idx: 0,
                                        show_reach_heatmap: &mut show_reach,
                                        force_open,
                                    },
                                );
                                used_h = ui.cursor().min.y - start_y;
                            });
                        },
                    );
                    out.textures_delta.clear();
                    heights.push(used_h);
                    for (txt, _) in collect_texts_and_rects(&out.shapes) {
                        assert!(
                            txt.is_ascii(),
                            "{theme:?} {label} machinability card must render plain ASCII only; got: {txt:?}"
                        );
                    }
                }
                assert!(
                    (heights[2] - heights[3]).abs() < 0.5,
                    "{theme:?} {label} machinability card must have zero resize creep; heights={heights:?}"
                );
            }
        }
    }

    #[test]
    fn clicking_apply_safe_fix_records_undo_marks_dirty_and_resolves_critical() {
        let (mut proj, machine) = make_hazardous_and_safe_project();
        let tools = ToolLibrary::default();
        let rs = proj.setups[0]
            .resolve_with_sketches(&CadDocument::empty(), &[], DisplayUnit::Millimeter)
            .unwrap();
        let initial_rep =
            evaluate_operation_machinability(&proj, 0, 0, &tools, Some(&machine), Some(&rs), None)
                .unwrap();
        assert_eq!(initial_rep.severity, MachinabilitySeverity::Critical);

        let ctx = egui::Context::default();
        apply_theme(&ctx, AppTheme::Dark);
        let mut history = UndoHistory::new(32);
        let mut tab = OperationTab::Tool;
        let mut pick_mode = ViewportPickMode::InspectEntity;
        let mut pick_target = GeometryPickTarget::CuttingArea;
        let mut editing_shared_avoid_id: Option<String> = None;
        let mut lock_chipload = true;
        let mut show_z_planes = false;
        let mut show_reach_heatmap = false;
        let mut auto_rotate = true;
        let mut open_tool_manager = false;
        let mut time = 0.0_f64;

        let mut run_frame = |proj: &mut ProjectDocument,
                             history: &mut UndoHistory,
                             events: Vec<egui::Event>|
         -> Vec<(String, egui::Rect)> {
            time += 0.25;
            let mut out = ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(
                        egui::Pos2::ZERO,
                        egui::vec2(420.0, 2200.0),
                    )),
                    time: Some(time),
                    events,
                    ..Default::default()
                },
                |ui| {
                    egui::CentralPanel::default().show(ui, |ui| {
                        let _ = show_operation_inspector(
                            ui,
                            OperationInspectorCtx {
                                project: proj,
                                setup_idx: 0,
                                op_idx: 0,
                                global_tools: &tools,
                                active_machine: Some(&machine),
                                history,
                                cad_doc: None,
                                resolved_setup: Some(&rs),
                                surface_mesh: None,
                                boundary_preview: None,
                                selected_face_normal_wcs: None,
                                extracted: None,
                                tab: &mut tab,
                                pick_mode: &mut pick_mode,
                                pick_target: &mut pick_target,
                                editing_shared_avoid_id: &mut editing_shared_avoid_id,
                                lock_chipload: &mut lock_chipload,
                                show_z_planes: &mut show_z_planes,
                                show_reach_heatmap: &mut show_reach_heatmap,
                                force_expand_machinability: false,
                                auto_rotate_indexed_op: &mut auto_rotate,
                                open_tool_manager: &mut open_tool_manager,
                                sim_engine: chip_sim::SimEngineKind::TriDexel,
                            },
                        );
                    });
                },
            );
            out.textures_delta.clear();
            collect_texts_and_rects(&out.shapes)
        };

        // Frame 1 & 2: auto-expanded card renders "Apply Safe Fix"
        let _ = run_frame(&mut proj, &mut history, Vec::new());
        let rects = run_frame(&mut proj, &mut history, Vec::new());
        let fix_btn = rects
            .iter()
            .find(|(t, _)| t == "Apply Safe Fix")
            .expect("Critical machinability card must render 'Apply Safe Fix' button")
            .1
            .center();

        // Click "Apply Safe Fix"
        let _ = run_frame(
            &mut proj,
            &mut history,
            vec![
                egui::Event::PointerMoved(fix_btn),
                egui::Event::PointerButton {
                    pos: fix_btn,
                    button: egui::PointerButton::Primary,
                    pressed: true,
                    modifiers: egui::Modifiers::NONE,
                },
                egui::Event::PointerButton {
                    pos: fix_btn,
                    button: egui::PointerButton::Primary,
                    pressed: false,
                    modifiers: egui::Modifiers::NONE,
                },
            ],
        );

        assert!(
            history.can_undo(),
            "Clicking Apply Safe Fix must record an atomic Undo snapshot"
        );
        assert_eq!(
            proj.setups[0].operations[0].runtime_status,
            OperationStatus::Dirty,
            "Clicking Apply Safe Fix must mark the operation Dirty"
        );
        assert!(
            proj.setups[0].operations[0].passes.max_stepdown.value_mm < 6.0,
            "Apply Safe Fix must reduce the hazardous 6.0 mm stepdown"
        );

        let after_rep =
            evaluate_operation_machinability(&proj, 0, 0, &tools, Some(&machine), Some(&rs), None)
                .unwrap();
        assert!(
            after_rep.severity < MachinabilitySeverity::Critical,
            "After applying safe fix, Critical finding must be resolved; got {:?} ({:?})",
            after_rep.severity,
            after_rep.findings
        );

        // Undo restores the original 6.0 mm stepdown
        let undo_label = history.undo(&mut proj).expect("undo must succeed");
        assert_eq!(undo_label, "Apply Safe Machinability Fix");
        assert!(
            (proj.setups[0].operations[0].passes.max_stepdown.value_mm - 6.0).abs() < 1e-6,
            "Undo must restore original 6.0 mm stepdown"
        );
    }

    #[test]
    fn drill_bore_passes_tab_exposes_helical_safety_controls_and_browser_tooltip_summary() {
        use chip_cam::{ExtractedSelectionSummary, HoleMachiningMode, StrategyParams};

        let (mut proj, machine) = make_hazardous_and_safe_project();
        let tools = ToolLibrary::default();
        let rs = proj.setups[0]
            .resolve_with_sketches(&CadDocument::empty(), &[], DisplayUnit::Millimeter)
            .unwrap();

        // Hazardous pocket produces a concise ASCII browser tooltip warning summary
        let haz_rep =
            evaluate_operation_machinability(&proj, 0, 0, &tools, Some(&machine), Some(&rs), None)
                .unwrap();
        let tip = compact_browser_warning_summary(&haz_rep)
            .expect("Critical op must produce a browser warning tooltip summary");
        assert!(tip.starts_with("Machinability [CRITICAL]: "));
        assert!(tip.is_ascii());

        // Switch op to DrillBore in HelicalBore mode and render [Passes] with a 5.0 mm hole
        let mut bore_op = Operation::new("op_bore", "Deep M5 Bore", OperationKind::DrillBore);
        bore_op.tool_id = Some(proj.tools[0].id.clone());
        if let StrategyParams::DrillBore(p) = bore_op.strategy_mut() {
            p.mode = HoleMachiningMode::HelicalBore;
            p.bore_inner_arc_feed_comp = true;
            p.bore_chip_clear_depth = Some(ParamLength::from_mm(6.0));
        }
        proj.setups[0].operations[0] = bore_op;

        let extracted = ExtractedSelectionSummary {
            primary_hole_diameter_mm: Some(5.0),
            ..ExtractedSelectionSummary::default()
        };

        let ctx = egui::Context::default();
        apply_theme(&ctx, AppTheme::Light);
        let mut history = UndoHistory::new(16);
        let mut tab = OperationTab::Passes;
        let mut pick_mode = ViewportPickMode::InspectEntity;
        let mut pick_target = GeometryPickTarget::CuttingArea;
        let mut editing_shared_avoid_id: Option<String> = None;
        let mut lock_chipload = true;
        let mut show_z_planes = false;
        let mut show_reach_heatmap = false;
        let mut auto_rotate = true;
        let mut open_tool_manager = false;

        let mut out = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(460.0, 1800.0),
                )),
                ..Default::default()
            },
            |ui| {
                egui::CentralPanel::default().show(ui, |ui| {
                    let _ = show_operation_inspector(
                        ui,
                        OperationInspectorCtx {
                            project: &mut proj,
                            setup_idx: 0,
                            op_idx: 0,
                            global_tools: &tools,
                            active_machine: Some(&machine),
                            history: &mut history,
                            cad_doc: None,
                            resolved_setup: Some(&rs),
                            surface_mesh: None,
                            boundary_preview: None,
                            selected_face_normal_wcs: None,
                            extracted: Some(Ok(&extracted)),
                            tab: &mut tab,
                            pick_mode: &mut pick_mode,
                            pick_target: &mut pick_target,
                            editing_shared_avoid_id: &mut editing_shared_avoid_id,
                            lock_chipload: &mut lock_chipload,
                            show_z_planes: &mut show_z_planes,
                            show_reach_heatmap: &mut show_reach_heatmap,
                            force_expand_machinability: false,
                            auto_rotate_indexed_op: &mut auto_rotate,
                            open_tool_manager: &mut open_tool_manager,
                            sim_engine: chip_sim::SimEngineKind::TriDexel,
                        },
                    );
                });
            },
        );
        out.textures_delta.clear();
        let texts: Vec<String> = collect_texts_and_rects(&out.shapes)
            .into_iter()
            .map(|(t, _)| t)
            .collect();
        let joined = texts.join("\n");
        assert!(
            joined.contains("Inner-Arc Feed Comp")
                && joined.contains("R_hole/r_orbit")
                && joined.contains("Chip-Clear Retract")
                && joined.contains("Clear Depth"),
            "DrillBore HelicalBore [Passes] must expose Inner-Arc Feed Comp + R_hole/r_orbit readout + Chip-Clear Retract; got:\n{joined}"
        );
    }
}
