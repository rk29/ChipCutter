use chip_cam::{
    ThreadCutDirection, ThreadFormKind, ThreadHandedness, ThreadMillParams, ThreadSpecEntry,
    ThreadStandardKind, compute_thread_mill_centerline_feed, compute_thread_minor_diameter_mm,
    find_thread_spec, iso60_pitch_diameter_mm, resolve_thread_helix_kinematics,
    thread_specs_for_standard,
};
use chip_math::{DisplayUnit, ParamLength};
use chip_tools::CutterGeometry;

use super::{COMBO_COL_WIDTH, PassItemCtx};
use crate::{
    theme::ThemeTokens,
    widgets::{BannerKind, inline_banner, section_card},
};

pub(super) struct ThreadMillParamsUiCtx<'a> {
    pub p: &'a mut ThreadMillParams,
    pub assigned_tool_geom: Option<&'a CutterGeometry>,
    pub tool_feed_preview: Option<(f64, f64)>,
    pub resolved_thread_depth_mm: Option<f64>,
    pub display_unit: DisplayUnit,
    pub item: PassItemCtx<'a>,
}

/// Formats the live helical kinematics badge accurately for all 8 combinations of
/// `(Internal|External) x (Right-Hand|Left-Hand) x (Climb|Conventional)` with a CW (`M3`) spindle.
pub fn format_thread_kinematics_badge(
    form: ThreadFormKind,
    handedness: ThreadHandedness,
    cut_direction: ThreadCutDirection,
) -> String {
    let kin = resolve_thread_helix_kinematics(form, handedness, cut_direction);
    let axial = if kin.bottom_up {
        "Bottom-Up"
    } else {
        "Top-Down"
    };
    let arc = if kin.ccw { "G3" } else { "G2" };
    let cut = match cut_direction {
        ThreadCutDirection::Climb => "Climb",
        ThreadCutDirection::Conventional => "Conventional",
    };
    format!("{axial} {arc} ({cut})")
}

/// Evaluates immediate inline safety and clearance warnings for a `ThreadMill` operation.
pub fn collect_thread_mill_warnings(
    p: &ThreadMillParams,
    assigned_tool_geom: Option<&CutterGeometry>,
    resolved_thread_depth_mm: Option<f64>,
    display_unit: DisplayUnit,
) -> Vec<(BannerKind, String)> {
    let mut warnings = Vec::new();
    let Some(geom) = assigned_tool_geom else {
        warnings.push((
            BannerKind::Warn,
            "No Thread Mill cutter assigned. Select a Thread Mill in the Tool tab before generating."
                .to_string(),
        ));
        return warnings;
    };

    let CutterGeometry::ThreadMill {
        diameter_mm,
        neck_length_mm,
        pitch_mm: tool_pitch_mm,
        teeth_count,
        thread_angle_deg,
        ..
    } = *geom
    else {
        warnings.push((
            BannerKind::Error,
            format!(
                "Assigned tool is a {} -- Thread Mill operations require a Thread Mill cutter.",
                geom.kind_label()
            ),
        ));
        return warnings;
    };

    let major_d = p.major_diameter.value_mm;
    let pitch = p.pitch.value_mm;
    let pd_offset = p.pitch_diameter_offset.value_mm;
    let d_minor_nom = compute_thread_minor_diameter_mm(p.form, major_d, pitch, thread_angle_deg);
    let d_minor_eff = d_minor_nom + pd_offset;

    if p.form == ThreadFormKind::InternalHole
        && (diameter_mm >= d_minor_nom - 1e-4 || diameter_mm >= d_minor_eff - 1e-4)
    {
        let limiting_minor = d_minor_nom.min(d_minor_eff);
        warnings.push((
            BannerKind::Error,
            format!(
                "Cutter diameter ({}) >= internal thread minor diameter ({}) -- choose a smaller Thread Mill.",
                display_unit.format_length(diameter_mm),
                display_unit.format_length(limiting_minor),
            ),
        ));
    }

    if teeth_count > 1 && (tool_pitch_mm - pitch).abs() > 0.02 {
        warnings.push((
            BannerKind::Error,
            format!(
                "Multi-form cutter pitch ({}, {} teeth) differs from target pitch ({}) by more than 0.02 mm.",
                display_unit.format_length(tool_pitch_mm),
                teeth_count,
                display_unit.format_length(pitch),
            ),
        ));
    }

    if let Some(depth_mm) = resolved_thread_depth_mm
        && depth_mm.is_finite()
        && depth_mm > neck_length_mm + 1e-4
    {
        warnings.push((
            BannerKind::Warn,
            format!(
                "Thread axial depth ({}) exceeds cutter neck reach ({}) -- shank collision risk.",
                display_unit.format_length(depth_mm),
                display_unit.format_length(neck_length_mm),
            ),
        ));
    }

    warnings
}

fn default_spec_for_standard(standard: ThreadStandardKind) -> Option<&'static ThreadSpecEntry> {
    let preferred = match standard {
        ThreadStandardKind::IsoMetricCoarse => Some("M8 x 1.25"),
        ThreadStandardKind::IsoMetricFine => Some("M8 x 1.0"),
        ThreadStandardKind::UnifiedUnc => Some("1/4-20 UNC"),
        ThreadStandardKind::UnifiedUnf => Some("1/4-28 UNF"),
        ThreadStandardKind::Custom => None,
    };
    preferred
        .and_then(find_thread_spec)
        .or_else(|| thread_specs_for_standard(standard).into_iter().next())
}

fn apply_thread_spec(p: &mut ThreadMillParams, spec: &ThreadSpecEntry) {
    p.standard_preset = Some(spec.designation.to_string());
    p.major_diameter = ParamLength::from_mm(spec.major_diameter_mm);
    p.pitch = ParamLength::from_mm(spec.pitch_mm);
}

pub(super) fn show_thread_mill_params(ui: &mut egui::Ui, ctx: ThreadMillParamsUiCtx<'_>) {
    let ThreadMillParamsUiCtx {
        p,
        assigned_tool_geom,
        tool_feed_preview,
        resolved_thread_depth_mm,
        display_unit,
        mut item,
    } = ctx;
    let (setup_idx, op_idx) = item.ids();
    let tok = ThemeTokens::from_ui(ui);

    // Inline Safety & Clearance Validation Banner(s)
    let warnings = collect_thread_mill_warnings(
        p,
        assigned_tool_geom,
        resolved_thread_depth_mm,
        display_unit,
    );
    for (kind, msg) in &warnings {
        inline_banner(ui, *kind, msg);
        ui.add_space(4.0);
    }

    let tool_thread_angle_deg = match assigned_tool_geom {
        Some(CutterGeometry::ThreadMill {
            thread_angle_deg, ..
        }) => *thread_angle_deg,
        _ => 60.0,
    };

    // Card 1: Thread Specification
    section_card(
        ui,
        ("passes_thread_spec_card", setup_idx, op_idx),
        "Thread Specification",
        true,
        |ui| {
            let matched_spec = p.standard_preset.as_deref().and_then(find_thread_spec);
            let mut active_standard = matched_spec
                .map(|s| s.standard)
                .unwrap_or(ThreadStandardKind::Custom);

            item.grid(ui, "passes_thread_spec_grid", |f| {
                f.set_tip("Cut internal threads inside a pre-drilled bore or external threads around a cylindrical boss");
                f.combo(
                    "Thread Form",
                    &mut p.form,
                    (&ThreadFormKind::ALL, |fk| fk.label()),
                    "Change Thread Form",
                );

                f.set_tip("Select a built-in ISO Metric or Unified UNC/UNF thread table, or Custom for arbitrary pitch/diameter");
                f.row("Standard", |ui, mutated| {
                    let prev_std = active_standard;
                    egui::ComboBox::from_id_salt(("tm_std_cb", setup_idx, op_idx))
                        .width(COMBO_COL_WIDTH)
                        .truncate()
                        .selected_text(active_standard.label())
                        .show_ui(ui, |ui| {
                            for cand in ThreadStandardKind::ALL {
                                if ui
                                    .selectable_value(&mut active_standard, cand, cand.label())
                                    .on_hover_text(format!("Use {} thread specifications", cand.label()))
                                    .clicked()
                                    && cand != prev_std
                                {
                                    if cand == ThreadStandardKind::Custom {
                                        p.standard_preset = None;
                                    } else if let Some(spec) = default_spec_for_standard(cand) {
                                        apply_thread_spec(p, spec);
                                    }
                                    *mutated = Some("Change Thread Standard");
                                }
                            }
                        })
                        .response
                        .on_hover_text("Choose standard thread family (ISO Metric Coarse/Fine, UNC, UNF, or Custom)");
                });

                if active_standard != ThreadStandardKind::Custom {
                    let specs = thread_specs_for_standard(active_standard);
                    let cur_size_lbl = p
                        .standard_preset
                        .clone()
                        .unwrap_or_else(|| "Select Thread Size...".to_string());
                    f.set_tip("Pick a standard thread size to auto-populate Major Diameter and Pitch");
                    f.row("Thread Size", |ui, mutated| {
                        egui::ComboBox::from_id_salt(("tm_size_cb", setup_idx, op_idx))
                            .width(COMBO_COL_WIDTH)
                            .truncate()
                            .selected_text(cur_size_lbl)
                            .show_ui(ui, |ui| {
                                for spec in &specs {
                                    let is_sel = p
                                        .standard_preset
                                        .as_deref()
                                        .is_some_and(|s| s.eq_ignore_ascii_case(spec.designation));
                                    let tip = format!(
                                        "{}: Major {}, Pitch {}, Tap Drill {}",
                                        spec.designation,
                                        display_unit.format_length(spec.major_diameter_mm),
                                        display_unit.format_length(spec.pitch_mm),
                                        display_unit.format_length(spec.recommended_tap_drill_mm),
                                    );
                                    if ui
                                        .selectable_label(is_sel, spec.designation)
                                        .on_hover_text(tip)
                                        .clicked()
                                        && !is_sel
                                    {
                                        apply_thread_spec(p, spec);
                                        *mutated = Some("Select Standard Thread Size");
                                    }
                                }
                            })
                            .response
                            .on_hover_text("Standard thread designation (auto-fills Major Dia and Pitch)");
                    });
                }

                // Crisp minor diameter & recommended tap-drill readout
                let active_spec = p.standard_preset.as_deref().and_then(find_thread_spec);
                let minor_dia_mm = compute_thread_minor_diameter_mm(
                    p.form,
                    p.major_diameter.value_mm,
                    p.pitch.value_mm,
                    tool_thread_angle_deg,
                );
                let pitch_dia_mm =
                    iso60_pitch_diameter_mm(p.major_diameter.value_mm, p.pitch.value_mm);
                let spec_readout = match (p.form, active_spec) {
                    (ThreadFormKind::InternalHole, Some(spec)) => format!(
                        "Tap Drill: {} | Minor Dia: {}",
                        display_unit.format_length(spec.recommended_tap_drill_mm),
                        display_unit.format_length(minor_dia_mm),
                    ),
                    (ThreadFormKind::InternalHole, None) => format!(
                        "Minor Dia: {} | Pitch Dia: {}",
                        display_unit.format_length(minor_dia_mm),
                        display_unit.format_length(pitch_dia_mm),
                    ),
                    (ThreadFormKind::ExternalBoss, _) => format!(
                        "Minor Dia: {} | Pitch Dia: {}",
                        display_unit.format_length(minor_dia_mm),
                        display_unit.format_length(pitch_dia_mm),
                    ),
                };
                f.set_tip("Computed thread minor diameter and recommended pre-drill (tap drill) diameter");
                f.row("Thread Info", |ui, _| {
                    ui.colored_label(tok.info_fg, &spec_readout)
                        .on_hover_text("Verify your pilot hole matches the recommended tap drill before threading");
                });

                let prev_major = p.major_diameter.value_mm;
                let prev_pitch = p.pitch.value_mm;

                f.set_tip("Nominal outer major diameter (D) of the thread");
                f.len("Major Dia", &mut p.major_diameter, "Edit Thread Major Diameter");

                f.set_tip("Axial thread pitch (P) advance per full 360-degree revolution");
                f.len("Pitch", &mut p.pitch, "Edit Thread Pitch");

                // Switch preset to Custom if Major Dia or Pitch was manually diverged from table
                if let Some(spec) = active_spec
                    && ((p.major_diameter.value_mm - prev_major).abs() > 1e-9
                        || (p.pitch.value_mm - prev_pitch).abs() > 1e-9)
                    && ((p.major_diameter.value_mm - spec.major_diameter_mm).abs() > 1e-4
                        || (p.pitch.value_mm - spec.pitch_mm).abs() > 1e-4)
                {
                    p.standard_preset = None;
                }

                f.set_tip("Right-hand (standard clockwise-tightening) or left-hand reverse thread helix");
                f.combo(
                    "Handedness",
                    &mut p.handedness,
                    (&ThreadHandedness::ALL, |h| h.label()),
                    "Change Thread Handedness",
                );

                f.set_tip("Climb milling (recommended for finish and chip evacuation) vs. Conventional cutting");
                f.combo(
                    "Cut Direction",
                    &mut p.cut_direction,
                    (&ThreadCutDirection::ALL, |d| d.label()),
                    "Change Thread Cut Direction",
                );

                let badge_txt = format_thread_kinematics_badge(p.form, p.handedness, p.cut_direction);
                let kin = resolve_thread_helix_kinematics(p.form, p.handedness, p.cut_direction);
                let kin_tip = format!(
                    "Resolved CW (M3) spindle helix: {} ({}) -> {} ({}) arc interpolation",
                    if kin.bottom_up { "Bottom-Up" } else { "Top-Down" },
                    if kin.bottom_up { "+Z" } else { "-Z" },
                    if kin.ccw { "G3" } else { "G2" },
                    if kin.ccw { "CCW" } else { "CW" },
                );
                f.set_tip("Resolved helical Z travel direction and G2/G3 arc sense for a CW (M3) spindle");
                f.row("Kinematics", |ui, _| {
                    egui::Frame::NONE
                        .fill(tok.info_bg)
                        .stroke(egui::Stroke::new(1.0, tok.info_border))
                        .corner_radius(egui::CornerRadius::same(4))
                        .inner_margin(egui::Margin::symmetric(6, 2))
                        .show(ui, |ui| {
                            ui.colored_label(tok.info_fg, &badge_txt)
                                .on_hover_text(&kin_tip);
                        });
                });
            });
        },
    );

    // Card 2: Helical Passes & Fit Tuning
    section_card(
        ui,
        ("passes_thread_passes_card", setup_idx, op_idx),
        "Helical Passes & Fit Tuning",
        true,
        |ui| {
            item.grid(ui, "passes_thread_passes_grid", |f| {
                f.set_tip("Number of progressive radial depth passes (equal-area sqrt progression) to reach full thread depth");
                f.drag(
                    "Radial Passes",
                    &mut p.radial_passes,
                    1..=8,
                    (0.15, ""),
                    "Edit Thread Radial Passes",
                );

                f.set_tip("Repeat zero-offset finishing passes at full thread depth to eliminate tool deflection");
                f.drag(
                    "Spring Passes",
                    &mut p.spring_passes,
                    0..=3,
                    (0.15, ""),
                    "Edit Thread Spring Passes",
                );

                f.set_tip("Signed thread gauge-fit offset (+ increases cut diameter: looser internal / tighter external; - decreases cut diameter)");
                f.len(
                    "Pitch Dia Offset",
                    &mut p.pitch_diameter_offset,
                    "Edit Thread Pitch Diameter Offset",
                );

                f.set_tip("Extra radial clearance outside external boss OD before tangent arc entry and exit");
                f.len(
                    "Lead-In Clearance",
                    &mut p.lead_in_clearance,
                    "Edit Thread Lead-In Clearance",
                );

                f.set_tip("Sort multiple thread locations via Nearest-Neighbor + 2-Opt to minimize rapid transit time");
                f.toggle(
                    "Optimize Order",
                    &mut p.optimize_hole_order,
                    "Toggle Thread Hole Order",
                );

                f.set_tip("Scale programmed centerline feedrate for internal/external arc curvature so cutting-edge chipload remains constant");
                f.check(
                    "Feed Comp",
                    &mut p.compensate_centerline_feed,
                    "Compensate Centerline Feed",
                    "Toggle Thread Feed Compensation",
                );
            });

            if let Some((tool_d, nom_feed)) = tool_feed_preview {
                let d_minor_nom = compute_thread_minor_diameter_mm(
                    p.form,
                    p.major_diameter.value_mm,
                    p.pitch.value_mm,
                    tool_thread_angle_deg,
                );
                let d_major_eff = p.major_diameter.value_mm + p.pitch_diameter_offset.value_mm;
                let d_minor_eff = d_minor_nom + p.pitch_diameter_offset.value_mm;
                let eff_feed = compute_thread_mill_centerline_feed(
                    p.form,
                    nom_feed,
                    d_major_eff,
                    d_minor_eff,
                    tool_d,
                    p.compensate_centerline_feed,
                );
                let ratio_pct = if nom_feed > 1e-6 {
                    (eff_feed / nom_feed) * 100.0
                } else {
                    100.0
                };
                let feed_msg = if p.compensate_centerline_feed {
                    format!(
                        "Edge {:.0} {} -> Centerline {:.0} {} ({:.0}% arc comp)",
                        display_unit.feed_from_mm_min(nom_feed),
                        display_unit.feed_suffix(),
                        display_unit.feed_from_mm_min(eff_feed),
                        display_unit.feed_suffix(),
                        ratio_pct,
                    )
                } else {
                    format!(
                        "Programmed Centerline Feed: {:.0} {} (arc compensation off)",
                        display_unit.feed_from_mm_min(eff_feed),
                        display_unit.feed_suffix(),
                    )
                };
                ui.add_space(4.0);
                inline_banner(ui, BannerKind::Info, feed_msg);
            }
        },
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thread_kinematics_badge_covers_all_eight_combinations() {
        assert_eq!(
            format_thread_kinematics_badge(
                ThreadFormKind::InternalHole,
                ThreadHandedness::RightHand,
                ThreadCutDirection::Climb,
            ),
            "Bottom-Up G3 (Climb)"
        );
        assert_eq!(
            format_thread_kinematics_badge(
                ThreadFormKind::InternalHole,
                ThreadHandedness::RightHand,
                ThreadCutDirection::Conventional,
            ),
            "Top-Down G2 (Conventional)"
        );
        assert_eq!(
            format_thread_kinematics_badge(
                ThreadFormKind::InternalHole,
                ThreadHandedness::LeftHand,
                ThreadCutDirection::Climb,
            ),
            "Top-Down G3 (Climb)"
        );
        assert_eq!(
            format_thread_kinematics_badge(
                ThreadFormKind::InternalHole,
                ThreadHandedness::LeftHand,
                ThreadCutDirection::Conventional,
            ),
            "Bottom-Up G2 (Conventional)"
        );
        assert_eq!(
            format_thread_kinematics_badge(
                ThreadFormKind::ExternalBoss,
                ThreadHandedness::RightHand,
                ThreadCutDirection::Climb,
            ),
            "Top-Down G2 (Climb)"
        );
        assert_eq!(
            format_thread_kinematics_badge(
                ThreadFormKind::ExternalBoss,
                ThreadHandedness::RightHand,
                ThreadCutDirection::Conventional,
            ),
            "Bottom-Up G3 (Conventional)"
        );
        assert_eq!(
            format_thread_kinematics_badge(
                ThreadFormKind::ExternalBoss,
                ThreadHandedness::LeftHand,
                ThreadCutDirection::Climb,
            ),
            "Bottom-Up G2 (Climb)"
        );
        assert_eq!(
            format_thread_kinematics_badge(
                ThreadFormKind::ExternalBoss,
                ThreadHandedness::LeftHand,
                ThreadCutDirection::Conventional,
            ),
            "Top-Down G3 (Conventional)"
        );
    }

    #[test]
    fn validation_banners_detect_incompatible_tool_oversize_pitch_and_reach() {
        let m8 = ThreadMillParams::default(); // M8 x 1.25 internal, minor ~6.647 mm

        // (a) Missing tool
        let w_none = collect_thread_mill_warnings(&m8, None, Some(10.0), DisplayUnit::Millimeter);
        assert_eq!(w_none.len(), 1);
        assert_eq!(w_none[0].0, BannerKind::Warn);

        // (a') Non-ThreadMill tool
        let flat = CutterGeometry::FlatEndmill {
            diameter_mm: 6.0,
            flute_length_mm: 18.0,
            overall_length_mm: 50.0,
            shank_diameter_mm: 6.0,
            flutes: 3,
        };
        let w_flat =
            collect_thread_mill_warnings(&m8, Some(&flat), Some(10.0), DisplayUnit::Millimeter);
        assert_eq!(w_flat.len(), 1);
        assert_eq!(w_flat[0].0, BannerKind::Error);

        // Clean compatible single-point thread mill
        let ok_tm = CutterGeometry::ThreadMill {
            diameter_mm: 4.75,
            neck_diameter_mm: 2.85,
            neck_length_mm: 16.0,
            pitch_mm: 1.0,
            teeth_count: 1,
            thread_angle_deg: 60.0,
            flute_length_mm: 1.2,
            overall_length_mm: 50.0,
            shank_diameter_mm: 6.0,
            flutes: 3,
        };
        assert!(
            collect_thread_mill_warnings(&m8, Some(&ok_tm), Some(12.0), DisplayUnit::Millimeter)
                .is_empty()
        );

        // (b)+(c)+(d) Oversized multi-form with mismatched pitch and insufficient neck reach
        let bad_tm = CutterGeometry::ThreadMill {
            diameter_mm: 7.0,
            neck_diameter_mm: 4.0,
            neck_length_mm: 10.0,
            pitch_mm: 1.5,
            teeth_count: 5,
            thread_angle_deg: 60.0,
            flute_length_mm: 7.5,
            overall_length_mm: 50.0,
            shank_diameter_mm: 6.0,
            flutes: 3,
        };
        let w_all =
            collect_thread_mill_warnings(&m8, Some(&bad_tm), Some(14.0), DisplayUnit::Millimeter);
        assert_eq!(w_all.len(), 3);
        assert!(w_all.iter().any(|(_, m)| m.contains("minor diameter")));
        assert!(w_all.iter().any(|(_, m)| m.contains("0.02 mm")));
        assert!(w_all.iter().any(|(_, m)| m.contains("neck reach")));
    }

    #[test]
    fn thread_mill_passes_inspector_renders_both_cards_cleanly() {
        let ctx = egui::Context::default();
        let mut params = ThreadMillParams::default();
        let tm_geom = CutterGeometry::ThreadMill {
            diameter_mm: 4.75,
            neck_diameter_mm: 2.85,
            neck_length_mm: 16.0,
            pitch_mm: 1.0,
            teeth_count: 1,
            thread_angle_deg: 60.0,
            flute_length_mm: 1.2,
            overall_length_mm: 50.0,
            shank_diameter_mm: 6.0,
            flutes: 3,
        };
        let eval_ctx = chip_math::EvalContext::new(DisplayUnit::Millimeter);
        let mut mutated: Option<&'static str> = None;

        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(420.0, 900.0),
            )),
            ..Default::default()
        };
        let mut out = ctx.run_ui(input, |ui| {
            egui::CentralPanel::default().show(ui, |ui| {
                show_thread_mill_params(
                    ui,
                    ThreadMillParamsUiCtx {
                        p: &mut params,
                        assigned_tool_geom: Some(&tm_geom),
                        tool_feed_preview: Some((4.75, 800.0)),
                        resolved_thread_depth_mm: Some(12.0),
                        display_unit: DisplayUnit::Millimeter,
                        item: PassItemCtx {
                            setup_idx: 0,
                            op_idx: 0,
                            eval_ctx: &eval_ctx,
                            mutated_label: &mut mutated,
                        },
                    },
                );
            });
        });
        out.textures_delta.clear();
        assert!(!out.shapes.is_empty());
        assert_eq!(mutated, None);
    }
}
