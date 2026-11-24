mod fixtures_tab;
mod heights_tab;
mod rotary_tab;
mod stock_tab;
mod wcs_tab;

use std::fmt::Debug;
use std::hash::Hash;
use std::path::Path;

use chip_cad::{BodyId, CadDocument};
use chip_cam::{
    AxisAnchor, BoxAnchor27, FixtureBody, ResolvedFixtureSolid, ResolvedSetup, RotaryConfig, Setup,
    SetupHeights, StockConfig, WcsConfig, WorkOffsetSlot,
};
use chip_math::{EvalContext, ParamLength};
use chip_post::{PostProfileSource, load_available_post_profiles};
use chip_project::{ModelEntry, ProjectDocument, UndoHistory};
use chip_tools::MachineDefinition;

use crate::{
    panels::{SetupTab, ViewportPickMode},
    widgets::{
        BannerKind, INPUT_COL_WIDTH, edit_param_length, grid_row, inline_banner, inspector_grid,
        section_card, static_card,
    },
};

use self::{
    fixtures_tab::{FixturesTabCtx, show_fixtures_tab},
    heights_tab::{HeightsTabCtx, show_heights_tab},
    rotary_tab::show_rotary_tab,
    stock_tab::{StockTabView, show_stock_tab},
    wcs_tab::{WcsTabCtx, show_wcs_tab},
};

/// Live rapid-avoidance and collision summary shown inside the `[Fixtures]` Setup tab (`M61`).
#[derive(Debug, Clone, Copy, Default)]
pub struct FixtureSafetySummary {
    pub rapid_lifts_inserted: usize,
    pub max_lifted_z_wcs: f64,
    pub colliding_fixture_count: usize,
}

/// Context passed to [`show_setup_inspector`].
pub struct SetupInspectorCtx<'a> {
    pub project: &'a mut ProjectDocument,
    pub setup_idx: usize,
    pub history: &'a mut UndoHistory,
    pub project_dir: Option<&'a Path>,
    pub cad_doc: Option<&'a CadDocument>,
    pub resolved: Option<Result<&'a ResolvedSetup, &'a str>>,
    pub resolved_fixtures: &'a [ResolvedFixtureSolid],
    pub fixture_safety: FixtureSafetySummary,
    pub tab: &'a mut SetupTab,
    pub pick_mode: &'a mut ViewportPickMode,
    pub selected_fixture_idx: &'a mut Option<usize>,
    pub show_fixtures: &'a mut bool,
    pub show_27pt_dots: &'a mut bool,
    pub show_z_planes: &'a mut bool,
    pub machines: &'a [MachineDefinition],
    pub open_machine_manager: &'a mut bool,
    pub open_post_inspector: &'a mut Option<String>,
    pub machine_changed: &'a mut bool,
    pub switch_machine_id: &'a mut Option<String>,
    pub import_fixture_requested: &'a mut bool,
    pub open_flip_jig_wizard: &'a mut bool,
    pub open_soft_jaw_wizard: &'a mut bool,
}

/// Lightweight pre-edit snapshot of only the Setup's header & WCS/Stock/Fixtures/Post/Heights
/// fields (never cloning `project.tools` or `setup.operations` on idle 60 FPS frames!).
struct SetupParamsSnapshot {
    name: String,
    target_bodies: Vec<BodyId>,
    wcs_slot: WorkOffsetSlot,
    post_profile: String,
    wcs: WcsConfig,
    stock: StockConfig,
    carryover_from_setup_id: Option<String>,
    rotary: RotaryConfig,
    heights: SetupHeights,
    fixtures: Vec<FixtureBody>,
    auto_avoid_fixtures_on_rapids: bool,
}

impl SetupParamsSnapshot {
    fn capture(setup: &Setup) -> Self {
        Self {
            name: setup.name.clone(),
            target_bodies: setup.target_bodies.clone(),
            wcs_slot: setup.wcs_slot,
            post_profile: setup.post_profile.clone(),
            wcs: setup.wcs.clone(),
            stock: setup.stock.clone(),
            carryover_from_setup_id: setup.carryover_from_setup_id.clone(),
            rotary: setup.rotary.clone(),
            heights: setup.heights.clone(),
            fixtures: setup.fixtures.clone(),
            auto_avoid_fixtures_on_rapids: setup.auto_avoid_fixtures_on_rapids,
        }
    }

    fn restore_into(self, setup: &mut Setup) {
        setup.name = self.name;
        setup.target_bodies = self.target_bodies;
        setup.wcs_slot = self.wcs_slot;
        setup.post_profile = self.post_profile;
        setup.wcs = self.wcs;
        setup.stock = self.stock;
        setup.carryover_from_setup_id = self.carryover_from_setup_id;
        setup.rotary = self.rotary;
        setup.heights = self.heights;
        setup.fixtures = self.fixtures;
        setup.auto_avoid_fixtures_on_rapids = self.auto_avoid_fixtures_on_rapids;
    }
}

pub(super) fn setup_field_tip(key: &str) -> &'static str {
    match key {
        "stk_sheet_z" => "Nominal flat stock Z thickness for Pure-2D sketch jobs",
        "fbox_x" | "fbox_y" | "fbox_z" => "Total fixed-size stock blank dimension in Setup WCS",
        "fcyl_d" => "Outer diameter of cylindrical bar stock",
        "fcyl_l" => "Total axial length of cylindrical bar stock",
        "stk_px" | "stk_nx" | "stk_py" | "stk_ny" => "Extra raw side stock outside model XY bounds",
        "stk_tz" | "rcyl_t" | "fcyl_to" => "Facing stock allowance above model top (+Z)",
        "stk_bz" | "rcyl_b" => "Vise, chuck, or sacrificial tab stock below model bottom (-Z)",
        "fbox_dx" | "fbox_dy" => "Model XY offset from fixed stock front-left (-X, -Y) corner",
        "fbox_ta" => "Distance from fixed stock top down to model top",
        "fbox_ba" => "Distance from fixed stock bottom up to model bottom",
        "rcyl_r" => "Extra radial stock outside part bounding radius",
        "setup_clr_off" => "Signed Z offset added to Clearance datum plane",
        "setup_ret_off" => "Signed Z offset added to Retract datum plane",
        _ => "Additional WCS shift from anchor datum",
    }
}

pub(super) struct SetupGrid<'a> {
    ui: &'a mut egui::Ui,
    setup_idx: usize,
    eval_ctx: &'a EvalContext,
    mutated: &'a mut Option<&'static str>,
}

impl SetupGrid<'_> {
    pub(super) fn len(
        &mut self,
        label: &str,
        key: &'static str,
        p: &mut ParamLength,
        undo: &'static str,
    ) -> bool {
        let salt = (key, self.setup_idx);
        let ctx = self.eval_ctx;
        let changed = grid_row(self.ui, label, Some(setup_field_tip(key)), |ui| {
            edit_param_length(ui, salt, p, ctx, INPUT_COL_WIDTH)
        });
        if changed {
            *self.mutated = Some(undo);
        }
        changed
    }
}

/// Renders the tabbed **Setup Inspector** (`[WCS & Post]`, `[Stock]`, `[Fixtures]`, `[4th Axis]`, `[Heights]`).
pub fn show_setup_inspector(ui: &mut egui::Ui, ctx: SetupInspectorCtx<'_>) -> bool {
    let SetupInspectorCtx {
        project,
        setup_idx,
        history,
        project_dir,
        cad_doc,
        resolved,
        resolved_fixtures,
        fixture_safety,
        tab,
        pick_mode,
        selected_fixture_idx,
        show_fixtures,
        show_27pt_dots,
        show_z_planes,
        machines,
        open_machine_manager,
        open_post_inspector,
        machine_changed,
        switch_machine_id,
        import_fixture_requested,
        open_flip_jig_wizard,
        open_soft_jaw_wizard,
    } = ctx;

    if setup_idx >= project.setups.len() {
        ui.weak("No Setup selected.");
        return false;
    }

    let display_unit = project.display_units;
    let mut align_camera_requested = false;
    let pre_setup = SetupParamsSnapshot::capture(&project.setups[setup_idx]);
    let mut mutated_label: Option<&'static str> = None;

    static_card(ui, None, |ui| {
        let setup = &mut project.setups[setup_idx];
        inspector_grid(ui, ("setup_hdr_grid", setup_idx), |ui| {
            grid_row(
                ui,
                "Setup Name",
                Some("Clamping setup title emitted in G-code headers"),
                |ui| {
                    let resp = ui.add(
                        egui::TextEdit::singleline(&mut setup.name).desired_width(INPUT_COL_WIDTH),
                    );
                    if resp.lost_focus() && setup.name != pre_setup.name {
                        mutated_label = Some("Rename Setup");
                    }
                    let tip = "Align viewport camera to this Setup's +Z axis";
                    if ui.small_button("Align Top").on_hover_text(tip).clicked() {
                        align_camera_requested = true;
                    }
                },
            );
            render_work_offset_row(
                ui,
                ("wcs_slot_hdr_cb", setup_idx),
                &mut setup.wcs_slot,
                &mut mutated_label,
            );
        });

        ui.add_space(4.0);
        ui.separator();
        ui.add_space(2.0);
        ui.horizontal_wrapped(|ui| {
            ui.selectable_value(tab, SetupTab::Wcs, "WCS & Post")
                .on_hover_text("Configure Work Coordinate System orientation, origin datum, target CNC machine, and post-processor");
            ui.selectable_value(tab, SetupTab::Stock, "Stock")
                .on_hover_text("Configure raw stock blank shape, fixed or relative dimensions, and side/Z margins");
            let fix_count = setup.fixtures.len();
            let fix_lbl = if fix_count > 0 {
                format!("Fixtures ({fix_count})")
            } else {
                "Fixtures".to_string()
            };
            ui.selectable_value(tab, SetupTab::Fixtures, fix_lbl)
                .on_hover_text("Configure 3D workholding hardware (vises, toe clamps, spoilboards, keep-outs, and imported fixtures)");
            ui.selectable_value(tab, SetupTab::Rotary, "4th Axis")
                .on_hover_text("Configure 4th-axis rotary kinematics, axis letter, chuck geometry, and cylindrical stock alignment");
            ui.selectable_value(tab, SetupTab::Heights, "Heights")
                .on_hover_text("Configure Setup-level safe Clearance and Retract Z heights");
        });
    });

    let eval_ctx = match resolved {
        Some(Ok(rs)) => rs.eval_context(display_unit),
        _ => EvalContext::new(display_unit),
    };

    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            if let Some(Err(err_msg)) = &resolved {
                inline_banner(ui, BannerKind::Error, *err_msg);
                ui.add_space(5.0);
            }
            if is_setup_wcs_pick_mode(*pick_mode, setup_idx)
                && let Some(msg) = pick_mode.banner_text()
            {
                inline_banner(ui, BannerKind::Info, msg);
                ui.add_space(5.0);
            }

            if *tab == SetupTab::Wcs && !machines.is_empty() {
                let current_mid = project.effective_machine_id().to_string();
                let current_machine = machines
                    .iter()
                    .find(|m| m.id.eq_ignore_ascii_case(&current_mid))
                    .or_else(|| machines.first());
                let mut picked_machine: Option<String> = None;
                let mut apply_defaults_clicked = false;
                section_card(
                    ui,
                    ("setup_machine_card", setup_idx),
                    "Target CNC Machine",
                    true,
                    |ui| {
                        inspector_grid(ui, ("setup_machine_grid", setup_idx), |ui| {
                            grid_row(
                                ui,
                                "Machine",
                                Some(
                                    "Target CNC machine envelope, spindle limits, kinematics, & tool set",
                                ),
                                |ui| {
                                    let label = current_machine
                                        .map_or(current_mid.as_str(), |m| m.name.as_str());
                                    egui::ComboBox::from_id_salt(("setup_machine_cb", setup_idx))
                                        .width(INPUT_COL_WIDTH + 54.0)
                                        .truncate()
                                        .selected_text(label)
                                        .show_ui(ui, |ui| {
                                            for m in machines {
                                                let tag = if m.is_builtin {
                                                    "[Built-In]"
                                                } else {
                                                    "[Custom]"
                                                };
                                                if ui
                                                    .selectable_label(
                                                        m.id.eq_ignore_ascii_case(&current_mid),
                                                        format!("{} {}", m.name, tag),
                                                    )
                                                    .clicked()
                                                {
                                                    picked_machine = Some(m.id.clone());
                                                }
                                            }
                                        })
                                        .response
                                        .on_hover_text(
                                            "Select target CNC machine profile (travel limits, RPM range, kinematics, and machine tool set)",
                                        );
                                },
                            );
                        });
                        if let Some(m) = current_machine {
                            ui.small(format!(
                                "Travel {:.0}×{:.0}×{:.0} mm | {:.0}–{:.0} RPM | {} tools | Post: {}",
                                m.travel_mm.x_mm,
                                m.travel_mm.y_mm,
                                m.travel_mm.z_mm,
                                m.spindle.min_rpm,
                                m.spindle.max_rpm,
                                m.tools.len(),
                                m.default_post_profile
                            ));
                        }
                        ui.add_space(2.0);
                        ui.horizontal_wrapped(|ui| {
                            if ui
                                .small_button("Apply Defaults")
                                .on_hover_text(
                                    "Apply this machine's default post-processor and 4th-axis chuck dimensions to setups",
                                )
                                .clicked()
                            {
                                apply_defaults_clicked = true;
                            }
                            if ui
                                .small_button("Manage Machines…")
                                .on_hover_text(
                                    "Open Machine Manager to inspect, clone, export, or import .machine.toml profiles",
                                )
                                .clicked()
                            {
                                *open_machine_manager = true;
                            }
                        });
                    },
                );

                if let Some(new_mid) = picked_machine {
                    if !new_mid.eq_ignore_ascii_case(&current_mid) {
                        *switch_machine_id = Some(new_mid);
                    }
                } else if apply_defaults_clicked
                    && let Some(m) = current_machine
                {
                    history.record_before_edit("Apply Machine Defaults", None, project);
                    project.apply_machine_defaults(m, true);
                    *machine_changed = true;
                }
            }

            let fixture_models: Vec<ModelEntry> =
                if *tab == SetupTab::Fixtures {
                    project.fixture_models().cloned().collect()
                } else {
                    Vec::new()
                };
            let preceding_setups: Vec<(String, String, bool)> =
                if *tab == SetupTab::Stock && setup_idx > 0 {
                    project.setups[..setup_idx]
                        .iter()
                        .map(|s| {
                            let all_clean = s.operations.iter().all(|o| {
                                o.suppressed
                                    || o.runtime_status == chip_cam::OperationStatus::Clean
                            });
                            (s.id.clone(), s.name.clone(), all_clean)
                        })
                        .collect()
                } else {
                    Vec::new()
                };
            let setup = &mut project.setups[setup_idx];
            match *tab {
                SetupTab::Wcs => show_wcs_tab(
                    ui,
                    WcsTabCtx {
                        setup,
                        setup_idx,
                        project_dir,
                        cad_doc,
                        eval_ctx: &eval_ctx,
                        pick_mode,
                        show_27pt_dots,
                        open_post_inspector,
                        mutated_label: &mut mutated_label,
                    },
                ),
                SetupTab::Stock => show_stock_tab(
                    ui,
                    setup,
                    StockTabView {
                        setup_idx,
                        is_pure_2d: cad_doc.is_none(),
                        eval_ctx: &eval_ctx,
                        display_unit,
                        resolved,
                        preceding_setups: &preceding_setups,
                    },
                    &mut mutated_label,
                ),
                SetupTab::Fixtures => show_fixtures_tab(
                    ui,
                    FixturesTabCtx {
                        setup,
                        setup_idx,
                        fixture_models: &fixture_models,
                        eval_ctx: &eval_ctx,
                        display_unit,
                        resolved,
                        resolved_fixtures,
                        fixture_safety,
                        selected_fixture_idx,
                        show_fixtures,
                        pick_mode,
                        import_fixture_requested,
                        open_flip_jig_wizard,
                        open_soft_jaw_wizard,
                        mutated_label: &mut mutated_label,
                    },
                ),
                SetupTab::Rotary => show_rotary_tab(
                    ui,
                    setup,
                    setup_idx,
                    &eval_ctx,
                    display_unit,
                    resolved,
                    &mut mutated_label,
                ),
                SetupTab::Heights => show_heights_tab(
                    ui,
                    HeightsTabCtx {
                        setup,
                        setup_idx,
                        eval_ctx: &eval_ctx,
                        display_unit,
                        resolved,
                        show_z_planes,
                        mutated_label: &mut mutated_label,
                    },
                ),
            }
        });

    if let Some(action) = mutated_label {
        let post_only = action == "Change Post Profile" || action == "Rename Setup";
        let mut pre_doc = project.clone();
        pre_setup.restore_into(&mut pre_doc.setups[setup_idx]);
        history.record_before_edit(action, None, &pre_doc);
        if !post_only {
            for op in &mut project.setups[setup_idx].operations {
                op.mark_dirty();
            }
        }
    }

    align_camera_requested
}

pub(super) fn render_work_offset_row(
    ui: &mut egui::Ui,
    id_salt: impl Hash + Debug,
    wcs_slot: &mut WorkOffsetSlot,
    mutated_label: &mut Option<&'static str>,
) {
    grid_row(
        ui,
        "Work Offset",
        Some("Controller work coordinate slot (G54..G59.3)"),
        |ui| {
            egui::ComboBox::from_id_salt(id_salt)
                .width(INPUT_COL_WIDTH)
                .selected_text(wcs_slot.label())
                .show_ui(ui, |ui| {
                    for slot in WorkOffsetSlot::ALL {
                        if ui
                            .selectable_value(wcs_slot, slot, slot.label())
                            .on_hover_text(format!("Use CNC work offset {}", slot.label()))
                            .changed()
                        {
                            *mutated_label = Some("Change Work Offset Slot");
                        }
                    }
                })
                .response
                .on_hover_text("Select CNC controller work coordinate offset (G54..G59.3)");
        },
    );
}

pub(super) fn is_setup_wcs_pick_mode(mode: ViewportPickMode, setup_idx: usize) -> bool {
    matches!(
        mode,
        ViewportPickMode::SetupWcsZ { setup_idx: s }
            | ViewportPickMode::SetupWcsX { setup_idx: s }
            | ViewportPickMode::SetupWcsXyPlane { setup_idx: s }
            | ViewportPickMode::SetupWcsXyPlaneXEdge { setup_idx: s }
            | ViewportPickMode::SetupWcsOriginFeature { setup_idx: s }
            if s == setup_idx
    )
}

pub(super) fn render_post_body(
    ui: &mut egui::Ui,
    setup: &mut Setup,
    setup_idx: usize,
    project_dir: Option<&Path>,
    open_post_inspector: &mut Option<String>,
    mutated_label: &mut Option<&'static str>,
) {
    let available_posts = load_available_post_profiles(project_dir);
    let active_prof = available_posts
        .iter()
        .find(|p| p.id.eq_ignore_ascii_case(&setup.post_profile));
    let selected_lbl = match active_prof {
        Some(p) if p.is_builtin => p.name.clone(),
        Some(p) => format!("{} [Custom]", p.name),
        None => post_profile_short_label(&setup.post_profile).to_string(),
    };

    inspector_grid(ui, ("setup_post_grid", setup_idx), |ui| {
        grid_row(
            ui,
            "Post Profile",
            Some("Target CNC controller dialect (.post.toml)"),
            |ui| {
                egui::ComboBox::from_id_salt(("setup_post_cb", setup_idx))
                    .width(INPUT_COL_WIDTH + 54.0)
                    .truncate()
                    .selected_text(selected_lbl)
                    .show_ui(ui, |ui| {
                        for prof in &available_posts {
                            let lbl = if prof.is_builtin {
                                format!("{} ({})", prof.name, prof.id)
                            } else {
                                format!("{} ({}) [Custom]", prof.name, prof.id)
                            };
                            let tip = match prof.source {
                                PostProfileSource::BuiltIn => {
                                    "Select this built-in CNC controller post-processor dialect"
                                }
                                PostProfileSource::UserConfig => {
                                    "Select this custom post-processor profile from user configuration"
                                }
                                PostProfileSource::ProjectOrBundle => {
                                    "Select this custom post-processor profile bundled with the project"
                                }
                                PostProfileSource::ExplicitFile => {
                                    "Select this custom post-processor profile loaded from disk"
                                }
                            };
                            if ui
                                .selectable_value(&mut setup.post_profile, prof.id.clone(), lbl)
                                .on_hover_text(tip)
                                .changed()
                            {
                                *mutated_label = Some("Change Post Profile");
                            }
                        }
                    })
                    .response
                    .on_hover_text(
                        "Choose the CNC controller G-code post-processor profile for this Setup (built-in or custom .post.toml)",
                    );
            },
        );
    });

    ui.add_space(3.0);
    if ui
        .small_button("Inspect / Customize Post...")
        .on_hover_text(
            "Open the Post Profile Inspector / Customizer to view built-in dialect settings or clone to a custom .post.toml",
        )
        .clicked()
    {
        *open_post_inspector = Some(setup.post_profile.clone());
    }

    ui.add_space(4.0);
    let active_prof = available_posts
        .iter()
        .find(|p| p.id.eq_ignore_ascii_case(&setup.post_profile));
    let extended_slot = matches!(
        setup.wcs_slot,
        WorkOffsetSlot::G59_1 | WorkOffsetSlot::G59_2 | WorkOffsetSlot::G59_3
    );
    let blocks_extended_wcs =
        active_prof.map_or(setup.post_profile == "grbl", |p| !p.allow_extended_wcs);
    if extended_slot && blocks_extended_wcs {
        inline_banner(
            ui,
            BannerKind::Warn,
            "Standard GRBL 1.1 only supports G54-G59; switch to grblHAL or LinuxCNC for G59.1-G59.3.",
        );
    } else if let Some(prof) = active_prof
        && prof.source == PostProfileSource::ProjectOrBundle
    {
        inline_banner(
            ui,
            BannerKind::Warn,
            format!(
                "[Custom] Project post profile '{}' loaded from project directory - review G-code before machining.",
                prof.id
            ),
        );
    } else if let Some(prof) = active_prof
        && !prof.is_builtin
        && !prof.description.trim().is_empty()
    {
        inline_banner(
            ui,
            BannerKind::Info,
            format!("[Custom] {}", prof.description.trim()),
        );
    } else {
        inline_banner(
            ui,
            BannerKind::Info,
            post_profile_summary(&setup.post_profile),
        );
    }
}

fn post_profile_short_label(profile_id: &str) -> &str {
    match profile_id {
        "grbl" => "GRBL 1.1 / FluidNC",
        "grblhal_atc" => "grblHAL / FluidNC ATC",
        "linuxcnc" => "LinuxCNC (RS274NGC)",
        other => other,
    }
}

fn post_profile_summary(profile_id: &str) -> &'static str {
    match profile_id {
        "grbl" => {
            "Splits multi-tool jobs into separate .nc files; <=80 chars/line, expanded peck cycles."
        }
        "grblhal_atc" => {
            "Single .nc program with M6 tool-setter macro changes and expanded drill cycles."
        }
        "linuxcnc" => {
            "RS274NGC (.ngc) with T# M6, G43 H# length offsets, G64 blending, and canned cycles."
        }
        _ => "Custom TOML post-processor profile configured for this Setup.",
    }
}

pub(super) fn show_27pt_xy_matrix(ui: &mut egui::Ui, anchor: &mut BoxAnchor27) -> bool {
    let mut changed = false;
    egui::Grid::new("box_anchor_3x3_xy")
        .spacing([3.0, 3.0])
        .show(ui, |ui| {
            for (y_name, y_a) in [
                ("Back (+Y)", AxisAnchor::Max),
                ("Center Y", AxisAnchor::Center),
                ("Front (-Y)", AxisAnchor::Min),
            ] {
                for (x_name, x_short, x_a) in [
                    ("Left (-X)", "L", AxisAnchor::Min),
                    ("Center X", "C", AxisAnchor::Center),
                    ("Right (+X)", "R", AxisAnchor::Max),
                ] {
                    let is_sel = anchor.x == x_a && anchor.y == y_a;
                    if ui
                        .selectable_label(is_sel, format!(" {x_short} "))
                        .on_hover_text(format!("Set XY origin to {y_name}, {x_name}"))
                        .clicked()
                        && !is_sel
                    {
                        anchor.x = x_a;
                        anchor.y = y_a;
                        changed = true;
                    }
                }
                ui.end_row();
            }
        });
    changed
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::path::Path;

    use chip_cad::{
        BodyId, CadDocument, EntitySignatureKind, GeometricSignature, SelectionBinding, SolidBody,
    };
    use chip_cam::{
        FixtureBody, HeightReference, Operation, OperationKind, OperationStatus, WcsAxisRef,
        WcsFeaturePointRef, WcsOrientationMode, WcsOriginMode,
    };
    use chip_math::{Aabb3, DVec3, ParamLength};
    use chip_project::{ProjectDocument, UndoHistory};

    use crate::panels::{
        SetupTab, ViewportPickMode,
        setup_inspector::{FixtureSafetySummary, SetupInspectorCtx, show_setup_inspector},
    };

    fn collect_texts(shapes: &[egui::epaint::ClippedShape], out: &mut Vec<String>) {
        fn visit(shape: &egui::Shape, out: &mut Vec<String>) {
            match shape {
                egui::Shape::Text(t) => out.push(t.galley.text().to_string()),
                egui::Shape::Vec(v) => {
                    for s in v {
                        visit(s, out);
                    }
                }
                _ => {}
            }
        }
        for cs in shapes {
            visit(&cs.shape, out);
        }
    }

    fn find_text_center(shape: &egui::Shape, needle: &str) -> Option<egui::Pos2> {
        match shape {
            egui::Shape::Text(t) if t.galley.text() == needle => {
                Some(t.pos + t.galley.size() * 0.5)
            }
            egui::Shape::Vec(v) => v.iter().find_map(|s| find_text_center(s, needle)),
            _ => None,
        }
    }

    fn find_text_containing(shape: &egui::Shape, needle: &str) -> Option<egui::Pos2> {
        match shape {
            egui::Shape::Text(t) if t.galley.text().contains(needle) => {
                Some(t.pos + t.galley.size() * 0.5)
            }
            egui::Shape::Vec(v) => v.iter().find_map(|s| find_text_containing(s, needle)),
            _ => None,
        }
    }

    fn click_at(pt: egui::Pos2) -> Vec<egui::Event> {
        vec![
            egui::Event::PointerMoved(pt),
            egui::Event::PointerButton {
                pos: pt,
                button: egui::PointerButton::Primary,
                pressed: true,
                modifiers: egui::Modifiers::NONE,
            },
            egui::Event::PointerButton {
                pos: pt,
                button: egui::PointerButton::Primary,
                pressed: false,
                modifiers: egui::Modifiers::NONE,
            },
        ]
    }

    fn dummy_binding(kind: EntitySignatureKind, idx: u32) -> SelectionBinding {
        SelectionBinding {
            body_id: BodyId(0),
            last_index: idx,
            signature: GeometricSignature {
                kind,
                normal_or_axis: DVec3::Z,
                centroid_mm: DVec3::ZERO,
                characteristic_measure: 10.0,
                aabb_mm: Aabb3::new(DVec3::ZERO, DVec3::new(10.0, 10.0, 10.0)),
            },
        }
    }

    fn two_body_cad_doc() -> CadDocument {
        let mut doc = CadDocument::empty();
        doc.bodies.push(SolidBody {
            id: BodyId(0),
            name: "Base Plate".to_string(),
            faces: Vec::new(),
            edges: Vec::new(),
            aabb_mm: Aabb3::new(DVec3::ZERO, DVec3::new(50.0, 50.0, 10.0)),
        });
        doc.bodies.push(SolidBody {
            id: BodyId(1),
            name: "Clamp Bracket".to_string(),
            faces: Vec::new(),
            edges: Vec::new(),
            aabb_mm: Aabb3::new(DVec3::new(60.0, 0.0, 0.0), DVec3::new(90.0, 40.0, 12.0)),
        });
        doc.aabb_mm = Aabb3::new(DVec3::ZERO, DVec3::new(90.0, 50.0, 12.0));
        doc
    }

    #[allow(clippy::too_many_arguments)]
    fn run_setup_frame(
        ctx: &egui::Context,
        project: &mut ProjectDocument,
        history: &mut UndoHistory,
        tab: &mut SetupTab,
        pick_mode: &mut ViewportPickMode,
        project_dir: Option<&Path>,
        cad_doc: Option<&CadDocument>,
        events: Vec<egui::Event>,
    ) -> Vec<egui::epaint::ClippedShape> {
        let mut sel_fix = Some(0);
        let mut show_fix = true;
        let mut show_27 = false;
        let mut show_z = false;
        let mut open_mm = false;
        let mut open_post: Option<String> = None;
        let mut m_changed = false;
        let mut sw_mid = None;
        let mut imp_fix = false;
        let mut open_fj = false;
        let mut open_sj = false;
        let mut out = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(480.0, 1400.0),
                )),
                events,
                ..Default::default()
            },
            |ui| {
                egui::CentralPanel::default().show(ui, |ui| {
                    let _ = show_setup_inspector(
                        ui,
                        SetupInspectorCtx {
                            project,
                            setup_idx: 0,
                            history,
                            project_dir,
                            cad_doc,
                            resolved: None,
                            resolved_fixtures: &[],
                            fixture_safety: FixtureSafetySummary::default(),
                            tab,
                            pick_mode,
                            selected_fixture_idx: &mut sel_fix,
                            show_fixtures: &mut show_fix,
                            show_27pt_dots: &mut show_27,
                            show_z_planes: &mut show_z,
                            machines: &[],
                            open_machine_manager: &mut open_mm,
                            open_post_inspector: &mut open_post,
                            machine_changed: &mut m_changed,
                            switch_machine_id: &mut sw_mid,
                            import_fixture_requested: &mut imp_fix,
                            open_flip_jig_wizard: &mut open_fj,
                            open_soft_jaw_wizard: &mut open_sj,
                        },
                    );
                });
            },
        );
        out.textures_delta.clear();
        out.shapes
    }

    #[test]
    fn m88_setup_inspector_custom_post_profile_selection_and_badge() {
        let tmp =
            std::env::temp_dir().join(format!("chipcutter_m88_custom_post_{}", std::process::id()));
        let posts_dir = tmp.join("posts");
        let _ = std::fs::remove_dir_all(&tmp);
        std::fs::create_dir_all(&posts_dir).unwrap();
        let custom_toml = chip_post::BUILTIN_GRBL_TOML
            .replace("id = \"grbl\"", "id = \"shop_custom_mill\"")
            .replace(
                "name = \"GRBL 1.1 / FluidNC (Split by Tool)\"",
                "name = \"Shop Custom Mill\"",
            );
        std::fs::write(posts_dir.join("shop_custom_mill.post.toml"), custom_toml).unwrap();

        let ctx = egui::Context::default();
        let mut proj = ProjectDocument::new("custom_post_test");
        let mut op = Operation::new("op1", "Face", OperationKind::Facing);
        op.runtime_status = OperationStatus::Clean;
        proj.setups[0].operations = vec![op];
        let mut hist = UndoHistory::default();
        let mut tab = SetupTab::Wcs;
        let mut pick_mode = ViewportPickMode::InspectEntity;

        let _ = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            Some(&tmp),
            None,
            Vec::new(),
        );
        let s1 = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            Some(&tmp),
            None,
            Vec::new(),
        );
        let post_cb_pt = s1
            .iter()
            .find_map(|cs| find_text_containing(&cs.shape, "GRBL 1.1"))
            .expect("Post Profile combo must display active GRBL profile");
        let _ = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            Some(&tmp),
            None,
            click_at(post_cb_pt),
        );
        let s_open = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            Some(&tmp),
            None,
            Vec::new(),
        );
        let custom_pt = s_open
            .iter()
            .find_map(|cs| find_text_containing(&cs.shape, "[Custom]"))
            .expect("Custom .post.toml profile must appear with [Custom] badge in Setup Inspector");
        let _ = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            Some(&tmp),
            None,
            click_at(custom_pt),
        );

        assert_eq!(proj.setups[0].post_profile, "shop_custom_mill");
        // Changing post profile is post-only and must not dirty Clean toolpaths
        assert_eq!(
            proj.setups[0].operations[0].runtime_status,
            OperationStatus::Clean
        );
        assert!(hist.can_undo());
        assert!(hist.undo(&mut proj).is_some());
        assert_eq!(proj.setups[0].post_profile, "grbl");

        let _ = std::fs::remove_dir_all(&tmp);
    }

    #[test]
    fn m88_setup_inspector_target_bodies_toggle_and_undo() {
        let doc = two_body_cad_doc();
        let ctx = egui::Context::default();
        let mut proj = ProjectDocument::new("multi_body_test");
        let mut op = Operation::new("op1", "Face", OperationKind::Facing);
        op.runtime_status = OperationStatus::Clean;
        proj.setups[0].operations = vec![op];
        let mut hist = UndoHistory::default();
        let mut tab = SetupTab::Wcs;
        let mut pick_mode = ViewportPickMode::InspectEntity;

        let _ = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            Some(&doc),
            Vec::new(),
        );
        let s1 = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            Some(&doc),
            Vec::new(),
        );
        let all_pt = s1
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "All Part Bodies"))
            .expect("Target Bodies card must render 'All Part Bodies' checkbox for 2-body CAD");

        // Uncheck "All Part Bodies" -> explicit mode with both bodies initially selected
        let _ = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            Some(&doc),
            click_at(all_pt),
        );
        assert_eq!(proj.setups[0].target_bodies, vec![BodyId(0), BodyId(1)]);
        assert_eq!(
            proj.setups[0].operations[0].runtime_status,
            OperationStatus::Dirty
        );

        let _ = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            Some(&doc),
            Vec::new(),
        );
        let s2 = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            Some(&doc),
            Vec::new(),
        );
        let body1_pt = s2
            .iter()
            .find_map(|cs| find_text_containing(&cs.shape, "Clamp Bracket"))
            .expect("Explicit mode must show per-body checkbox for Clamp Bracket");
        let _ = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            Some(&doc),
            click_at(body1_pt),
        );
        assert_eq!(proj.setups[0].target_bodies, vec![BodyId(0)]);

        // Undo restores [0, 1], second Undo restores [] (All Part Bodies)
        assert!(hist.undo(&mut proj).is_some());
        assert_eq!(proj.setups[0].target_bodies, vec![BodyId(0), BodyId(1)]);
        assert!(hist.undo(&mut proj).is_some());
        assert!(proj.setups[0].target_bodies.is_empty());
    }

    #[test]
    fn m88_wcs_xy_plane_x_edge_z_and_x_clear_and_edge_origin_segments() {
        let doc = two_body_cad_doc();
        let ctx = egui::Context::default();
        let mut proj = ProjectDocument::new("wcs_picking_test");
        let face_bind = dummy_binding(EntitySignatureKind::PlanarFace, 2);
        let edge_bind = dummy_binding(EntitySignatureKind::LineEdge, 7);

        // 1. SelectXYPlane: Pick X Edge activates SetupWcsXyPlaneXEdge; Clear (Auto) clears x_edge_ref with Undo
        proj.setups[0].wcs.orientation = WcsOrientationMode::SelectXYPlane {
            face_ref: face_bind.clone(),
            flip_z: false,
            x_edge_ref: Some(edge_bind.clone()),
            clock_90: 0,
            extra_z_rot_deg: 0.0,
        };
        proj.setups[0].wcs.origin = WcsOriginMode::SelectedCadFeature {
            feature: WcsFeaturePointRef::EdgeMidpoint(edge_bind.clone()),
        };

        let mut hist = UndoHistory::default();
        let mut tab = SetupTab::Wcs;
        let mut pick_mode = ViewportPickMode::InspectEntity;

        let _ = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            Some(&doc),
            Vec::new(),
        );
        let s1 = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            Some(&doc),
            Vec::new(),
        );
        let mut texts = Vec::new();
        collect_texts(&s1, &mut texts);
        assert!(
            texts.iter().any(|t| t == "Edge #7"),
            "SelectXYPlane must render bound x_edge_ref readout Edge #7"
        );

        let pick_x_pt = s1
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Pick X Edge"))
            .expect("SelectXYPlane must show 'Pick X Edge' button");
        let _ = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            Some(&doc),
            click_at(pick_x_pt),
        );
        assert_eq!(
            pick_mode,
            ViewportPickMode::SetupWcsXyPlaneXEdge { setup_idx: 0 }
        );

        // Re-render after pick banner appears (banner shifts content down) and click "Clear (Auto)"
        let _ = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            Some(&doc),
            Vec::new(),
        );
        let s_after_pick = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            Some(&doc),
            Vec::new(),
        );
        let clear_pt = s_after_pick
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Clear (Auto)"))
            .expect("SelectXYPlane with bound x_edge_ref must show 'Clear (Auto)'");
        let _ = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            Some(&doc),
            click_at(clear_pt),
        );
        assert!(matches!(
            proj.setups[0].wcs.orientation,
            WcsOrientationMode::SelectXYPlane {
                x_edge_ref: None,
                ..
            }
        ));
        assert_eq!(pick_mode, ViewportPickMode::InspectEntity);
        assert!(hist.undo(&mut proj).is_some());
        assert!(matches!(
            proj.setups[0].wcs.orientation,
            WcsOrientationMode::SelectXYPlane {
                x_edge_ref: Some(_),
                ..
            }
        ));

        // 2. EdgeStart / EdgeMidpoint / EdgeEnd segmented buttons preserve exact SelectionBinding
        let _ = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            Some(&doc),
            Vec::new(),
        );
        let s_edge = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            Some(&doc),
            Vec::new(),
        );
        let start_pt = s_edge
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Start"))
            .expect("Edge origin must render 'Start' segment");
        let _ = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            Some(&doc),
            click_at(start_pt),
        );
        assert_eq!(
            proj.setups[0].wcs.origin,
            WcsOriginMode::SelectedCadFeature {
                feature: WcsFeaturePointRef::EdgeStart(edge_bind.clone())
            }
        );

        let end_pt = s_edge
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "End"))
            .expect("Edge origin must render 'End' segment");
        let _ = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            Some(&doc),
            click_at(end_pt),
        );
        assert_eq!(
            proj.setups[0].wcs.origin,
            WcsOriginMode::SelectedCadFeature {
                feature: WcsFeaturePointRef::EdgeEnd(edge_bind.clone())
            }
        );
        assert!(hist.undo(&mut proj).is_some());
        assert_eq!(
            proj.setups[0].wcs.origin,
            WcsOriginMode::SelectedCadFeature {
                feature: WcsFeaturePointRef::EdgeStart(edge_bind.clone())
            }
        );

        // 3. SelectZAndX: Clear (Auto) button beside x_ref clears to None with Undo
        proj.setups[0].wcs.orientation = WcsOrientationMode::SelectZAndX {
            z_ref: WcsAxisRef::FaceAxis(face_bind),
            flip_z: false,
            x_ref: Some(WcsAxisRef::EdgeDirection(edge_bind)),
            flip_x: false,
            clock_90: 0,
            extra_z_rot_deg: 0.0,
        };
        let _ = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            Some(&doc),
            Vec::new(),
        );
        let s_zx = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            Some(&doc),
            Vec::new(),
        );
        let clear_zx_pt = s_zx
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Clear (Auto)"))
            .expect("SelectZAndX with set x_ref must render 'Clear (Auto)'");
        let _ = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            Some(&doc),
            click_at(clear_zx_pt),
        );
        assert!(matches!(
            proj.setups[0].wcs.orientation,
            WcsOrientationMode::SelectZAndX { x_ref: None, .. }
        ));
        assert!(hist.undo(&mut proj).is_some());
        assert!(matches!(
            proj.setups[0].wcs.orientation,
            WcsOrientationMode::SelectZAndX { x_ref: Some(_), .. }
        ));
    }

    #[test]
    fn m88_heights_tab_retract_setup_clearance_and_fixtures_tab_color_reset() {
        // 1. Heights tab: Retract Datum offers Setup Clearance and resolves cleanly
        let ctx = egui::Context::default();
        let mut proj = ProjectDocument::new("heights_fixtures_test");
        proj.setups[0].heights.clearance.reference = HeightReference::StockTop;
        proj.setups[0].heights.clearance.offset = ParamLength::from_mm(12.0);
        proj.setups[0].heights.retract.reference = HeightReference::ModelTop;
        proj.setups[0].heights.retract.offset = ParamLength::from_mm(-3.0);
        let mut hist = UndoHistory::default();
        let mut tab = SetupTab::Heights;
        let mut pick_mode = ViewportPickMode::InspectEntity;

        let _ = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            None,
            Vec::new(),
        );
        let s_h = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            None,
            Vec::new(),
        );
        let ret_cb_pt = s_h
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Model Top"))
            .expect("Retract Datum combo must display Model Top");
        let _ = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            None,
            click_at(ret_cb_pt),
        );
        let s_h_open = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            None,
            Vec::new(),
        );
        let sc_pt = s_h_open
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Setup Clearance"))
            .expect("Retract Datum dropdown must include Setup Clearance");
        let _ = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            None,
            click_at(sc_pt),
        );
        assert_eq!(
            proj.setups[0].heights.retract.reference,
            HeightReference::SetupClearance
        );
        let aabb = Aabb3::new(DVec3::new(0.0, 0.0, -10.0), DVec3::new(50.0, 50.0, 0.0));
        let (clr_z, ret_z) = proj.setups[0].heights.resolve(&aabb, &aabb).unwrap();
        assert!((clr_z - 12.0).abs() < 1e-6);
        assert!((ret_z - 9.0).abs() < 1e-6);
        assert!(hist.undo(&mut proj).is_some());
        assert_eq!(
            proj.setups[0].heights.retract.reference,
            HeightReference::ModelTop
        );

        // 2. Fixtures tab: color swatch row + Reset button clears color_rgba with Undo
        let mut vise = FixtureBody::default_vise("vise_1", "Machinist Vise 1");
        vise.color_rgba = Some([0.2, 0.75, 0.35, 0.5]);
        proj.setups[0].fixtures.push(vise);
        tab = SetupTab::Fixtures;
        let _ = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            None,
            Vec::new(),
        );
        let s_f = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            None,
            Vec::new(),
        );
        let mut f_texts = Vec::new();
        collect_texts(&s_f, &mut f_texts);
        assert!(
            f_texts.iter().any(|t| t == "Color"),
            "Fixtures tab must render Color row"
        );
        let reset_pt = s_f
            .iter()
            .find_map(|cs| find_text_center(&cs.shape, "Reset"))
            .expect("Custom fixture color must show 'Reset' button");
        let _ = run_setup_frame(
            &ctx,
            &mut proj,
            &mut hist,
            &mut tab,
            &mut pick_mode,
            None,
            None,
            click_at(reset_pt),
        );
        assert_eq!(proj.setups[0].fixtures[0].color_rgba, None);
        assert!(hist.undo(&mut proj).is_some());
        assert_eq!(
            proj.setups[0].fixtures[0].color_rgba,
            Some([0.2, 0.75, 0.35, 0.5])
        );
    }
}
