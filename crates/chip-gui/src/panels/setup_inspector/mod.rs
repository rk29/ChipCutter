mod fixtures_tab;
mod heights_tab;
mod rotary_tab;
mod stock_tab;
mod wcs_tab;

use std::fmt::Debug;
use std::hash::Hash;

use chip_cad::CadDocument;
use chip_cam::{
    AxisAnchor, BoxAnchor27, FixtureBody, ResolvedFixtureSolid, ResolvedSetup, RotaryConfig, Setup,
    SetupHeights, StockConfig, WcsConfig, WorkOffsetSlot,
};
use chip_math::{EvalContext, ParamLength};
use chip_post::builtin_post_profiles;
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

/// Context passed to [`show_setup_inspector`].
pub struct SetupInspectorCtx<'a> {
    pub project: &'a mut ProjectDocument,
    pub setup_idx: usize,
    pub history: &'a mut UndoHistory,
    pub cad_doc: Option<&'a CadDocument>,
    pub resolved: Option<Result<&'a ResolvedSetup, &'a str>>,
    pub resolved_fixtures: &'a [ResolvedFixtureSolid],
    pub tab: &'a mut SetupTab,
    pub pick_mode: &'a mut ViewportPickMode,
    pub selected_fixture_idx: &'a mut Option<usize>,
    pub show_fixtures: &'a mut bool,
    pub show_27pt_dots: &'a mut bool,
    pub show_z_planes: &'a mut bool,
    pub machines: &'a [MachineDefinition],
    pub open_machine_manager: &'a mut bool,
    pub machine_changed: &'a mut bool,
    pub import_fixture_requested: &'a mut bool,
}

/// Lightweight pre-edit snapshot of only the Setup's header & WCS/Stock/Fixtures/Post/Heights
/// fields (never cloning `project.tools` or `setup.operations` on idle 60 FPS frames!).
struct SetupParamsSnapshot {
    name: String,
    wcs_slot: WorkOffsetSlot,
    post_profile: String,
    wcs: WcsConfig,
    stock: StockConfig,
    rotary: RotaryConfig,
    heights: SetupHeights,
    fixtures: Vec<FixtureBody>,
    auto_avoid_fixtures_on_rapids: bool,
}

impl SetupParamsSnapshot {
    fn capture(setup: &Setup) -> Self {
        Self {
            name: setup.name.clone(),
            wcs_slot: setup.wcs_slot,
            post_profile: setup.post_profile.clone(),
            wcs: setup.wcs.clone(),
            stock: setup.stock.clone(),
            rotary: setup.rotary.clone(),
            heights: setup.heights.clone(),
            fixtures: setup.fixtures.clone(),
            auto_avoid_fixtures_on_rapids: setup.auto_avoid_fixtures_on_rapids,
        }
    }

    fn restore_into(self, setup: &mut Setup) {
        setup.name = self.name;
        setup.wcs_slot = self.wcs_slot;
        setup.post_profile = self.post_profile;
        setup.wcs = self.wcs;
        setup.stock = self.stock;
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
        cad_doc,
        resolved,
        resolved_fixtures,
        tab,
        pick_mode,
        selected_fixture_idx,
        show_fixtures,
        show_27pt_dots,
        show_z_planes,
        machines,
        open_machine_manager,
        machine_changed,
        import_fixture_requested,
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
                    history.record_before_edit("Change Target Machine", None, project);
                    project.machine_id = Some(new_mid);
                    *machine_changed = true;
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
            let setup = &mut project.setups[setup_idx];
            match *tab {
                SetupTab::Wcs => show_wcs_tab(
                    ui,
                    WcsTabCtx {
                        setup,
                        setup_idx,
                        cad_doc,
                        eval_ctx: &eval_ctx,
                        pick_mode,
                        show_27pt_dots,
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
                        selected_fixture_idx,
                        show_fixtures,
                        pick_mode,
                        import_fixture_requested,
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
            | ViewportPickMode::SetupWcsOriginFeature { setup_idx: s }
            if s == setup_idx
    )
}

pub(super) fn render_post_body(
    ui: &mut egui::Ui,
    setup: &mut Setup,
    setup_idx: usize,
    mutated_label: &mut Option<&'static str>,
) {
    inspector_grid(ui, ("setup_post_grid", setup_idx), |ui| {
        grid_row(
            ui,
            "Post Profile",
            Some("Target CNC controller dialect (.post.toml)"),
            |ui| {
                egui::ComboBox::from_id_salt(("setup_post_cb", setup_idx))
                    .width(INPUT_COL_WIDTH + 54.0)
                    .truncate()
                    .selected_text(post_profile_short_label(&setup.post_profile))
                    .show_ui(ui, |ui| {
                        for prof in builtin_post_profiles() {
                            let lbl = format!("{} ({})", prof.name, prof.id);
                            if ui
                                .selectable_value(&mut setup.post_profile, prof.id, lbl)
                                .on_hover_text("Select this CNC controller post-processor dialect")
                                .changed()
                            {
                                *mutated_label = Some("Change Post Profile");
                            }
                        }
                    })
                    .response
                    .on_hover_text(
                        "Choose the CNC controller G-code post-processor profile for this Setup",
                    );
            },
        );
    });

    ui.add_space(4.0);
    let extended_slot = matches!(
        setup.wcs_slot,
        WorkOffsetSlot::G59_1 | WorkOffsetSlot::G59_2 | WorkOffsetSlot::G59_3
    );
    if extended_slot && setup.post_profile == "grbl" {
        inline_banner(
            ui,
            BannerKind::Warn,
            "Standard GRBL 1.1 only supports G54–G59; switch to grblHAL or LinuxCNC for G59.1–G59.3.",
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
