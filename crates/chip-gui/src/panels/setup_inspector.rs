use std::fmt::Debug;
use std::hash::Hash;

use chip_cad::CadDocument;
use chip_cam::{
    AxisAnchor, BoxAnchor27, FixedBoxXyAlign, FixedBoxZAlign, HeightReference, ResolvedSetup,
    RotaryAxisKind, RotaryConfig, RotaryDirection, RotaryDriveMode, RotaryUnwindMode, Setup,
    SetupHeights, StockAxis, StockConfig, WcsAxisRef, WcsConfig, WcsFeaturePointRef,
    WcsOrientationMode, WcsOriginMode, WorkOffsetSlot,
};
use chip_math::{DisplayUnit, EvalContext, ParamLength};
use chip_post::builtin_post_profiles;
use chip_project::{ProjectDocument, UndoHistory};
use chip_tools::MachineDefinition;

use crate::{
    panels::{SetupTab, ViewportPickMode},
    theme::ThemeTokens,
    widgets::{
        BannerKind, INPUT_COL_WIDTH, edit_param_length, grid_row, inline_banner, inspector_grid,
        section_card, static_card,
    },
};

/// Context passed to [`show_setup_inspector`].
pub struct SetupInspectorCtx<'a> {
    pub project: &'a mut ProjectDocument,
    pub setup_idx: usize,
    pub history: &'a mut UndoHistory,
    pub cad_doc: Option<&'a CadDocument>,
    pub resolved: Option<Result<&'a ResolvedSetup, &'a str>>,
    pub tab: &'a mut SetupTab,
    pub pick_mode: &'a mut ViewportPickMode,
    pub show_27pt_dots: &'a mut bool,
    pub show_z_planes: &'a mut bool,
    pub machines: &'a [MachineDefinition],
    pub open_machine_manager: &'a mut bool,
    pub machine_changed: &'a mut bool,
}

/// Lightweight (~200-byte) pre-edit snapshot of only the Setup's header & WCS/Stock/Post/Heights
/// fields (never cloning `project.tools` or `setup.operations` on idle 60 FPS frames!).
struct SetupParamsSnapshot {
    name: String,
    wcs_slot: WorkOffsetSlot,
    post_profile: String,
    wcs: WcsConfig,
    stock: StockConfig,
    rotary: RotaryConfig,
    heights: SetupHeights,
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
    }
}

fn setup_field_tip(key: &str) -> &'static str {
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

struct SetupGrid<'a> {
    ui: &'a mut egui::Ui,
    setup_idx: usize,
    eval_ctx: &'a EvalContext,
    mutated: &'a mut Option<&'static str>,
}

impl SetupGrid<'_> {
    fn len(
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

/// Renders the tabbed **Setup Inspector** (`[WCS]`, `[Stock]`, `[Heights]`).
pub fn show_setup_inspector(ui: &mut egui::Ui, ctx: SetupInspectorCtx<'_>) -> bool {
    let SetupInspectorCtx {
        project,
        setup_idx,
        history,
        cad_doc,
        resolved,
        tab,
        pick_mode,
        show_27pt_dots,
        show_z_planes,
        machines,
        open_machine_manager,
        machine_changed,
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
        ui.horizontal(|ui| {
            ui.selectable_value(tab, SetupTab::Wcs, "WCS & Post");
            ui.selectable_value(tab, SetupTab::Stock, "Stock");
            ui.selectable_value(tab, SetupTab::Rotary, "4th Axis");
            ui.selectable_value(tab, SetupTab::Heights, "Heights");
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
                static_card(ui, Some("Target CNC Machine"), |ui| {
                    inspector_grid(ui, ("setup_machine_grid", setup_idx), |ui| {
                        grid_row(
                            ui,
                            "Machine",
                            Some("Target CNC machine envelope, spindle limits, kinematics, & tool set"),
                            |ui| {
                                let label = current_machine
                                    .map_or(current_mid.as_str(), |m| m.name.as_str());
                                egui::ComboBox::from_id_salt(("setup_machine_cb", setup_idx))
                                    .width(INPUT_COL_WIDTH)
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
                                    });
                            },
                        );
                    });
                    if let Some(m) = current_machine {
                        ui.small(format!(
                            "Travel {:.0}×{:.0}×{:.0} mm  |  {:.0}–{:.0} RPM  |  {} tools  |  Post: {}",
                            m.travel_mm.x_mm,
                            m.travel_mm.y_mm,
                            m.travel_mm.z_mm,
                            m.spindle.min_rpm,
                            m.spindle.max_rpm,
                            m.tools.len(),
                            m.default_post_profile
                        ));
                    }
                    ui.horizontal(|ui| {
                        if ui
                            .small_button("Apply Machine Defaults")
                            .on_hover_text(
                                "Apply this machine's default post-processor and 4th-axis chuck dimensions to setups",
                            )
                            .clicked()
                        {
                            apply_defaults_clicked = true;
                        }
                        if ui.small_button("Manage Machines…").clicked() {
                            *open_machine_manager = true;
                        }
                    });
                });
                ui.add_space(5.0);

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

fn render_work_offset_row(
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
                        if ui.selectable_value(wcs_slot, slot, slot.label()).changed() {
                            *mutated_label = Some("Change Work Offset Slot");
                        }
                    }
                });
        },
    );
}

fn is_setup_wcs_pick_mode(mode: ViewportPickMode, setup_idx: usize) -> bool {
    matches!(
        mode,
        ViewportPickMode::SetupWcsZ { setup_idx: s }
            | ViewportPickMode::SetupWcsX { setup_idx: s }
            | ViewportPickMode::SetupWcsXyPlane { setup_idx: s }
            | ViewportPickMode::SetupWcsOriginFeature { setup_idx: s }
            if s == setup_idx
    )
}

struct WcsTabCtx<'a> {
    setup: &'a mut Setup,
    setup_idx: usize,
    cad_doc: Option<&'a CadDocument>,
    eval_ctx: &'a EvalContext,
    pick_mode: &'a mut ViewportPickMode,
    show_27pt_dots: &'a mut bool,
    mutated_label: &'a mut Option<&'static str>,
}

fn show_wcs_tab(ui: &mut egui::Ui, mut ctx: WcsTabCtx<'_>) {
    let idx = ctx.setup_idx;
    section_card(
        ui,
        ("wcs_orient_card", idx),
        "Work Offset & Orientation",
        true,
        |ui| {
            render_wcs_orient_body(ui, &mut ctx);
        },
    );
    section_card(
        ui,
        ("wcs_origin_card", idx),
        "WCS Origin (X0 Y0 Z0)",
        true,
        |ui| {
            render_wcs_origin_body(ui, &mut ctx);
        },
    );
    section_card(
        ui,
        ("setup_post_card", idx),
        "Post-Processor & Output",
        true,
        |ui| {
            render_post_body(ui, ctx.setup, idx, ctx.mutated_label);
        },
    );
}

fn render_wcs_orient_body(ui: &mut egui::Ui, ctx: &mut WcsTabCtx<'_>) {
    let setup_idx = ctx.setup_idx;
    let tok = ThemeTokens::from_ui(ui);
    let is_model = matches!(
        ctx.setup.wcs.orientation,
        WcsOrientationMode::ModelOrientation { .. }
    );
    let is_zx = matches!(
        ctx.setup.wcs.orientation,
        WcsOrientationMode::SelectZAndX { .. }
    );
    let is_xy = matches!(
        ctx.setup.wcs.orientation,
        WcsOrientationMode::SelectXYPlane { .. }
    );

    inspector_grid(ui, ("wcs_orient_grid", setup_idx), |ui| {
        render_work_offset_row(
            ui,
            ("wcs_slot_cb", setup_idx),
            &mut ctx.setup.wcs_slot,
            ctx.mutated_label,
        );

        grid_row(
            ui,
            "Orientation Mode",
            Some("Align spindle +Z and table +X to CAD geometry"),
            |ui| {
                if ui.selectable_label(is_model, "Model XYZ").clicked() && !is_model {
                    ctx.setup.wcs.orientation = WcsOrientationMode::default();
                    *ctx.pick_mode = ViewportPickMode::InspectEntity;
                    *ctx.mutated_label = Some("Reset WCS to Model Orientation");
                }
                let has_cad = ctx.cad_doc.is_some();
                if ui
                    .add_enabled_ui(has_cad, |ui| ui.selectable_label(is_zx, "Z & X"))
                    .inner
                    .clicked()
                    && !is_zx
                    && let Some(bind) = ctx
                        .cad_doc
                        .and_then(|d| d.faces.first().and_then(|f| d.bind_face(f.id)))
                {
                    ctx.setup.wcs.orientation = WcsOrientationMode::SelectZAndX {
                        z_ref: WcsAxisRef::FaceAxis(bind),
                        flip_z: false,
                        x_ref: None,
                        flip_x: false,
                        clock_90: 0,
                        extra_z_rot_deg: 0.0,
                    };
                    *ctx.pick_mode = ViewportPickMode::SetupWcsZ { setup_idx };
                    *ctx.mutated_label = Some("Switch WCS to Select Z & X");
                }
                if ui
                    .add_enabled_ui(has_cad, |ui| ui.selectable_label(is_xy, "XY Plane"))
                    .inner
                    .clicked()
                    && !is_xy
                    && let Some(bind) = ctx
                        .cad_doc
                        .and_then(|d| d.faces.first().and_then(|f| d.bind_face(f.id)))
                {
                    ctx.setup.wcs.orientation = WcsOrientationMode::SelectXYPlane {
                        face_ref: bind,
                        flip_z: false,
                        x_edge_ref: None,
                        clock_90: 0,
                        extra_z_rot_deg: 0.0,
                    };
                    *ctx.pick_mode = ViewportPickMode::SetupWcsXyPlane { setup_idx };
                    *ctx.mutated_label = Some("Switch WCS to Select XY Plane");
                }
            },
        );

        let extra_z_rot = match &mut ctx.setup.wcs.orientation {
            WcsOrientationMode::ModelOrientation {
                flip_z,
                flip_x,
                clock_90,
                extra_z_rot_deg,
            } => {
                grid_row(
                    ui,
                    "Axis Controls",
                    Some("Invert +Z/+X or rotate +X by 90°"),
                    |ui| {
                        if ui.checkbox(flip_z, "Flip +Z").changed() {
                            *ctx.mutated_label = Some("Flip WCS Z Axis");
                        }
                        if ui.checkbox(flip_x, "Flip +X").changed() {
                            *ctx.mutated_label = Some("Flip WCS X Axis");
                        }
                        if ui.small_button("Clock X 90°").clicked() {
                            *clock_90 = (*clock_90 + 1) % 4;
                            *ctx.mutated_label = Some("Clock WCS X Axis 90°");
                        }
                        if *clock_90 > 0 {
                            ui.colored_label(
                                tok.text_muted,
                                format!("{}°", u16::from(*clock_90) * 90),
                            );
                        }
                    },
                );
                extra_z_rot_deg
            }
            WcsOrientationMode::SelectZAndX {
                z_ref,
                flip_z,
                x_ref,
                flip_x,
                clock_90,
                extra_z_rot_deg,
            } => {
                let z_desc = describe_axis_ref(Some(z_ref));
                render_axis_pick_spec_row(
                    ui,
                    AxisPickSpec {
                        label: "+Z Axis",
                        desc: &z_desc,
                        pick_btn: "Pick Z",
                        target_mode: ViewportPickMode::SetupWcsZ { setup_idx },
                        flip_lbl: "Flip Z",
                        flip_undo: "Flip WCS Z Axis",
                    },
                    ctx.pick_mode,
                    flip_z,
                    ctx.mutated_label,
                );
                let x_desc = describe_axis_ref(x_ref.as_ref());
                render_axis_pick_spec_row(
                    ui,
                    AxisPickSpec {
                        label: "+X Axis",
                        desc: &x_desc,
                        pick_btn: "Pick X",
                        target_mode: ViewportPickMode::SetupWcsX { setup_idx },
                        flip_lbl: "Flip X",
                        flip_undo: "Flip WCS X Axis",
                    },
                    ctx.pick_mode,
                    flip_x,
                    ctx.mutated_label,
                );
                grid_row(
                    ui,
                    "X Clocking",
                    Some("Rotate +X in 90° steps around +Z"),
                    |ui| {
                        if ui.small_button("Clock X 90°").clicked() {
                            *clock_90 = (*clock_90 + 1) % 4;
                            *ctx.mutated_label = Some("Clock WCS X Axis 90°");
                        }
                        ui.colored_label(tok.text_muted, format!("{}°", u16::from(*clock_90) * 90));
                    },
                );
                extra_z_rot_deg
            }
            WcsOrientationMode::SelectXYPlane {
                face_ref,
                flip_z,
                clock_90,
                extra_z_rot_deg,
                ..
            } => {
                let target = ViewportPickMode::SetupWcsXyPlane { setup_idx };
                grid_row(
                    ui,
                    "XY Plane Face",
                    Some("Planar B-Rep face defining Setup XY"),
                    |ui| {
                        ui.label(format!("Face #{}", face_ref.last_index));
                        let on = *ctx.pick_mode == target;
                        if ui.selectable_label(on, "Pick Face").clicked() {
                            *ctx.pick_mode = if on {
                                ViewportPickMode::InspectEntity
                            } else {
                                target
                            };
                        }
                    },
                );
                grid_row(
                    ui,
                    "Axis Controls",
                    Some("Flip +Z normal or rotate +X by 90°"),
                    |ui| {
                        if ui.checkbox(flip_z, "Flip +Z").changed() {
                            *ctx.mutated_label = Some("Flip WCS Z Axis");
                        }
                        if ui.small_button("Clock 90°").clicked() {
                            *clock_90 = (*clock_90 + 1) % 4;
                            *ctx.mutated_label = Some("Clock WCS X Axis 90°");
                        }
                        if *clock_90 > 0 {
                            ui.colored_label(
                                tok.text_muted,
                                format!("{}°", u16::from(*clock_90) * 90),
                            );
                        }
                    },
                );
                extra_z_rot_deg
            }
        };

        grid_row(
            ui,
            "Fine Z Rot (°)",
            Some("Additional arbitrary rotation around Setup +Z (e.g., 15° or 45° vise clocking)"),
            |ui| {
                let resp = ui.add(
                    egui::DragValue::new(extra_z_rot)
                        .speed(0.5)
                        .range(-360.0..=360.0)
                        .suffix("°")
                        .max_decimals(2),
                );
                if resp.changed() {
                    if !extra_z_rot.is_finite() {
                        *extra_z_rot = 0.0;
                    }
                    *ctx.mutated_label = Some("Adjust WCS Fine Z Rotation");
                }
                if extra_z_rot.abs() > 1e-6 && ui.small_button("0°").clicked() {
                    *extra_z_rot = 0.0;
                    *ctx.mutated_label = Some("Reset WCS Fine Z Rotation");
                }
            },
        );
    });
}

struct AxisPickSpec<'a> {
    label: &'static str,
    desc: &'a str,
    pick_btn: &'static str,
    target_mode: ViewportPickMode,
    flip_lbl: &'static str,
    flip_undo: &'static str,
}

fn render_axis_pick_spec_row(
    ui: &mut egui::Ui,
    spec: AxisPickSpec<'_>,
    pick_mode: &mut ViewportPickMode,
    flip: &mut bool,
    mutated: &mut Option<&'static str>,
) {
    grid_row(
        ui,
        spec.label,
        Some("Reference B-Rep face normal or straight edge"),
        |ui| {
            ui.label(spec.desc);
            let on = *pick_mode == spec.target_mode;
            if ui.selectable_label(on, spec.pick_btn).clicked() {
                *pick_mode = if on {
                    ViewportPickMode::InspectEntity
                } else {
                    spec.target_mode
                };
            }
            if ui.checkbox(flip, spec.flip_lbl).changed() {
                *mutated = Some(spec.flip_undo);
            }
        },
    );
}

fn describe_axis_ref(axis_ref: Option<&WcsAxisRef>) -> String {
    match axis_ref {
        Some(WcsAxisRef::FaceAxis(b)) => format!("Face #{}", b.last_index),
        Some(WcsAxisRef::EdgeDirection(b)) => format!("Edge #{}", b.last_index),
        None => "Auto".to_string(),
    }
}

fn render_wcs_origin_body(ui: &mut egui::Ui, ctx: &mut WcsTabCtx<'_>) {
    let setup_idx = ctx.setup_idx;
    let is_stock_pt = matches!(ctx.setup.wcs.origin, WcsOriginMode::StockBoxPoint { .. });
    let is_model_pt = matches!(ctx.setup.wcs.origin, WcsOriginMode::ModelBoxPoint { .. });
    let is_cad_pt = matches!(
        ctx.setup.wcs.origin,
        WcsOriginMode::SelectedCadFeature { .. }
    );

    inspector_grid(ui, ("wcs_origin_top_grid", setup_idx), |ui| {
        grid_row(
            ui,
            "Origin Reference",
            Some("Datum probed for X0 Y0 Z0 on the machine"),
            |ui| {
                if ui.selectable_label(is_stock_pt, "Stock 27-Pt").clicked() && !is_stock_pt {
                    ctx.setup.wcs.origin = WcsOriginMode::StockBoxPoint {
                        anchor: BoxAnchor27::TOP_FRONT_LEFT,
                    };
                    *ctx.mutated_label = Some("Set WCS Origin to Stock Box");
                }
                if ui.selectable_label(is_model_pt, "Model 27-Pt").clicked() && !is_model_pt {
                    ctx.setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
                        anchor: BoxAnchor27::TOP_FRONT_LEFT,
                    };
                    *ctx.mutated_label = Some("Set WCS Origin to Model Box");
                }
                let has_cad = ctx.cad_doc.is_some();
                if ui
                    .add_enabled_ui(has_cad, |ui| ui.selectable_label(is_cad_pt, "CAD Feature"))
                    .inner
                    .clicked()
                    && !is_cad_pt
                    && let Some(doc) = ctx.cad_doc
                {
                    select_default_cad_origin_feature(doc, &mut ctx.setup.wcs.origin);
                    *ctx.pick_mode = ViewportPickMode::SetupWcsOriginFeature { setup_idx };
                    *ctx.mutated_label = Some("Set WCS Origin to CAD Feature");
                }
            },
        );

        match &mut ctx.setup.wcs.origin {
            WcsOriginMode::StockBoxPoint { anchor } | WcsOriginMode::ModelBoxPoint { anchor } => {
                grid_row(
                    ui,
                    "Active Anchor",
                    Some("Selected 27-point bounding-box datum"),
                    |ui| {
                        ui.strong(anchor.label());
                    },
                );
                grid_row(
                    ui,
                    "3D Viewport Dots",
                    Some("Show clickable 27-point origin dots"),
                    |ui| {
                        ui.checkbox(ctx.show_27pt_dots, "Interactive 27-pt overlay");
                    },
                );
            }
            WcsOriginMode::SelectedCadFeature { feature } => {
                let target = ViewportPickMode::SetupWcsOriginFeature { setup_idx };
                let desc = describe_origin_feature(feature);
                grid_row(
                    ui,
                    "CAD Anchor",
                    Some("Associative B-Rep hole or edge datum"),
                    |ui| {
                        ui.label(desc);
                    },
                );
                grid_row(
                    ui,
                    "Feature Pick",
                    Some("Click a hole wall or edge in 3D viewport"),
                    |ui| {
                        let on = *ctx.pick_mode == target;
                        if ui.selectable_label(on, "Pick CAD Feature").clicked() {
                            *ctx.pick_mode = if on {
                                ViewportPickMode::InspectEntity
                            } else {
                                target
                            };
                        }
                        if let WcsFeaturePointRef::HoleAxisCenter { top_rim, .. } = feature
                            && ui.checkbox(top_rim, "Top Rim").changed()
                        {
                            *ctx.mutated_label = Some("Toggle Bore Top/Bottom Origin");
                        }
                    },
                );
            }
        }
    });

    if let WcsOriginMode::StockBoxPoint { anchor } | WcsOriginMode::ModelBoxPoint { anchor } =
        &mut ctx.setup.wcs.origin
    {
        ui.add_space(4.0);
        if show_27pt_grid_selector(ui, anchor) {
            *ctx.mutated_label = Some("Change 27-Point WCS Origin");
        }
    }

    ui.add_space(4.0);
    ui.separator();
    ui.add_space(3.0);

    inspector_grid(ui, ("wcs_origin_offset_grid", setup_idx), |ui| {
        let mut g = SetupGrid {
            ui,
            setup_idx,
            eval_ctx: ctx.eval_ctx,
            mutated: ctx.mutated_label,
        };
        g.len(
            "ΔX Offset",
            "wcs_off_x",
            &mut ctx.setup.wcs.additional_offset_wcs[0],
            "Edit WCS Origin Offset",
        );
        g.len(
            "ΔY Offset",
            "wcs_off_y",
            &mut ctx.setup.wcs.additional_offset_wcs[1],
            "Edit WCS Origin Offset",
        );
        g.len(
            "ΔZ Offset",
            "wcs_off_z",
            &mut ctx.setup.wcs.additional_offset_wcs[2],
            "Edit WCS Origin Offset",
        );
    });
}

fn select_default_cad_origin_feature(doc: &CadDocument, origin: &mut WcsOriginMode) {
    if let Some(hole) = doc.faces.iter().find(|f| {
        matches!(
            f.surface,
            chip_cad::SurfaceKind::Cylinder {
                is_internal: true,
                ..
            }
        )
    }) && let Some(b) = doc.bind_face(hole.id)
    {
        *origin = WcsOriginMode::SelectedCadFeature {
            feature: WcsFeaturePointRef::HoleAxisCenter {
                hole_face: b,
                top_rim: true,
            },
        };
    } else if let Some(edge) = doc.edges.first()
        && let Some(b) = doc.bind_edge(edge.id)
    {
        *origin = WcsOriginMode::SelectedCadFeature {
            feature: WcsFeaturePointRef::EdgeMidpoint(b),
        };
    }
}

fn describe_origin_feature(feature: &WcsFeaturePointRef) -> String {
    match feature {
        WcsFeaturePointRef::HoleAxisCenter { hole_face, top_rim } => format!(
            "Bore #{} ({})",
            hole_face.last_index,
            if *top_rim {
                "Top Center"
            } else {
                "Bottom Center"
            }
        ),
        WcsFeaturePointRef::CircleCenter(b) => format!("Circle Edge #{} Center", b.last_index),
        WcsFeaturePointRef::EdgeStart(b) => format!("Edge #{} Start", b.last_index),
        WcsFeaturePointRef::EdgeMidpoint(b) => format!("Edge #{} Midpoint", b.last_index),
        WcsFeaturePointRef::EdgeEnd(b) => format!("Edge #{} End", b.last_index),
    }
}

fn render_post_body(
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
                    .width(INPUT_COL_WIDTH + 44.0)
                    .selected_text(post_profile_short_label(&setup.post_profile))
                    .show_ui(ui, |ui| {
                        for prof in builtin_post_profiles() {
                            let lbl = format!("{} ({})", prof.name, prof.id);
                            if ui
                                .selectable_value(&mut setup.post_profile, prof.id, lbl)
                                .changed()
                            {
                                *mutated_label = Some("Change Post Profile");
                            }
                        }
                    });
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

#[derive(Clone, Copy)]
struct StockTabView<'a> {
    setup_idx: usize,
    is_pure_2d: bool,
    eval_ctx: &'a EvalContext,
    display_unit: DisplayUnit,
    resolved: Option<Result<&'a ResolvedSetup, &'a str>>,
}

fn show_stock_tab(
    ui: &mut egui::Ui,
    setup: &mut Setup,
    view: StockTabView<'_>,
    mutated_label: &mut Option<&'static str>,
) {
    let idx = view.setup_idx;
    section_card(
        ui,
        ("stock_geom_card", idx),
        "Stock Geometry & Dimensions",
        true,
        |ui| {
            render_stock_geom_body(ui, setup, view, mutated_label);
        },
    );
    section_card(
        ui,
        ("stock_offsets_card", idx),
        "Stock Offsets",
        true,
        |ui| {
            render_stock_offsets_body(ui, setup, view, mutated_label);
        },
    );
}

fn render_stock_geom_body(
    ui: &mut egui::Ui,
    setup: &mut Setup,
    view: StockTabView<'_>,
    mutated_label: &mut Option<&'static str>,
) {
    let StockTabView {
        setup_idx,
        is_pure_2d,
        eval_ctx,
        display_unit,
        resolved,
    } = view;

    inspector_grid(ui, ("stock_geom_grid", setup_idx), |ui| {
        grid_row(
            ui,
            "Stock Mode",
            Some("Raw stock blank shape and sizing mode"),
            |ui| {
                render_stock_mode_combo(ui, setup_idx, &mut setup.stock, mutated_label);
            },
        );

        let mut g = SetupGrid {
            ui,
            setup_idx,
            eval_ctx,
            mutated: mutated_label,
        };
        match &mut setup.stock {
            StockConfig::RelativeBox {
                sheet_thickness_z,
                round_increment_mm,
                ..
            } => {
                if is_pure_2d {
                    g.len(
                        "Sheet Thickness (Z)",
                        "stk_sheet_z",
                        sheet_thickness_z,
                        "Edit Pure-2D Sheet Thickness",
                    );
                }
                let mut round_on = round_increment_mm.is_some();
                if grid_row(
                    g.ui,
                    "Round Saw Blank",
                    Some("Round total stock up to 5 mm step"),
                    |ui| ui.checkbox(&mut round_on, "Up to 5 mm step").changed(),
                ) {
                    *round_increment_mm = round_on.then_some(5.0);
                    *g.mutated = Some("Toggle Stock Rounding");
                }
            }
            StockConfig::FixedBox {
                width_x,
                depth_y,
                height_z,
                ..
            } => {
                g.len("Width (WCS X)", "fbox_x", width_x, "Edit Fixed Stock Width");
                g.len("Depth (WCS Y)", "fbox_y", depth_y, "Edit Fixed Stock Depth");
                g.len(
                    "Height (WCS Z)",
                    "fbox_z",
                    height_z,
                    "Edit Fixed Stock Height",
                );
            }
            StockConfig::RelativeCylinder { axis, .. } => {
                render_cyl_axis_row(g.ui, ("rcyl_axis", setup_idx), axis, g.mutated);
            }
            StockConfig::FixedCylinder {
                axis,
                diameter,
                length,
                ..
            } => {
                render_cyl_axis_row(g.ui, ("fcyl_axis", setup_idx), axis, g.mutated);
                g.len("Bar Diameter", "fcyl_d", diameter, "Edit Cylinder Diameter");
                g.len("Bar Length", "fcyl_l", length, "Edit Cylinder Length");
            }
        }
    });

    if let Some(Ok(rs)) = resolved {
        let s = rs.stock.aabb_wcs().size();
        let m = rs.model_aabb_wcs.size();
        let fmt = |v| display_unit.format_length(v);
        ui.add_space(5.0);
        inline_banner(
            ui,
            BannerKind::Info,
            format!(
                "Model: {} × {} × {}\nStock: {} × {} × {}",
                fmt(m.x),
                fmt(m.y),
                fmt(m.z),
                fmt(s.x),
                fmt(s.y),
                fmt(s.z)
            ),
        );
    }
}

fn render_stock_mode_combo(
    ui: &mut egui::Ui,
    setup_idx: usize,
    stock: &mut StockConfig,
    mutated_label: &mut Option<&'static str>,
) {
    let mm = ParamLength::from_mm;
    egui::ComboBox::from_id_salt(("stock_mode_cb", setup_idx))
        .width(INPUT_COL_WIDTH + 44.0)
        .selected_text(stock.kind_label())
        .show_ui(ui, |ui| {
            let is_rbox = matches!(stock, StockConfig::RelativeBox { .. });
            if ui.selectable_label(is_rbox, "Relative Box Stock").clicked() && !is_rbox {
                *stock = StockConfig::default();
                *mutated_label = Some("Change Stock Mode to Relative Box");
            }
            let is_fbox = matches!(stock, StockConfig::FixedBox { .. });
            if ui
                .selectable_label(is_fbox, "Fixed-Size Box Stock")
                .clicked()
                && !is_fbox
            {
                *stock = StockConfig::FixedBox {
                    width_x: mm(100.0),
                    depth_y: mm(80.0),
                    height_z: mm(25.0),
                    xy_align: FixedBoxXyAlign::CenterXy,
                    z_align: FixedBoxZAlign::default(),
                };
                *mutated_label = Some("Change Stock Mode to Fixed Box");
            }
            let is_rcyl = matches!(stock, StockConfig::RelativeCylinder { .. });
            if ui
                .selectable_label(is_rcyl, "Relative Cylindrical Stock")
                .clicked()
                && !is_rcyl
            {
                *stock = StockConfig::RelativeCylinder {
                    axis: StockAxis::Z,
                    radial_offset: mm(3.0),
                    top_offset: mm(1.0),
                    bottom_offset: mm(5.0),
                };
                *mutated_label = Some("Change Stock Mode to Relative Cylinder");
            }
            let is_fcyl = matches!(stock, StockConfig::FixedCylinder { .. });
            if ui
                .selectable_label(is_fcyl, "Fixed Cylindrical Stock")
                .clicked()
                && !is_fcyl
            {
                *stock = StockConfig::FixedCylinder {
                    axis: StockAxis::Z,
                    diameter: mm(130.0),
                    length: mm(30.0),
                    axial_top_offset: mm(1.0),
                };
                *mutated_label = Some("Change Stock Mode to Fixed Cylinder");
            }
        });
}

fn render_cyl_axis_row(
    ui: &mut egui::Ui,
    id_salt: impl Hash + Debug,
    axis: &mut StockAxis,
    mutated_label: &mut Option<&'static str>,
) {
    grid_row(
        ui,
        "Bar Axis",
        Some("Principal WCS axis of round bar stock"),
        |ui| {
            egui::ComboBox::from_id_salt(id_salt)
                .width(INPUT_COL_WIDTH + 44.0)
                .selected_text(axis.label())
                .show_ui(ui, |ui| {
                    for a in StockAxis::ALL {
                        if ui.selectable_value(axis, a, a.label()).changed() {
                            *mutated_label = Some("Change Cylinder Stock Axis");
                        }
                    }
                });
        },
    );
}

fn render_stock_offsets_body(
    ui: &mut egui::Ui,
    setup: &mut Setup,
    view: StockTabView<'_>,
    mutated_label: &mut Option<&'static str>,
) {
    let setup_idx = view.setup_idx;
    inspector_grid(ui, ("stock_offsets_grid", setup_idx), |ui| {
        let mut g = SetupGrid {
            ui,
            setup_idx,
            eval_ctx: view.eval_ctx,
            mutated: mutated_label,
        };
        match &mut setup.stock {
            StockConfig::RelativeBox {
                side_pos_x,
                side_neg_x,
                side_pos_y,
                side_neg_y,
                symmetric_xy,
                top_offset_z,
                bottom_offset_z,
                ..
            } => {
                if grid_row(
                    g.ui,
                    "Symmetric XY",
                    Some("Apply identical side stock on ±X and ±Y"),
                    |ui| ui.checkbox(symmetric_xy, "Mirror ±X / ±Y").changed(),
                ) {
                    if *symmetric_xy {
                        *side_neg_x = side_pos_x.clone();
                        *side_neg_y = side_pos_y.clone();
                    }
                    *g.mutated = Some("Toggle Symmetric Stock Margins");
                }
                let px_lbl = if *symmetric_xy {
                    "Side X Margin (±X)"
                } else {
                    "Side +X Margin"
                };
                if g.len(px_lbl, "stk_px", side_pos_x, "Edit Stock Side Margin") && *symmetric_xy {
                    *side_neg_x = side_pos_x.clone();
                }
                if !*symmetric_xy {
                    g.len(
                        "Side -X Margin",
                        "stk_nx",
                        side_neg_x,
                        "Edit Stock -X Margin",
                    );
                }
                let py_lbl = if *symmetric_xy {
                    "Side Y Margin (±Y)"
                } else {
                    "Side +Y Margin"
                };
                if g.len(py_lbl, "stk_py", side_pos_y, "Edit Stock Y Margin") && *symmetric_xy {
                    *side_neg_y = side_pos_y.clone();
                }
                if !*symmetric_xy {
                    g.len(
                        "Side -Y Margin",
                        "stk_ny",
                        side_neg_y,
                        "Edit Stock -Y Margin",
                    );
                }
                g.len(
                    "Top Allowance (+Z)",
                    "stk_tz",
                    top_offset_z,
                    "Edit Stock Top Offset",
                );
                g.len(
                    "Bottom Holding (-Z)",
                    "stk_bz",
                    bottom_offset_z,
                    "Edit Stock Bottom Offset",
                );
            }
            StockConfig::FixedBox {
                xy_align, z_align, ..
            } => {
                match xy_align {
                    FixedBoxXyAlign::CenterXy => {
                        grid_row(
                            g.ui,
                            "XY Alignment",
                            Some("Part XY placement inside fixed stock"),
                            |ui| {
                                ui.label("Centered in XY");
                            },
                        );
                    }
                    FixedBoxXyAlign::OffsetFromFrontLeft { dx, dy } => {
                        g.len("Front-Left ΔX", "fbox_dx", dx, "Edit Fixed Stock XY Offset");
                        g.len("Front-Left ΔY", "fbox_dy", dy, "Edit Fixed Stock XY Offset");
                    }
                }
                match z_align {
                    FixedBoxZAlign::OffsetFromTop { top_allowance } => {
                        g.len(
                            "Top Allowance (+Z)",
                            "fbox_ta",
                            top_allowance,
                            "Edit Fixed Stock Top Allowance",
                        );
                    }
                    FixedBoxZAlign::OffsetFromBottom { bottom_allowance } => {
                        g.len(
                            "Bottom Allowance (-Z)",
                            "fbox_ba",
                            bottom_allowance,
                            "Edit Fixed Stock Bottom Allowance",
                        );
                    }
                    FixedBoxZAlign::CenterZ => {
                        grid_row(g.ui, "Z Alignment", None, |ui| {
                            ui.label("Centered in Z");
                        });
                    }
                }
            }
            StockConfig::RelativeCylinder {
                radial_offset,
                top_offset,
                bottom_offset,
                ..
            } => {
                g.len(
                    "Radial Margin",
                    "rcyl_r",
                    radial_offset,
                    "Edit Cylinder Radial Offset",
                );
                g.len(
                    "Top Margin (+Z)",
                    "rcyl_t",
                    top_offset,
                    "Edit Cylinder Top Offset",
                );
                g.len(
                    "Bottom Chuck (-Z)",
                    "rcyl_b",
                    bottom_offset,
                    "Edit Cylinder Bottom Offset",
                );
            }
            StockConfig::FixedCylinder {
                axial_top_offset, ..
            } => {
                g.len(
                    "Top Facing Offset",
                    "fcyl_to",
                    axial_top_offset,
                    "Edit Cylinder Top Offset",
                );
            }
        }
    });
}

struct HeightsTabCtx<'a> {
    setup: &'a mut Setup,
    setup_idx: usize,
    eval_ctx: &'a EvalContext,
    display_unit: DisplayUnit,
    resolved: Option<Result<&'a ResolvedSetup, &'a str>>,
    show_z_planes: &'a mut bool,
    mutated_label: &'a mut Option<&'static str>,
}

fn show_heights_tab(ui: &mut egui::Ui, ctx: HeightsTabCtx<'_>) {
    let HeightsTabCtx {
        setup,
        setup_idx,
        eval_ctx,
        display_unit,
        resolved,
        show_z_planes,
        mutated_label,
    } = ctx;

    section_card(
        ui,
        ("setup_safe_z_card", setup_idx),
        "Safe Z Clearance & Retract",
        true,
        |ui| {
            inspector_grid(ui, ("setup_heights_grid", setup_idx), |ui| {
                grid_row(
                    ui,
                    "3D Z Planes",
                    Some("Render translucent Z planes in 3D viewport"),
                    |ui| {
                        ui.checkbox(show_z_planes, "Preview in viewport");
                    },
                );

                render_setup_height_ref_row(
                    ui,
                    "Clearance Datum",
                    ("setup_clr_ref", setup_idx),
                    &mut setup.heights.clearance.reference,
                    "Change Setup Clearance Reference",
                    mutated_label,
                );
                let mut g = SetupGrid {
                    ui,
                    setup_idx,
                    eval_ctx,
                    mutated: mutated_label,
                };
                g.len(
                    "Clearance Offset",
                    "setup_clr_off",
                    &mut setup.heights.clearance.offset,
                    "Edit Setup Clearance Offset",
                );
                render_setup_height_ref_row(
                    g.ui,
                    "Retract Datum",
                    ("setup_ret_ref", setup_idx),
                    &mut setup.heights.retract.reference,
                    "Change Setup Retract Reference",
                    g.mutated,
                );
                g.len(
                    "Retract Offset",
                    "setup_ret_off",
                    &mut setup.heights.retract.offset,
                    "Edit Setup Retract Offset",
                );
            });

            if let Some(Ok(rs)) = resolved {
                let fmt = |v| display_unit.format_length(v);
                ui.add_space(5.0);
                inline_banner(
                    ui,
                    BannerKind::Info,
                    format!(
                        "Clearance Z: {}  ·  Retract Z: {}\nStock Top: {}  ·  Model Top: {}  ·  Stock Bot: {}",
                        fmt(rs.clearance_z_wcs),
                        fmt(rs.retract_z_wcs),
                        fmt(rs.stock.aabb_wcs().max.z),
                        fmt(rs.model_aabb_wcs.max.z),
                        fmt(rs.stock.aabb_wcs().min.z),
                    ),
                );
            }
        },
    );
}

fn render_setup_height_ref_row(
    ui: &mut egui::Ui,
    label: &str,
    salt: (&str, usize),
    val: &mut HeightReference,
    undo: &'static str,
    mutated: &mut Option<&'static str>,
) {
    grid_row(
        ui,
        label,
        Some("Reference datum plane for Setup Z rapid moves"),
        |ui| {
            egui::ComboBox::from_id_salt(salt)
                .width(INPUT_COL_WIDTH + 30.0)
                .selected_text(val.label())
                .show_ui(ui, |ui| {
                    for r in HeightReference::SETUP_REFS {
                        if ui.selectable_value(val, r, r.label()).changed() {
                            *mutated = Some(undo);
                        }
                    }
                });
        },
    );
}

fn show_27pt_grid_selector(ui: &mut egui::Ui, anchor: &mut BoxAnchor27) -> bool {
    let mut changed = false;

    ui.horizontal(|ui| {
        ui.weak("Z Level:");
        for (lbl, z_a) in [
            ("Top", AxisAnchor::Max),
            ("Mid-Z", AxisAnchor::Center),
            ("Bottom", AxisAnchor::Min),
        ] {
            if ui.selectable_label(anchor.z == z_a, lbl).clicked() && anchor.z != z_a {
                anchor.z = z_a;
                changed = true;
            }
        }
    });

    egui::Grid::new("box_anchor_3x3_xy")
        .spacing([4.0, 4.0])
        .show(ui, |ui| {
            for (y_lbl, y_a) in [
                ("Back (+Y)", AxisAnchor::Max),
                ("Center Y", AxisAnchor::Center),
                ("Front (-Y)", AxisAnchor::Min),
            ] {
                ui.weak(y_lbl);
                for (x_short, x_a) in [
                    ("L", AxisAnchor::Min),
                    ("C", AxisAnchor::Center),
                    ("R", AxisAnchor::Max),
                ] {
                    let is_sel = anchor.x == x_a && anchor.y == y_a;
                    if ui
                        .selectable_label(is_sel, format!(" {x_short} "))
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

fn show_rotary_tab(
    ui: &mut egui::Ui,
    setup: &mut Setup,
    setup_idx: usize,
    eval_ctx: &EvalContext,
    display_unit: DisplayUnit,
    resolved: Option<Result<&ResolvedSetup, &str>>,
    mutated_label: &mut Option<&'static str>,
) {
    section_card(
        ui,
        ("setup_rotary_kinematics", setup_idx),
        "4th-Axis Rotary Kinematics",
        true,
        |ui| {
            inspector_grid(ui, ("setup_rotary_grid", setup_idx), |ui| {
                grid_row(
                    ui,
                    "Rotary Mode",
                    Some(
                        "Select 3-axis (disabled), 2-linear + rotary (Z+X/Z+Y), indexed 3+1, or full simultaneous 4-axis",
                    ),
                    |ui| {
                        let prev = setup.rotary.mode;
                        egui::ComboBox::from_id_salt(("rot_mode_cb", setup_idx))
                            .width(INPUT_COL_WIDTH)
                            .selected_text(setup.rotary.mode.label())
                            .show_ui(ui, |ui| {
                                for m in RotaryDriveMode::ALL {
                                    ui.selectable_value(&mut setup.rotary.mode, m, m.label());
                                }
                            });
                        if setup.rotary.mode != prev {
                            *mutated_label = Some("Change Rotary Drive Mode");
                        }
                    },
                );

                if setup.rotary.is_enabled() {
                    grid_row(
                        ui,
                        "Rotary Axis",
                        Some("Principal Setup WCS axis aligned with the rotary chuck centerline"),
                        |ui| {
                            let prev = setup.rotary.axis;
                            egui::ComboBox::from_id_salt(("rot_axis_cb", setup_idx))
                                .width(INPUT_COL_WIDTH)
                                .selected_text(setup.rotary.axis.label())
                                .show_ui(ui, |ui| {
                                    for a in RotaryAxisKind::ALL {
                                        ui.selectable_value(&mut setup.rotary.axis, a, a.label());
                                    }
                                });
                            if setup.rotary.axis != prev {
                                *mutated_label = Some("Change Rotary Axis");
                            }
                        },
                    );

                    grid_row(
                        ui,
                        "Direction",
                        Some("Positive rotation polarity around the rotary axis"),
                        |ui| {
                            let prev = setup.rotary.direction;
                            egui::ComboBox::from_id_salt(("rot_dir_cb", setup_idx))
                                .width(INPUT_COL_WIDTH)
                                .selected_text(setup.rotary.direction.label())
                                .show_ui(ui, |ui| {
                                    for d in RotaryDirection::ALL {
                                        ui.selectable_value(
                                            &mut setup.rotary.direction,
                                            d,
                                            d.label(),
                                        );
                                    }
                                });
                            if setup.rotary.direction != prev {
                                *mutated_label = Some("Change Rotary Polarity");
                            }
                        },
                    );

                    grid_row(
                        ui,
                        "G-Code Letter",
                        Some("Rotary axis address word emitted by 4-axis post-processors"),
                        |ui| {
                            let cur_letter = setup.rotary.effective_axis_letter();
                            let prev = setup.rotary.axis_letter_override;
                            egui::ComboBox::from_id_salt(("rot_letter_cb", setup_idx))
                                .width(92.0)
                                .selected_text(format!("{cur_letter}-Axis"))
                                .show_ui(ui, |ui| {
                                    let def = setup.rotary.axis.default_letter();
                                    ui.selectable_value(
                                        &mut setup.rotary.axis_letter_override,
                                        None,
                                        format!("Auto ({def})"),
                                    );
                                    for c in ['A', 'B', 'C'] {
                                        ui.selectable_value(
                                            &mut setup.rotary.axis_letter_override,
                                            Some(c),
                                            format!("{c}-Axis"),
                                        );
                                    }
                                });
                            if setup.rotary.axis_letter_override != prev {
                                *mutated_label = Some("Change Rotary Axis Letter");
                            }
                        },
                    );

                    grid_row(
                        ui,
                        "Unwind Mode",
                        Some(
                            "Angular rollover policy for multi-turn spirals vs. shortest-path indexing",
                        ),
                        |ui| {
                            let prev = setup.rotary.unwind_mode;
                            egui::ComboBox::from_id_salt(("rot_unwind_cb", setup_idx))
                                .width(INPUT_COL_WIDTH)
                                .selected_text(setup.rotary.unwind_mode.label())
                                .show_ui(ui, |ui| {
                                    for u in RotaryUnwindMode::ALL {
                                        ui.selectable_value(
                                            &mut setup.rotary.unwind_mode,
                                            u,
                                            u.label(),
                                        );
                                    }
                                });
                            if setup.rotary.unwind_mode != prev {
                                *mutated_label = Some("Change Rotary Unwind Mode");
                            }
                        },
                    );
                }
            });
        },
    );

    if !setup.rotary.is_enabled() {
        ui.add_space(4.0);
        inline_banner(
            ui,
            BannerKind::Info,
            "Enable 4th-Axis Rotary above to configure A/B/C centerline kinematics, chuck clearance, polar stock simulation, and 4-axis G-code output.",
        );
        return;
    }

    ui.add_space(5.0);
    let (off0_lbl, off1_lbl) = setup.rotary.axis.offset_axis_labels();
    section_card(
        ui,
        ("setup_rotary_chuck_card", setup_idx),
        "Centerline, Clearance & Chuck",
        true,
        |ui| {
            inspector_grid(ui, ("setup_rotary_chuck_grid", setup_idx), |ui| {
                let mut sg = SetupGrid {
                    ui,
                    setup_idx,
                    eval_ctx,
                    mutated: mutated_label,
                };
                sg.len(
                    off0_lbl,
                    "rot_off0",
                    &mut setup.rotary.center_offset_wcs[0],
                    "Edit Rotary Center Offset",
                );
                sg.len(
                    off1_lbl,
                    "rot_off1",
                    &mut setup.rotary.center_offset_wcs[1],
                    "Edit Rotary Center Offset",
                );
                sg.len(
                    "Clearance Margin",
                    "rot_clr_off",
                    &mut setup.rotary.clearance_radial_offset,
                    "Edit Rotary Clearance Offset",
                );
                sg.len(
                    "Chuck Diameter",
                    "rot_chuck_d",
                    &mut setup.rotary.chuck_diameter,
                    "Edit Rotary Chuck Diameter",
                );
                sg.len(
                    "Chuck Length",
                    "rot_chuck_l",
                    &mut setup.rotary.chuck_length,
                    "Edit Rotary Chuck Length",
                );
                grid_row(
                    sg.ui,
                    "Chuck Side",
                    Some("Place rotary chuck at the negative or positive end of the stock axis"),
                    |ui| {
                        let prev = setup.rotary.chuck_end_positive;
                        ui.horizontal(|ui| {
                            ui.selectable_value(
                                &mut setup.rotary.chuck_end_positive,
                                false,
                                "-End",
                            );
                            ui.selectable_value(&mut setup.rotary.chuck_end_positive, true, "+End");
                        });
                        if setup.rotary.chuck_end_positive != prev {
                            *sg.mutated = Some("Flip Rotary Chuck End");
                        }
                    },
                );
            });

            ui.add_space(4.0);
            if ui
                .button("Align Cylindrical Stock to Rotary Axis")
                .on_hover_text(
                    "Switch Setup stock to a Relative Cylinder concentric with the configured rotary axis and WCS center",
                )
                .clicked()
            {
                setup.stock = StockConfig::RelativeCylinder {
                    axis: setup.rotary.axis.to_stock_axis(),
                    radial_offset: ParamLength::from_mm(2.0),
                    top_offset: ParamLength::from_mm(2.0),
                    bottom_offset: ParamLength::from_mm(2.0),
                };
                setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
                    anchor: BoxAnchor27 {
                        x: AxisAnchor::Center,
                        y: AxisAnchor::Center,
                        z: AxisAnchor::Center,
                    },
                };
                *mutated_label = Some("Align Cylindrical Stock to Rotary Axis");
            }
        },
    );

    if let Some(Ok(rs)) = resolved
        && let Some(rot) = rs.rotary
    {
        ui.add_space(5.0);
        static_card(ui, Some("Resolved Rotary Envelope"), |ui| {
            ui.monospace(format!(
                "Axis:      {}-Axis ({})",
                rot.axis_letter,
                rot.axis.label()
            ));
            ui.monospace(format!(
                "Stock R:   {}   Clearance R: {}",
                display_unit.format_length(rot.stock_outer_radius_mm),
                display_unit.format_length(rot.clearance_radius_mm)
            ));
            ui.monospace(format!(
                "Axial U:   {} .. {}",
                display_unit.format_length(rot.stock_u_min),
                display_unit.format_length(rot.stock_u_max)
            ));
            ui.monospace(format!(
                "Chuck:     {} .. {} (Ø{})",
                display_unit.format_length(rot.chuck_u_min),
                display_unit.format_length(rot.chuck_u_max),
                display_unit.format_length(2.0 * rot.chuck_radius_mm)
            ));
        });
    }
}
