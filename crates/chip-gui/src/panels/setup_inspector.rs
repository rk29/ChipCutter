use chip_cad::CadDocument;
use chip_cam::{
    AxisAnchor, BoxAnchor27, FixedBoxXyAlign, FixedBoxZAlign, HeightReference, ResolvedSetup,
    Setup, SetupHeights, StockAxis, StockConfig, WcsAxisRef, WcsConfig, WcsFeaturePointRef,
    WcsOrientationMode, WcsOriginMode, WorkOffsetSlot,
};
use chip_math::{DisplayUnit, EvalContext, ParamLength};
use chip_project::{ProjectDocument, UndoHistory};

use crate::{
    panels::{SetupTab, ViewportPickMode},
    widgets::edit_param_length,
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
}

/// Lightweight (~200-byte) pre-edit snapshot of only the Setup's header & WCS/Stock/Heights fields
/// (never cloning `project.tools` or `setup.operations` on idle 60 FPS frames!).
struct SetupParamsSnapshot {
    name: String,
    wcs_slot: WorkOffsetSlot,
    wcs: WcsConfig,
    stock: StockConfig,
    heights: SetupHeights,
}

impl SetupParamsSnapshot {
    fn capture(setup: &Setup) -> Self {
        Self {
            name: setup.name.clone(),
            wcs_slot: setup.wcs_slot,
            wcs: setup.wcs.clone(),
            stock: setup.stock.clone(),
            heights: setup.heights.clone(),
        }
    }

    fn restore_into(self, setup: &mut Setup) {
        setup.name = self.name;
        setup.wcs_slot = self.wcs_slot;
        setup.wcs = self.wcs;
        setup.stock = self.stock;
        setup.heights = self.heights;
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
    } = ctx;

    if setup_idx >= project.setups.len() {
        ui.weak("No Setup selected.");
        return false;
    }

    let display_unit = project.display_units;
    let mut align_camera_requested = false;
    let pre_setup = SetupParamsSnapshot::capture(&project.setups[setup_idx]);
    let mut mutated_label: Option<&'static str> = None;

    {
        let setup = &mut project.setups[setup_idx];
        ui.horizontal(|ui| {
            ui.strong("Setup:");
            let resp = ui.text_edit_singleline(&mut setup.name);
            if resp.lost_focus() && setup.name != pre_setup.name {
                mutated_label = Some("Rename Setup");
            }
        });

        ui.horizontal(|ui| {
            ui.label("Work Offset:");
            egui::ComboBox::from_id_salt(("wcs_slot_cb", setup_idx))
                .selected_text(setup.wcs_slot.label())
                .show_ui(ui, |ui| {
                    for slot in WorkOffsetSlot::ALL {
                        if ui
                            .selectable_value(&mut setup.wcs_slot, slot, slot.label())
                            .changed()
                        {
                            mutated_label = Some("Change Work Offset Slot");
                        }
                    }
                });
            if ui
                .small_button("Align Top")
                .on_hover_text("Align viewport camera to this Setup's +Z axis")
                .clicked()
            {
                align_camera_requested = true;
            }
        });
    }

    ui.separator();
    ui.horizontal(|ui| {
        ui.selectable_value(tab, SetupTab::Wcs, "WCS");
        ui.selectable_value(tab, SetupTab::Stock, "Stock");
        ui.selectable_value(tab, SetupTab::Heights, "Heights");
    });
    ui.separator();

    let eval_ctx = match resolved {
        Some(Ok(rs)) => rs.eval_context(display_unit),
        _ => EvalContext::new(display_unit),
    };

    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            if let Some(Err(err_msg)) = &resolved {
                ui.colored_label(
                    egui::Color32::from_rgb(248, 98, 98),
                    format!("[!] {err_msg}"),
                );
                ui.separator();
            }

            let setup = &mut project.setups[setup_idx];
            match *tab {
                SetupTab::Wcs => show_wcs_tab(
                    ui,
                    (setup, setup_idx),
                    cad_doc,
                    &eval_ctx,
                    pick_mode,
                    show_27pt_dots,
                    &mut mutated_label,
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
                SetupTab::Heights => show_heights_tab(
                    ui,
                    (setup, setup_idx),
                    &eval_ctx,
                    display_unit,
                    resolved,
                    show_z_planes,
                    &mut mutated_label,
                ),
            }
        });

    if let Some(action) = mutated_label {
        let mut pre_doc = project.clone();
        pre_setup.restore_into(&mut pre_doc.setups[setup_idx]);
        history.record_before_edit(action, None, &pre_doc);
        for op in &mut project.setups[setup_idx].operations {
            op.mark_dirty();
        }
    }

    align_camera_requested
}

fn show_wcs_tab(
    ui: &mut egui::Ui,
    (setup, setup_idx): (&mut Setup, usize),
    cad_doc: Option<&CadDocument>,
    eval_ctx: &EvalContext,
    pick_mode: &mut ViewportPickMode,
    show_27pt_dots: &mut bool,
    mutated_label: &mut Option<&'static str>,
) {
    ui.strong("1. Machine Orientation (T_body_to_wcs)");
    let is_model = matches!(
        setup.wcs.orientation,
        WcsOrientationMode::ModelOrientation { .. }
    );
    let is_zx = matches!(
        setup.wcs.orientation,
        WcsOrientationMode::SelectZAndX { .. }
    );
    let is_xy = matches!(
        setup.wcs.orientation,
        WcsOrientationMode::SelectXYPlane { .. }
    );

    ui.horizontal_wrapped(|ui| {
        if ui.selectable_label(is_model, "Model XYZ").clicked() && !is_model {
            setup.wcs.orientation = WcsOrientationMode::default();
            *pick_mode = ViewportPickMode::InspectEntity;
            *mutated_label = Some("Reset WCS to Model Orientation");
        }
        if ui.selectable_label(is_zx, "Select Z & X").clicked()
            && !is_zx
            && let Some(doc) = cad_doc
            && let Some(default_face) = doc.faces.first()
            && let Some(bind) = doc.bind_face(default_face.id)
        {
            setup.wcs.orientation = WcsOrientationMode::SelectZAndX {
                z_ref: WcsAxisRef::FaceAxis(bind),
                flip_z: false,
                x_ref: None,
                flip_x: false,
                clock_90: 0,
            };
            *pick_mode = ViewportPickMode::SetupWcsZ { setup_idx };
            *mutated_label = Some("Switch WCS to Select Z & X");
        }
        if ui.selectable_label(is_xy, "Select XY Plane").clicked()
            && !is_xy
            && let Some(doc) = cad_doc
            && let Some(default_face) = doc.faces.first()
            && let Some(bind) = doc.bind_face(default_face.id)
        {
            setup.wcs.orientation = WcsOrientationMode::SelectXYPlane {
                face_ref: bind,
                flip_z: false,
                x_edge_ref: None,
                clock_90: 0,
            };
            *pick_mode = ViewportPickMode::SetupWcsXyPlane { setup_idx };
            *mutated_label = Some("Switch WCS to Select XY Plane");
        }
    });

    match &mut setup.wcs.orientation {
        WcsOrientationMode::ModelOrientation {
            flip_z,
            flip_x,
            clock_90,
        } => {
            ui.horizontal(|ui| {
                if ui.checkbox(flip_z, "Flip +Z").changed() {
                    *mutated_label = Some("Flip WCS Z Axis");
                }
                if ui.checkbox(flip_x, "Flip +X").changed() {
                    *mutated_label = Some("Flip WCS X Axis");
                }
                if ui.small_button("Clock X 90°").clicked() {
                    *clock_90 = (*clock_90 + 1) % 4;
                    *mutated_label = Some("Clock WCS X Axis 90°");
                }
            });
        }
        WcsOrientationMode::SelectZAndX {
            z_ref,
            flip_z,
            x_ref,
            flip_x,
            clock_90,
        } => {
            let picking_z = *pick_mode == ViewportPickMode::SetupWcsZ { setup_idx };
            let picking_x = *pick_mode == ViewportPickMode::SetupWcsX { setup_idx };

            ui.horizontal(|ui| {
                let z_desc = match z_ref {
                    WcsAxisRef::FaceAxis(b) => format!("Face #{}", b.last_index),
                    WcsAxisRef::EdgeDirection(b) => format!("Edge #{}", b.last_index),
                };
                ui.label(format!("+Z Axis: {z_desc}"));
                if ui.selectable_label(picking_z, "Pick Z").clicked() {
                    *pick_mode = if picking_z {
                        ViewportPickMode::InspectEntity
                    } else {
                        ViewportPickMode::SetupWcsZ { setup_idx }
                    };
                }
                if ui.checkbox(flip_z, "Flip Z").changed() {
                    *mutated_label = Some("Flip WCS Z Axis");
                }
            });

            ui.horizontal(|ui| {
                let x_desc = match x_ref {
                    Some(WcsAxisRef::FaceAxis(b)) => format!("Face #{}", b.last_index),
                    Some(WcsAxisRef::EdgeDirection(b)) => format!("Edge #{}", b.last_index),
                    None => "Auto".to_string(),
                };
                ui.label(format!("+X Axis: {x_desc}"));
                if ui.selectable_label(picking_x, "Pick X").clicked() {
                    *pick_mode = if picking_x {
                        ViewportPickMode::InspectEntity
                    } else {
                        ViewportPickMode::SetupWcsX { setup_idx }
                    };
                }
                if ui.checkbox(flip_x, "Flip X").changed() {
                    *mutated_label = Some("Flip WCS X Axis");
                }
            });

            if ui.small_button("Clock X 90°").clicked() {
                *clock_90 = (*clock_90 + 1) % 4;
                *mutated_label = Some("Clock WCS X Axis 90°");
            }
        }
        WcsOrientationMode::SelectXYPlane {
            face_ref,
            flip_z,
            clock_90,
            ..
        } => {
            let picking_xy = *pick_mode == ViewportPickMode::SetupWcsXyPlane { setup_idx };
            ui.horizontal(|ui| {
                ui.label(format!("XY Face: #{}", face_ref.last_index));
                if ui.selectable_label(picking_xy, "Pick Face").clicked() {
                    *pick_mode = if picking_xy {
                        ViewportPickMode::InspectEntity
                    } else {
                        ViewportPickMode::SetupWcsXyPlane { setup_idx }
                    };
                }
                if ui.checkbox(flip_z, "Flip +Z").changed() {
                    *mutated_label = Some("Flip WCS Z Axis");
                }
                if ui.small_button("Clock 90°").clicked() {
                    *clock_90 = (*clock_90 + 1) % 4;
                    *mutated_label = Some("Clock WCS X Axis 90°");
                }
            });
        }
    }

    ui.separator();
    ui.strong("2. WCS Origin (X0 Y0 Z0)");

    let is_stock_pt = matches!(setup.wcs.origin, WcsOriginMode::StockBoxPoint { .. });
    let is_model_pt = matches!(setup.wcs.origin, WcsOriginMode::ModelBoxPoint { .. });
    let is_cad_pt = matches!(setup.wcs.origin, WcsOriginMode::SelectedCadFeature { .. });

    ui.horizontal_wrapped(|ui| {
        if ui.selectable_label(is_stock_pt, "Stock 27-Pt").clicked() && !is_stock_pt {
            setup.wcs.origin = WcsOriginMode::StockBoxPoint {
                anchor: BoxAnchor27::TOP_FRONT_LEFT,
            };
            *mutated_label = Some("Set WCS Origin to Stock Box");
        }
        if ui.selectable_label(is_model_pt, "Model 27-Pt").clicked() && !is_model_pt {
            setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
                anchor: BoxAnchor27::TOP_FRONT_LEFT,
            };
            *mutated_label = Some("Set WCS Origin to Model Box");
        }
        if ui.selectable_label(is_cad_pt, "CAD Feature").clicked()
            && !is_cad_pt
            && let Some(doc) = cad_doc
        {
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
                setup.wcs.origin = WcsOriginMode::SelectedCadFeature {
                    feature: WcsFeaturePointRef::HoleAxisCenter {
                        hole_face: b,
                        top_rim: true,
                    },
                };
            } else if let Some(edge) = doc.edges.first()
                && let Some(b) = doc.bind_edge(edge.id)
            {
                setup.wcs.origin = WcsOriginMode::SelectedCadFeature {
                    feature: WcsFeaturePointRef::EdgeMidpoint(b),
                };
            }
            *pick_mode = ViewportPickMode::SetupWcsOriginFeature { setup_idx };
            *mutated_label = Some("Set WCS Origin to CAD Feature");
        }
    });

    match &mut setup.wcs.origin {
        WcsOriginMode::StockBoxPoint { anchor } | WcsOriginMode::ModelBoxPoint { anchor } => {
            ui.checkbox(
                show_27pt_dots,
                "Show interactive 27-point dots in 3D viewport",
            );
            ui.label(format!("Active Point: {}", anchor.label()));
            if show_27pt_grid_selector(ui, anchor) {
                *mutated_label = Some("Change 27-Point WCS Origin");
            }
        }
        WcsOriginMode::SelectedCadFeature { feature } => {
            let picking_feat = *pick_mode == ViewportPickMode::SetupWcsOriginFeature { setup_idx };
            let desc = match feature {
                WcsFeaturePointRef::HoleAxisCenter { hole_face, top_rim } => {
                    format!(
                        "Bore #{} ({})",
                        hole_face.last_index,
                        if *top_rim {
                            "Top Center"
                        } else {
                            "Bottom Center"
                        }
                    )
                }
                WcsFeaturePointRef::CircleCenter(b) => {
                    format!("Circle Edge #{} Center", b.last_index)
                }
                WcsFeaturePointRef::EdgeStart(b) => format!("Edge #{} Start", b.last_index),
                WcsFeaturePointRef::EdgeMidpoint(b) => format!("Edge #{} Midpoint", b.last_index),
                WcsFeaturePointRef::EdgeEnd(b) => format!("Edge #{} End", b.last_index),
            };
            ui.label(format!("Anchor: {desc}"));
            ui.horizontal(|ui| {
                if ui
                    .selectable_label(picking_feat, "Pick CAD Feature")
                    .clicked()
                {
                    *pick_mode = if picking_feat {
                        ViewportPickMode::InspectEntity
                    } else {
                        ViewportPickMode::SetupWcsOriginFeature { setup_idx }
                    };
                }
                if let WcsFeaturePointRef::HoleAxisCenter { top_rim, .. } = feature
                    && ui.checkbox(top_rim, "Top Rim").changed()
                {
                    *mutated_label = Some("Toggle Bore Top/Bottom Origin");
                }
            });
        }
    }

    ui.add_space(4.0);
    ui.strong("3. Additional XYZ Offset (WCS)");
    for (axis_i, lbl) in [(0, "ΔX:"), (1, "ΔY:"), (2, "ΔZ:")] {
        ui.horizontal(|ui| {
            ui.label(lbl);
            if edit_param_length(
                ui,
                ("wcs_off", setup_idx, axis_i),
                &mut setup.wcs.additional_offset_wcs[axis_i],
                eval_ctx,
                110.0,
            ) {
                *mutated_label = Some("Edit WCS Origin Offset");
            }
        });
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
    let StockTabView {
        setup_idx,
        is_pure_2d,
        eval_ctx,
        display_unit,
        resolved,
    } = view;
    ui.strong("Stock Geometry Mode");
    let cur_kind = setup.stock.kind_label();
    egui::ComboBox::from_id_salt(("stock_mode_cb", setup_idx))
        .selected_text(cur_kind)
        .show_ui(ui, |ui| {
            if ui
                .selectable_label(
                    matches!(setup.stock, StockConfig::RelativeBox { .. }),
                    "Relative Box Stock",
                )
                .clicked()
                && !matches!(setup.stock, StockConfig::RelativeBox { .. })
            {
                setup.stock = StockConfig::default();
                *mutated_label = Some("Change Stock Mode to Relative Box");
            }
            if ui
                .selectable_label(
                    matches!(setup.stock, StockConfig::FixedBox { .. }),
                    "Fixed-Size Box Stock",
                )
                .clicked()
                && !matches!(setup.stock, StockConfig::FixedBox { .. })
            {
                setup.stock = StockConfig::FixedBox {
                    width_x: ParamLength::from_mm(100.0),
                    depth_y: ParamLength::from_mm(80.0),
                    height_z: ParamLength::from_mm(25.0),
                    xy_align: FixedBoxXyAlign::CenterXy,
                    z_align: FixedBoxZAlign::default(),
                };
                *mutated_label = Some("Change Stock Mode to Fixed Box");
            }
            if ui
                .selectable_label(
                    matches!(setup.stock, StockConfig::RelativeCylinder { .. }),
                    "Relative Cylindrical Stock",
                )
                .clicked()
                && !matches!(setup.stock, StockConfig::RelativeCylinder { .. })
            {
                setup.stock = StockConfig::RelativeCylinder {
                    axis: StockAxis::Z,
                    radial_offset: ParamLength::from_mm(3.0),
                    top_offset: ParamLength::from_mm(1.0),
                    bottom_offset: ParamLength::from_mm(5.0),
                };
                *mutated_label = Some("Change Stock Mode to Relative Cylinder");
            }
            if ui
                .selectable_label(
                    matches!(setup.stock, StockConfig::FixedCylinder { .. }),
                    "Fixed Cylindrical Stock",
                )
                .clicked()
                && !matches!(setup.stock, StockConfig::FixedCylinder { .. })
            {
                setup.stock = StockConfig::FixedCylinder {
                    axis: StockAxis::Z,
                    diameter: ParamLength::from_mm(130.0),
                    length: ParamLength::from_mm(30.0),
                    axial_top_offset: ParamLength::from_mm(1.0),
                };
                *mutated_label = Some("Change Stock Mode to Fixed Cylinder");
            }
        });

    ui.separator();

    match &mut setup.stock {
        StockConfig::RelativeBox {
            side_pos_x,
            side_neg_x,
            side_pos_y,
            side_neg_y,
            symmetric_xy,
            top_offset_z,
            bottom_offset_z,
            sheet_thickness_z,
            round_increment_mm,
        } => {
            if is_pure_2d {
                ui.label("Pure-2D Sheet Thickness (Z):");
                if edit_param_length(
                    ui,
                    ("stk_sheet_z", setup_idx),
                    sheet_thickness_z,
                    eval_ctx,
                    115.0,
                ) {
                    *mutated_label = Some("Edit Pure-2D Sheet Thickness");
                }
                ui.add_space(2.0);
            }
            if ui.checkbox(symmetric_xy, "Symmetric XY Margins").changed() {
                *mutated_label = Some("Toggle Symmetric Stock Margins");
            }
            ui.label("Side X Margin (+X):");
            if edit_param_length(ui, ("stk_px", setup_idx), side_pos_x, eval_ctx, 115.0) {
                if *symmetric_xy {
                    *side_neg_x = side_pos_x.clone();
                }
                *mutated_label = Some("Edit Stock Side Margin");
            }
            if !*symmetric_xy {
                ui.label("Side -X Margin:");
                if edit_param_length(ui, ("stk_nx", setup_idx), side_neg_x, eval_ctx, 115.0) {
                    *mutated_label = Some("Edit Stock -X Margin");
                }
            }
            ui.label("Side Y Margin (+Y):");
            if edit_param_length(ui, ("stk_py", setup_idx), side_pos_y, eval_ctx, 115.0) {
                if *symmetric_xy {
                    *side_neg_y = side_pos_y.clone();
                }
                *mutated_label = Some("Edit Stock Y Margin");
            }
            if !*symmetric_xy {
                ui.label("Side -Y Margin:");
                if edit_param_length(ui, ("stk_ny", setup_idx), side_neg_y, eval_ctx, 115.0) {
                    *mutated_label = Some("Edit Stock -Y Margin");
                }
            }
            ui.label("Top Facing Allowance (+Z):");
            if edit_param_length(ui, ("stk_tz", setup_idx), top_offset_z, eval_ctx, 115.0) {
                *mutated_label = Some("Edit Stock Top Offset");
            }
            ui.label("Bottom Vise/Tab Holding (-Z):");
            if edit_param_length(ui, ("stk_bz", setup_idx), bottom_offset_z, eval_ctx, 115.0) {
                *mutated_label = Some("Edit Stock Bottom Offset");
            }

            let mut round_on = round_increment_mm.is_some();
            if ui
                .checkbox(&mut round_on, "Round total stock up to 5 mm increment")
                .changed()
            {
                *round_increment_mm = if round_on { Some(5.0) } else { None };
                *mutated_label = Some("Toggle Stock Rounding");
            }
        }
        StockConfig::FixedBox {
            width_x,
            depth_y,
            height_z,
            z_align,
            ..
        } => {
            ui.label("Width (WCS X):");
            if edit_param_length(ui, ("fbox_x", setup_idx), width_x, eval_ctx, 115.0) {
                *mutated_label = Some("Edit Fixed Stock Width");
            }
            ui.label("Depth (WCS Y):");
            if edit_param_length(ui, ("fbox_y", setup_idx), depth_y, eval_ctx, 115.0) {
                *mutated_label = Some("Edit Fixed Stock Depth");
            }
            ui.label("Height (WCS Z):");
            if edit_param_length(ui, ("fbox_z", setup_idx), height_z, eval_ctx, 115.0) {
                *mutated_label = Some("Edit Fixed Stock Height");
            }
            if let FixedBoxZAlign::OffsetFromTop { top_allowance } = z_align {
                ui.label("Model Offset from Stock Top:");
                if edit_param_length(ui, ("fbox_ta", setup_idx), top_allowance, eval_ctx, 115.0) {
                    *mutated_label = Some("Edit Fixed Stock Top Allowance");
                }
            }
        }
        StockConfig::RelativeCylinder {
            axis,
            radial_offset,
            top_offset,
            bottom_offset,
        } => {
            egui::ComboBox::from_label("Bar Axis")
                .selected_text(axis.label())
                .show_ui(ui, |ui| {
                    for a in StockAxis::ALL {
                        if ui.selectable_value(axis, a, a.label()).changed() {
                            *mutated_label = Some("Change Cylinder Stock Axis");
                        }
                    }
                });
            ui.label("Radial Margin:");
            if edit_param_length(ui, ("rcyl_r", setup_idx), radial_offset, eval_ctx, 115.0) {
                *mutated_label = Some("Edit Cylinder Radial Offset");
            }
            ui.label("Top Margin:");
            if edit_param_length(ui, ("rcyl_t", setup_idx), top_offset, eval_ctx, 115.0) {
                *mutated_label = Some("Edit Cylinder Top Offset");
            }
            ui.label("Bottom Chuck Margin:");
            if edit_param_length(ui, ("rcyl_b", setup_idx), bottom_offset, eval_ctx, 115.0) {
                *mutated_label = Some("Edit Cylinder Bottom Offset");
            }
        }
        StockConfig::FixedCylinder {
            axis,
            diameter,
            length,
            axial_top_offset,
        } => {
            egui::ComboBox::from_label("Bar Axis")
                .selected_text(axis.label())
                .show_ui(ui, |ui| {
                    for a in StockAxis::ALL {
                        if ui.selectable_value(axis, a, a.label()).changed() {
                            *mutated_label = Some("Change Cylinder Stock Axis");
                        }
                    }
                });
            ui.label("Bar Diameter:");
            if edit_param_length(ui, ("fcyl_d", setup_idx), diameter, eval_ctx, 115.0) {
                *mutated_label = Some("Edit Cylinder Diameter");
            }
            ui.label("Bar Length:");
            if edit_param_length(ui, ("fcyl_l", setup_idx), length, eval_ctx, 115.0) {
                *mutated_label = Some("Edit Cylinder Length");
            }
            ui.label("Top Facing Offset:");
            if edit_param_length(
                ui,
                ("fcyl_to", setup_idx),
                axial_top_offset,
                eval_ctx,
                115.0,
            ) {
                *mutated_label = Some("Edit Cylinder Top Offset");
            }
        }
    }

    if let Some(Ok(rs)) = resolved {
        ui.separator();
        let s_sz = rs.stock.aabb_wcs().size();
        let m_sz = rs.model_aabb_wcs.size();
        ui.strong("Resolved WCS Dimensions");
        ui.label(format!(
            "Model: {} × {} × {}",
            display_unit.format_length(m_sz.x),
            display_unit.format_length(m_sz.y),
            display_unit.format_length(m_sz.z),
        ));
        ui.label(format!(
            "Stock: {} × {} × {}",
            display_unit.format_length(s_sz.x),
            display_unit.format_length(s_sz.y),
            display_unit.format_length(s_sz.z),
        ));
    }
}

fn show_heights_tab(
    ui: &mut egui::Ui,
    (setup, setup_idx): (&mut Setup, usize),
    eval_ctx: &EvalContext,
    display_unit: DisplayUnit,
    resolved: Option<Result<&ResolvedSetup, &str>>,
    show_z_planes: &mut bool,
    mutated_label: &mut Option<&'static str>,
) {
    ui.checkbox(
        show_z_planes,
        "Preview translucent Z-height planes in 3D viewport",
    );
    ui.separator();

    ui.strong("Clearance Height (First/Last G0 Rapids)");
    egui::ComboBox::from_id_salt(("setup_clr_ref", setup_idx))
        .selected_text(setup.heights.clearance.reference.label())
        .show_ui(ui, |ui| {
            for r in HeightReference::SETUP_REFS {
                if ui
                    .selectable_value(&mut setup.heights.clearance.reference, r, r.label())
                    .changed()
                {
                    *mutated_label = Some("Change Setup Clearance Reference");
                }
            }
        });
    if edit_param_length(
        ui,
        ("setup_clr_off", setup_idx),
        &mut setup.heights.clearance.offset,
        eval_ctx,
        120.0,
    ) {
        *mutated_label = Some("Edit Setup Clearance Offset");
    }

    ui.add_space(6.0);
    ui.strong("Retract Height (Local G0 Jumps)");
    egui::ComboBox::from_id_salt(("setup_ret_ref", setup_idx))
        .selected_text(setup.heights.retract.reference.label())
        .show_ui(ui, |ui| {
            for r in HeightReference::SETUP_REFS {
                if ui
                    .selectable_value(&mut setup.heights.retract.reference, r, r.label())
                    .changed()
                {
                    *mutated_label = Some("Change Setup Retract Reference");
                }
            }
        });
    if edit_param_length(
        ui,
        ("setup_ret_off", setup_idx),
        &mut setup.heights.retract.offset,
        eval_ctx,
        120.0,
    ) {
        *mutated_label = Some("Edit Setup Retract Offset");
    }

    if let Some(Ok(rs)) = resolved {
        ui.separator();
        ui.strong("Resolved WCS Z Planes");
        ui.label(format!(
            "• Clearance Z: {}",
            display_unit.format_length(rs.clearance_z_wcs)
        ));
        ui.label(format!(
            "• Retract Z:   {}",
            display_unit.format_length(rs.retract_z_wcs)
        ));
        ui.label(format!(
            "• Stock Top:   {}",
            display_unit.format_length(rs.stock.aabb_wcs().max.z)
        ));
        ui.label(format!(
            "• Model Top:   {}",
            display_unit.format_length(rs.model_aabb_wcs.max.z)
        ));
        ui.label(format!(
            "• Stock Bot:   {}",
            display_unit.format_length(rs.stock.aabb_wcs().min.z)
        ));
    }
}

fn show_27pt_grid_selector(ui: &mut egui::Ui, anchor: &mut BoxAnchor27) -> bool {
    let mut changed = false;

    ui.horizontal(|ui| {
        ui.weak("Z Plane:");
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
