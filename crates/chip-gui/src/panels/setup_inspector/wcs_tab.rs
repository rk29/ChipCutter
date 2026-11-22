use chip_cad::CadDocument;
use chip_cam::{
    AxisAnchor, BoxAnchor27, Setup, WcsAxisRef, WcsFeaturePointRef, WcsOrientationMode,
    WcsOriginMode,
};
use chip_math::EvalContext;

use crate::{
    panels::ViewportPickMode,
    theme::ThemeTokens,
    widgets::{INPUT_COL_WIDTH, grid_row, inspector_grid, section_card},
};

use super::{SetupGrid, render_post_body, render_work_offset_row, show_27pt_xy_matrix};

pub(super) struct WcsTabCtx<'a> {
    pub(super) setup: &'a mut Setup,
    pub(super) setup_idx: usize,
    pub(super) cad_doc: Option<&'a CadDocument>,
    pub(super) eval_ctx: &'a EvalContext,
    pub(super) pick_mode: &'a mut ViewportPickMode,
    pub(super) show_27pt_dots: &'a mut bool,
    pub(super) mutated_label: &'a mut Option<&'static str>,
}

pub(super) fn show_wcs_tab(ui: &mut egui::Ui, mut ctx: WcsTabCtx<'_>) {
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
            "Orientation",
            Some("Align spindle +Z and table +X to CAD geometry"),
            |ui| {
                let orient_lbl = if is_model {
                    "Model XYZ"
                } else if is_zx {
                    "Select Z & X Axes"
                } else {
                    "Select XY Plane"
                };
                let has_cad = ctx.cad_doc.is_some();
                egui::ComboBox::from_id_salt(("wcs_orient_mode_cb", setup_idx))
                    .width(INPUT_COL_WIDTH + 54.0)
                    .truncate()
                    .selected_text(orient_lbl)
                    .show_ui(ui, |ui| {
                        if ui
                            .selectable_label(is_model, "Model XYZ")
                            .on_hover_text("Use the model's native X, Y, Z axes directly")
                            .clicked()
                            && !is_model
                        {
                            ctx.setup.wcs.orientation = WcsOrientationMode::default();
                            *ctx.pick_mode = ViewportPickMode::InspectEntity;
                            *ctx.mutated_label = Some("Reset WCS to Model Orientation");
                        }
                        if ui
                            .add_enabled_ui(has_cad, |ui| {
                                ui.selectable_label(is_zx, "Select Z & X Axes")
                                    .on_hover_text(
                                        "Align +Z spindle and +X table axis to picked B-Rep faces or edges",
                                    )
                            })
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
                            .add_enabled_ui(has_cad, |ui| {
                                ui.selectable_label(is_xy, "Select XY Plane")
                                    .on_hover_text(
                                        "Align Setup XY plane directly onto a picked planar B-Rep face",
                                    )
                            })
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
                    })
                    .response
                    .on_hover_text("Choose how Setup +Z spindle and +X table axes are oriented");
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
                    "Flip +Z",
                    Some("Invert spindle +Z 180 degrees (e.g., flip part upside down for Op 2)"),
                    |ui| {
                        if ui
                            .checkbox(flip_z, "Invert Spindle (+Z)")
                            .on_hover_text(
                                "Flip the part upside down relative to the spindle +Z axis",
                            )
                            .changed()
                        {
                            *ctx.mutated_label = Some("Flip WCS Z Axis");
                        }
                    },
                );
                grid_row(
                    ui,
                    "Flip +X",
                    Some("Invert table +X direction 180 degrees"),
                    |ui| {
                        if ui
                            .checkbox(flip_x, "Invert Table (+X)")
                            .on_hover_text("Reverse Setup +X direction by 180 degrees")
                            .changed()
                        {
                            *ctx.mutated_label = Some("Flip WCS X Axis");
                        }
                    },
                );
                grid_row(
                    ui,
                    "X Clocking",
                    Some("Rotate Setup +X/+Y around +Z in 90-degree increments"),
                    |ui| {
                        if ui
                            .small_button("Clock +90°")
                            .on_hover_text("Rotate Setup +X/+Y by 90 degrees around +Z")
                            .clicked()
                        {
                            *clock_90 = (*clock_90 + 1) % 4;
                            *ctx.mutated_label = Some("Clock WCS X Axis 90°");
                        }
                        ui.colored_label(tok.text_muted, format!("{}°", u16::from(*clock_90) * 90));
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
                        if ui
                            .small_button("Clock X 90°")
                            .on_hover_text(
                                "Rotate Setup +X/+Y by 90 degrees around the +Z spindle axis",
                            )
                            .clicked()
                        {
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
                        if ui
                            .selectable_label(on, "Pick Face")
                            .on_hover_text("Click a planar 3D CAD face in the viewport to define the Setup XY plane")
                            .clicked()
                        {
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
                    "Flip +Z",
                    Some("Invert the picked plane's +Z spindle normal 180 degrees"),
                    |ui| {
                        if ui
                            .checkbox(flip_z, "Invert Spindle (+Z)")
                            .on_hover_text("Flip the spindle +Z normal 180 degrees")
                            .changed()
                        {
                            *ctx.mutated_label = Some("Flip WCS Z Axis");
                        }
                    },
                );
                grid_row(
                    ui,
                    "X Clocking",
                    Some("Rotate Setup +X/+Y around +Z in 90-degree increments"),
                    |ui| {
                        if ui
                            .small_button("Clock +90°")
                            .on_hover_text("Rotate Setup +X/+Y by 90 degrees around +Z")
                            .clicked()
                        {
                            *clock_90 = (*clock_90 + 1) % 4;
                            *ctx.mutated_label = Some("Clock WCS X Axis 90°");
                        }
                        ui.colored_label(tok.text_muted, format!("{}°", u16::from(*clock_90) * 90));
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
                let resp = ui
                    .add(
                        egui::DragValue::new(extra_z_rot)
                            .speed(0.5)
                            .range(-360.0..=360.0)
                            .suffix("°")
                            .max_decimals(2),
                    )
                    .on_hover_text("Fine rotation angle around Setup +Z in degrees");
                if resp.changed() {
                    if !extra_z_rot.is_finite() {
                        *extra_z_rot = 0.0;
                    }
                    *ctx.mutated_label = Some("Adjust WCS Fine Z Rotation");
                }
                if extra_z_rot.abs() > 1e-6
                    && ui
                        .small_button("0°")
                        .on_hover_text("Reset fine Z rotation back to 0 degrees")
                        .clicked()
                {
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
            if ui
                .selectable_label(on, spec.pick_btn)
                .on_hover_text(
                    "Click a 3D face or straight edge in the viewport to align this WCS axis",
                )
                .clicked()
            {
                *pick_mode = if on {
                    ViewportPickMode::InspectEntity
                } else {
                    spec.target_mode
                };
            }
            if ui
                .checkbox(flip, spec.flip_lbl)
                .on_hover_text("Invert the direction of this WCS axis 180 degrees")
                .changed()
            {
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
            "Origin Source",
            Some("Datum probed for X0 Y0 Z0 on the machine"),
            |ui| {
                let origin_lbl = if is_stock_pt {
                    "Stock Box (27-Pt)"
                } else if is_model_pt {
                    "Model Box (27-Pt)"
                } else {
                    "CAD Feature"
                };
                let has_cad = ctx.cad_doc.is_some();
                egui::ComboBox::from_id_salt(("wcs_origin_mode_cb", setup_idx))
                    .width(INPUT_COL_WIDTH + 54.0)
                    .truncate()
                    .selected_text(origin_lbl)
                    .show_ui(ui, |ui| {
                        if ui
                            .selectable_label(is_stock_pt, "Stock Box (27-Pt)")
                            .on_hover_text("Place X0 Y0 Z0 on any corner, edge, face center, or center of the raw stock box")
                            .clicked()
                            && !is_stock_pt
                        {
                            ctx.setup.wcs.origin = WcsOriginMode::StockBoxPoint {
                                anchor: BoxAnchor27::TOP_FRONT_LEFT,
                            };
                            *ctx.mutated_label = Some("Set WCS Origin to Stock Box");
                        }
                        if ui
                            .selectable_label(is_model_pt, "Model Box (27-Pt)")
                            .on_hover_text("Place X0 Y0 Z0 on any of the 27 bounding-box points of the target CAD model")
                            .clicked()
                            && !is_model_pt
                        {
                            ctx.setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
                                anchor: BoxAnchor27::TOP_FRONT_LEFT,
                            };
                            *ctx.mutated_label = Some("Set WCS Origin to Model Box");
                        }
                        if ui
                            .add_enabled_ui(has_cad, |ui| {
                                ui.selectable_label(is_cad_pt, "CAD Feature")
                                    .on_hover_text("Anchor X0 Y0 Z0 directly to a B-Rep hole center or edge point")
                            })
                            .inner
                            .clicked()
                            && !is_cad_pt
                            && let Some(doc) = ctx.cad_doc
                        {
                            select_default_cad_origin_feature(doc, &mut ctx.setup.wcs.origin);
                            *ctx.pick_mode = ViewportPickMode::SetupWcsOriginFeature { setup_idx };
                            *ctx.mutated_label = Some("Set WCS Origin to CAD Feature");
                        }
                    })
                    .response
                    .on_hover_text("Choose whether WCS zero (X0 Y0 Z0) is anchored to Stock bounds, Model bounds, or a CAD feature");
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
                    "Z Anchor",
                    Some("Select Top, Mid-Z, or Bottom plane of the 27-point datum box"),
                    |ui| {
                        for (lbl, z_a) in [
                            ("Top", AxisAnchor::Max),
                            ("Mid", AxisAnchor::Center),
                            ("Bot", AxisAnchor::Min),
                        ] {
                            if ui
                                .selectable_label(anchor.z == z_a, lbl)
                                .on_hover_text(format!("Set WCS Z0 plane to {lbl}"))
                                .clicked()
                                && anchor.z != z_a
                            {
                                anchor.z = z_a;
                                *ctx.mutated_label = Some("Change 27-Point WCS Origin");
                            }
                        }
                    },
                );
                grid_row(
                    ui,
                    "XY Anchor",
                    Some("Pick any of the 9 XY positions (Left/Center/Right x Back/Center/Front)"),
                    |ui| {
                        if show_27pt_xy_matrix(ui, anchor) {
                            *ctx.mutated_label = Some("Change 27-Point WCS Origin");
                        }
                    },
                );
                grid_row(
                    ui,
                    "3D Origin Dots",
                    Some("Show clickable 27-point origin dots directly in the 3D viewport"),
                    |ui| {
                        ui.checkbox(ctx.show_27pt_dots, "Show 27-pt overlay")
                            .on_hover_text(
                                "Display clickable 27-point WCS anchor spheres in the 3D viewport",
                            );
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
                        if ui
                            .selectable_label(on, "Pick CAD Feature")
                            .on_hover_text("Click a cylindrical hole or edge in the 3D viewport to anchor WCS zero")
                            .clicked()
                        {
                            *ctx.pick_mode = if on {
                                ViewportPickMode::InspectEntity
                            } else {
                                target
                            };
                        }
                    },
                );
                if let WcsFeaturePointRef::HoleAxisCenter { top_rim, .. } = feature {
                    grid_row(
                        ui,
                        "Bore Rim",
                        Some(
                            "Anchor WCS Z0 at the top rim or bottom floor of the cylindrical bore",
                        ),
                        |ui| {
                            if ui
                                .checkbox(top_rim, "Top Rim")
                                .on_hover_text(
                                    "Checked = top rim of bore; unchecked = bottom of bore",
                                )
                                .changed()
                            {
                                *ctx.mutated_label = Some("Toggle Bore Top/Bottom Origin");
                            }
                        },
                    );
                }
            }
        }
    });

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
