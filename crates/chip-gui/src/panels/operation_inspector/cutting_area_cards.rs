use chip_cad::CadDocument;
use chip_cam::{
    BoundaryPreviewOverlay, ExtractedSelectionSummary, MachiningBoundaryMode, Operation,
    OperationKind, PocketBoundaryMode, ResolvedHeights, ResolvedRotaryWindow, ResolvedSetup,
    RotaryAxialBoundaryMode, StrategyParams, SurfaceMesh3D, ToolContainmentMode,
    resolve_rotary_window_for_preview,
};
use chip_math::{DisplayUnit, EvalContext};
use chip_tools::Tool;

use crate::{
    theme::ThemeTokens,
    widgets::{
        BannerKind, INPUT_COL_WIDTH, edit_param_length, grid_row, inline_banner, inspector_grid,
        section_card,
    },
};

const COMBO_COL_WIDTH: f32 = INPUT_COL_WIDTH + 54.0;

const CONTAINMENT_ORDER: [ToolContainmentMode; 3] = [
    ToolContainmentMode::ToolInsideBoundary,
    ToolContainmentMode::ToolCenterOnBoundary,
    ToolContainmentMode::ToolOutsideBoundary,
];

fn boundary_source_ui_label(mode: MachiningBoundaryMode) -> &'static str {
    match mode {
        MachiningBoundaryMode::StockBox => "Stock Bounds",
        MachiningBoundaryMode::ModelBoundingBox => "Model Bounds",
        MachiningBoundaryMode::ModelSilhouette => "Model Silhouette",
        MachiningBoundaryMode::SelectedRegions => "Selected 2D Contours",
    }
}

fn rotary_source_ui_label(mode: RotaryAxialBoundaryMode) -> &'static str {
    match mode {
        RotaryAxialBoundaryMode::StockBounds => "Stock Bounds",
        RotaryAxialBoundaryMode::ModelBounds => "Model Bounds",
        RotaryAxialBoundaryMode::SelectedGeometry => "Selected Geometry (-U..+U)",
        RotaryAxialBoundaryMode::ExplicitRange => "Explicit Range",
    }
}

fn rotary_containment_ui_label(mode: ToolContainmentMode) -> &'static str {
    match mode {
        ToolContainmentMode::ToolInsideBoundary => "Tool Inside",
        ToolContainmentMode::ToolCenterOnBoundary => "Tool Center",
        ToolContainmentMode::ToolOutsideBoundary => "Tool Outside",
    }
}

fn pocket_area_ui_label(mode: PocketBoundaryMode) -> &'static str {
    match mode {
        PocketBoundaryMode::SelectedPockets => "Inside Selected Pockets",
        PocketBoundaryMode::ClearOutsideSelectedToStock => "Clear Outside Selected Bosses to Stock",
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn show_machining_boundary_card(
    ui: &mut egui::Ui,
    op: &mut Operation,
    setup_idx: usize,
    op_idx: usize,
    display_unit: DisplayUnit,
    eval_ctx: &EvalContext,
    active_tool: Option<&Tool>,
    boundary_preview: Option<&BoundaryPreviewOverlay>,
    mutated_label: &mut Option<&'static str>,
) {
    let op_kind = op.kind;
    let selections_empty = op.selections.is_empty();
    let Some(boundary) = op.machining_boundary_mut() else {
        return;
    };

    section_card(
        ui,
        ("geom_cutting_area_card", setup_idx, op_idx),
        "Cutting Area / Machining Boundary",
        true,
        |ui| {
            inspector_grid(ui, ("geom_cutting_area_grid", setup_idx, op_idx), |ui| {
                grid_row(
                    ui,
                    "Boundary Source",
                    Some(
                        "2D XY region constraining this operation (Stock Bounds, Model Bounds rectangle, projected Model Silhouette, or Selected 2D Contours)",
                    ),
                    |ui| {
                        let prev = boundary.boundary_mode;
                        egui::ComboBox::from_id_salt(("geom_bmode", setup_idx, op_idx))
                            .width(COMBO_COL_WIDTH)
                            .truncate()
                            .selected_text(boundary_source_ui_label(boundary.boundary_mode))
                            .show_ui(ui, |ui| {
                                for m in MachiningBoundaryMode::ALL {
                                    ui.selectable_value(
                                        &mut boundary.boundary_mode,
                                        m,
                                        boundary_source_ui_label(m),
                                    )
                                    .on_hover_text(format!(
                                        "Constrain XY cutting area using {}",
                                        boundary_source_ui_label(m)
                                    ));
                                }
                            })
                            .response
                            .on_hover_text("Select 2D XY machining boundary source");
                        if boundary.boundary_mode != prev {
                            *mutated_label = Some("Change Machining Boundary Source");
                        }
                    },
                );

                if boundary.boundary_mode == MachiningBoundaryMode::ModelBoundingBox {
                    grid_row(
                        ui,
                        "-X Offset",
                        Some(
                            "Signed offset on the -X side of Model Bounds (+ expands outward toward -X, - insets inward toward +X)",
                        ),
                        |ui| {
                            if edit_param_length(
                                ui,
                                ("geom_mb_xneg", setup_idx, op_idx),
                                &mut boundary.model_bounds_offset_x_neg,
                                eval_ctx,
                                INPUT_COL_WIDTH,
                            ) {
                                *mutated_label = Some("Edit Model Bounds -X Offset");
                            }
                        },
                    );
                    grid_row(
                        ui,
                        "+X Offset",
                        Some(
                            "Signed offset on the +X side of Model Bounds (+ expands outward toward +X, - insets inward toward -X)",
                        ),
                        |ui| {
                            if edit_param_length(
                                ui,
                                ("geom_mb_xpos", setup_idx, op_idx),
                                &mut boundary.model_bounds_offset_x_pos,
                                eval_ctx,
                                INPUT_COL_WIDTH,
                            ) {
                                *mutated_label = Some("Edit Model Bounds +X Offset");
                            }
                        },
                    );
                    grid_row(
                        ui,
                        "-Y Offset",
                        Some(
                            "Signed offset on the -Y side of Model Bounds (+ expands outward toward -Y, - insets inward toward +Y)",
                        ),
                        |ui| {
                            if edit_param_length(
                                ui,
                                ("geom_mb_yneg", setup_idx, op_idx),
                                &mut boundary.model_bounds_offset_y_neg,
                                eval_ctx,
                                INPUT_COL_WIDTH,
                            ) {
                                *mutated_label = Some("Edit Model Bounds -Y Offset");
                            }
                        },
                    );
                    grid_row(
                        ui,
                        "+Y Offset",
                        Some(
                            "Signed offset on the +Y side of Model Bounds (+ expands outward toward +Y, - insets inward toward -Y)",
                        ),
                        |ui| {
                            if edit_param_length(
                                ui,
                                ("geom_mb_ypos", setup_idx, op_idx),
                                &mut boundary.model_bounds_offset_y_pos,
                                eval_ctx,
                                INPUT_COL_WIDTH,
                            ) {
                                *mutated_label = Some("Edit Model Bounds +Y Offset");
                            }
                        },
                    );
                }

                if boundary.boundary_mode == MachiningBoundaryMode::ModelSilhouette {
                    grid_row(
                        ui,
                        "Interior Holes",
                        Some(
                            "Fill interior through-holes in the projected Model Silhouette so 3D passes machine through voids instead of leaving uncut stock pillars",
                        ),
                        |ui| {
                            if ui
                                .checkbox(&mut boundary.silhouette_fill_holes, "Fill through-holes")
                                .on_hover_text(
                                    "Checked: machine across interior through-holes inside the outer silhouette. Unchecked: keep interior through-hole voids outside the cutting boundary.",
                                )
                                .changed()
                            {
                                *mutated_label = Some("Toggle Silhouette Fill Through-Holes");
                            }
                        },
                    );
                }

                grid_row(
                    ui,
                    "Tool Containment",
                    Some(
                        "Controls whether tool edge stays inside, center rides on, or edge extends outside the nominal boundary",
                    ),
                    |ui| {
                        let prev = boundary.tool_containment;
                        egui::ComboBox::from_id_salt(("geom_tcont", setup_idx, op_idx))
                            .width(COMBO_COL_WIDTH)
                            .truncate()
                            .selected_text(boundary.tool_containment.label())
                            .show_ui(ui, |ui| {
                                for c in CONTAINMENT_ORDER {
                                    ui.selectable_value(
                                        &mut boundary.tool_containment,
                                        c,
                                        c.label(),
                                    )
                                    .on_hover_text(format!("Position cutter {}", c.label()));
                                }
                            })
                            .response
                            .on_hover_text(
                                "Control cutter position relative to the machining boundary",
                            );
                        if boundary.tool_containment != prev {
                            *mutated_label = Some("Change Tool Containment Mode");
                        }
                    },
                );

                grid_row(
                    ui,
                    "Boundary Offset",
                    Some(
                        "Signed XY offset applied to the machining boundary (+ expands outward, - insets inward)",
                    ),
                    |ui| {
                        if edit_param_length(
                            ui,
                            ("geom_boff", setup_idx, op_idx),
                            &mut boundary.boundary_offset,
                            eval_ctx,
                            INPUT_COL_WIDTH,
                        ) {
                            *mutated_label = Some("Edit Boundary Offset");
                        }
                    },
                );
            });

            // Live resolved shift / XY span readout
            let tok = ThemeTokens::from_ui(ui);
            let tool_r = active_tool.map(|t| 0.5 * t.geometry.cutting_diameter_mm());
            let shift_str = tool_r.map(|r| {
                let c_shift = match boundary.tool_containment {
                    ToolContainmentMode::ToolInsideBoundary => -r,
                    ToolContainmentMode::ToolCenterOnBoundary => 0.0,
                    ToolContainmentMode::ToolOutsideBoundary => r,
                };
                let net = c_shift + boundary.boundary_offset.value_mm;
                let sign = if net > 1e-6 { "+" } else { "" };
                format!(
                    "Tool-Center Shift: {sign}{} (Ø{})",
                    display_unit.format_length(net),
                    display_unit.format_length(2.0 * r)
                )
            });
            let span_str = if let Some(BoundaryPreviewOverlay::Planar2D { nominal_region, .. }) =
                boundary_preview
                && let Some(aabb) = region_set_aabb(nominal_region)
            {
                let dx = aabb.max.x - aabb.min.x;
                let dy = aabb.max.y - aabb.min.y;
                Some(format!(
                    "XY: {} × {}",
                    display_unit.format_length(dx),
                    display_unit.format_length(dy)
                ))
            } else {
                None
            };
            if shift_str.is_some() || span_str.is_some() {
                ui.add_space(4.0);
                let summary = match (span_str, shift_str) {
                    (Some(a), Some(b)) => format!("{a}  ·  {b}"),
                    (Some(a), None) => a,
                    (None, Some(b)) => b,
                    (None, None) => String::new(),
                };
                ui.colored_label(tok.text_secondary, summary);
            }

            if boundary.boundary_mode == MachiningBoundaryMode::SelectedRegions && selections_empty
            {
                ui.add_space(4.0);
                inline_banner(
                    ui,
                    BannerKind::Warn,
                    "Selected 2D Contours is active with no geometry selected. Pick closed 2D contours, planar faces, or sketch regions below.",
                );
            } else if let Some(BoundaryPreviewOverlay::Planar2D { warnings, .. }) = boundary_preview
            {
                for w in warnings {
                    ui.add_space(4.0);
                    inline_banner(ui, BannerKind::Warn, w);
                }
            }

            if op_kind == OperationKind::Deburr3D {
                ui.add_space(4.0);
                inline_banner(
                    ui,
                    BannerKind::Info,
                    "XY Machining Boundary constrains Auto-Detect Edges mode. Pick explicit 3D edge loops, open chains, or faces below to override.",
                );
            } else if op_kind == OperationKind::Undercut3D {
                ui.add_space(4.0);
                inline_banner(
                    ui,
                    BannerKind::Info,
                    "Scans overhanging downward-facing surfaces within the Heights Z-band and XY Machining Boundary while trimming neck/shank collisions.",
                );
            }
        },
    );
}

#[allow(clippy::too_many_arguments)]
pub(super) fn show_rotary_cutting_area_card(
    ui: &mut egui::Ui,
    op: &mut Operation,
    setup_idx: usize,
    op_idx: usize,
    display_unit: DisplayUnit,
    eval_ctx: &EvalContext,
    cad_doc: Option<&CadDocument>,
    resolved_setup: Option<&ResolvedSetup>,
    active_tool: Option<&Tool>,
    surface_mesh: Option<&SurfaceMesh3D>,
    resolved_heights: Option<&ResolvedHeights>,
    boundary_preview: Option<&BoundaryPreviewOverlay>,
    extracted: Option<&ExtractedSelectionSummary>,
    mutated_label: &mut Option<&'static str>,
) {
    let selections_empty = op.selections.is_empty();
    let mut mode_after = RotaryAxialBoundaryMode::ModelBounds;
    let mut changed_any = false;

    section_card(
        ui,
        ("geom_rotary_cutting_area_card", setup_idx, op_idx),
        "4th-Axis Rotary Cutting Area (-U / +U)",
        true,
        |ui| {
            if let Some(rb) = op.rotary_axial_boundary_mut() {
                inspector_grid(ui, ("geom_rotary_area_grid", setup_idx, op_idx), |ui| {
                    grid_row(
                        ui,
                        "Axial Source",
                        Some(
                            "Source determining the nominal -U..+U axial span along the rotary centerline",
                        ),
                        |ui| {
                            let prev = rb.axial_boundary_mode;
                            egui::ComboBox::from_id_salt(("geom_rot_bmode", setup_idx, op_idx))
                                .width(COMBO_COL_WIDTH)
                                .truncate()
                                .selected_text(rotary_source_ui_label(rb.axial_boundary_mode))
                                .show_ui(ui, |ui| {
                                    for m in RotaryAxialBoundaryMode::ALL {
                                        ui.selectable_value(
                                            &mut rb.axial_boundary_mode,
                                            m,
                                            rotary_source_ui_label(m),
                                        )
                                        .on_hover_text(
                                            format!(
                                                "Bound rotary cut along -U/+U using {}",
                                                rotary_source_ui_label(m)
                                            ),
                                        );
                                    }
                                })
                                .response
                                .on_hover_text(
                                    "Select how -U..+U axial cutting limits are derived",
                                );
                            if rb.axial_boundary_mode != prev {
                                *mutated_label = Some("Change Rotary Axial Boundary Source");
                                changed_any = true;
                            }
                        },
                    );

                    grid_row(
                        ui,
                        "Axial Containment",
                        Some(
                            "Tool Inside keeps full cutter diameter inside -U/+U; Tool Center rides on -U/+U; Tool Outside extends by tool radius for shoulder relief (chuck-safe)",
                        ),
                        |ui| {
                            let prev = rb.axial_containment;
                            egui::ComboBox::from_id_salt(("geom_rot_tcont", setup_idx, op_idx))
                                .width(COMBO_COL_WIDTH)
                                .truncate()
                                .selected_text(rotary_containment_ui_label(rb.axial_containment))
                                .show_ui(ui, |ui| {
                                    for c in CONTAINMENT_ORDER {
                                        ui.selectable_value(
                                            &mut rb.axial_containment,
                                            c,
                                            rotary_containment_ui_label(c),
                                        )
                                        .on_hover_text(c.label());
                                    }
                                })
                                .response
                                .on_hover_text(
                                    "Position tool perimeter relative to -U / +U axial limits",
                                );
                            if rb.axial_containment != prev {
                                *mutated_label = Some("Change Rotary Axial Containment");
                                changed_any = true;
                            }
                        },
                    );

                    if rb.axial_boundary_mode == RotaryAxialBoundaryMode::ExplicitRange {
                        grid_row(
                            ui,
                            "Start U (-U)",
                            Some("Explicit starting coordinate along the rotary axis (-U end)"),
                            |ui| {
                                if edit_param_length(
                                    ui,
                                    ("geom_rot_u_start", setup_idx, op_idx),
                                    &mut rb.explicit_u_start,
                                    eval_ctx,
                                    INPUT_COL_WIDTH,
                                ) {
                                    *mutated_label = Some("Edit Rotary Explicit Start U");
                                    changed_any = true;
                                }
                            },
                        );
                        grid_row(
                            ui,
                            "End U (+U)",
                            Some("Explicit ending coordinate along the rotary axis (+U end)"),
                            |ui| {
                                if edit_param_length(
                                    ui,
                                    ("geom_rot_u_end", setup_idx, op_idx),
                                    &mut rb.explicit_u_end,
                                    eval_ctx,
                                    INPUT_COL_WIDTH,
                                ) {
                                    *mutated_label = Some("Edit Rotary Explicit End U");
                                    changed_any = true;
                                }
                            },
                        );
                    }

                    grid_row(
                        ui,
                        "Start Margin (-U)",
                        Some(
                            "Signed axial margin at the -U end: positive (> 0) trims inward, negative (< 0) extends outward for shoulder relief while strictly respecting the rotary chuck keep-out guard",
                        ),
                        |ui| {
                            if edit_param_length(
                                ui,
                                ("geom_rot_m_start", setup_idx, op_idx),
                                &mut rb.axial_margin_start,
                                eval_ctx,
                                INPUT_COL_WIDTH,
                            ) {
                                *mutated_label = Some("Edit Rotary Start Margin (-U)");
                                changed_any = true;
                            }
                        },
                    );

                    grid_row(
                        ui,
                        "End Margin (+U)",
                        Some(
                            "Signed axial margin at the +U end: positive (> 0) trims inward, negative (< 0) extends outward for shoulder relief while strictly respecting the rotary chuck keep-out guard",
                        ),
                        |ui| {
                            if edit_param_length(
                                ui,
                                ("geom_rot_m_end", setup_idx, op_idx),
                                &mut rb.axial_margin_end,
                                eval_ctx,
                                INPUT_COL_WIDTH,
                            ) {
                                *mutated_label = Some("Edit Rotary End Margin (+U)");
                                changed_any = true;
                            }
                        },
                    );

                    grid_row(
                        ui,
                        "Start Angle (°)",
                        Some(
                            "Starting angular boundary around the 4th rotary axis (0° to 360° = full circumference)",
                        ),
                        |ui| {
                            if ui
                                .add(
                                    egui::DragValue::new(&mut rb.start_angle_deg)
                                        .speed(2.0)
                                        .range(-360.0..=360.0)
                                        .suffix("°"),
                                )
                                .on_hover_text("Start angle around the 4th rotary axis")
                                .changed()
                            {
                                if rb.end_angle_deg <= rb.start_angle_deg + 1.0 {
                                    rb.end_angle_deg = rb.start_angle_deg + 1.0;
                                }
                                *mutated_label = Some("Edit Rotary Start Angle");
                                changed_any = true;
                            }
                        },
                    );

                    grid_row(
                        ui,
                        "End Angle (°)",
                        Some(
                            "Ending angular boundary around the 4th rotary axis (0° to 360° = full circumference)",
                        ),
                        |ui| {
                            if ui
                                .add(
                                    egui::DragValue::new(&mut rb.end_angle_deg)
                                        .speed(2.0)
                                        .range((rb.start_angle_deg + 1.0)..=720.0)
                                        .suffix("°"),
                                )
                                .on_hover_text("End angle around the 4th rotary axis")
                                .changed()
                            {
                                *mutated_label = Some("Edit Rotary End Angle");
                                changed_any = true;
                            }
                        },
                    );

                    grid_row(
                        ui,
                        "Angular Presets",
                        Some("Quick angular sector presets around the 4th rotary axis"),
                        |ui| {
                            for (lbl, s_deg, e_deg) in [
                                ("Full 360°", 0.0_f64, 360.0_f64),
                                ("0–180°", 0.0_f64, 180.0_f64),
                                ("±90°", -90.0_f64, 90.0_f64),
                            ] {
                                let active = (rb.start_angle_deg - s_deg).abs() < 0.05
                                    && (rb.end_angle_deg - e_deg).abs() < 0.05;
                                if ui
                                    .selectable_label(active, lbl)
                                    .on_hover_text(format!(
                                        "Set angular window to {s_deg:.0}°..{e_deg:.0}°"
                                    ))
                                    .clicked()
                                {
                                    rb.start_angle_deg = s_deg;
                                    rb.end_angle_deg = e_deg;
                                    *mutated_label = Some("Apply Rotary Angular Preset");
                                    changed_any = true;
                                }
                            }
                        },
                    );
                });
                mode_after = rb.axial_boundary_mode;
            }

            // Live resolved rotary window readout (reuse cached overlay unless mutated this frame)
            let resolved_window: Option<Result<ResolvedRotaryWindow, String>> = if !changed_any
                && let Some(BoundaryPreviewOverlay::RotarySector { window, .. }) = boundary_preview
            {
                Some(Ok(window.clone()))
            } else if let Some(rs) = resolved_setup {
                resolve_rotary_window_for_preview(
                    op,
                    rs,
                    cad_doc,
                    surface_mesh,
                    extracted,
                    active_tool,
                    resolved_heights,
                    eval_ctx,
                )
            } else {
                None
            };

            ui.add_space(5.0);
            let tok = ThemeTokens::from_ui(ui);
            match resolved_window {
                Some(Ok(win)) => {
                    let u_span_str = format!(
                        "Effective U: [{:.2}, {:.2}] {} · Chuck Guard: {:.2} {}",
                        display_unit.from_mm(win.u_start),
                        display_unit.from_mm(win.u_end),
                        display_unit.suffix(),
                        display_unit.from_mm(win.chuck_guard_u),
                        display_unit.suffix(),
                    );
                    ui.colored_label(tok.text_primary, u_span_str);
                    ui.colored_label(
                        tok.text_secondary,
                        format!(
                            "Sector: {:.0}°..{:.0}° · R: {:.2}..{:.2} {}",
                            win.start_angle_deg,
                            win.end_angle_deg,
                            display_unit.from_mm(win.r_floor),
                            display_unit.from_mm(win.r_ceiling),
                            display_unit.suffix(),
                        ),
                    );
                    if win.clamped_by_chuck {
                        ui.add_space(4.0);
                        inline_banner(
                            ui,
                            BannerKind::Warn,
                            format!(
                                "Clamped by Rotary Chuck Keep-Out Guard at {} (tool radius + 0.5 mm safety clearance).",
                                display_unit.format_length(win.chuck_guard_u)
                            ),
                        );
                    }
                }
                Some(Err(err)) => {
                    inline_banner(ui, BannerKind::Error, err);
                }
                None => {
                    if resolved_setup.and_then(|rs| rs.rotary).is_none() {
                        inline_banner(
                            ui,
                            BannerKind::Warn,
                            "4th-Axis Rotary is not enabled on the active Setup. Enable it in Setup → Rotary.",
                        );
                    }
                }
            }

            if mode_after == RotaryAxialBoundaryMode::SelectedGeometry && selections_empty {
                ui.add_space(4.0);
                inline_banner(
                    ui,
                    BannerKind::Warn,
                    "Selected Geometry (-U..+U) is active with no selections. Pick 3D faces, loops, edges, or sketches below (currently falling back to Model Bounds).",
                );
            }
        },
    );
}

pub(super) fn show_pocket_cutting_area_card(
    ui: &mut egui::Ui,
    op: &mut Operation,
    setup_idx: usize,
    op_idx: usize,
    mutated_label: &mut Option<&'static str>,
) {
    let selections_empty = op.selections.is_empty();
    let StrategyParams::Pocket2D(p) = op.strategy_mut() else {
        return;
    };

    section_card(
        ui,
        ("geom_pocket_area_card", setup_idx, op_idx),
        "Cutting Area / Machining Boundary",
        true,
        |ui| {
            inspector_grid(ui, ("geom_pocket_area_grid", setup_idx, op_idx), |ui| {
                grid_row(
                    ui,
                    "Pocket Area Mode",
                    Some(
                        "Clear inside selected pockets, or treat selected contours/faces as raised bosses and clear outside them to Setup Stock XY bounds",
                    ),
                    |ui| {
                        let prev = p.boundary_mode;
                        egui::ComboBox::from_id_salt(("geom_pkt_bmode", setup_idx, op_idx))
                            .width(COMBO_COL_WIDTH)
                            .truncate()
                            .selected_text(pocket_area_ui_label(p.boundary_mode))
                            .show_ui(ui, |ui| {
                                for m in PocketBoundaryMode::ALL {
                                    ui.selectable_value(
                                        &mut p.boundary_mode,
                                        m,
                                        pocket_area_ui_label(m),
                                    )
                                    .on_hover_text(pocket_area_ui_label(m));
                                }
                            })
                            .response
                            .on_hover_text("Choose whether to clear inside selected pockets or outside selected bosses to stock");
                        if p.boundary_mode != prev {
                            *mutated_label = Some("Change Pocket Area Mode");
                        }
                    },
                );

                if p.boundary_mode == PocketBoundaryMode::SelectedPockets {
                    grid_row(
                        ui,
                        "Open Outer Boundary",
                        Some(
                            "Treat the outer boundary of selected 2D sketches or closed loops as open air so the cutter enters smoothly from outside",
                        ),
                        |ui| {
                            if ui
                                .checkbox(
                                    &mut p.treat_outer_boundary_as_open,
                                    "Treat Outer Loop as Open Air",
                                )
                                .on_hover_text(
                                    "Allow horizontal open-air entry across the outer boundary of 2D sketch or closed-loop selections",
                                )
                                .changed()
                            {
                                *mutated_label = Some("Toggle Open Outer Boundary");
                            }
                        },
                    );
                }
            });

            if p.boundary_mode == PocketBoundaryMode::ClearOutsideSelectedToStock {
                ui.add_space(4.0);
                if selections_empty {
                    inline_banner(
                        ui,
                        BannerKind::Warn,
                        "Select one or more planar faces or closed 2D contours below to leave standing as raised bosses while clearing surrounding stock.",
                    );
                } else {
                    inline_banner(
                        ui,
                        BannerKind::Info,
                        "Clearing stock outside selected bosses out to Setup Stock XY bounds (shown in 3D viewport) with open-air entry.",
                    );
                }
            }
        },
    );
}

pub(super) fn show_contour_cutting_area_card(
    ui: &mut egui::Ui,
    op: &mut Operation,
    setup_idx: usize,
    op_idx: usize,
    eval_ctx: &EvalContext,
    mutated_label: &mut Option<&'static str>,
) {
    let StrategyParams::Contour2D(p) = op.strategy_mut() else {
        return;
    };

    section_card(
        ui,
        ("geom_contour_area_card", setup_idx, op_idx),
        "Cutting Area / Machining Boundary",
        true,
        |ui| {
            inspector_grid(ui, ("geom_contour_area_grid", setup_idx, op_idx), |ui| {
                grid_row(
                    ui,
                    "Stock Clip",
                    Some("Trim generated 2D contour passes to remain inside Setup Stock XY bounds"),
                    |ui| {
                        if ui
                            .checkbox(&mut p.clip_to_stock_bounds, "Clip to Stock Bounds")
                            .on_hover_text(
                                "Keep 2D contour toolpaths clipped inside Setup Stock XY bounds",
                            )
                            .changed()
                        {
                            *mutated_label = Some("Toggle Contour Stock Clipping");
                        }
                    },
                );
                grid_row(
                    ui,
                    "Open Chain Extension",
                    Some("Extend open 2D profile start and end tangentially past stock edges"),
                    |ui| {
                        if edit_param_length(
                            ui,
                            ("geom_contour_open_ext", setup_idx, op_idx),
                            &mut p.open_chain_extension,
                            eval_ctx,
                            INPUT_COL_WIDTH,
                        ) {
                            *mutated_label = Some("Edit Open Chain Extension");
                        }
                    },
                );
            });
        },
    );
}

fn region_set_aabb(region: &chip_cam::RegionSet2D) -> Option<chip_math::Aabb2> {
    region.exact_aabb()
}
