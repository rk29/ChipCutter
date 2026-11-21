use chip_cad::{CadDocument, MAX_MODEL_SCALE, MIN_MODEL_SCALE, ModelTransform};
use chip_math::{DVec3, DisplayUnit};
use chip_project::{ProjectDocument, UndoHistory};

use crate::{
    panels::{InspectorFocus, ViewportPickMode},
    theme::ThemeTokens,
    widgets::{BannerKind, grid_row, inline_banner, inspector_grid, section_card, static_card},
};

/// Context passed to [`show_model_inspector`].
pub struct ModelInspectorCtx<'a> {
    pub project: &'a mut ProjectDocument,
    pub model_idx: usize,
    pub history: &'a mut UndoHistory,
    pub cad_doc: Option<&'a CadDocument>,
    pub lock_uniform_scale: &'a mut bool,
    pub pick_mode: &'a mut ViewportPickMode,
    pub focus: &'a mut InspectorFocus,
}

/// Actions emitted by [`show_model_inspector`] for the parent `ChipCutterApp`.
#[derive(Debug, Default, Clone, Copy)]
pub struct ModelInspectorResult {
    pub transform_changed: bool,
    pub fit_camera_requested: bool,
}

/// Renders the **Model Transform, Orientation, & Scaling** Inspector panel for both STEP solids
/// and STL/OBJ/3MF triangle meshes.
pub fn show_model_inspector(ui: &mut egui::Ui, ctx: ModelInspectorCtx<'_>) -> ModelInspectorResult {
    let ModelInspectorCtx {
        project,
        model_idx,
        history,
        cad_doc,
        lock_uniform_scale,
        pick_mode,
        focus,
    } = ctx;

    let mut result = ModelInspectorResult::default();
    let Some(doc) = cad_doc else {
        ui.weak("No 3D STEP or mesh model loaded.");
        return result;
    };
    if model_idx >= project.models.len() {
        ui.weak("Model entry not found in project.");
        return result;
    }

    let unit = project.display_units;
    let tok = ThemeTokens::from_ui(ui);
    let is_mesh_only = !doc.meshes.is_empty() && doc.bodies.is_empty();
    let cur_aabb = doc.aabb_mm;
    let cur_size = cur_aabb.size();
    let pre_transform = project.models[model_idx].transform;
    let mut new_transform = pre_transform;
    let mut undo_label: Option<&'static str> = None;

    // 1. Model Info & Live Bounding Box header card
    static_card(ui, None, |ui| {
        ui.strong(&doc.source_name);
        let fmt_summary = if let Some(mesh) = doc.meshes.first() {
            format!(
                "{} · {} triangles · {} vertices",
                mesh.format.label(),
                mesh.diagnostics.valid_triangle_count,
                mesh.diagnostics.welded_vertex_count
            )
        } else {
            format!(
                "STEP B-Rep Solid · {} faces · {} loops · {} edges",
                doc.faces.len(),
                doc.loops.len(),
                doc.edges.len()
            )
        };
        ui.colored_label(tok.text_muted, fmt_summary);
        ui.add_space(3.0);
        ui.horizontal_wrapped(|ui| {
            if ui
                .small_button("Fit View")
                .on_hover_text("Frame transformed model in 3D viewport (F)")
                .clicked()
            {
                result.fit_camera_requested = true;
            }
            if ui
                .small_button("Inspect Entities")
                .on_hover_text("Switch to B-Rep / Mesh Entity Inspector")
                .clicked()
            {
                *focus = InspectorFocus::CadEntity;
                *pick_mode = ViewportPickMode::InspectEntity;
            }
        });
        ui.add_space(4.0);
        let (u_scale, u_sfx, prec) = match unit {
            DisplayUnit::Millimeter => (1.0, "mm", 1usize),
            DisplayUnit::Inch => (1.0 / 25.4, "in", 3usize),
        };
        inspector_grid(ui, ("model_bbox_grid", model_idx), |ui| {
            grid_row(
                ui,
                "Size (X × Y × Z)",
                Some("Live transformed bounding box in active display units"),
                |ui| {
                    ui.monospace(format!(
                        "{:.prec$}×{:.prec$}×{:.prec$} {u_sfx}",
                        cur_size.x * u_scale,
                        cur_size.y * u_scale,
                        cur_size.z * u_scale,
                    ));
                },
            );
            grid_row(
                ui,
                "Z Range",
                Some("Lowest to highest Z coordinates of the transformed model"),
                |ui| {
                    ui.monospace(format!(
                        "[{:.prec$} .. {:.prec$}] {u_sfx}",
                        cur_aabb.min.z * u_scale,
                        cur_aabb.max.z * u_scale,
                    ));
                },
            );
        });
    });

    ui.add_space(5.0);

    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            let is_picking_lay_flat =
                *pick_mode == ViewportPickMode::PickModelLayFlatFace { model_idx };
            if is_picking_lay_flat && let Some(msg) = pick_mode.banner_text() {
                inline_banner(ui, BannerKind::Info, msg);
                ui.add_space(5.0);
            }

            // 2. Quick Orientation card
            section_card(
                ui,
                ("model_orient_card", model_idx),
                "Quick Orientation",
                true,
                |ui| {
                    let pivot = if cur_aabb.is_valid() {
                        cur_aabb.center()
                    } else {
                        DVec3::ZERO
                    };
                    ui.horizontal_wrapped(|ui| {
                        if ui
                            .button("Y-Up -> Z-Up (+90° X)")
                            .on_hover_text(
                                "Convert Y-up exports (Blender, 3D print STLs) to CNC Z-up (+90° around X)",
                            )
                            .clicked()
                        {
                            if pre_transform.is_identity() {
                                new_transform = ModelTransform::y_up_to_z_up();
                            } else {
                                new_transform =
                                    pre_transform.rotated_world_axis_deg(DVec3::X, 90.0, pivot);
                            }
                            undo_label = Some("Convert Model Y-Up to Z-Up");
                            result.fit_camera_requested = true;
                        }

                        let lay_flat_btn = ui
                            .selectable_label(is_picking_lay_flat, "Lay Flat on Clicked Face…")
                            .on_hover_text(
                                "Click any planar STEP face or STL triangle in the 3D viewport to lay it flat (-Z) on Z=0",
                            );
                        if lay_flat_btn.clicked() {
                            *pick_mode = if is_picking_lay_flat {
                                ViewportPickMode::InspectEntity
                            } else {
                                ViewportPickMode::PickModelLayFlatFace { model_idx }
                            };
                        }
                    });

                    ui.add_space(4.0);
                    inspector_grid(ui, ("model_rot90_grid", model_idx), |ui| {
                        for (axis_idx, axis_vec, axis_lbl) in [
                            (0, DVec3::X, "Rotate X"),
                            (1, DVec3::Y, "Rotate Y"),
                            (2, DVec3::Z, "Rotate Z"),
                        ] {
                            grid_row(
                                ui,
                                axis_lbl,
                                Some("Increment world-axis rotation by ±90° around model center"),
                                |ui| {
                                    if ui
                                        .small_button("-90°")
                                        .on_hover_text(format!("Rotate model -90 degrees around the {axis_lbl} axis"))
                                        .clicked()
                                    {
                                        new_transform = pre_transform.rotated_world_axis_deg(
                                            axis_vec, -90.0, pivot,
                                        );
                                        undo_label = Some("Rotate Model -90°");
                                    }
                                    if ui
                                        .small_button("+90°")
                                        .on_hover_text(format!("Rotate model +90 degrees around the {axis_lbl} axis"))
                                        .clicked()
                                    {
                                        new_transform = pre_transform.rotated_world_axis_deg(
                                            axis_vec, 90.0, pivot,
                                        );
                                        undo_label = Some("Rotate Model +90°");
                                    }
                                    ui.colored_label(
                                        tok.text_muted,
                                        format!("{:.1}°", new_transform.rotate_deg[axis_idx]),
                                    );
                                },
                            );
                        }
                    });
                },
            );

            // 3. Scale & Target Dimensions card
            section_card(
                ui,
                ("model_scale_card", model_idx),
                "Scale & Target Dimensions",
                true,
                |ui| {
                    ui.horizontal(|ui| {
                        if ui
                            .checkbox(lock_uniform_scale, "Lock Uniform Scale")
                            .on_hover_text(
                                "Scale X, Y, and Z proportionally (recommended for STEP solids with cylindrical holes)",
                            )
                            .changed()
                            && *lock_uniform_scale
                            && new_transform.uniform_scale().is_none()
                        {
                            let u_scale = new_transform.scale[0];
                            new_transform = new_transform.with_uniform_scale(u_scale);
                            undo_label = Some("Lock Uniform Model Scale");
                        }
                    });

                    ui.add_space(3.0);
                    inspector_grid(ui, ("model_scale_grid", model_idx), |ui| {
                        if *lock_uniform_scale {
                            grid_row(
                                ui,
                                "Uniform Scale",
                                Some("Proportional scale multiplier (1.0 = 100%)"),
                                |ui| {
                                    let mut s = new_transform.scale[0];
                                    let resp = ui
                                        .add(
                                            egui::DragValue::new(&mut s)
                                                .speed(0.01)
                                                .range(MIN_MODEL_SCALE..=MAX_MODEL_SCALE)
                                                .suffix("×")
                                                .max_decimals(4),
                                        )
                                        .on_hover_text("Proportional scale multiplier across X, Y, and Z (1.0x = original size)");
                                    if resp.changed() && s.is_finite() {
                                        new_transform = new_transform.with_uniform_scale(s);
                                        undo_label = Some("Scale Model Uniformly");
                                    }
                                    ui.colored_label(tok.text_muted, format!("{:.1}%", s * 100.0));
                                },
                            );
                        } else {
                            for (axis_idx, lbl) in [(0, "Scale X"), (1, "Scale Y"), (2, "Scale Z")] {
                                grid_row(
                                    ui,
                                    lbl,
                                    Some("Independent scale factor along this local model axis"),
                                    |ui| {
                                        let mut s = new_transform.scale[axis_idx];
                                        let resp = ui
                                            .add(
                                                egui::DragValue::new(&mut s)
                                                    .speed(0.01)
                                                    .range(MIN_MODEL_SCALE..=MAX_MODEL_SCALE)
                                                    .suffix("×")
                                                    .max_decimals(4),
                                            )
                                            .on_hover_text(format!("Independent scale factor along {lbl}"));
                                        if resp.changed() && s.is_finite() {
                                            new_transform.scale[axis_idx] =
                                                s.clamp(MIN_MODEL_SCALE, MAX_MODEL_SCALE);
                                            undo_label = Some("Scale Model Non-Uniformly");
                                        }
                                    },
                                );
                            }
                        }

                        grid_row(
                            ui,
                            "Presets",
                            Some("Quick uniform scale multipliers & inch/mm unit rescue"),
                            |ui| {
                                ui.horizontal_wrapped(|ui| {
                                    ui.spacing_mut().item_spacing.x = 4.0;
                                    for (lbl, tip, factor, is_abs) in [
                                        ("0.5×", "Half size (×0.5)", 0.5, false),
                                        ("2×", "Double size (×2.0)", 2.0, false),
                                        ("×25.4", "Convert inches to millimeters (×25.4)", 25.4, false),
                                        ("/25.4", "Convert millimeters to inches (÷25.4)", 1.0 / 25.4, false),
                                        ("1×", "Reset scale to 1.0× (100%)", 1.0, true),
                                    ] {
                                        if ui.small_button(lbl).on_hover_text(tip).clicked() {
                                            if is_abs {
                                                new_transform =
                                                    new_transform.with_uniform_scale(1.0);
                                            } else {
                                                new_transform.scale = [
                                                    (new_transform.scale[0] * factor)
                                                        .clamp(MIN_MODEL_SCALE, MAX_MODEL_SCALE),
                                                    (new_transform.scale[1] * factor)
                                                        .clamp(MIN_MODEL_SCALE, MAX_MODEL_SCALE),
                                                    (new_transform.scale[2] * factor)
                                                        .clamp(MIN_MODEL_SCALE, MAX_MODEL_SCALE),
                                                ];
                                            }
                                            undo_label = Some("Apply Model Scale Preset");
                                            result.fit_camera_requested = true;
                                        }
                                    }
                                });
                            },
                        );

                        // Target Size X / Y / Z (auto-computes exact uniform scale to fit stock)
                        let (unit_scale, suffix, max_dec) = match unit {
                            DisplayUnit::Millimeter => (1.0, " mm", 2),
                            DisplayUnit::Inch => (1.0 / 25.4, " in", 3),
                        };
                        for (axis_idx, lbl, cur_mm) in [
                            (0, "Target Size X", cur_size.x),
                            (1, "Target Size Y", cur_size.y),
                            (2, "Target Size Z", cur_size.z),
                        ] {
                            grid_row(
                                ui,
                                lbl,
                                Some(
                                    "Type a desired finished dimension to automatically scale the model to fit",
                                ),
                                |ui| {
                                    let mut disp_val = (cur_mm * unit_scale).max(0.001);
                                    let resp = ui
                                        .add(
                                            egui::DragValue::new(&mut disp_val)
                                                .speed(if unit == DisplayUnit::Inch {
                                                    0.05
                                                } else {
                                                    0.5
                                                })
                                                .range(0.01..=100_000.0)
                                                .suffix(suffix)
                                                .max_decimals(max_dec),
                                        )
                                        .on_hover_text(format!("Enter desired {lbl} to uniformly scale the model to match"));
                                    if resp.changed() && disp_val.is_finite() && disp_val > 1e-4 {
                                        let target_mm = disp_val / unit_scale;
                                        let fitted = pre_transform.scaled_uniformly_to_fit_axis(
                                            cur_aabb, axis_idx, target_mm,
                                        );
                                        if fitted != pre_transform {
                                            new_transform = fitted;
                                            undo_label = Some("Scale Model to Target Size");
                                        }
                                    }
                                },
                            );
                        }
                    });

                    if !is_mesh_only && new_transform.uniform_scale().is_none() {
                        ui.add_space(4.0);
                        inline_banner(
                            ui,
                            BannerKind::Warn,
                            "Non-uniform scaling on a STEP solid stretches circular holes & arcs into general NURBS surfaces. Keep 'Lock Uniform Scale' enabled if you plan to drill or bore circular features.",
                        );
                    }
                },
            );

            // 4. Quick Placement card
            section_card(
                ui,
                ("model_place_card", model_idx),
                "Quick Placement",
                true,
                |ui| {
                    ui.horizontal_wrapped(|ui| {
                        if ui
                            .button("Center XY on Origin")
                            .on_hover_text("Center model bounding box at X=0, Y=0")
                            .clicked()
                        {
                            new_transform = pre_transform.centered_xy_for_aabb(cur_aabb);
                            undo_label = Some("Center Model XY");
                            result.fit_camera_requested = true;
                        }
                        if ui
                            .button("Drop Bottom to Z = 0")
                            .on_hover_text("Shift model vertically so its lowest point rests at Z = 0")
                            .clicked()
                        {
                            new_transform = pre_transform.dropped_bottom_to_z0_for_aabb(cur_aabb);
                            undo_label = Some("Drop Model Bottom to Z=0");
                            result.fit_camera_requested = true;
                        }
                        if ui
                            .button("Align Top to Z = 0")
                            .on_hover_text("Shift model vertically so its top face sits at Z = 0")
                            .clicked()
                        {
                            new_transform = pre_transform.aligned_top_to_z0_for_aabb(cur_aabb);
                            undo_label = Some("Align Model Top to Z=0");
                            result.fit_camera_requested = true;
                        }
                        if ui
                            .add_enabled(
                                !pre_transform.is_identity(),
                                egui::Button::new("Reset Transform"),
                            )
                            .on_hover_text("Restore raw untransformed CAD / STL coordinates")
                            .clicked()
                        {
                            new_transform = ModelTransform::IDENTITY;
                            undo_label = Some("Reset Model Transform");
                            result.fit_camera_requested = true;
                        }
                    });
                },
            );

            // 5. Exact Rotation & Translation card
            section_card(
                ui,
                ("model_exact_card", model_idx),
                "Exact Rotation & Translation",
                true,
                |ui| {
                    let (unit_scale, suffix, max_dec, drag_speed) = match unit {
                        DisplayUnit::Millimeter => (1.0, " mm", 3, 0.5),
                        DisplayUnit::Inch => (1.0 / 25.4, " in", 4, 0.02),
                    };
                    inspector_grid(ui, ("model_exact_grid", model_idx), |ui| {
                        for (axis_idx, lbl) in [
                            (0, "Rotate X (°)"),
                            (1, "Rotate Y (°)"),
                            (2, "Rotate Z (°)"),
                        ] {
                            grid_row(
                                ui,
                                lbl,
                                Some("Intrinsic XYZ Euler rotation angle in degrees (applied after Scale)"),
                                |ui| {
                                    let mut deg = new_transform.rotate_deg[axis_idx];
                                    let resp = ui
                                        .add(
                                            egui::DragValue::new(&mut deg)
                                                .speed(0.5)
                                                .range(-360.0..=360.0)
                                                .suffix("°")
                                                .max_decimals(2),
                                        )
                                        .on_hover_text(format!("Exact {lbl} Euler rotation angle in degrees"));
                                    if resp.changed() && deg.is_finite() {
                                        new_transform.rotate_deg[axis_idx] = deg;
                                        undo_label = Some("Edit Model Rotation");
                                    }
                                },
                            );
                        }
                        for (axis_idx, lbl) in [(0, "Move X"), (1, "Move Y"), (2, "Move Z")] {
                            grid_row(
                                ui,
                                lbl,
                                Some("World translation offset applied after Scale and Rotation"),
                                |ui| {
                                    let mut disp = new_transform.translate_mm[axis_idx] * unit_scale;
                                    let resp = ui
                                        .add(
                                            egui::DragValue::new(&mut disp)
                                                .speed(drag_speed)
                                                .range(-100_000.0..=100_000.0)
                                                .suffix(suffix)
                                                .max_decimals(max_dec),
                                        )
                                        .on_hover_text(format!("Exact world {lbl} translation offset"));
                                    if resp.changed() && disp.is_finite() {
                                        new_transform.translate_mm[axis_idx] = disp / unit_scale;
                                        undo_label = Some("Edit Model Translation");
                                    }
                                },
                            );
                        }
                    });
                },
            );

            ui.add_space(4.0);
            inline_banner(
                ui,
                BannerKind::Info,
                "Model Transform fixes raw STEP/STL export orientation, size, and base position non-destructively across the whole project (preserved on Ctrl+R reload). Use Setup -> WCS & Post for per-operation fixture datums (G54 Op-1 / G55 Op-2 flip).",
            );
        });

    if let Some(action_lbl) = undo_label
        && new_transform != pre_transform
        && new_transform.validate().is_ok()
    {
        history.record_before_edit(action_lbl, None, project);
        project.models[model_idx].transform = new_transform;
        for setup in &mut project.setups {
            for op in &mut setup.operations {
                op.mark_dirty();
            }
        }
        result.transform_changed = true;
    }

    result
}
