//! Parametric 2D Sketch & Entity Inspector (`11-sketches-vectors.md`):
//! - Anchoring (`SetupXy` vs. associative `PlanarFace`), `Look At Sketch` camera button, and 2D Snap Grid controls;
//! - Interactive viewport draw tool toggles (`Rect`, `Circle`, `Slot`, `Polygon`, `Bolt Circle`, `Text`) + 1-click presets;
//! - Full expression-backed parametric editors for all 7 `SketchEntity` variants + TrueType/Hershey typography controls;
//! - Instant `rebuild_regions()` + Undo/Redo snapshot recording.

use std::path::Path;

use chip_cam::{
    ChamferTraceMode, ContourCompensation, GeometrySelectionItem, Operation, OperationKind,
    StrategyParams,
};
use chip_math::{EvalContext, ParamLength};
use chip_project::{ProjectDocument, UndoHistory, portable_model_path};
use chip_sketch::{
    FaceSketchOriginMode, HersheyFontKind, PointPatternSpec, PolygonSizingMode, RectAnchorMode,
    SketchAnchor, SketchEntity, SketchEntityEntry, SketchFontSource, SketchPlaneRef,
    SketchTextSpec, SlotLengthMode, TextHAlign, TextVAlign, compute_sketch_sha256_hex,
};

use crate::{
    panels::{SketchAuthorTool, ViewportPickMode},
    widgets::edit_param_length,
};

/// High-level action requested by [`show_sketch_inspector`].
#[derive(Debug, Clone)]
pub enum SketchInspectorAction {
    LookAtSketch(usize),
    InvalidateFrameCache,
    FocusOperation { setup_idx: usize, op_idx: usize },
    Log(String),
}

/// Context passed into [`show_sketch_inspector`].
pub struct SketchInspectorCtx<'a> {
    pub project: &'a mut ProjectDocument,
    pub sketch_idx: usize,
    pub active_setup_idx: usize,
    pub selected_entity_idx: &'a mut Option<usize>,
    pub snap_grid_mm: &'a mut Option<f64>,
    pub author_start_xy: &'a mut Option<chip_math::DVec2>,
    pub history: &'a mut UndoHistory,
    pub pick_mode: &'a mut ViewportPickMode,
    pub eval_ctx: &'a EvalContext,
    pub has_cad_model: bool,
    pub project_file: Option<&'a Path>,
}

/// Renders the parametric **2D Sketch & Entity Inspector** pane.
pub fn show_sketch_inspector(
    ui: &mut egui::Ui,
    ctx: SketchInspectorCtx<'_>,
) -> Option<SketchInspectorAction> {
    let SketchInspectorCtx {
        project,
        sketch_idx,
        active_setup_idx,
        selected_entity_idx,
        snap_grid_mm,
        author_start_xy,
        history,
        pick_mode,
        eval_ctx,
        has_cad_model,
        project_file,
    } = ctx;

    if sketch_idx >= project.sketches.len() {
        ui.weak("No Sketch Layer selected.");
        return None;
    }

    let u = project.display_units;
    let pre_project = project.clone();
    let mut undo_info: Option<(&'static str, Option<String>)> = None;
    let mut rebuild_needed = false;
    let mut action: Option<SketchInspectorAction> = None;

    egui::ScrollArea::vertical().show(ui, |ui| {
        // 1. Header + Visibility + Look At Sketch
        ui.horizontal(|ui| {
            ui.heading("2D Sketch");
            let sk = &mut project.sketches[sketch_idx];
            if ui.checkbox(&mut sk.visible, "Visible").changed() {
                undo_info = Some(("Toggle Sketch Visibility", None));
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui
                    .button("Look At Sketch")
                    .on_hover_text(
                        "Align orthographic camera perpendicular to this Sketch plane",
                    )
                    .clicked()
                {
                    action = Some(SketchInspectorAction::LookAtSketch(sketch_idx));
                }
            });
        });
        ui.separator();

        // 2. Sketch Name, Anchor Mode, & Snap Grid
        {
            let sk = &mut project.sketches[sketch_idx];
            ui.horizontal(|ui| {
                ui.label("Name:");
                let prev_name = sk.name.clone();
                if ui.text_edit_singleline(&mut sk.name).changed() && sk.name != prev_name {
                    undo_info = Some(("Rename Sketch", Some(format!("sk_name_{sketch_idx}"))));
                }
            });

            match &mut sk.anchor {
                SketchAnchor::SetupXy {
                    plane_ref,
                    z_offset,
                    ..
                } => {
                    ui.horizontal(|ui| {
                        ui.label("Anchor:");
                        egui::ComboBox::from_id_salt(("sk_plane_ref", sketch_idx))
                            .selected_text(format!("Setup XY — {}", plane_ref.label()))
                            .show_ui(ui, |ui| {
                                for r in SketchPlaneRef::ALL {
                                    if ui.selectable_value(plane_ref, r, r.label()).changed() {
                                        undo_info = Some(("Change Sketch Plane", None));
                                    }
                                }
                            });
                    });
                    ui.horizontal(|ui| {
                        ui.label("Z Offset:");
                        if edit_param_length(
                            ui,
                            ("sk_z_off_xy", sketch_idx),
                            z_offset,
                            eval_ctx,
                            110.0,
                        ) {
                            undo_info = Some((
                                "Edit Sketch Z Offset",
                                Some(format!("sk_zoff_{sketch_idx}")),
                            ));
                        }
                        if has_cad_model
                            && ui
                                .small_button("Attach to 3D Face…")
                                .on_hover_text(
                                    "Click a planar STEP B-Rep face in the 3D viewport to anchor this sketch associatively",
                                )
                                .clicked()
                        {
                            *pick_mode = ViewportPickMode::RebindSketchHostFace { sketch_idx };
                        }
                    });
                }
                SketchAnchor::PlanarFace {
                    setup_id,
                    face_binding,
                    origin_mode,
                    x_axis_clock_deg,
                    z_offset,
                } => {
                    let mut switch_to_setup_xy: Option<String> = None;
                    ui.horizontal(|ui| {
                        ui.strong(format!(
                            "Anchor: 3D Planar Face #{}",
                            face_binding.last_face_index
                        ));
                        if has_cad_model && ui.small_button("Re-Pick…").clicked() {
                            *pick_mode = ViewportPickMode::RebindSketchHostFace { sketch_idx };
                        }
                        if ui.small_button("Use Setup XY").clicked() {
                            switch_to_setup_xy = Some(setup_id.clone());
                        }
                    });
                    ui.horizontal(|ui| {
                        ui.label("Origin:");
                        egui::ComboBox::from_id_salt(("sk_face_orig", sketch_idx))
                            .selected_text(origin_mode.label())
                            .show_ui(ui, |ui| {
                                for m in FaceSketchOriginMode::ALL {
                                    if ui.selectable_value(origin_mode, m, m.label()).changed() {
                                        undo_info = Some(("Change Face Sketch Origin", None));
                                    }
                                }
                            });
                    });
                    ui.horizontal(|ui| {
                        ui.label("+X Clock:");
                        if ui
                            .add(
                                egui::DragValue::new(x_axis_clock_deg)
                                    .speed(1.0)
                                    .suffix("°")
                                    .range(-180.0..=180.0),
                            )
                            .changed()
                        {
                            undo_info = Some((
                                "Clock Face Sketch +X",
                                Some(format!("sk_clk_{sketch_idx}")),
                            ));
                        }
                        ui.label("Z Off:");
                        if edit_param_length(
                            ui,
                            ("sk_z_off_face", sketch_idx),
                            z_offset,
                            eval_ctx,
                            85.0,
                        ) {
                            undo_info = Some((
                                "Edit Sketch Z Offset",
                                Some(format!("sk_zoff_{sketch_idx}")),
                            ));
                        }
                    });
                    if let Some(sid) = switch_to_setup_xy {
                        sk.anchor = SketchAnchor::SetupXy {
                            setup_id: sid,
                            plane_ref: SketchPlaneRef::StockTop,
                            z_offset: ParamLength::from_mm(0.0),
                        };
                        undo_info = Some(("Switch Sketch to Setup XY", None));
                    }
                }
            }

            ui.horizontal(|ui| {
                ui.label("2D Snap Grid:");
                for (lbl, val) in [
                    ("Off", None),
                    ("1 mm", Some(1.0)),
                    ("5 mm", Some(5.0)),
                    ("10 mm", Some(10.0)),
                ] {
                    if ui.selectable_label(*snap_grid_mm == val, lbl).clicked() {
                        *snap_grid_mm = val;
                    }
                }
            });
        }

        // 3. Interactive Viewport Authoring Toolbar + 1-Click Quick Primitives
        ui.separator();
        ui.strong("Draw in Viewport (2D Snap Enabled)");
        ui.horizontal_wrapped(|ui| {
            for tool in SketchAuthorTool::ALL {
                let is_active = *pick_mode
                    == ViewportPickMode::SketchAuthor {
                        sketch_idx,
                        tool,
                    };
                if ui
                    .selectable_label(is_active, format!("+ {}", tool.short_label()))
                    .on_hover_text(tool.tooltip())
                    .clicked()
                {
                    *author_start_xy = None;
                    if is_active {
                        *pick_mode = ViewportPickMode::InspectEntity;
                    } else {
                        *pick_mode = ViewportPickMode::SketchAuthor { sketch_idx, tool };
                    }
                }
            }
        });
        ui.horizontal_wrapped(|ui| {
            ui.weak("Quick Insert at (0, 0):");
            let sk = &mut project.sketches[sketch_idx];
            if ui.small_button("Rect").clicked() {
                let n = sk.entities.len() + 1;
                sk.entities.push(SketchEntityEntry {
                    id: format!("rect_{n}"),
                    name: format!("Rectangle {n}"),
                    sub_layer: None,
                    is_construction: false,
                    entity: SketchEntity::Rectangle {
                        anchor: RectAnchorMode::Center,
                        x: ParamLength::from_mm(0.0),
                        y: ParamLength::from_mm(0.0),
                        width: ParamLength::from_mm(50.0),
                        height: ParamLength::from_mm(30.0),
                        corner_radius: ParamLength::from_mm(4.0),
                        rotation_deg: 0.0,
                    },
                });
                *selected_entity_idx = Some(sk.entities.len() - 1);
                undo_info = Some(("Add Sketch Rectangle", None));
                rebuild_needed = true;
            }
            if ui.small_button("Circle").clicked() {
                let n = sk.entities.len() + 1;
                sk.entities.push(SketchEntityEntry {
                    id: format!("circle_{n}"),
                    name: format!("Circle {n}"),
                    sub_layer: None,
                    is_construction: false,
                    entity: SketchEntity::new_circle(
                        ParamLength::from_mm(0.0),
                        ParamLength::from_mm(0.0),
                        ParamLength::from_mm(12.0),
                    ),
                });
                *selected_entity_idx = Some(sk.entities.len() - 1);
                undo_info = Some(("Add Sketch Circle", None));
                rebuild_needed = true;
            }
            if ui.small_button("Slot").clicked() {
                let n = sk.entities.len() + 1;
                sk.entities.push(SketchEntityEntry {
                    id: format!("slot_{n}"),
                    name: format!("Slot {n}"),
                    sub_layer: None,
                    is_construction: false,
                    entity: SketchEntity::Slot {
                        mode: SlotLengthMode::CenterToCenter,
                        x: ParamLength::from_mm(0.0),
                        y: ParamLength::from_mm(0.0),
                        length: ParamLength::from_mm(24.0),
                        width: ParamLength::from_mm(8.0),
                        angle_deg: 0.0,
                    },
                });
                *selected_entity_idx = Some(sk.entities.len() - 1);
                undo_info = Some(("Add Sketch Slot", None));
                rebuild_needed = true;
            }
            if ui.small_button("Hex").clicked() {
                let n = sk.entities.len() + 1;
                sk.entities.push(SketchEntityEntry {
                    id: format!("poly_{n}"),
                    name: format!("Hexagon {n}"),
                    sub_layer: None,
                    is_construction: false,
                    entity: SketchEntity::RegularPolygon {
                        center_x: ParamLength::from_mm(0.0),
                        center_y: ParamLength::from_mm(0.0),
                        sides: 6,
                        sizing: PolygonSizingMode::AcrossFlats,
                        diameter: ParamLength::from_mm(19.0),
                        corner_radius: ParamLength::from_mm(0.0),
                        rotation_deg: 0.0,
                    },
                });
                *selected_entity_idx = Some(sk.entities.len() - 1);
                undo_info = Some(("Add Sketch Hexagon", None));
                rebuild_needed = true;
            }
            if ui.small_button("Bolt Circle").clicked() {
                let n = sk.entities.len() + 1;
                sk.entities.push(SketchEntityEntry {
                    id: format!("bolts_{n}"),
                    name: format!("Bolt Circle {n}"),
                    sub_layer: Some("HOLES".to_string()),
                    is_construction: false,
                    entity: SketchEntity::PointPattern(PointPatternSpec::BoltCircle {
                        center_x: ParamLength::from_mm(0.0),
                        center_y: ParamLength::from_mm(0.0),
                        bolt_circle_diameter: ParamLength::from_mm(40.0),
                        hole_diameter: ParamLength::from_mm(6.0),
                        count: 6,
                        start_angle_deg: 0.0,
                    }),
                });
                *selected_entity_idx = Some(sk.entities.len() - 1);
                undo_info = Some(("Add Bolt Circle", None));
                rebuild_needed = true;
            }
            if ui.small_button("Text").clicked() {
                let n = sk.entities.len() + 1;
                sk.entities.push(SketchEntityEntry {
                    id: format!("text_{n}"),
                    name: format!("Text {n}"),
                    sub_layer: None,
                    is_construction: false,
                    entity: SketchEntity::Text(SketchTextSpec::default()),
                });
                *selected_entity_idx = Some(sk.entities.len() - 1);
                undo_info = Some(("Add CAM Text", None));
                rebuild_needed = true;
            }
        });

        // 4. Entities List & Parametric Property Editor
        ui.separator();
        let ent_len = project.sketches[sketch_idx].entities.len();
        if let Some(sel) = *selected_entity_idx
            && sel >= ent_len
        {
            *selected_entity_idx = ent_len.checked_sub(1);
        }
        if selected_entity_idx.is_none() && ent_len > 0 {
            *selected_entity_idx = Some(0);
        }

        ui.strong(format!("Entities ({ent_len})"));
        if ent_len == 0 {
            ui.weak("Click a Draw tool above or Quick Insert to add 2D primitives or CAM text.");
        } else {
            let mut remove_idx: Option<usize> = None;
            let mut dup_idx: Option<usize> = None;
            egui::ScrollArea::vertical()
                .id_salt(("sk_ent_scroll", sketch_idx))
                .max_height(155.0)
                .show(ui, |ui| {
                    let sk = &project.sketches[sketch_idx];
                    for (ei, entry) in sk.entities.iter().enumerate() {
                        let is_sel = *selected_entity_idx == Some(ei);
                        ui.horizontal(|ui| {
                            let tag = if entry.is_construction { "[C] " } else { "" };
                            let row_lbl = format!(
                                "{tag}[{}] {}",
                                entry.entity.kind_label(),
                                entry.name
                            );
                            let avail_w = (ui.available_width() - 56.0).max(60.0);
                            if ui
                                .add_sized(
                                    [avail_w, 18.0],
                                    egui::Button::selectable(is_sel, row_lbl).truncate(),
                                )
                                .clicked()
                            {
                                *selected_entity_idx = Some(ei);
                            }
                            ui.with_layout(
                                egui::Layout::right_to_left(egui::Align::Center),
                                |ui| {
                                    if ui
                                        .small_button("×")
                                        .on_hover_text("Delete entity")
                                        .clicked()
                                    {
                                        remove_idx = Some(ei);
                                    }
                                    if ui
                                        .small_button("Dup")
                                        .on_hover_text("Duplicate entity")
                                        .clicked()
                                    {
                                        dup_idx = Some(ei);
                                    }
                                },
                            );
                        });
                    }
                });

            if let Some(di) = dup_idx {
                let sk = &mut project.sketches[sketch_idx];
                let mut cloned = sk.entities[di].clone();
                let next_n = sk.entities.len() + 1;
                cloned.id = format!("{}_copy_{next_n}", cloned.id);
                cloned.name = format!("{} Copy", cloned.name);
                sk.entities.push(cloned);
                *selected_entity_idx = Some(sk.entities.len() - 1);
                undo_info = Some(("Duplicate Sketch Entity", None));
                rebuild_needed = true;
            }
            if let Some(ri) = remove_idx {
                let sk = &mut project.sketches[sketch_idx];
                sk.entities.remove(ri);
                *selected_entity_idx = sk.entities.len().checked_sub(1);
                undo_info = Some(("Delete Sketch Entity", None));
                rebuild_needed = true;
            }
        }

        if let Some(ei) = *selected_entity_idx
            && ei < project.sketches[sketch_idx].entities.len()
        {
            ui.separator();
            let entry = &mut project.sketches[sketch_idx].entities[ei];
            let edit_res = render_entity_property_editor(ui, sketch_idx, ei, entry, eval_ctx, project_file);
            if let Some((lbl, coal)) = edit_res {
                undo_info = Some((lbl, coal));
                rebuild_needed = true;
            }
        }

        // 5. Summary & Quick CAM Operation Creation
        ui.separator();
        {
            let sk = &project.sketches[sketch_idx];
            let aabb = sk.exact_aabb();
            let sz = aabb.size();
            let islands: usize = sk.regions.iter().map(|r| r.islands_cw.len()).sum();
            ui.label(format!(
                "Bounds: {} × {}  |  {} region(s) ({islands} island(s))",
                u.format_length(sz.x),
                u.format_length(sz.y),
                sk.regions.len()
            ));
            if !sk.diagnostics.is_empty() {
                for d in &sk.diagnostics {
                    ui.small(format!("• {}", d.message));
                }
            }
        }

        ui.horizontal_wrapped(|ui| {
            if ui.button("+ 2D Contour").clicked() {
                action = create_quick_sketch_op(
                    project,
                    history,
                    sketch_idx,
                    active_setup_idx,
                    *selected_entity_idx,
                    OperationKind::Contour2D,
                );
            }
            if ui.button("+ 2D Pocket").clicked() {
                action = create_quick_sketch_op(
                    project,
                    history,
                    sketch_idx,
                    active_setup_idx,
                    *selected_entity_idx,
                    OperationKind::Pocket2D,
                );
            }
            if ui.button("+ Drill Holes").clicked() {
                action = create_quick_sketch_op(
                    project,
                    history,
                    sketch_idx,
                    active_setup_idx,
                    *selected_entity_idx,
                    OperationKind::DrillBore,
                );
            }
            if ui
                .button("+ 1-Pass Trace")
                .on_hover_text("Create a 1-pass 2D Curve Trace / Engraving operation")
                .clicked()
            {
                action = create_quick_sketch_op(
                    project,
                    history,
                    sketch_idx,
                    active_setup_idx,
                    *selected_entity_idx,
                    OperationKind::ChamferTrace,
                );
            }
            if ui
                .button("+ V-Carve")
                .on_hover_text("Create a variable-depth 3D V-Carve / Bevel Inlay operation")
                .clicked()
            {
                action = create_quick_sketch_op(
                    project,
                    history,
                    sketch_idx,
                    active_setup_idx,
                    *selected_entity_idx,
                    OperationKind::VCarve,
                );
            }
        });
    });

    if let Some((lbl, coal)) = undo_info {
        history.record_before_edit(lbl, coal.as_deref(), &pre_project);
        if rebuild_needed {
            let proj_dir = project_file.and_then(Path::parent);
            let font_map = project.load_custom_fonts(proj_dir);
            let fonts_opt = (!font_map.is_empty()).then_some(&font_map);
            project.sketches[sketch_idx].reevaluate(eval_ctx);
            project.sketches[sketch_idx].rebuild_regions(fonts_opt);
            let sk_id = project.sketches[sketch_idx].id.clone();
            mark_sketch_dependent_ops_dirty(project, &sk_id);
        }
        if action.is_none() {
            action = Some(SketchInspectorAction::InvalidateFrameCache);
        }
    }

    action
}

fn render_entity_property_editor(
    ui: &mut egui::Ui,
    sk_idx: usize,
    ei: usize,
    entry: &mut SketchEntityEntry,
    eval_ctx: &EvalContext,
    project_file: Option<&Path>,
) -> Option<(&'static str, Option<String>)> {
    let mut changed: Option<(&'static str, Option<String>)> = None;
    let ckey = |field: &str| Some(format!("sk_{sk_idx}_ent_{ei}_{field}"));

    ui.strong(format!("Selected {} Properties", entry.entity.kind_label()));
    ui.horizontal(|ui| {
        ui.label("Name:");
        if ui.text_edit_singleline(&mut entry.name).changed() {
            changed = Some(("Rename Sketch Entity", ckey("name")));
        }
    });
    ui.horizontal(|ui| {
        let mut layer_buf = entry.sub_layer.clone().unwrap_or_default();
        ui.label("Layer:");
        if ui
            .add(egui::TextEdit::singleline(&mut layer_buf).desired_width(95.0))
            .changed()
        {
            let trimmed = layer_buf.trim();
            entry.sub_layer = (!trimmed.is_empty()).then(|| trimmed.to_string());
            changed = Some(("Edit Entity Sub-Layer", ckey("layer")));
        }
        if ui
            .checkbox(&mut entry.is_construction, "Construction")
            .on_hover_text(
                "Construction geometry snaps in viewport but is excluded from CAM regions",
            )
            .changed()
        {
            changed = Some(("Toggle Construction Entity", None));
        }
    });

    match &mut entry.entity {
        SketchEntity::Rectangle {
            anchor,
            x,
            y,
            width,
            height,
            corner_radius,
            rotation_deg,
        } => {
            ui.horizontal(|ui| {
                ui.label("Anchor:");
                egui::ComboBox::from_id_salt(("rect_anc", sk_idx, ei))
                    .selected_text(anchor.label())
                    .show_ui(ui, |ui| {
                        for m in RectAnchorMode::ALL {
                            if ui.selectable_value(anchor, m, m.label()).changed() {
                                changed = Some(("Change Rect Anchor", None));
                            }
                        }
                    });
            });
            ui.horizontal(|ui| {
                ui.label("X:");
                if edit_param_length(ui, ("rx", sk_idx, ei), x, eval_ctx, 88.0) {
                    changed = Some(("Edit Rect X", ckey("rx")));
                }
                ui.label("Y:");
                if edit_param_length(ui, ("ry", sk_idx, ei), y, eval_ctx, 88.0) {
                    changed = Some(("Edit Rect Y", ckey("ry")));
                }
            });
            ui.horizontal(|ui| {
                ui.label("Width:");
                if edit_param_length(ui, ("rw", sk_idx, ei), width, eval_ctx, 82.0) {
                    changed = Some(("Edit Rect Width", ckey("rw")));
                }
                ui.label("Height:");
                if edit_param_length(ui, ("rh", sk_idx, ei), height, eval_ctx, 82.0) {
                    changed = Some(("Edit Rect Height", ckey("rh")));
                }
            });
            ui.horizontal(|ui| {
                ui.label("Fillet R:");
                if edit_param_length(ui, ("rr", sk_idx, ei), corner_radius, eval_ctx, 78.0) {
                    changed = Some(("Edit Rect Fillet", ckey("rr")));
                }
                ui.label("Rot:");
                if ui
                    .add(egui::DragValue::new(rotation_deg).speed(1.0).suffix("°"))
                    .changed()
                {
                    changed = Some(("Rotate Rect", ckey("rrot")));
                }
            });
        }
        SketchEntity::Circle {
            center_x,
            center_y,
            center_xy,
            diameter,
        } => {
            if let Some(legacy) = center_xy.take() {
                *center_x = ParamLength::from_mm(legacy.x);
                *center_y = ParamLength::from_mm(legacy.y);
            }
            ui.horizontal(|ui| {
                ui.label("Center X:");
                if edit_param_length(ui, ("cx", sk_idx, ei), center_x, eval_ctx, 82.0) {
                    changed = Some(("Edit Circle X", ckey("cx")));
                }
                ui.label("Y:");
                if edit_param_length(ui, ("cy", sk_idx, ei), center_y, eval_ctx, 82.0) {
                    changed = Some(("Edit Circle Y", ckey("cy")));
                }
            });
            ui.horizontal(|ui| {
                ui.label("Diameter Ø:");
                if edit_param_length(ui, ("cd", sk_idx, ei), diameter, eval_ctx, 110.0) {
                    changed = Some(("Edit Circle Diameter", ckey("cd")));
                }
            });
        }
        SketchEntity::Slot {
            mode,
            x,
            y,
            length,
            width,
            angle_deg,
        } => {
            ui.horizontal(|ui| {
                ui.label("Length Mode:");
                egui::ComboBox::from_id_salt(("slot_m", sk_idx, ei))
                    .selected_text(mode.label())
                    .show_ui(ui, |ui| {
                        for m in SlotLengthMode::ALL {
                            if ui.selectable_value(mode, m, m.label()).changed() {
                                changed = Some(("Change Slot Mode", None));
                            }
                        }
                    });
            });
            ui.horizontal(|ui| {
                ui.label("Center X:");
                if edit_param_length(ui, ("sx", sk_idx, ei), x, eval_ctx, 82.0) {
                    changed = Some(("Edit Slot X", ckey("sx")));
                }
                ui.label("Y:");
                if edit_param_length(ui, ("sy", sk_idx, ei), y, eval_ctx, 82.0) {
                    changed = Some(("Edit Slot Y", ckey("sy")));
                }
            });
            ui.horizontal(|ui| {
                ui.label("Length:");
                if edit_param_length(ui, ("sl", sk_idx, ei), length, eval_ctx, 76.0) {
                    changed = Some(("Edit Slot Length", ckey("sl")));
                }
                ui.label("Width:");
                if edit_param_length(ui, ("sw", sk_idx, ei), width, eval_ctx, 76.0) {
                    changed = Some(("Edit Slot Width", ckey("sw")));
                }
            });
            ui.horizontal(|ui| {
                ui.label("Angle:");
                if ui
                    .add(egui::DragValue::new(angle_deg).speed(1.0).suffix("°"))
                    .changed()
                {
                    changed = Some(("Rotate Slot", ckey("sang")));
                }
            });
        }
        SketchEntity::RegularPolygon {
            center_x,
            center_y,
            sides,
            sizing,
            diameter,
            corner_radius,
            rotation_deg,
        } => {
            ui.horizontal(|ui| {
                ui.label("Sides:");
                if ui.add(egui::DragValue::new(sides).range(3..=64)).changed() {
                    changed = Some(("Edit Polygon Sides", ckey("psides")));
                }
                egui::ComboBox::from_id_salt(("poly_sz", sk_idx, ei))
                    .selected_text(sizing.label())
                    .show_ui(ui, |ui| {
                        for s in PolygonSizingMode::ALL {
                            if ui.selectable_value(sizing, s, s.label()).changed() {
                                changed = Some(("Change Polygon Sizing", None));
                            }
                        }
                    });
            });
            ui.horizontal(|ui| {
                ui.label("Center X:");
                if edit_param_length(ui, ("px", sk_idx, ei), center_x, eval_ctx, 82.0) {
                    changed = Some(("Edit Polygon X", ckey("px")));
                }
                ui.label("Y:");
                if edit_param_length(ui, ("py", sk_idx, ei), center_y, eval_ctx, 82.0) {
                    changed = Some(("Edit Polygon Y", ckey("py")));
                }
            });
            ui.horizontal(|ui| {
                ui.label("Size / AF:");
                if edit_param_length(ui, ("pd", sk_idx, ei), diameter, eval_ctx, 78.0) {
                    changed = Some(("Edit Polygon Diameter", ckey("pd")));
                }
                ui.label("Fillet R:");
                if edit_param_length(ui, ("pr", sk_idx, ei), corner_radius, eval_ctx, 68.0) {
                    changed = Some(("Edit Polygon Fillet", ckey("pr")));
                }
            });
            ui.horizontal(|ui| {
                ui.label("Rotation:");
                if ui
                    .add(egui::DragValue::new(rotation_deg).speed(1.0).suffix("°"))
                    .changed()
                {
                    changed = Some(("Rotate Polygon", ckey("prot")));
                }
            });
        }
        SketchEntity::Polyline {
            polyline,
            translate_x,
            translate_y,
            scale,
            rotation_deg,
        } => {
            ui.weak(format!(
                "{} segment(s) · {}",
                polyline.segments.len(),
                if polyline.closed {
                    "Closed Loop"
                } else {
                    "Open Chain"
                }
            ));
            ui.horizontal(|ui| {
                ui.label("Shift X:");
                if edit_param_length(ui, ("plx", sk_idx, ei), translate_x, eval_ctx, 82.0) {
                    changed = Some(("Translate Polyline X", ckey("plx")));
                }
                ui.label("Y:");
                if edit_param_length(ui, ("ply", sk_idx, ei), translate_y, eval_ctx, 82.0) {
                    changed = Some(("Translate Polyline Y", ckey("ply")));
                }
            });
            ui.horizontal(|ui| {
                ui.label("Scale:");
                if ui
                    .add(egui::DragValue::new(scale).speed(0.02).range(0.01..=100.0))
                    .changed()
                {
                    changed = Some(("Scale Polyline", ckey("plsc")));
                }
                ui.label("Rot:");
                if ui
                    .add(egui::DragValue::new(rotation_deg).speed(1.0).suffix("°"))
                    .changed()
                {
                    changed = Some(("Rotate Polyline", ckey("plrot")));
                }
            });
        }
        SketchEntity::PointPattern(pat) => {
            if let Some(res) = render_point_pattern_editor(ui, sk_idx, ei, pat, eval_ctx) {
                changed = Some(res);
            }
        }
        SketchEntity::Text(txt) => {
            if let Some(res) = render_text_editor(ui, sk_idx, ei, txt, eval_ctx, project_file) {
                changed = Some(res);
            }
        }
    }

    changed
}

fn render_point_pattern_editor(
    ui: &mut egui::Ui,
    sk_idx: usize,
    ei: usize,
    pat: &mut PointPatternSpec,
    eval_ctx: &EvalContext,
) -> Option<(&'static str, Option<String>)> {
    let mut changed: Option<(&'static str, Option<String>)> = None;
    let ckey = |field: &str| Some(format!("sk_{sk_idx}_pat_{ei}_{field}"));

    let kind_idx = match pat {
        PointPatternSpec::Single { .. } => 0,
        PointPatternSpec::LinearGrid { .. } => 1,
        PointPatternSpec::BoltCircle { .. } => 2,
    };
    ui.horizontal(|ui| {
        ui.label("Pattern:");
        for (idx, lbl) in [(0, "Single"), (1, "Linear Grid"), (2, "Bolt Circle")] {
            if ui.selectable_label(kind_idx == idx, lbl).clicked() && kind_idx != idx {
                *pat = match idx {
                    0 => PointPatternSpec::Single {
                        x: ParamLength::from_mm(0.0),
                        y: ParamLength::from_mm(0.0),
                        hole_diameter: ParamLength::from_mm(6.0),
                    },
                    1 => PointPatternSpec::LinearGrid {
                        origin_x: ParamLength::from_mm(-20.0),
                        origin_y: ParamLength::from_mm(-15.0),
                        count_x: 3,
                        count_y: 2,
                        spacing_x: ParamLength::from_mm(20.0),
                        spacing_y: ParamLength::from_mm(30.0),
                        angle_deg: 0.0,
                        hole_diameter: ParamLength::from_mm(6.0),
                    },
                    _ => PointPatternSpec::BoltCircle {
                        center_x: ParamLength::from_mm(0.0),
                        center_y: ParamLength::from_mm(0.0),
                        bolt_circle_diameter: ParamLength::from_mm(40.0),
                        hole_diameter: ParamLength::from_mm(6.0),
                        count: 6,
                        start_angle_deg: 0.0,
                    },
                };
                changed = Some(("Switch Point Pattern Kind", None));
            }
        }
    });

    match pat {
        PointPatternSpec::Single {
            x,
            y,
            hole_diameter,
        } => {
            ui.horizontal(|ui| {
                ui.label("X:");
                if edit_param_length(ui, ("psx", sk_idx, ei), x, eval_ctx, 84.0) {
                    changed = Some(("Edit Hole X", ckey("psx")));
                }
                ui.label("Y:");
                if edit_param_length(ui, ("psy", sk_idx, ei), y, eval_ctx, 84.0) {
                    changed = Some(("Edit Hole Y", ckey("psy")));
                }
            });
            ui.horizontal(|ui| {
                ui.label("Hole Ø:");
                if edit_param_length(ui, ("psd", sk_idx, ei), hole_diameter, eval_ctx, 100.0) {
                    changed = Some(("Edit Hole Diameter", ckey("psd")));
                }
            });
        }
        PointPatternSpec::LinearGrid {
            origin_x,
            origin_y,
            count_x,
            count_y,
            spacing_x,
            spacing_y,
            angle_deg,
            hole_diameter,
        } => {
            ui.horizontal(|ui| {
                ui.label("Origin X:");
                if edit_param_length(ui, ("pgx", sk_idx, ei), origin_x, eval_ctx, 78.0) {
                    changed = Some(("Edit Grid X", ckey("pgx")));
                }
                ui.label("Y:");
                if edit_param_length(ui, ("pgy", sk_idx, ei), origin_y, eval_ctx, 78.0) {
                    changed = Some(("Edit Grid Y", ckey("pgy")));
                }
            });
            ui.horizontal(|ui| {
                ui.label("Count X:");
                if ui
                    .add(egui::DragValue::new(count_x).range(1..=128))
                    .changed()
                {
                    changed = Some(("Edit Grid Count X", ckey("pgnx")));
                }
                ui.label("Count Y:");
                if ui
                    .add(egui::DragValue::new(count_y).range(1..=128))
                    .changed()
                {
                    changed = Some(("Edit Grid Count Y", ckey("pgny")));
                }
            });
            ui.horizontal(|ui| {
                ui.label("Pitch X:");
                if edit_param_length(ui, ("pgsx", sk_idx, ei), spacing_x, eval_ctx, 78.0) {
                    changed = Some(("Edit Grid Pitch X", ckey("pgsx")));
                }
                ui.label("Y:");
                if edit_param_length(ui, ("pgsy", sk_idx, ei), spacing_y, eval_ctx, 78.0) {
                    changed = Some(("Edit Grid Pitch Y", ckey("pgsy")));
                }
            });
            ui.horizontal(|ui| {
                ui.label("Hole Ø:");
                if edit_param_length(ui, ("pgd", sk_idx, ei), hole_diameter, eval_ctx, 82.0) {
                    changed = Some(("Edit Grid Hole Ø", ckey("pgd")));
                }
                ui.label("Angle:");
                if ui
                    .add(egui::DragValue::new(angle_deg).speed(1.0).suffix("°"))
                    .changed()
                {
                    changed = Some(("Rotate Hole Grid", ckey("pga")));
                }
            });
        }
        PointPatternSpec::BoltCircle {
            center_x,
            center_y,
            bolt_circle_diameter,
            hole_diameter,
            count,
            start_angle_deg,
        } => {
            ui.horizontal(|ui| {
                ui.label("Center X:");
                if edit_param_length(ui, ("bcx", sk_idx, ei), center_x, eval_ctx, 78.0) {
                    changed = Some(("Edit Bolt Circle X", ckey("bcx")));
                }
                ui.label("Y:");
                if edit_param_length(ui, ("bcy", sk_idx, ei), center_y, eval_ctx, 78.0) {
                    changed = Some(("Edit Bolt Circle Y", ckey("bcy")));
                }
            });
            ui.horizontal(|ui| {
                ui.label("BCD Ø:");
                if edit_param_length(
                    ui,
                    ("bcd", sk_idx, ei),
                    bolt_circle_diameter,
                    eval_ctx,
                    80.0,
                ) {
                    changed = Some(("Edit Bolt Circle Ø", ckey("bcd")));
                }
                ui.label("Hole Ø:");
                if edit_param_length(ui, ("bchd", sk_idx, ei), hole_diameter, eval_ctx, 72.0) {
                    changed = Some(("Edit Bolt Hole Ø", ckey("bchd")));
                }
            });
            ui.horizontal(|ui| {
                ui.label("Holes:");
                if ui.add(egui::DragValue::new(count).range(1..=180)).changed() {
                    changed = Some(("Edit Bolt Count", ckey("bcn")));
                }
                ui.label("Start °:");
                if ui
                    .add(egui::DragValue::new(start_angle_deg).speed(1.0).suffix("°"))
                    .changed()
                {
                    changed = Some(("Edit Bolt Start Angle", ckey("bcsa")));
                }
            });
        }
    }

    changed
}

fn render_text_editor(
    ui: &mut egui::Ui,
    sk_idx: usize,
    ei: usize,
    txt: &mut SketchTextSpec,
    eval_ctx: &EvalContext,
    project_file: Option<&Path>,
) -> Option<(&'static str, Option<String>)> {
    let mut changed: Option<(&'static str, Option<String>)> = None;
    let ckey = |field: &str| Some(format!("sk_{sk_idx}_txt_{ei}_{field}"));

    ui.label("Text Content (multi-line):");
    if ui
        .add(egui::TextEdit::multiline(&mut txt.content).desired_rows(2))
        .changed()
    {
        changed = Some(("Edit CAM Text Content", ckey("content")));
    }

    ui.horizontal(|ui| {
        ui.label("Font:");
        egui::ComboBox::from_id_salt(("txt_font", sk_idx, ei))
            .selected_text(txt.font.label())
            .show_ui(ui, |ui| {
                if ui
                    .selectable_value(
                        &mut txt.font,
                        SketchFontSource::BuiltinSansBold,
                        "DejaVu Sans Bold (Outline)",
                    )
                    .changed()
                {
                    changed = Some(("Switch CAM Font", None));
                }
                for hf in HersheyFontKind::ALL {
                    if ui
                        .selectable_value(
                            &mut txt.font,
                            SketchFontSource::Hershey { font: hf },
                            hf.label(),
                        )
                        .changed()
                    {
                        changed = Some(("Switch Hershey Engraving Font", None));
                    }
                }
            });
        if ui
            .small_button(".ttf/.otf…")
            .on_hover_text(
                "Load a custom TrueType or OpenType font file (packed into .chipz under fonts/)",
            )
            .clicked()
            && let Some(picked) = rfd::FileDialog::new()
                .add_filter("TrueType / OpenType Font", &["ttf", "otf"])
                .pick_file()
            && let Ok(bytes) = std::fs::read(&picked)
        {
            txt.font = SketchFontSource::CustomFile {
                path: portable_model_path(project_file, &picked),
                sha256: compute_sketch_sha256_hex(&bytes),
                face_index: 0,
            };
            changed = Some(("Load Custom TTF/OTF Font", None));
        }
    });

    ui.horizontal(|ui| {
        ui.label("Cap Height:");
        if edit_param_length(
            ui,
            ("tcap", sk_idx, ei),
            &mut txt.cap_height,
            eval_ctx,
            80.0,
        ) {
            changed = Some(("Edit Text Cap Height", ckey("tcap")));
        }
        ui.label("Spacing:");
        if edit_param_length(
            ui,
            ("tlsp", sk_idx, ei),
            &mut txt.letter_spacing,
            eval_ctx,
            68.0,
        ) {
            changed = Some(("Edit Letter Spacing", ckey("tlsp")));
        }
    });

    ui.horizontal(|ui| {
        ui.label("Origin X:");
        if edit_param_length(ui, ("tx", sk_idx, ei), &mut txt.x, eval_ctx, 82.0) {
            changed = Some(("Edit Text X", ckey("tx")));
        }
        ui.label("Y:");
        if edit_param_length(ui, ("ty", sk_idx, ei), &mut txt.y, eval_ctx, 82.0) {
            changed = Some(("Edit Text Y", ckey("ty")));
        }
    });

    ui.horizontal(|ui| {
        ui.label("Align:");
        egui::ComboBox::from_id_salt(("txt_ha", sk_idx, ei))
            .width(72.0)
            .selected_text(txt.h_align.label())
            .show_ui(ui, |ui| {
                for a in TextHAlign::ALL {
                    if ui
                        .selectable_value(&mut txt.h_align, a, a.label())
                        .changed()
                    {
                        changed = Some(("Change Text H-Align", None));
                    }
                }
            });
        egui::ComboBox::from_id_salt(("txt_va", sk_idx, ei))
            .width(78.0)
            .selected_text(txt.v_align.label())
            .show_ui(ui, |ui| {
                for a in TextVAlign::ALL {
                    if ui
                        .selectable_value(&mut txt.v_align, a, a.label())
                        .changed()
                    {
                        changed = Some(("Change Text V-Align", None));
                    }
                }
            });
    });

    ui.horizontal(|ui| {
        ui.label("Rot:");
        if ui
            .add(
                egui::DragValue::new(&mut txt.rotation_deg)
                    .speed(1.0)
                    .suffix("°"),
            )
            .changed()
        {
            changed = Some(("Rotate Text", ckey("trot")));
        }
        ui.label("Line×:");
        if ui
            .add(
                egui::DragValue::new(&mut txt.line_spacing_factor)
                    .speed(0.05)
                    .range(0.8..=4.0),
            )
            .changed()
        {
            changed = Some(("Edit Text Line Spacing", ckey("tls")));
        }
        if ui
            .checkbox(&mut txt.mirror_x, "Mirror X")
            .on_hover_text("Mirror horizontally for reverse-side acrylic engraving or V-inlays")
            .changed()
        {
            changed = Some(("Toggle Text Mirror X", None));
        }
    });

    changed
}

fn mark_sketch_dependent_ops_dirty(project: &mut ProjectDocument, sketch_id: &str) {
    for setup in &mut project.setups {
        for op in &mut setup.operations {
            let uses_sketch = op.selections.iter().any(|s| match s {
                GeometrySelectionItem::SketchRegion { sketch_id: id, .. }
                | GeometrySelectionItem::SketchEntity { sketch_id: id, .. }
                | GeometrySelectionItem::SketchLayerAll { sketch_id: id, .. } => id == sketch_id,
                _ => false,
            });
            if uses_sketch {
                op.mark_dirty();
            }
        }
    }
}

fn create_quick_sketch_op(
    project: &mut ProjectDocument,
    history: &mut UndoHistory,
    sketch_idx: usize,
    active_setup_idx: usize,
    selected_entity_idx: Option<usize>,
    kind: OperationKind,
) -> Option<SketchInspectorAction> {
    if active_setup_idx >= project.setups.len() || sketch_idx >= project.sketches.len() {
        return None;
    }
    history.record_before_edit(format!("Add {} from Sketch", kind.label()), None, project);
    let sk = &project.sketches[sketch_idx];
    let sketch_id = sk.id.clone();
    let sk_name = sk.name.clone();
    let sel_item = if let Some(ei) = selected_entity_idx
        && let Some(ent) = sk.entities.get(ei)
    {
        GeometrySelectionItem::SketchEntity {
            sketch_id,
            entity_id: ent.id.clone(),
            reverse: false,
            side_flip: false,
        }
    } else {
        GeometrySelectionItem::SketchLayerAll {
            sketch_id,
            sub_layer: None,
            reverse: false,
            side_flip: false,
        }
    };

    let n = project.setups[active_setup_idx].operations.len() + 1;
    let op_label = match kind {
        OperationKind::ChamferTrace => format!("2D Trace {n} ({sk_name})"),
        _ => format!("{} {n} ({sk_name})", kind.default_name()),
    };
    let mut op = Operation::new(format!("op_{active_setup_idx}_{n}"), op_label, kind);
    op.selections.push(sel_item);
    if kind == OperationKind::ChamferTrace
        && let StrategyParams::ChamferTrace(p) = op.strategy_mut()
    {
        p.mode = ChamferTraceMode::CurveTrace;
        p.trace_compensation = ContourCompensation::CenterOff;
        p.trace_axial_offset = ParamLength::from_mm(0.3);
    }
    if let Some(t) = project
        .tools
        .iter()
        .find(|t| kind.is_compatible_with_tool(t))
    {
        op.tool_id = Some(t.id.clone());
        op.preset_id = t.presets.first().map(|p| p.id.clone());
    }
    project.setups[active_setup_idx].operations.push(op);
    let op_idx = project.setups[active_setup_idx].operations.len() - 1;
    Some(SketchInspectorAction::FocusOperation {
        setup_idx: active_setup_idx,
        op_idx,
    })
}
