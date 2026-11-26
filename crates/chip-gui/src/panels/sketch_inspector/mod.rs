//! Parametric 2D Sketch & Entity Inspector (`11-sketches-vectors.md`):
//! - Anchoring (`SetupXy` vs. associative `PlanarFace`), `Look At Sketch` camera button, and 2D Snap Grid controls;
//! - Interactive viewport draw tool toggles (`Rect`, `Circle`, `Slot`, `Polygon`, `Bolt Circle`, `Text`) + 1-click presets;
//! - Full expression-backed parametric editors for all 7 `SketchEntity` variants + TrueType/Hershey typography controls;
//! - Instant `rebuild_regions()` + Undo/Redo snapshot recording.

mod entity_editors;

use std::path::Path;

use chip_cam::{
    ChamferTraceMode, ContourCompensation, GeometrySelectionItem, Operation, OperationKind,
    StrategyParams,
};
use chip_math::{EvalContext, ParamLength};
use chip_project::{ProjectDocument, UndoHistory};
use chip_sketch::{
    FaceSketchOriginMode, PointPatternSpec, PolygonSizingMode, RectAnchorMode, SketchAnchor,
    SketchEntity, SketchEntityEntry, SketchPlaneRef, SketchTextSpec, SlotLengthMode,
};

use crate::{
    panels::{SketchAuthorTool, ViewportPickMode},
    theme::ThemeTokens,
    widgets::{
        BannerKind, INPUT_COL_WIDTH, edit_param_length, grid_row, inline_banner, inspector_grid,
        section_card, static_card,
    },
};

use self::entity_editors::{
    EntityEditorCtx, EnumChoice, mini_choice_combo, show_selected_entity_card,
};

impl EnumChoice for SketchPlaneRef {
    const ALL: &'static [Self] = &Self::ALL;
    fn label(self) -> &'static str {
        self.label()
    }
}
impl EnumChoice for FaceSketchOriginMode {
    const ALL: &'static [Self] = &Self::ALL;
    fn label(self) -> &'static str {
        self.label()
    }
}

const SNAP_STEPS: [(&str, Option<f64>); 4] = [
    ("Off", None),
    ("1 mm", Some(1.0)),
    ("5 mm", Some(5.0)),
    ("10 mm", Some(10.0)),
];

const QUICK_OPS: [(&str, OperationKind); 5] = [
    ("+ 2D Contour", OperationKind::Contour2D),
    ("+ 2D Pocket", OperationKind::Pocket2D),
    ("+ Drill Holes", OperationKind::DrillBore),
    ("+ 1-Pass Trace", OperationKind::ChamferTrace),
    ("+ V-Carve", OperationKind::VCarve),
];

const QUICK_SPECS: [(u8, &str, &str, Option<&str>); 6] = [
    (0, "Rect", "Add Sketch Rectangle", None),
    (1, "Circle", "Add Sketch Circle", None),
    (2, "Slot", "Add Sketch Slot", None),
    (3, "Hex", "Add Sketch Hexagon", None),
    (4, "Bolt Circle", "Add Bolt Circle", Some("HOLES")),
    (5, "Text", "Add CAM Text", None),
];

fn quick_op_tip(kind: OperationKind) -> &'static str {
    match kind {
        OperationKind::ChamferTrace => "Create a 1-pass 2D Curve Trace / Engraving operation",
        OperationKind::VCarve => "Create a variable-depth 3D V-Carve / Bevel Inlay operation",
        _ => "Create a new CAM operation from this Sketch selection",
    }
}

/// High-level action requested by [`show_sketch_inspector`].
#[derive(Debug, Clone)]
pub enum SketchInspectorAction {
    LookAtSketch(usize),
    InvalidateFrameCache,
    FocusOperation { setup_idx: usize, op_idx: usize },
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

struct SketchUiState<'a> {
    ctx: SketchInspectorCtx<'a>,
    undo_info: Option<(&'static str, Option<String>)>,
    rebuild_needed: bool,
    action: Option<SketchInspectorAction>,
}

/// Renders the parametric **2D Sketch & Entity Inspector** pane.
pub fn show_sketch_inspector(
    ui: &mut egui::Ui,
    ctx: SketchInspectorCtx<'_>,
) -> Option<SketchInspectorAction> {
    if ctx.sketch_idx >= ctx.project.sketches.len() {
        ui.weak("No Sketch Layer selected.");
        return None;
    }

    let pre_project = ctx.project.clone();
    let mut st = SketchUiState {
        ctx,
        undo_info: None,
        rebuild_needed: false,
        action: None,
    };
    let idx = st.ctx.sketch_idx;

    static_card(ui, None, |ui| {
        ui.horizontal(|ui| {
            ui.strong("2D Sketch Layer");
            let vis = &mut st.ctx.project.sketches[idx].visible;
            if ui.checkbox(vis, "Visible").changed() {
                st.undo_info = Some(("Toggle Sketch Visibility", None));
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let tip = "Align orthographic camera perpendicular to this Sketch plane";
                if ui
                    .small_button("Look At Sketch")
                    .on_hover_text(tip)
                    .clicked()
                {
                    st.action = Some(SketchInspectorAction::LookAtSketch(idx));
                }
            });
        });
    });

    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| render_sketch_scroll_body(ui, &mut st));

    commit_sketch_mutations(st, &pre_project)
}

fn render_sketch_scroll_body(ui: &mut egui::Ui, st: &mut SketchUiState<'_>) {
    let idx = st.ctx.sketch_idx;
    if is_sketch_pick_mode(*st.ctx.pick_mode, idx)
        && let Some(msg) = st.ctx.pick_mode.banner_text()
    {
        inline_banner(ui, BannerKind::Info, msg);
        ui.add_space(5.0);
    }

    let c1 = ("sk_anc", idx);
    let t1 = "Sketch Anchor & Snap Grid";
    section_card(ui, c1, t1, true, |ui| render_anchor_card(ui, st));
    let c2 = ("sk_auth", idx);
    let t2 = "Interactive Authoring Tools";
    section_card(ui, c2, t2, true, |ui| render_authoring_card(ui, st));

    let ent_len = st.ctx.project.sketches[idx].entities.len();
    if st.ctx.selected_entity_idx.is_some_and(|s| s >= ent_len) {
        *st.ctx.selected_entity_idx = ent_len.checked_sub(1);
    } else if st.ctx.selected_entity_idx.is_none() && ent_len > 0 {
        *st.ctx.selected_entity_idx = Some(0);
    }

    let c3 = ("sk_ents", idx);
    let hdr = format!("Entities & Regions ({ent_len})");
    section_card(ui, c3, &hdr, true, |ui| render_entities_card(ui, st));

    if let Some(ei) = *st.ctx.selected_entity_idx
        && let Some(entry) = st.ctx.project.sketches[idx].entities.get_mut(ei)
    {
        let (eval_ctx, project_file) = (st.ctx.eval_ctx, st.ctx.project_file);
        let ed = EntityEditorCtx {
            sketch_idx: idx,
            entity_idx: ei,
            entry,
            eval_ctx,
            project_file,
        };
        if let Some(info) = show_selected_entity_card(ui, ed) {
            st.undo_info = Some(info);
            st.rebuild_needed = true;
        }
    }
}

fn commit_sketch_mutations(
    mut st: SketchUiState<'_>,
    pre: &ProjectDocument,
) -> Option<SketchInspectorAction> {
    let idx = st.ctx.sketch_idx;
    if let Some((lbl, coal)) = st.undo_info {
        st.ctx.history.record_before_edit(lbl, coal.as_deref(), pre);
        if st.rebuild_needed {
            let dir = st.ctx.project_file.and_then(Path::parent);
            let fonts = st.ctx.project.load_custom_fonts(dir);
            st.ctx.project.sketches[idx].reevaluate(st.ctx.eval_ctx);
            st.ctx.project.sketches[idx].rebuild_regions((!fonts.is_empty()).then_some(&fonts));
            let sk_id = st.ctx.project.sketches[idx].id.clone();
            mark_sketch_dependent_ops_dirty(st.ctx.project, &sk_id);
        }
        if st.action.is_none() {
            st.action = Some(SketchInspectorAction::InvalidateFrameCache);
        }
    }
    st.action
}

fn render_anchor_card(ui: &mut egui::Ui, st: &mut SketchUiState<'_>) {
    let idx = st.ctx.sketch_idx;
    let mut switch_to_setup_xy: Option<String> = None;

    inspector_grid(ui, ("sk_anchor_grid", idx), |ui| {
        if grid_row(
            ui,
            "Sketch Name",
            Some("Display label in the CAM Browser"),
            |ui| {
                let name = &mut st.ctx.project.sketches[idx].name;
                let prev = name.clone();
                let w = INPUT_COL_WIDTH + 36.0;
                ui.add(egui::TextEdit::singleline(name).desired_width(w))
                    .on_hover_text("Display label for this 2D sketch layer in the CAM Browser")
                    .changed()
                    && *name != prev
            },
        ) {
            st.undo_info = Some(("Rename Sketch", Some(format!("sk_name_{idx}"))));
        }

        render_anchor_variant_rows(ui, st, &mut switch_to_setup_xy);

        grid_row(
            ui,
            "2D Snap Grid",
            Some("Cursor grid step during viewport drawing"),
            |ui| {
                let snap = &mut *st.ctx.snap_grid_mm;
                for (lbl, val) in SNAP_STEPS {
                    if ui
                        .selectable_label(*snap == val, lbl)
                        .on_hover_text(format!("Set 2D viewport drawing grid snap to {lbl}"))
                        .clicked()
                    {
                        *snap = val;
                    }
                }
            },
        );
    });

    if let Some(sid) = switch_to_setup_xy {
        st.ctx.project.sketches[idx].anchor = SketchAnchor::SetupXy {
            setup_id: sid,
            plane_ref: SketchPlaneRef::StockTop,
            z_offset: ParamLength::from_mm(0.0),
        };
        st.undo_info = Some(("Switch Sketch to Setup XY", None));
    }
}

fn z_offset_row(
    ui: &mut egui::Ui,
    salt: (&str, usize),
    p: &mut ParamLength,
    ctx: &EvalContext,
) -> bool {
    grid_row(
        ui,
        "Z Offset",
        Some("Signed normal offset from host plane"),
        |ui| edit_param_length(ui, salt, p, ctx, INPUT_COL_WIDTH),
    )
}

fn render_anchor_variant_rows(
    ui: &mut egui::Ui,
    st: &mut SketchUiState<'_>,
    switch_to_setup_xy: &mut Option<String>,
) {
    let sketch_idx = st.ctx.sketch_idx;
    let eval_ctx = st.ctx.eval_ctx;
    let has_cad = st.ctx.has_cad_model;
    let rebind = ViewportPickMode::RebindSketchHostFace { sketch_idx };
    let cb_w = INPUT_COL_WIDTH + 36.0;
    let z_undo = || {
        Some((
            "Edit Sketch Z Offset",
            Some(format!("sk_zoff_{sketch_idx}")),
        ))
    };

    if st.ctx.project.setups.len() > 1 {
        let setups_list: Vec<(String, String)> = st
            .ctx
            .project
            .setups
            .iter()
            .map(|s| {
                (
                    s.id.clone(),
                    format!("{} [{}]", s.name, s.wcs_slot.short_code()),
                )
            })
            .collect();
        let cur_setup_id = st.ctx.project.sketches[sketch_idx]
            .anchor
            .setup_id()
            .to_string();
        let cur_label = setups_list
            .iter()
            .find(|(id, _)| id == &cur_setup_id)
            .map_or(cur_setup_id.as_str(), |(_, lbl)| lbl.as_str());
        let mut chosen_setup_id = cur_setup_id.clone();
        grid_row(
            ui,
            "Setup",
            Some(
                "Reassign this 2D Sketch layer to another clamping Setup's WCS frame & variable scope",
            ),
            |ui| {
                egui::ComboBox::from_id_salt(("sk_setup_reassign", sketch_idx))
                    .width(cb_w)
                    .truncate()
                    .selected_text(cur_label)
                    .show_ui(ui, |ui| {
                        for (sid, slbl) in &setups_list {
                            ui.selectable_value(&mut chosen_setup_id, sid.clone(), slbl)
                                .on_hover_text(format!("Anchor this 2D Sketch to {slbl}"));
                        }
                    })
                    .response
                    .on_hover_text("Choose which Setup WCS frame hosts this 2D Sketch");
            },
        );
        if chosen_setup_id != cur_setup_id {
            st.ctx.project.sketches[sketch_idx]
                .anchor
                .set_setup_id(chosen_setup_id);
            st.undo_info = Some(("Reassign Sketch Setup", None));
            st.rebuild_needed = true;
        }
    }

    match &mut st.ctx.project.sketches[sketch_idx].anchor {
        SketchAnchor::SetupXy {
            plane_ref,
            z_offset,
            ..
        } => {
            if grid_row(ui, "Anchor Plane", Some("Setup XY reference plane"), |ui| {
                mini_choice_combo(
                    ui,
                    ("sk_pr", sketch_idx),
                    cb_w,
                    plane_ref,
                    "Reference Z plane within the Setup for this 2D sketch",
                )
            }) {
                st.undo_info = Some(("Change Sketch Plane", None));
            }
            if z_offset_row(ui, ("sk_z_off_xy", sketch_idx), z_offset, eval_ctx) {
                st.undo_info = z_undo();
            }
            if has_cad {
                grid_row(
                    ui,
                    "3D Host Face",
                    Some("Click a planar STEP face to anchor"),
                    |ui| {
                        let on = *st.ctx.pick_mode == rebind;
                        if ui
                            .selectable_label(on, "Attach to 3D Face…")
                            .on_hover_text("Click a planar 3D CAD face in the viewport to anchor this sketch onto it")
                            .clicked()
                        {
                            *st.ctx.pick_mode = if on {
                                ViewportPickMode::InspectEntity
                            } else {
                                rebind
                            };
                        }
                    },
                );
            }
        }
        SketchAnchor::PlanarFace {
            setup_id,
            face_binding,
            origin_mode,
            x_axis_clock_deg,
            z_offset,
        } => {
            grid_row(
                ui,
                "Host 3D Face",
                Some("Associative planar B-Rep face"),
                |ui| {
                    ui.strong(format!("Face #{}", face_binding.last_face_index));
                    if has_cad
                        && ui
                            .small_button("Re-Pick…")
                            .on_hover_text("Select a different planar 3D face to host this sketch")
                            .clicked()
                    {
                        *st.ctx.pick_mode = rebind;
                    }
                    if ui
                        .small_button("Use Setup XY")
                        .on_hover_text(
                            "Detach from the 3D face and anchor this sketch to the Setup XY plane",
                        )
                        .clicked()
                    {
                        *switch_to_setup_xy = Some(setup_id.clone());
                    }
                },
            );
            if grid_row(
                ui,
                "Face Origin",
                Some("2D (0, 0) datum on host face"),
                |ui| {
                    mini_choice_combo(
                        ui,
                        ("sk_fo", sketch_idx),
                        cb_w,
                        origin_mode,
                        "2D (0, 0) coordinate datum on the host 3D face",
                    )
                },
            ) {
                st.undo_info = Some(("Change Face Sketch Origin", None));
            }
            if grid_row(
                ui,
                "+X Clocking",
                Some("In-plane +X rotation around face normal"),
                |ui| {
                    ui.add(
                        egui::DragValue::new(x_axis_clock_deg)
                            .speed(1.0)
                            .suffix("°")
                            .range(-180.0..=180.0),
                    )
                    .on_hover_text(
                        "Rotate the sketch +X axis in-plane around the host face normal (degrees)",
                    )
                    .changed()
                },
            ) {
                st.undo_info = Some(("Clock Face Sketch +X", Some(format!("sk_clk_{sketch_idx}"))));
            }
            if z_offset_row(ui, ("sk_z_off_face", sketch_idx), z_offset, eval_ctx) {
                st.undo_info = z_undo();
            }
        }
    }
}

fn render_authoring_card(ui: &mut egui::Ui, st: &mut SketchUiState<'_>) {
    let sketch_idx = st.ctx.sketch_idx;
    let tok = ThemeTokens::from_ui(ui);

    ui.colored_label(tok.text_secondary, "Draw in Viewport (2D Snap Enabled):");
    ui.horizontal_wrapped(|ui| {
        for tool in SketchAuthorTool::ALL {
            let target = ViewportPickMode::SketchAuthor { sketch_idx, tool };
            let on = *st.ctx.pick_mode == target;
            let lbl = format!("+ {}", tool.short_label());
            if ui
                .selectable_label(on, lbl)
                .on_hover_text(tool.tooltip())
                .clicked()
            {
                *st.ctx.author_start_xy = None;
                *st.ctx.pick_mode = if on {
                    ViewportPickMode::InspectEntity
                } else {
                    target
                };
            }
        }
    });

    ui.add_space(4.0);
    ui.colored_label(tok.text_secondary, "Quick Insert at (0, 0):");
    ui.horizontal_wrapped(|ui| {
        let sk = &mut st.ctx.project.sketches[sketch_idx];
        for (spec, btn_lbl, undo_lbl, sub_layer) in QUICK_SPECS {
            if ui
                .small_button(btn_lbl)
                .on_hover_text(format!(
                    "Insert a default {btn_lbl} entity centered at (0, 0)"
                ))
                .clicked()
            {
                let n = sk.entities.len() + 1;
                sk.entities.push(ent_entry(n, spec, sub_layer));
                *st.ctx.selected_entity_idx = Some(sk.entities.len() - 1);
                st.undo_info = Some((undo_lbl, None));
                st.rebuild_needed = true;
            }
        }
    });
}

fn render_entities_card(ui: &mut egui::Ui, st: &mut SketchUiState<'_>) {
    let idx = st.ctx.sketch_idx;
    let tok = ThemeTokens::from_ui(ui);
    let u = st.ctx.project.display_units;

    if st.ctx.project.sketches[idx].entities.is_empty() {
        inline_banner(
            ui,
            BannerKind::Info,
            "Click a Draw tool or Quick Insert button above to add 2D primitives or CAM text.",
        );
    } else {
        render_entity_scroll_list(ui, st);
    }

    ui.add_space(4.0);
    {
        let sk = &st.ctx.project.sketches[idx];
        let sz = sk.exact_aabb().size();
        let islands: usize = sk.regions.iter().map(|r| r.islands_cw.len()).sum();
        let (sx, sy, nr) = (
            u.format_length(sz.x),
            u.format_length(sz.y),
            sk.regions.len(),
        );
        inline_banner(
            ui,
            BannerKind::Info,
            format!("Bounds: {sx} × {sy}  ·  {nr} region(s) ({islands} island(s))"),
        );
        if !sk.diagnostics.is_empty() {
            ui.add_space(3.0);
            let joined = sk
                .diagnostics
                .iter()
                .map(|d| format!("• {}", d.message))
                .collect::<Vec<_>>()
                .join("\n");
            inline_banner(ui, BannerKind::Warn, joined);
        }
    }

    ui.add_space(5.0);
    ui.colored_label(tok.text_secondary, "1-Click CAM Operation from Sketch:");
    ui.horizontal_wrapped(|ui| {
        for (lbl, kind) in QUICK_OPS {
            if ui
                .small_button(lbl)
                .on_hover_text(quick_op_tip(kind))
                .clicked()
            {
                st.action = create_quick_sketch_op(&mut st.ctx, kind);
            }
        }
    });
}

fn render_entity_scroll_list(ui: &mut egui::Ui, st: &mut SketchUiState<'_>) {
    let idx = st.ctx.sketch_idx;
    let (mut remove_idx, mut dup_idx) = (None, None);

    egui::ScrollArea::vertical()
        .id_salt(("sk_ent_scroll", idx))
        .max_height(155.0)
        .show(ui, |ui| {
            for (ei, entry) in st.ctx.project.sketches[idx].entities.iter().enumerate() {
                let (sel, dup, del) =
                    render_entity_row(ui, entry, *st.ctx.selected_entity_idx == Some(ei));
                if sel {
                    *st.ctx.selected_entity_idx = Some(ei);
                }
                if dup {
                    dup_idx = Some(ei);
                }
                if del {
                    remove_idx = Some(ei);
                }
            }
        });

    if let Some(di) = dup_idx {
        let sk = &mut st.ctx.project.sketches[idx];
        let mut c = sk.entities[di].clone();
        c.id = format!("{}_copy_{}", c.id, sk.entities.len() + 1);
        c.name = format!("{} Copy", c.name);
        sk.entities.push(c);
        *st.ctx.selected_entity_idx = Some(sk.entities.len() - 1);
        st.undo_info = Some(("Duplicate Sketch Entity", None));
        st.rebuild_needed = true;
    }
    if let Some(ri) = remove_idx {
        let sk = &mut st.ctx.project.sketches[idx];
        sk.entities.remove(ri);
        *st.ctx.selected_entity_idx = sk.entities.len().checked_sub(1);
        st.undo_info = Some(("Delete Sketch Entity", None));
        st.rebuild_needed = true;
    }
}

fn render_entity_row(
    ui: &mut egui::Ui,
    entry: &SketchEntityEntry,
    is_sel: bool,
) -> (bool, bool, bool) {
    let (mut sel, mut dup, mut del) = (false, false, false);
    ui.horizontal(|ui| {
        let tag = if entry.is_construction { "[C] " } else { "" };
        let btn = egui::Button::selectable(
            is_sel,
            format!("{tag}[{}] {}", entry.entity.kind_label(), entry.name),
        )
        .truncate();
        sel = ui
            .add_sized([(ui.available_width() - 56.0).max(60.0), 19.0], btn)
            .clicked();
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            del = ui
                .small_button("×")
                .on_hover_text("Delete entity")
                .clicked();
            dup = ui
                .small_button("Dup")
                .on_hover_text("Duplicate entity")
                .clicked();
        });
    });
    (sel, dup, del)
}

fn is_sketch_pick_mode(mode: ViewportPickMode, sketch_idx: usize) -> bool {
    matches!(
        mode,
        ViewportPickMode::RebindSketchHostFace { sketch_idx: s }
            | ViewportPickMode::SketchAuthor { sketch_idx: s, .. }
            if s == sketch_idx
    )
}

pub(crate) fn create_default_sketch_entity(
    n: usize,
    spec: u8,
    sub_layer: Option<&str>,
) -> SketchEntityEntry {
    ent_entry(n, spec, sub_layer)
}

fn ent_entry(n: usize, spec: u8, sub_layer: Option<&str>) -> SketchEntityEntry {
    let mm = ParamLength::from_mm;
    let (p, title) = match spec {
        0 => ("rect", "Rectangle"),
        1 => ("circle", "Circle"),
        2 => ("slot", "Slot"),
        3 => ("poly", "Hexagon"),
        4 => ("bolts", "Bolt Circle"),
        _ => ("text", "Text"),
    };
    let entity = match spec {
        0 => SketchEntity::Rectangle {
            anchor: RectAnchorMode::Center,
            x: mm(0.0),
            y: mm(0.0),
            width: mm(50.0),
            height: mm(30.0),
            corner_radius: mm(4.0),
            rotation_deg: 0.0,
        },
        1 => SketchEntity::new_circle(mm(0.0), mm(0.0), mm(12.0)),
        2 => SketchEntity::Slot {
            mode: SlotLengthMode::CenterToCenter,
            x: mm(0.0),
            y: mm(0.0),
            length: mm(24.0),
            width: mm(8.0),
            angle_deg: 0.0,
        },
        3 => SketchEntity::RegularPolygon {
            center_x: mm(0.0),
            center_y: mm(0.0),
            sides: 6,
            sizing: PolygonSizingMode::AcrossFlats,
            diameter: mm(19.0),
            corner_radius: mm(0.0),
            rotation_deg: 0.0,
        },
        4 => SketchEntity::PointPattern(PointPatternSpec::BoltCircle {
            center_x: mm(0.0),
            center_y: mm(0.0),
            bolt_circle_diameter: mm(40.0),
            hole_diameter: mm(6.0),
            count: 6,
            start_angle_deg: 0.0,
        }),
        _ => SketchEntity::Text(SketchTextSpec::default()),
    };
    SketchEntityEntry {
        id: format!("{p}_{n}"),
        name: format!("{title} {n}"),
        sub_layer: sub_layer.map(str::to_string),
        is_construction: false,
        entity,
    }
}

pub(crate) fn mark_sketch_dependent_ops_dirty(project: &mut ProjectDocument, sketch_id: &str) {
    for setup in &mut project.setups {
        for op in &mut setup.operations {
            let uses = op.selections.iter().any(|s| match s {
                GeometrySelectionItem::SketchRegion { sketch_id: id, .. }
                | GeometrySelectionItem::SketchEntity { sketch_id: id, .. }
                | GeometrySelectionItem::SketchLayerAll { sketch_id: id, .. } => id == sketch_id,
                _ => false,
            });
            if uses {
                op.mark_dirty();
            }
        }
    }
}

fn create_quick_sketch_op(
    ctx: &mut SketchInspectorCtx<'_>,
    kind: OperationKind,
) -> Option<SketchInspectorAction> {
    let (s_idx, sk_idx) = (ctx.active_setup_idx, ctx.sketch_idx);
    if s_idx >= ctx.project.setups.len() || sk_idx >= ctx.project.sketches.len() {
        return None;
    }
    let undo_msg = format!("Add {} from Sketch", kind.label());
    ctx.history.record_before_edit(undo_msg, None, ctx.project);
    let sk = &ctx.project.sketches[sk_idx];
    let (sketch_id, sk_name) = (sk.id.clone(), sk.name.clone());
    let sel_item = if let Some(ei) = *ctx.selected_entity_idx
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

    let n = ctx.project.setups[s_idx].operations.len() + 1;
    let op_label = match kind {
        OperationKind::ChamferTrace => format!("2D Trace {n} ({sk_name})"),
        _ => format!("{} {n} ({sk_name})", kind.default_name()),
    };
    let mut op = Operation::new(format!("op_{s_idx}_{n}"), op_label, kind);
    op.selections.push(sel_item);
    if kind == OperationKind::ChamferTrace
        && let StrategyParams::ChamferTrace(p) = op.strategy_mut()
    {
        p.mode = ChamferTraceMode::CurveTrace;
        p.trace_compensation = ContourCompensation::CenterOff;
        p.trace_axial_offset = ParamLength::from_mm(0.3);
    }
    if let Some(t) = ctx
        .project
        .tools
        .iter()
        .find(|t| kind.is_compatible_with_tool(t))
    {
        let material = ctx.project.effective_workpiece_material();
        op.tool_id = Some(t.id.clone());
        op.preset_id =
            chip_tools::match_best_preset_for_material(t, &material).map(|p| p.id.clone());
    }
    let ops = &mut ctx.project.setups[s_idx].operations;
    ops.push(op);
    Some(SketchInspectorAction::FocusOperation {
        setup_idx: s_idx,
        op_idx: ops.len() - 1,
    })
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cam::{
        GeometrySelectionItem, Operation, OperationKind, OperationStatus, Setup, WorkOffsetSlot,
    };
    use chip_math::{DisplayUnit, EvalContext, ParamLength};
    use chip_project::{ProjectDocument, UndoHistory};
    use chip_sketch::{
        SketchAnchor, SketchEntity, SketchEntityEntry, SketchFontSource, SketchLayer,
        SketchPlaneRef, SketchTextSpec,
    };

    use super::{SketchInspectorCtx, show_sketch_inspector};
    use crate::panels::ViewportPickMode;

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

    fn find_text(shape: &egui::Shape, needle: &str) -> Option<egui::Pos2> {
        match shape {
            egui::Shape::Text(t) if t.galley.text().contains(needle) => {
                Some(t.pos + t.galley.size() * 0.5)
            }
            egui::Shape::Vec(v) => v.iter().find_map(|s| find_text(s, needle)),
            _ => None,
        }
    }

    fn click(pt: egui::Pos2) -> Vec<egui::Event> {
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

    #[test]
    fn sketch_inspector_reassigns_setup_and_exposes_text_arc_fit_and_face_index() {
        let mut project = ProjectDocument::new("Sketch M89 Test");
        project.setups.push(Setup::new(
            "setup_2",
            "Setup 2 — Flip Op",
            WorkOffsetSlot::G55,
        ));

        let mut op = Operation::new("op_1", "Engrave", OperationKind::VCarve);
        op.runtime_status = OperationStatus::Clean;
        op.selections.push(GeometrySelectionItem::SketchLayerAll {
            sketch_id: "sk_1".to_string(),
            sub_layer: None,
            reverse: false,
            side_flip: false,
        });
        project.setups[0].operations.push(op);

        let mut sk = SketchLayer {
            id: "sk_1".to_string(),
            name: "Logo Sketch".to_string(),
            visible: true,
            anchor: SketchAnchor::SetupXy {
                setup_id: "setup_1".to_string(),
                plane_ref: SketchPlaneRef::StockTop,
                z_offset: ParamLength::from_mm(0.0),
            },
            source_asset: None,
            cut_depth_default: ParamLength::from_mm(3.0),
            entities: vec![SketchEntityEntry {
                id: "txt_1".to_string(),
                name: "Label Text".to_string(),
                sub_layer: None,
                is_construction: false,
                entity: SketchEntity::Text(SketchTextSpec {
                    font: SketchFontSource::CustomFile {
                        path: "fonts/custom_collection.ttc".to_string(),
                        sha256: String::new(),
                        face_index: 2,
                    },
                    arc_fit_tolerance_mm: 0.02,
                    ..SketchTextSpec::default()
                }),
            }],
            regions: Vec::new(),
            diagnostics: Vec::new(),
        };
        sk.rebuild_regions(None);
        project.sketches.push(sk);

        let ctx = egui::Context::default();
        let eval_ctx = EvalContext::new(DisplayUnit::Millimeter);
        let mut history = UndoHistory::default();
        let mut sel_ent = Some(0);
        let mut snap = Some(1.0);
        let mut author_start = None;
        let mut pick_mode = ViewportPickMode::InspectEntity;

        let mut run_frame = |events: Vec<egui::Event>| {
            let mut out = ctx.run_ui(
                egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(
                        egui::Pos2::ZERO,
                        egui::vec2(520.0, 1200.0),
                    )),
                    events,
                    ..Default::default()
                },
                |ui| {
                    egui::CentralPanel::default().show(ui, |ui| {
                        let _ = show_sketch_inspector(
                            ui,
                            SketchInspectorCtx {
                                project: &mut project,
                                sketch_idx: 0,
                                active_setup_idx: 0,
                                selected_entity_idx: &mut sel_ent,
                                snap_grid_mm: &mut snap,
                                author_start_xy: &mut author_start,
                                history: &mut history,
                                pick_mode: &mut pick_mode,
                                eval_ctx: &eval_ctx,
                                has_cad_model: false,
                                project_file: None,
                            },
                        );
                    });
                },
            );
            out.textures_delta.clear();
            out
        };

        let out1 = run_frame(vec![]);
        let mut texts = Vec::new();
        collect_texts(&out1.shapes, &mut texts);
        assert!(
            texts.iter().any(|t| t.contains("Font Face Index")),
            "expected Font Face Index control for CustomFile font, got: {texts:?}"
        );
        assert!(
            texts.iter().any(|t| t.contains("Arc-Fit Tolerance")),
            "expected Arc-Fit Tolerance control for outline text, got: {texts:?}"
        );

        // Open Setup reassignment ComboBox ("Setup 1 — Top Op [G54]") and pick Setup 2
        let setup_cb_pt = out1
            .shapes
            .iter()
            .find_map(|cs| find_text(&cs.shape, "Setup 1"))
            .expect("Setup reassignment combo must render when project.setups.len() > 1");
        let _ = run_frame(click(setup_cb_pt));
        let out2 = run_frame(vec![]);
        let setup2_pt = out2
            .shapes
            .iter()
            .find_map(|cs| find_text(&cs.shape, "Setup 2"))
            .expect("Setup 2 must appear in reassignment dropdown");
        let _ = run_frame(click(setup2_pt));

        assert_eq!(project.sketches[0].anchor.setup_id(), "setup_2");
        assert_eq!(
            project.setups[0].operations[0].runtime_status,
            OperationStatus::Dirty,
            "sketch-dependent operation must be marked Dirty after Setup reassignment"
        );
        assert_eq!(
            history.undo(&mut project).as_deref(),
            Some("Reassign Sketch Setup")
        );
        assert_eq!(project.sketches[0].anchor.setup_id(), "setup_1");
    }
}
