//! Two-column parametric property editors for selected 2D [`SketchEntity`] items, including
//! TrueType/OpenType outline & Hershey single-stroke CAM typography (`11-sketches-vectors.md`).

use std::path::Path;

use chip_math::{EvalContext, ParamLength};
use chip_project::portable_model_path;
use chip_sketch::{
    HersheyFontKind, PointPatternSpec, PolygonSizingMode, RectAnchorMode, SketchEntity,
    SketchEntityEntry, SketchFontSource, SketchTextSpec, SlotLengthMode, TextHAlign, TextVAlign,
    compute_sketch_sha256_hex,
};

use crate::{
    theme::ThemeTokens,
    widgets::{
        BannerKind, INPUT_COL_WIDTH, edit_param_length, grid_row, inline_banner, inspector_grid,
        section_card,
    },
};

pub(super) struct EntityEditorCtx<'a> {
    pub sketch_idx: usize,
    pub entity_idx: usize,
    pub entry: &'a mut SketchEntityEntry,
    pub eval_ctx: &'a EvalContext,
    pub project_file: Option<&'a Path>,
}

type EditOutcome = Option<(&'static str, Option<String>)>;

pub(super) trait EnumChoice: Copy + PartialEq + 'static {
    const ALL: &'static [Self];
    fn label(self) -> &'static str;
}

impl EnumChoice for RectAnchorMode {
    const ALL: &'static [Self] = &Self::ALL;
    fn label(self) -> &'static str {
        self.label()
    }
}
impl EnumChoice for SlotLengthMode {
    const ALL: &'static [Self] = &Self::ALL;
    fn label(self) -> &'static str {
        self.label()
    }
}
impl EnumChoice for PolygonSizingMode {
    const ALL: &'static [Self] = &Self::ALL;
    fn label(self) -> &'static str {
        self.label()
    }
}
impl EnumChoice for TextHAlign {
    const ALL: &'static [Self] = &Self::ALL;
    fn label(self) -> &'static str {
        self.label()
    }
}
impl EnumChoice for TextVAlign {
    const ALL: &'static [Self] = &Self::ALL;
    fn label(self) -> &'static str {
        self.label()
    }
}

fn field_tip(key: &str) -> &'static str {
    match key {
        "ranc" => "Reference point controlled by X and Y coordinates",
        "rx" | "ry" | "tx" | "ty" => "Origin/anchor coordinate on the 2D Sketch plane",
        "rw" | "rh" => "Rectangle dimension along local Sketch axes",
        "rr" | "pr" => "Exact circular corner fillet radius (0 = sharp corners)",
        "rrot" | "prot" | "plrot" | "sang" | "pga" => "Rotation angle in degrees",
        "cx" | "cy" | "px" | "py" | "sx" | "sy" | "bcx" | "bcy" => {
            "Center coordinate on Sketch plane"
        }
        "cd" => "Exact circle diameter",
        "slot_m" => "Center-to-center arc distance vs. overall stadium length",
        "sl" | "sw" => "Parametric stadium slot dimension",
        "psides" => "Number of equal polygon edges (3..=64)",
        "poly_sz" => "Across Flats (inscribed) vs. Circumscribed corner circle",
        "pd" => "Across-flats wrench size or circumscribed diameter",
        "plx" | "ply" => "2D translation offset applied to imported polyline",
        "plsc" => "Uniform 2D scale multiplier",
        "psx" | "psy" | "pgx" | "pgy" => "Hole reference coordinate on the 2D Sketch plane",
        "psd" | "pgd" | "bchd" => "Nominal hole diameter (populates hole_d)",
        "pgsx" | "pgsy" => "Center-to-center hole pitch along grid axis",
        "bcd" => "Bolt circle pitch diameter (BCD / PCD)",
        "tcap" => "Uppercase letter cap height",
        "tlsp" => "Extra tracking spacing between adjacent glyphs",
        "tls" => "Multi-line baseline step multiplier (0.8..=4.0)",
        _ => "Parametric 2D Sketch dimension",
    }
}

struct EntityGrid<'a> {
    ui: &'a mut egui::Ui,
    sk_idx: usize,
    ei: usize,
    eval_ctx: &'a EvalContext,
    changed: EditOutcome,
}

impl EntityGrid<'_> {
    fn coal(&self, key: &str) -> Option<String> {
        Some(format!("sk_{}_e_{}_{key}", self.sk_idx, self.ei))
    }

    fn len(&mut self, label: &str, key: &'static str, p: &mut ParamLength, undo: &'static str) {
        let salt = (key, self.sk_idx, self.ei);
        let ctx = self.eval_ctx;
        if grid_row(self.ui, label, Some(field_tip(key)), |ui| {
            edit_param_length(ui, salt, p, ctx, INPUT_COL_WIDTH)
        }) {
            self.changed = Some((undo, self.coal(key)));
        }
    }

    fn deg(&mut self, label: &str, key: &'static str, deg: &mut f64, undo: &'static str) {
        let tip = field_tip(key);
        if grid_row(self.ui, label, Some(tip), |ui| {
            ui.add(egui::DragValue::new(deg).speed(1.0).suffix("°"))
                .on_hover_text(tip)
                .changed()
        }) {
            self.changed = Some((undo, self.coal(key)));
        }
    }

    fn choice<T: EnumChoice>(
        &mut self,
        label: &str,
        key: &'static str,
        val: &mut T,
        undo: &'static str,
    ) {
        let salt = (key, self.sk_idx, self.ei);
        let tip = field_tip(key);
        if grid_row(self.ui, label, Some(tip), |ui| {
            mini_choice_combo(ui, salt, INPUT_COL_WIDTH + 30.0, val, tip)
        }) {
            self.changed = Some((undo, None));
        }
    }
}

pub(super) fn mini_choice_combo<T: EnumChoice>(
    ui: &mut egui::Ui,
    salt: impl std::hash::Hash + std::fmt::Debug,
    width: f32,
    val: &mut T,
    tooltip: &str,
) -> bool {
    let mut c = false;
    egui::ComboBox::from_id_salt(salt)
        .width(width)
        .selected_text(val.label())
        .show_ui(ui, |ui| {
            for &item in T::ALL {
                c |= ui
                    .selectable_value(val, item, item.label())
                    .on_hover_text(tooltip)
                    .changed();
            }
        })
        .response
        .on_hover_text(tooltip);
    c
}

pub(super) fn show_selected_entity_card(
    ui: &mut egui::Ui,
    ctx: EntityEditorCtx<'_>,
) -> EditOutcome {
    let EntityEditorCtx {
        sketch_idx: sk_idx,
        entity_idx: ei,
        entry,
        eval_ctx,
        project_file,
    } = ctx;
    let title = if matches!(entry.entity, SketchEntity::Text(_)) {
        "TrueType & Hershey Typography".to_string()
    } else {
        format!("Parametric {} Editor", entry.entity.kind_label())
    };
    let salt = ("sk_ent_editor_card", sk_idx);
    section_card(ui, salt, &title, true, |ui| {
        render_entity_body(ui, sk_idx, ei, entry, eval_ctx, project_file)
    })
    .flatten()
}

fn render_entity_body(
    ui: &mut egui::Ui,
    sk_idx: usize,
    ei: usize,
    entry: &mut SketchEntityEntry,
    eval_ctx: &EvalContext,
    project_file: Option<&Path>,
) -> EditOutcome {
    let meta = render_meta_grid(ui, sk_idx, ei, entry);
    ui.add_space(3.0);
    ui.separator();
    ui.add_space(3.0);

    let specific = match &mut entry.entity {
        SketchEntity::PointPattern(pat) => {
            render_point_pattern_editor(ui, sk_idx, ei, pat, eval_ctx)
        }
        SketchEntity::Text(txt) => render_text_editor(ui, sk_idx, ei, txt, eval_ctx, project_file),
        other => render_primitive_grid(ui, sk_idx, ei, other, eval_ctx),
    };
    specific.or(meta)
}

fn render_meta_grid(
    ui: &mut egui::Ui,
    sk_idx: usize,
    ei: usize,
    entry: &mut SketchEntityEntry,
) -> EditOutcome {
    let mut changed: EditOutcome = None;
    let ckey = |f: &str| Some(format!("sk_{sk_idx}_ent_{ei}_{f}"));
    inspector_grid(ui, ("sk_ent_meta_grid", sk_idx, ei), |ui| {
        if grid_row(
            ui,
            "Entity Name",
            Some("Label in the Sketch Entities list"),
            |ui| {
                ui.add(
                    egui::TextEdit::singleline(&mut entry.name)
                        .desired_width(INPUT_COL_WIDTH + 36.0),
                )
                .on_hover_text("Display label for this 2D sketch entity")
                .changed()
            },
        ) {
            changed = Some(("Rename Sketch Entity", ckey("name")));
        }
        grid_row(
            ui,
            "Sub-Layer",
            Some("Optional DXF/SVG sub-layer tag"),
            |ui| {
                let mut buf = entry.sub_layer.clone().unwrap_or_default();
                if ui
                    .add(egui::TextEdit::singleline(&mut buf).desired_width(INPUT_COL_WIDTH))
                    .on_hover_text(
                        "Optional DXF/SVG sub-layer tag used for batch CAM region selection",
                    )
                    .changed()
                {
                    let t = buf.trim();
                    entry.sub_layer = (!t.is_empty()).then(|| t.to_string());
                    changed = Some(("Edit Entity Sub-Layer", ckey("layer")));
                }
            },
        );
        if grid_row(
            ui,
            "Geometry Role",
            Some("Snaps in viewport; excluded from CAM regions"),
            |ui| {
                ui.checkbox(&mut entry.is_construction, "Construction")
                    .on_hover_text("Mark as reference/construction geometry (snappable in viewport, excluded from CAM machining regions)")
                    .changed()
            },
        ) {
            changed = Some(("Toggle Construction Entity", None));
        }
    });
    changed
}

fn render_primitive_grid(
    ui: &mut egui::Ui,
    sk_idx: usize,
    ei: usize,
    entity: &mut SketchEntity,
    eval_ctx: &EvalContext,
) -> EditOutcome {
    inspector_grid(ui, ("sk_ent_prim_grid", sk_idx, ei), |ui| {
        let mut g = EntityGrid {
            ui,
            sk_idx,
            ei,
            eval_ctx,
            changed: None,
        };
        match entity {
            SketchEntity::Rectangle {
                anchor,
                x,
                y,
                width,
                height,
                corner_radius,
                rotation_deg,
            } => {
                g.choice("Anchor Mode", "ranc", anchor, "Change Rect Anchor");
                g.len("Position X", "rx", x, "Edit Rect X");
                g.len("Position Y", "ry", y, "Edit Rect Y");
                g.len("Width", "rw", width, "Edit Rect Width");
                g.len("Height", "rh", height, "Edit Rect Height");
                g.len("Fillet R", "rr", corner_radius, "Edit Rect Fillet");
                g.deg("Rotation", "rrot", rotation_deg, "Rotate Rect");
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
                g.len("Center X", "cx", center_x, "Edit Circle X");
                g.len("Center Y", "cy", center_y, "Edit Circle Y");
                g.len("Diameter Ø", "cd", diameter, "Edit Circle Diameter");
            }
            SketchEntity::Slot {
                mode,
                x,
                y,
                length,
                width,
                angle_deg,
            } => {
                g.choice("Length Mode", "slot_m", mode, "Change Slot Mode");
                g.len("Center X", "sx", x, "Edit Slot X");
                g.len("Center Y", "sy", y, "Edit Slot Y");
                g.len("Slot Length", "sl", length, "Edit Slot Length");
                g.len("Slot Width", "sw", width, "Edit Slot Width");
                g.deg("Angle", "sang", angle_deg, "Rotate Slot");
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
                if grid_row(g.ui, "Polygon Sides", Some(field_tip("psides")), |ui| {
                    ui.add(egui::DragValue::new(sides).range(3..=64))
                        .on_hover_text(field_tip("psides"))
                        .changed()
                }) {
                    g.changed = Some(("Edit Polygon Sides", g.coal("psides")));
                }
                g.choice("Sizing Mode", "poly_sz", sizing, "Change Polygon Sizing");
                g.len("Center X", "px", center_x, "Edit Polygon X");
                g.len("Center Y", "py", center_y, "Edit Polygon Y");
                g.len("Size / AF Ø", "pd", diameter, "Edit Polygon Diameter");
                g.len("Fillet R", "pr", corner_radius, "Edit Polygon Fillet");
                g.deg("Rotation", "prot", rotation_deg, "Rotate Polygon");
            }
            SketchEntity::Polyline {
                polyline,
                translate_x,
                translate_y,
                scale,
                rotation_deg,
            } => {
                let tok = ThemeTokens::from_ui(g.ui);
                let kind = if polyline.closed {
                    "Closed Loop"
                } else {
                    "Open Chain"
                };
                let desc = format!("{} segment(s) · {kind}", polyline.segments.len());
                grid_row(g.ui, "Topology", Some("Imported DXF/SVG topology"), |ui| {
                    ui.colored_label(tok.text_muted, desc);
                });
                g.len("Translate X", "plx", translate_x, "Translate Polyline X");
                g.len("Translate Y", "ply", translate_y, "Translate Polyline Y");
                if grid_row(g.ui, "Scale Factor", Some(field_tip("plsc")), |ui| {
                    ui.add(egui::DragValue::new(scale).speed(0.02).range(0.01..=100.0))
                        .on_hover_text(field_tip("plsc"))
                        .changed()
                }) {
                    g.changed = Some(("Scale Polyline", g.coal("plsc")));
                }
                g.deg("Rotation", "plrot", rotation_deg, "Rotate Polyline");
            }
            SketchEntity::PointPattern(_) | SketchEntity::Text(_) => {}
        }
        g.changed
    })
}

fn render_point_pattern_editor(
    ui: &mut egui::Ui,
    sk_idx: usize,
    ei: usize,
    pat: &mut PointPatternSpec,
    eval_ctx: &EvalContext,
) -> EditOutcome {
    let kind_idx = match pat {
        PointPatternSpec::Single { .. } => 0,
        PointPatternSpec::LinearGrid { .. } => 1,
        PointPatternSpec::BoltCircle { .. } => 2,
    };

    inspector_grid(ui, ("sk_ent_pat_grid", sk_idx, ei), |ui| {
        let mut changed = None;
        if let Some(idx) = grid_row(
            ui,
            "Pattern Type",
            Some("Single hole, grid, or bolt circle"),
            |ui| {
                [(0, "Single"), (1, "Grid"), (2, "Bolt Circle")]
                    .into_iter()
                    .find(|&(i, lbl)| {
                        ui.selectable_label(kind_idx == i, lbl)
                            .on_hover_text(format!("Switch hole pattern type to {lbl}"))
                            .clicked()
                            && kind_idx != i
                    })
                    .map(|(i, _)| i)
            },
        ) {
            *pat = default_point_pattern_for_idx(idx);
            changed = Some(("Switch Point Pattern Kind", None));
        }

        let mut g = EntityGrid {
            ui,
            sk_idx,
            ei,
            eval_ctx,
            changed,
        };
        match pat {
            PointPatternSpec::Single {
                x,
                y,
                hole_diameter,
            } => {
                g.len("Hole X", "psx", x, "Edit Hole X");
                g.len("Hole Y", "psy", y, "Edit Hole Y");
                g.len("Hole Ø", "psd", hole_diameter, "Edit Hole Diameter");
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
                g.len("Origin X", "pgx", origin_x, "Edit Grid X");
                g.len("Origin Y", "pgy", origin_y, "Edit Grid Y");
                let (cx, cy) = grid_row(g.ui, "Count (X × Y)", Some("Columns × rows"), |ui| {
                    let a = ui
                        .add(egui::DragValue::new(count_x).range(1..=128))
                        .on_hover_text("Number of hole columns along X")
                        .changed();
                    ui.label("×");
                    (
                        a,
                        ui.add(egui::DragValue::new(count_y).range(1..=128))
                            .on_hover_text("Number of hole rows along Y")
                            .changed(),
                    )
                });
                if cx {
                    g.changed = Some(("Edit Grid Count X", g.coal("pgnx")));
                } else if cy {
                    g.changed = Some(("Edit Grid Count Y", g.coal("pgny")));
                }
                g.len("Pitch X", "pgsx", spacing_x, "Edit Grid Pitch X");
                g.len("Pitch Y", "pgsy", spacing_y, "Edit Grid Pitch Y");
                g.len("Hole Ø", "pgd", hole_diameter, "Edit Grid Hole Ø");
                g.deg("Grid Angle", "pga", angle_deg, "Rotate Hole Grid");
            }
            PointPatternSpec::BoltCircle {
                center_x,
                center_y,
                bolt_circle_diameter,
                hole_diameter,
                count,
                start_angle_deg,
            } => {
                g.len("Center X", "bcx", center_x, "Edit Bolt Circle X");
                g.len("Center Y", "bcy", center_y, "Edit Bolt Circle Y");
                g.len("BCD Ø", "bcd", bolt_circle_diameter, "Edit Bolt Circle Ø");
                g.len("Hole Ø", "bchd", hole_diameter, "Edit Bolt Hole Ø");
                let (cnt, sa) = grid_row(
                    g.ui,
                    "Count & Start °",
                    Some("Hole count and start angle"),
                    |ui| {
                        let a = ui
                            .add(egui::DragValue::new(count).range(1..=180))
                            .on_hover_text("Number of equally spaced holes around the bolt circle")
                            .changed();
                        (
                            a,
                            ui.add(egui::DragValue::new(start_angle_deg).speed(1.0).suffix("°"))
                                .on_hover_text(
                                    "Angular offset of the first bolt hole from +X (degrees)",
                                )
                                .changed(),
                        )
                    },
                );
                if cnt {
                    g.changed = Some(("Edit Bolt Count", g.coal("bcn")));
                } else if sa {
                    g.changed = Some(("Edit Bolt Start Angle", g.coal("bcsa")));
                }
            }
        }
        g.changed
    })
}

fn default_point_pattern_for_idx(idx: usize) -> PointPatternSpec {
    let mm = ParamLength::from_mm;
    match idx {
        0 => PointPatternSpec::Single {
            x: mm(0.0),
            y: mm(0.0),
            hole_diameter: mm(6.0),
        },
        1 => PointPatternSpec::LinearGrid {
            origin_x: mm(-20.0),
            origin_y: mm(-15.0),
            count_x: 3,
            count_y: 2,
            spacing_x: mm(20.0),
            spacing_y: mm(30.0),
            angle_deg: 0.0,
            hole_diameter: mm(6.0),
        },
        _ => PointPatternSpec::BoltCircle {
            center_x: mm(0.0),
            center_y: mm(0.0),
            bolt_circle_diameter: mm(40.0),
            hole_diameter: mm(6.0),
            count: 6,
            start_angle_deg: 0.0,
        },
    }
}

fn render_text_editor(
    ui: &mut egui::Ui,
    sk_idx: usize,
    ei: usize,
    txt: &mut SketchTextSpec,
    eval_ctx: &EvalContext,
    project_file: Option<&Path>,
) -> EditOutcome {
    let tok = ThemeTokens::from_ui(ui);
    ui.colored_label(tok.text_secondary, "Text Content (multi-line):");
    let ml = egui::TextEdit::multiline(&mut txt.content)
        .desired_rows(2)
        .desired_width(f32::INFINITY);
    let changed = ui
        .add(ml)
        .on_hover_text(
            "Multi-line text string converted into outline or single-stroke CAM engraving vectors",
        )
        .changed()
        .then(|| {
            (
                "Edit CAM Text Content",
                Some(format!("sk_{sk_idx}_txt_{ei}_content")),
            )
        });
    ui.add_space(4.0);

    let grid_changed = inspector_grid(ui, ("sk_ent_txt_grid", sk_idx, ei), |ui| {
        let font_changed = grid_row(
            ui,
            "Font Family",
            Some("TrueType/OpenType or Hershey font"),
            |ui| render_font_picker_controls(ui, sk_idx, ei, &mut txt.font, project_file),
        )
        .map(|lbl| (lbl, None));

        let mut g = EntityGrid {
            ui,
            sk_idx,
            ei,
            eval_ctx,
            changed: font_changed,
        };
        g.len(
            "Cap Height",
            "tcap",
            &mut txt.cap_height,
            "Edit Text Cap Height",
        );
        g.len(
            "Spacing",
            "tlsp",
            &mut txt.letter_spacing,
            "Edit Letter Spacing",
        );
        g.len("Origin X", "tx", &mut txt.x, "Edit Text X");
        g.len("Origin Y", "ty", &mut txt.y, "Edit Text Y");
        let (ha, va) = grid_row(
            g.ui,
            "Alignment",
            Some("Horizontal & vertical alignment"),
            |ui| {
                let a = mini_choice_combo(
                    ui,
                    ("txt_ha", sk_idx, ei),
                    72.0,
                    &mut txt.h_align,
                    "Horizontal text alignment relative to Origin X",
                );
                (
                    a,
                    mini_choice_combo(
                        ui,
                        ("txt_va", sk_idx, ei),
                        78.0,
                        &mut txt.v_align,
                        "Vertical text alignment relative to Origin Y",
                    ),
                )
            },
        );
        if ha {
            g.changed = Some(("Change Text H-Align", None));
        } else if va {
            g.changed = Some(("Change Text V-Align", None));
        }
        if grid_row(g.ui, "Line Spacing ×", Some(field_tip("tls")), |ui| {
            ui.add(
                egui::DragValue::new(&mut txt.line_spacing_factor)
                    .speed(0.05)
                    .range(0.8..=4.0),
            )
            .on_hover_text(field_tip("tls"))
            .changed()
        }) {
            g.changed = Some(("Edit Text Line Spacing", g.coal("tls")));
        }
        if let Some((lbl, k)) = render_text_orient_row(g.ui, txt) {
            g.changed = Some((lbl, k.and_then(|key| g.coal(key))));
        }
        g.changed
    });

    if matches!(txt.font, SketchFontSource::Hershey { .. }) {
        ui.add_space(4.0);
        inline_banner(
            ui,
            BannerKind::Info,
            "Single-stroke Hershey font active: pair with '+ 1-Pass Trace' for centerline engraving.",
        );
    }
    grid_changed.or(changed)
}

fn render_text_orient_row(
    ui: &mut egui::Ui,
    txt: &mut SketchTextSpec,
) -> Option<(&'static str, Option<&'static str>)> {
    grid_row(
        ui,
        "Orientation",
        Some("Rotation angle and reverse-engraving Mirror X"),
        |ui| {
            if ui
                .add(
                    egui::DragValue::new(&mut txt.rotation_deg)
                        .speed(1.0)
                        .suffix("°"),
                )
                .on_hover_text("Rotate text baseline angle in degrees")
                .changed()
            {
                return Some(("Rotate Text", Some("trot")));
            }
            let tip = "Mirror horizontally for reverse-side acrylic engraving or V-inlays";
            ui.checkbox(&mut txt.mirror_x, "Mirror X")
                .on_hover_text(tip)
                .changed()
                .then_some(("Toggle Text Mirror X", None))
        },
    )
}

fn render_font_picker_controls(
    ui: &mut egui::Ui,
    sk_idx: usize,
    ei: usize,
    font: &mut SketchFontSource,
    project_file: Option<&Path>,
) -> Option<&'static str> {
    let mut out = None;
    egui::ComboBox::from_id_salt(("txt_font", sk_idx, ei))
        .width(INPUT_COL_WIDTH + 16.0)
        .selected_text(font.label())
        .show_ui(ui, |ui| {
            if ui
                .selectable_value(
                    font,
                    SketchFontSource::BuiltinSansBold,
                    "DejaVu Sans Bold (Outline)",
                )
                .on_hover_text(
                    "Built-in TrueType outline font suitable for V-Carve and 2D Pocket lettering",
                )
                .changed()
            {
                out = Some("Switch CAM Font");
            }
            for hf in HersheyFontKind::ALL {
                if ui
                    .selectable_value(font, SketchFontSource::Hershey { font: hf }, hf.label())
                    .on_hover_text(
                        "Single-stroke Hershey engraving font for fast 1-pass trace engraving",
                    )
                    .changed()
                {
                    out = Some("Switch Hershey Engraving Font");
                }
            }
        })
        .response
        .on_hover_text("Select built-in outline font or single-stroke Hershey engraving font");
    let tip = "Load a custom TrueType or OpenType font file (packed into .chipz under fonts/)";
    if ui.small_button(".ttf/.otf…").on_hover_text(tip).clicked()
        && let Some(picked) = rfd::FileDialog::new()
            .add_filter("TrueType / OpenType Font", &["ttf", "otf"])
            .pick_file()
        && let Ok(bytes) = std::fs::read(&picked)
    {
        *font = SketchFontSource::CustomFile {
            path: portable_model_path(project_file, &picked),
            sha256: compute_sketch_sha256_hex(&bytes),
            face_index: 0,
        };
        out = Some("Load Custom TTF/OTF Font");
    }
    out
}
