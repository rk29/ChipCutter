use std::{collections::HashSet, path::PathBuf};

use chip_math::{DisplayUnit, MM_PER_INCH, ParamLength};
use chip_project::{ProjectDocument, UndoHistory, portable_model_path};
use chip_sketch::{
    DxfImportOptions, SketchAnchor, SketchDiagnosticKind, SketchLayer, SketchPlaneRef,
    SvgImportOptions, VectorAssetFormat, VectorImportUnits, import_dxf_bytes, import_svg_bytes,
    sample_badge_logo_svg, sample_router_panel_dxf,
};
use chip_tools::ToolLibrary;

/// Modal dialog state for importing `.dxf` and `.svg` 2D vector files with live scale,
/// unit/DPI overrides, layer filtering, and gap-healing diagnostics (`11-sketches-vectors.md`).
pub struct VectorImportModalState {
    pub open: bool,
    source_path: Option<PathBuf>,
    source_label: String,
    raw_bytes: Vec<u8>,
    format: VectorAssetFormat,

    sketch_name: String,
    plane_ref: SketchPlaneRef,
    z_offset_mm: f64,
    units_choice: UnitsChoice,
    svg_dpi: f64,
    join_tolerance_mm: f64,
    arc_fit_tolerance_mm: f64,
    sheet_thickness_mm: f64,

    available_layers: Vec<String>,
    enabled_layers: HashSet<String>,

    preview_sketch: Option<SketchLayer>,
    preview_scale_info: String,
    preview_error: Option<String>,
    dirty_preview: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum UnitsChoice {
    AutoMm,
    AutoInch,
    ForceMm,
    ForceInch,
    ForceCm,
}

impl UnitsChoice {
    const ALL: [Self; 5] = [
        Self::AutoMm,
        Self::AutoInch,
        Self::ForceMm,
        Self::ForceInch,
        Self::ForceCm,
    ];

    const fn label(self) -> &'static str {
        match self {
            Self::AutoMm => "Auto-Detect (Fallback: mm)",
            Self::AutoInch => "Auto-Detect (Fallback: in)",
            Self::ForceMm => "Force Millimeters (mm)",
            Self::ForceInch => "Force Inches (in)",
            Self::ForceCm => "Force Centimeters (cm)",
        }
    }

    const fn to_policy(self) -> VectorImportUnits {
        match self {
            Self::AutoMm => VectorImportUnits::AutoDetect {
                fallback: DisplayUnit::Millimeter,
            },
            Self::AutoInch => VectorImportUnits::AutoDetect {
                fallback: DisplayUnit::Inch,
            },
            Self::ForceMm => VectorImportUnits::Millimeters,
            Self::ForceInch => VectorImportUnits::Inches,
            Self::ForceCm => VectorImportUnits::Centimeters,
        }
    }
}

/// Outcome returned when the user commits an import in [`VectorImportModalState::show`].
pub enum VectorModalAction {
    AppendedSketch { sketch_name: String },
    CreatedPure2dJob { project_name: String },
}

/// Mutable app references passed into [`VectorImportModalState::show`].
pub struct VectorModalCtx<'a> {
    pub project: &'a mut ProjectDocument,
    pub project_file_path: Option<&'a std::path::Path>,
    pub active_setup_idx: usize,
    pub global_tools: &'a ToolLibrary,
    pub history: &'a mut UndoHistory,
    pub logs: &'a mut Vec<String>,
}

impl Default for VectorImportModalState {
    fn default() -> Self {
        let mut s = Self {
            open: false,
            source_path: None,
            source_label: "sample_router_panel.dxf (Built-In Fixture)".to_string(),
            raw_bytes: sample_router_panel_dxf(),
            format: VectorAssetFormat::Dxf,
            sketch_name: "Router Front Panel".to_string(),
            plane_ref: SketchPlaneRef::StockTop,
            z_offset_mm: 0.0,
            units_choice: UnitsChoice::AutoMm,
            svg_dpi: 96.0,
            join_tolerance_mm: 0.05,
            arc_fit_tolerance_mm: 0.01,
            sheet_thickness_mm: 6.0,
            available_layers: Vec::new(),
            enabled_layers: HashSet::new(),
            preview_sketch: None,
            preview_scale_info: String::new(),
            preview_error: None,
            dirty_preview: true,
        };
        s.refresh_available_layers_from_raw();
        s
    }
}

impl VectorImportModalState {
    pub fn open_with_file_picker(&mut self) {
        self.open = true;
        if let Some(path) = rfd::FileDialog::new()
            .add_filter(
                "2D Vector Drawings (.dxf, .svg)",
                &["dxf", "DXF", "svg", "SVG"],
            )
            .pick_file()
        {
            self.load_from_disk_path(path);
        } else if self.preview_sketch.is_none() {
            self.dirty_preview = true;
        }
    }

    pub fn load_from_disk_path(&mut self, path: PathBuf) {
        let ext = path
            .extension()
            .and_then(|s| s.to_str())
            .unwrap_or_default()
            .to_ascii_lowercase();
        match std::fs::read(&path) {
            Ok(bytes) => {
                self.format = if ext == "svg" {
                    VectorAssetFormat::Svg
                } else {
                    VectorAssetFormat::Dxf
                };
                self.sketch_name = path
                    .file_stem()
                    .and_then(|s| s.to_str())
                    .unwrap_or("Imported Sketch")
                    .to_string();
                self.source_label = path.display().to_string();
                self.source_path = Some(path);
                self.raw_bytes = bytes;
                self.preview_error = None;
                self.refresh_available_layers_from_raw();
                self.dirty_preview = true;
            }
            Err(e) => {
                self.preview_error = Some(format!("Failed to read '{}': {e}", path.display()));
                self.preview_sketch = None;
            }
        }
    }

    fn load_sample_dxf(&mut self) {
        self.source_path = None;
        self.source_label = "sample_router_panel.dxf (Built-In Fixture)".to_string();
        self.raw_bytes = sample_router_panel_dxf();
        self.format = VectorAssetFormat::Dxf;
        self.sketch_name = "Router Front Panel".to_string();
        self.refresh_available_layers_from_raw();
        self.dirty_preview = true;
    }

    fn load_sample_svg(&mut self) {
        self.source_path = None;
        self.source_label = "sample_badge_logo.svg (Built-In Fixture)".to_string();
        self.raw_bytes = sample_badge_logo_svg().into_bytes();
        self.format = VectorAssetFormat::Svg;
        self.sketch_name = "Badge Emblem Logo".to_string();
        self.refresh_available_layers_from_raw();
        self.dirty_preview = true;
    }

    fn refresh_available_layers_from_raw(&mut self) {
        self.available_layers.clear();
        self.enabled_layers.clear();
        if self.format == VectorAssetFormat::Dxf
            && let Ok(rep) = import_dxf_bytes(&self.raw_bytes, &DxfImportOptions::default())
        {
            self.available_layers = rep.available_layers;
            self.enabled_layers = self.available_layers.iter().cloned().collect();
        }
    }

    fn recompute_preview(&mut self, setup_id: &str, stored_path: Option<String>) {
        self.dirty_preview = false;
        self.preview_error = None;

        match self.format {
            VectorAssetFormat::Dxf => {
                let include_layers = if !self.available_layers.is_empty()
                    && self.enabled_layers.len() < self.available_layers.len()
                {
                    Some(self.enabled_layers.clone())
                } else {
                    None
                };
                let opts = DxfImportOptions {
                    sketch_id: "sketch_preview".to_string(),
                    sketch_name: self.sketch_name.clone(),
                    setup_id: setup_id.to_string(),
                    source_path: stored_path,
                    units: self.units_choice.to_policy(),
                    join_tolerance_mm: self.join_tolerance_mm,
                    arc_fit_tolerance_mm: self.arc_fit_tolerance_mm,
                    include_layers,
                };
                match import_dxf_bytes(&self.raw_bytes, &opts) {
                    Ok(mut rep) => {
                        rep.sketch.anchor = SketchAnchor::SetupXy {
                            setup_id: setup_id.to_string(),
                            plane_ref: self.plane_ref,
                            z_offset: ParamLength::from_mm(self.z_offset_mm),
                        };
                        self.preview_scale_info = rep.detected_dxf_units;
                        self.preview_sketch = Some(rep.sketch);
                    }
                    Err(e) => {
                        self.preview_sketch = None;
                        self.preview_error = Some(e.to_string());
                    }
                }
            }
            VectorAssetFormat::Svg => {
                let units_override = match self.units_choice {
                    UnitsChoice::ForceMm | UnitsChoice::ForceInch | UnitsChoice::ForceCm => {
                        Some(self.units_choice.to_policy())
                    }
                    UnitsChoice::AutoMm | UnitsChoice::AutoInch => None,
                };
                let opts = SvgImportOptions {
                    sketch_id: "sketch_preview".to_string(),
                    sketch_name: self.sketch_name.clone(),
                    setup_id: setup_id.to_string(),
                    source_path: stored_path,
                    svg_dpi: self.svg_dpi,
                    units_override,
                    join_tolerance_mm: self.join_tolerance_mm,
                    arc_fit_tolerance_mm: self.arc_fit_tolerance_mm,
                };
                match import_svg_bytes(&self.raw_bytes, &opts) {
                    Ok(mut rep) => {
                        rep.sketch.anchor = SketchAnchor::SetupXy {
                            setup_id: setup_id.to_string(),
                            plane_ref: self.plane_ref,
                            z_offset: ParamLength::from_mm(self.z_offset_mm),
                        };
                        self.preview_scale_info = rep.scale_description;
                        self.preview_sketch = Some(rep.sketch);
                    }
                    Err(e) => {
                        self.preview_sketch = None;
                        self.preview_error = Some(e.to_string());
                    }
                }
            }
        }
    }

    pub fn show(
        &mut self,
        egui_ctx: &egui::Context,
        mctx: VectorModalCtx<'_>,
    ) -> Option<VectorModalAction> {
        if !self.open {
            return None;
        }
        let VectorModalCtx {
            project,
            project_file_path,
            active_setup_idx,
            global_tools,
            history,
            logs,
        } = mctx;

        let setup_id = project
            .setups
            .get(active_setup_idx)
            .or_else(|| project.setups.first())
            .map_or_else(|| "setup_1".to_string(), |s| s.id.clone());
        let stored_path = self
            .source_path
            .as_deref()
            .map(|sp| portable_model_path(project_file_path, sp));

        if self.dirty_preview {
            self.recompute_preview(&setup_id, stored_path.clone());
        }

        let display_unit = project.display_units;
        let mut keep_open = self.open;
        let mut committed: Option<VectorModalAction> = None;

        egui::Window::new("Import 2D Vector Drawing (.dxf / .svg)")
            .open(&mut keep_open)
            .collapsible(false)
            .resizable(true)
            .default_width(520.0)
            .show(egui_ctx, |ui| {
                ui.horizontal_wrapped(|ui| {
                    if ui
                        .button("Browse .dxf / .svg…")
                        .on_hover_text("Choose a 2D AutoCAD DXF or SVG vector drawing file from disk")
                        .clicked()
                        && let Some(path) = rfd::FileDialog::new()
                            .add_filter(
                                "2D Vector Drawings (.dxf, .svg)",
                                &["dxf", "DXF", "svg", "SVG"],
                            )
                            .pick_file()
                    {
                        self.load_from_disk_path(path);
                        self.recompute_preview(&setup_id, None);
                    }
                    if ui
                        .small_button("Sample Panel (.dxf)")
                        .on_hover_text("Load the built-in electronics faceplate sample DXF for testing")
                        .clicked()
                    {
                        self.load_sample_dxf();
                        self.recompute_preview(&setup_id, None);
                    }
                    if ui
                        .small_button("Sample Emblem (.svg)")
                        .on_hover_text("Load the built-in decorative emblem sample SVG for testing")
                        .clicked()
                    {
                        self.load_sample_svg();
                        self.recompute_preview(&setup_id, None);
                    }
                });
                ui.weak(format!("Source: {}", self.source_label));
                ui.separator();

                egui::Grid::new("vec_import_grid")
                    .num_columns(2)
                    .spacing([12.0, 6.0])
                    .show(ui, |ui| {
                        ui.label("Sketch Name:");
                        if ui
                            .text_edit_singleline(&mut self.sketch_name)
                            .on_hover_text("Name for the imported 2D sketch layer")
                            .changed()
                        {
                            self.dirty_preview = true;
                        }
                        ui.end_row();

                        ui.label("Sketch Z Plane:");
                        ui.horizontal(|ui| {
                            egui::ComboBox::from_id_salt("vec_plane_ref")
                                .selected_text(self.plane_ref.label())
                                .show_ui(ui, |ui| {
                                    for r in SketchPlaneRef::ALL {
                                        if ui
                                            .selectable_value(&mut self.plane_ref, r, r.label())
                                            .on_hover_text("Anchor imported 2D vectors on this Setup Z plane")
                                            .changed()
                                        {
                                            self.dirty_preview = true;
                                        }
                                    }
                                })
                                .response
                                .on_hover_text("Choose which Setup Z datum plane hosts the imported 2D sketch");
                            ui.label("ΔZ (mm):");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut self.z_offset_mm)
                                        .speed(0.1)
                                        .range(-200.0..=200.0),
                                )
                                .on_hover_text("Additional Z offset above or below the selected Sketch Z Plane (mm)")
                                .changed()
                            {
                                self.dirty_preview = true;
                            }
                        });
                        ui.end_row();

                        ui.label("Units / Scale:");
                        egui::ComboBox::from_id_salt("vec_units_choice")
                            .selected_text(self.units_choice.label())
                            .show_ui(ui, |ui| {
                                for u in UnitsChoice::ALL {
                                    if ui
                                        .selectable_value(&mut self.units_choice, u, u.label())
                                        .on_hover_text("Interpret imported vector coordinates in these physical units")
                                        .changed()
                                    {
                                        self.dirty_preview = true;
                                    }
                                }
                            })
                            .response
                            .on_hover_text("Select physical units or auto-detect scale for the DXF/SVG drawing");
                        ui.end_row();

                        if self.format == VectorAssetFormat::Svg {
                            ui.label("SVG Pixel DPI:");
                            ui.horizontal(|ui| {
                                for (lbl, preset) in
                                    [("96 (Inkscape/CSS)", 96.0), ("72 (Illustrator)", 72.0), ("90", 90.0)]
                                {
                                    if ui
                                        .selectable_label(
                                            (self.svg_dpi - preset).abs() < 0.05,
                                            lbl,
                                        )
                                        .on_hover_text(format!("Set SVG pixel-to-inch density to {preset} DPI"))
                                        .clicked()
                                    {
                                        self.svg_dpi = preset;
                                        self.dirty_preview = true;
                                    }
                                }
                                if ui
                                    .add(
                                        egui::DragValue::new(&mut self.svg_dpi)
                                            .speed(1.0)
                                            .range(10.0..=1200.0),
                                    )
                                    .on_hover_text("Custom SVG pixels-per-inch (DPI) used when SVG coordinates are unitless px")
                                    .changed()
                                {
                                    self.dirty_preview = true;
                                }
                            });
                            ui.end_row();
                        }

                        let (u_scale, u_sfx, tol_dec, tol_spd) = match display_unit {
                            DisplayUnit::Millimeter => (1.0, " mm", 3usize, 0.002),
                            DisplayUnit::Inch => (1.0 / MM_PER_INCH, " in", 4usize, 0.0001),
                        };
                        ui.label("Join / Arc-Fit Tolerance:");
                        ui.horizontal_wrapped(|ui| {
                            ui.label("Join:")
                                .on_hover_text("Maximum gap between open segment endpoints automatically stitched into closed loops");
                            let mut join_disp =
                                (self.join_tolerance_mm * u_scale).clamp(0.0, 0.50 * u_scale);
                            if ui
                                .add(
                                    egui::DragValue::new(&mut join_disp)
                                        .speed(tol_spd)
                                        .range(0.0..=(0.50 * u_scale))
                                        .fixed_decimals(tol_dec)
                                        .suffix(u_sfx),
                                )
                                .on_hover_text("Maximum gap between open segment endpoints automatically stitched into closed loops")
                                .changed()
                                && join_disp.is_finite()
                            {
                                self.join_tolerance_mm = (join_disp / u_scale).clamp(0.0, 0.50);
                                self.dirty_preview = true;
                            }

                            ui.separator();
                            ui.label("Arc-Fit Tolerance:")
                                .on_hover_text("Maximum chord/biarc deviation when converting Bezier splines and curves into true G2/G3 arcs");
                            let mut arc_disp = (self.arc_fit_tolerance_mm * u_scale)
                                .clamp(0.001 * u_scale, 1.0 * u_scale);
                            if ui
                                .add(
                                    egui::DragValue::new(&mut arc_disp)
                                        .speed(tol_spd)
                                        .range((0.001 * u_scale)..=(1.0 * u_scale))
                                        .fixed_decimals(tol_dec)
                                        .suffix(u_sfx),
                                )
                                .on_hover_text(
                                    "Maximum deviation when fitting DXF splines and SVG Bezier curves to circular arcs (smaller = finer arcs)",
                                )
                                .changed()
                                && arc_disp.is_finite()
                            {
                                self.arc_fit_tolerance_mm = (arc_disp / u_scale).clamp(0.001, 1.0);
                                self.dirty_preview = true;
                            }
                        });
                        ui.end_row();

                        ui.label("Pure-2D Sheet Thickness (mm):");
                        ui.add(
                            egui::DragValue::new(&mut self.sheet_thickness_mm)
                                .speed(0.25)
                                .range(0.5..=200.0),
                        )
                        .on_hover_text("Raw stock sheet thickness used when starting a Pure-2D job from this vector file (mm)");
                        ui.end_row();
                    });

                if self.format == VectorAssetFormat::Dxf && !self.available_layers.is_empty() {
                    ui.add_space(4.0);
                    ui.strong("DXF Layer Filter:");
                    ui.horizontal_wrapped(|ui| {
                        for lname in self.available_layers.clone() {
                            let mut on = self.enabled_layers.contains(&lname);
                            if ui
                                .checkbox(&mut on, &lname)
                                .on_hover_text(format!("Include or exclude DXF layer '{lname}' in the imported sketch"))
                                .changed()
                            {
                                if on {
                                    self.enabled_layers.insert(lname);
                                } else {
                                    self.enabled_layers.remove(&lname);
                                }
                                self.dirty_preview = true;
                            }
                        }
                    });
                }

                if self.dirty_preview {
                    self.recompute_preview(&setup_id, stored_path);
                }

                ui.separator();
                let tok = crate::theme::ThemeTokens::from_ui(ui);
                if let Some(err) = &self.preview_error {
                    ui.horizontal(|ui| {
                        crate::widgets::status_badge_icon(
                            ui,
                            crate::widgets::StatusBadgeKind::Error,
                            None,
                        );
                        ui.colored_label(tok.error_fg, err);
                    });
                } else if let Some(sk) = &self.preview_sketch {
                    let aabb = sk.exact_aabb();
                    let sz = aabb.size();
                    let islands: usize = sk.regions.iter().map(|r| r.islands_cw.len()).sum();
                    let healed_gaps = sk
                        .diagnostics
                        .iter()
                        .filter(|d| d.kind == SketchDiagnosticKind::HealedEndpointGap)
                        .count();

                    egui::Frame::NONE
                        .fill(tok.card_bg)
                        .stroke(egui::Stroke::new(1.0, tok.border_strong))
                        .inner_margin(8.0)
                        .corner_radius(4.0)
                        .show(ui, |ui| {
                            ui.strong("Live Geometry & Scale Preview");
                            ui.label(format!("Scale Policy: {}", self.preview_scale_info));
                            ui.label(format!(
                                "2D Footprint: X = {} × Y = {}",
                                display_unit.format_length(sz.x),
                                display_unit.format_length(sz.y)
                            ));
                            ui.label(format!(
                                "Entities:     {} total ({} circles, {} closed loops, {} open chains)",
                                sk.entities.len(),
                                sk.circle_count(),
                                sk.closed_loop_count(),
                                sk.open_chain_count()
                            ));
                            ui.label(format!(
                                "Regions:      {} closed machinable region(s), {} interior island(s)",
                                sk.regions.len(),
                                islands
                            ));
                            if healed_gaps > 0 {
                                ui.colored_label(
                                    tok.clean_fg,
                                    format!("Gap-Healer: closed {healed_gaps} micro-gap(s) within {:.3} mm.", self.join_tolerance_mm),
                                );
                            }
                            if sk.open_chain_count() > 0 {
                                ui.colored_label(
                                    tok.warn_fg,
                                    format!(
                                        "Note: {} unclosed open chain(s) remain (usable for 2D Contour / Engraving).",
                                        sk.open_chain_count()
                                    ),
                                );
                            }
                        });

                    ui.add_space(8.0);
                    ui.horizontal(|ui| {
                        if ui
                            .button("Add Sketch to Current Project")
                            .on_hover_text(
                                "Append this 2D Sketch Layer to the current project (works on both 3D STEP and Pure-2D jobs!)",
                            )
                            .clicked()
                        {
                            history.record_before_edit("Import 2D Sketch Layer", None, project);
                            let mut final_sk = sk.clone();
                            final_sk.id = format!("sketch_{}", project.sketches.len() + 1);
                            let sk_name = final_sk.name.clone();
                            project.sketches.push(final_sk);
                            logs.push(format!("Imported 2D Sketch Layer '{sk_name}'."));
                            committed = Some(VectorModalAction::AppendedSketch {
                                sketch_name: sk_name,
                            });
                        }

                        if ui
                            .button("Start New Pure-2D Sheet Job")
                            .on_hover_text(
                                "Create a fresh Pure-2D CNC sheet job (zero STEP files required) sized around this 2D drawing",
                            )
                            .clicked()
                        {
                            let mut fresh = ProjectDocument::new_pure_2d_sheet(
                                &self.sketch_name,
                                self.sheet_thickness_mm,
                                display_unit,
                                &global_tools.tools,
                            );
                            let mut final_sk = sk.clone();
                            final_sk.id = "sketch_1".to_string();
                            final_sk.anchor = SketchAnchor::SetupXy {
                                setup_id: "setup_1".to_string(),
                                plane_ref: SketchPlaneRef::StockTop,
                                z_offset: ParamLength::from_mm(0.0),
                            };
                            let job_name = fresh.project_name.clone();
                            fresh.sketches.push(final_sk);
                            *project = fresh;
                            logs.push(format!(
                                "Created Pure-2D CNC job '{job_name}' ({:.1} mm sheet).",
                                self.sheet_thickness_mm
                            ));
                            committed = Some(VectorModalAction::CreatedPure2dJob {
                                project_name: job_name,
                            });
                        }
                    });
                }
            });

        if committed.is_some() {
            keep_open = false;
        }
        self.open = keep_open;
        committed
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::DisplayUnit;
    use chip_project::{ProjectDocument, UndoHistory};
    use chip_tools::default_starter_tool_library;

    use super::{VectorImportModalState, VectorModalCtx};

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

    #[test]
    fn vector_import_modal_renders_unit_aware_arc_fit_tolerance_beside_join_tolerance() {
        let ctx = egui::Context::default();
        let mut modal = VectorImportModalState {
            open: true,
            arc_fit_tolerance_mm: 0.025,
            ..Default::default()
        };
        let mut project = ProjectDocument::new("Vector Modal Test");
        project.display_units = DisplayUnit::Millimeter;
        let tools = default_starter_tool_library();
        let mut history = UndoHistory::default();
        let mut logs = Vec::new();

        // Warmup frame so egui::Window completes initial auto-sizing before measuring text shapes
        let mut warmup = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(800.0, 700.0),
                )),
                ..Default::default()
            },
            |ui| {
                let _ = modal.show(
                    ui.ctx(),
                    VectorModalCtx {
                        project: &mut project,
                        project_file_path: None,
                        active_setup_idx: 0,
                        global_tools: &tools,
                        history: &mut history,
                        logs: &mut logs,
                    },
                );
            },
        );
        warmup.textures_delta.clear();

        let mut out_mm = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(800.0, 700.0),
                )),
                ..Default::default()
            },
            |ui| {
                let _ = modal.show(
                    ui.ctx(),
                    VectorModalCtx {
                        project: &mut project,
                        project_file_path: None,
                        active_setup_idx: 0,
                        global_tools: &tools,
                        history: &mut history,
                        logs: &mut logs,
                    },
                );
            },
        );
        out_mm.textures_delta.clear();
        let mut texts_mm = Vec::new();
        collect_texts(&out_mm.shapes, &mut texts_mm);
        assert!(
            texts_mm.iter().any(|t| t.contains("Arc-Fit Tolerance")),
            "expected Arc-Fit Tolerance widget in vector import modal, got: {texts_mm:?}"
        );
        assert!(
            texts_mm.iter().any(|t| t == "0.025") && texts_mm.iter().any(|t| t == " mm"),
            "expected arc_fit_tolerance_mm formatted in mm, got: {texts_mm:?}"
        );

        // Switch project display units to Inch and verify unit conversion at UI boundary
        project.display_units = DisplayUnit::Inch;
        let mut out_in = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(800.0, 700.0),
                )),
                ..Default::default()
            },
            |ui| {
                let _ = modal.show(
                    ui.ctx(),
                    VectorModalCtx {
                        project: &mut project,
                        project_file_path: None,
                        active_setup_idx: 0,
                        global_tools: &tools,
                        history: &mut history,
                        logs: &mut logs,
                    },
                );
            },
        );
        out_in.textures_delta.clear();
        let mut texts_in = Vec::new();
        collect_texts(&out_in.shapes, &mut texts_in);
        assert!(
            texts_in.iter().any(|t| t == "0.0010") && texts_in.iter().any(|t| t == " in"),
            "expected arc_fit_tolerance_mm (0.025 mm ~= 0.0010 in) formatted in inches, got: {texts_in:?}"
        );
    }
}
