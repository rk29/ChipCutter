use std::{collections::HashSet, path::PathBuf};

use chip_cam::{BoxAnchor27, StockConfig, WcsOriginMode};
use chip_math::{DisplayUnit, ParamLength};
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
                    if ui.button("Browse .dxf / .svg…").clicked()
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
                    if ui.small_button("Sample Panel (.dxf)").clicked() {
                        self.load_sample_dxf();
                        self.recompute_preview(&setup_id, None);
                    }
                    if ui.small_button("Sample Emblem (.svg)").clicked() {
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
                        if ui.text_edit_singleline(&mut self.sketch_name).changed() {
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
                                            .changed()
                                        {
                                            self.dirty_preview = true;
                                        }
                                    }
                                });
                            ui.label("ΔZ (mm):");
                            if ui
                                .add(
                                    egui::DragValue::new(&mut self.z_offset_mm)
                                        .speed(0.1)
                                        .range(-200.0..=200.0),
                                )
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
                                        .changed()
                                    {
                                        self.dirty_preview = true;
                                    }
                                }
                            });
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
                                    .changed()
                                {
                                    self.dirty_preview = true;
                                }
                            });
                            ui.end_row();
                        }

                        ui.label("Gap-Heal Tolerance (mm):");
                        if ui
                            .add(
                                egui::Slider::new(&mut self.join_tolerance_mm, 0.0..=0.50)
                                    .fixed_decimals(3),
                            )
                            .changed()
                        {
                            self.dirty_preview = true;
                        }
                        ui.end_row();

                        ui.label("Pure-2D Sheet Thickness (mm):");
                        ui.add(
                            egui::DragValue::new(&mut self.sheet_thickness_mm)
                                .speed(0.25)
                                .range(0.5..=200.0),
                        );
                        ui.end_row();
                    });

                if self.format == VectorAssetFormat::Dxf && !self.available_layers.is_empty() {
                    ui.add_space(4.0);
                    ui.strong("DXF Layer Filter:");
                    ui.horizontal_wrapped(|ui| {
                        for lname in self.available_layers.clone() {
                            let mut on = self.enabled_layers.contains(&lname);
                            if ui.checkbox(&mut on, &lname).changed() {
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
                if let Some(err) = &self.preview_error {
                    ui.colored_label(egui::Color32::from_rgb(248, 96, 96), format!("[!] {err}"));
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
                        .fill(egui::Color32::from_rgb(22, 28, 36))
                        .stroke(egui::Stroke::new(1.0, egui::Color32::from_rgb(56, 78, 108)))
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
                                    egui::Color32::from_rgb(106, 224, 152),
                                    format!("Gap-Healer: closed {healed_gaps} micro-gap(s) within {:.3} mm.", self.join_tolerance_mm),
                                );
                            }
                            if sk.open_chain_count() > 0 {
                                ui.colored_label(
                                    egui::Color32::from_rgb(248, 186, 78),
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
                            let mut fresh = ProjectDocument::new(&self.sketch_name);
                            fresh.display_units = display_unit;
                            fresh.setups[0].name = format!(
                                "Setup 1 — {:.1}mm Sheet (G54)",
                                self.sheet_thickness_mm
                            );
                            fresh.setups[0].wcs.origin = WcsOriginMode::StockBoxPoint {
                                anchor: BoxAnchor27::TOP_FRONT_LEFT,
                            };
                            fresh.setups[0].stock = StockConfig::RelativeBox {
                                side_pos_x: ParamLength::from_mm(3.0),
                                side_neg_x: ParamLength::from_mm(3.0),
                                side_pos_y: ParamLength::from_mm(3.0),
                                side_neg_y: ParamLength::from_mm(3.0),
                                symmetric_xy: true,
                                top_offset_z: ParamLength::from_mm(0.0),
                                bottom_offset_z: ParamLength::from_mm(0.0),
                                sheet_thickness_z: ParamLength::from_mm(
                                    self.sheet_thickness_mm.max(0.5),
                                ),
                                round_increment_mm: None,
                            };
                            for t in &global_tools.tools {
                                fresh.ensure_tool_embedded(t);
                            }
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
