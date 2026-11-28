use std::path::PathBuf;

use chip_cad::{
    MAX_MODEL_SCALE, MIN_MODEL_SCALE, MeshImportOptions, MeshUnitKind, ModelTransform,
    TriangleMeshBody, load_mesh_bytes,
};
use chip_fixtures::{sample_relief_wave_obj_text, sample_sculpted_dome_stl_bytes};
use chip_math::{Aabb3, DisplayUnit};

use crate::{
    theme::ThemeTokens,
    widgets::{IconKind, inline_icon, primary_icon_button, static_card},
};

/// Committed mesh import payload emitted by [`MeshImportModalState::show`].
pub struct MeshImportCommit {
    pub source_name: String,
    pub source_path: Option<PathBuf>,
    pub raw_bytes: Vec<u8>,
    pub unit_override: Option<MeshUnitKind>,
    pub transform: ModelTransform,
}

/// Modal dialog for importing `.stl`, `.obj`, and `.3mf` 3D triangle meshes with live physical
/// bounding-box preview and unit confirmation (preventing `25.4×` scale mistakes on unitless files).
pub struct MeshImportModalState {
    pub open: bool,
    source_path: Option<PathBuf>,
    source_label: String,
    raw_bytes: Vec<u8>,
    selected_unit: MeshUnitKind,
    detected_unit: Option<MeshUnitKind>,
    preview_mesh: Option<TriangleMeshBody>,
    preview_error: Option<String>,
    dirty_preview: bool,
    rotate_deg: [f64; 3],
    uniform_scale: f64,
    center_xy: bool,
    drop_z0: bool,
}

impl Default for MeshImportModalState {
    fn default() -> Self {
        Self {
            open: false,
            source_path: None,
            source_label: "sculpted_dome.stl".to_string(),
            raw_bytes: sample_sculpted_dome_stl_bytes(),
            selected_unit: MeshUnitKind::Millimeter,
            detected_unit: None,
            preview_mesh: None,
            preview_error: None,
            dirty_preview: true,
            rotate_deg: [0.0; 3],
            uniform_scale: 1.0,
            center_xy: false,
            drop_z0: false,
        }
    }
}

impl MeshImportModalState {
    /// Opens a native OS file picker for `.stl`/`.obj`/`.3mf`, opening the preview modal if chosen.
    pub fn open_with_file_picker(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter(
                "3D Triangle Meshes (.stl, .obj, .3mf)",
                &["stl", "obj", "3mf", "STL", "OBJ", "3MF"],
            )
            .pick_file()
        {
            self.load_from_disk_path(path);
            self.open = true;
        }
    }

    /// Loads raw bytes from `path` on disk and pre-populates unit detection for live preview.
    pub fn load_from_disk_path(&mut self, path: PathBuf) {
        let name = path
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("mesh.stl")
            .to_string();
        match std::fs::read(&path) {
            Ok(bytes) => {
                self.source_path = Some(path);
                self.source_label = name;
                self.raw_bytes = bytes;
                self.preview_error = None;
                self.probe_initial_unit();
            }
            Err(err) => {
                self.source_path = Some(path);
                self.source_label = name;
                self.raw_bytes.clear();
                self.preview_mesh = None;
                self.preview_error = Some(format!("Failed to read mesh file: {err}"));
                self.dirty_preview = false;
            }
        }
    }

    fn load_builtin_fixture(&mut self, name: &str, bytes: Vec<u8>) {
        self.source_path = None;
        self.source_label = name.to_string();
        self.raw_bytes = bytes;
        self.preview_error = None;
        self.probe_initial_unit();
    }

    fn probe_initial_unit(&mut self) {
        self.rotate_deg = [0.0; 3];
        self.uniform_scale = 1.0;
        self.center_xy = false;
        self.drop_z0 = false;
        match load_mesh_bytes(
            &self.source_label,
            &self.raw_bytes,
            &MeshImportOptions::default(),
        ) {
            Ok(mesh) => {
                self.detected_unit = mesh.detected_unit;
                self.selected_unit = mesh.applied_unit;
                self.preview_mesh = Some(mesh);
                self.preview_error = None;
                self.dirty_preview = false;
            }
            Err(err) => {
                self.detected_unit = None;
                self.preview_mesh = None;
                self.preview_error = Some(err.to_string());
                self.dirty_preview = false;
            }
        }
    }

    fn rebuild_preview(&mut self) {
        if self.raw_bytes.is_empty() {
            self.dirty_preview = false;
            return;
        }
        match load_mesh_bytes(
            &self.source_label,
            &self.raw_bytes,
            &MeshImportOptions {
                unit_override: Some(self.selected_unit),
                ..MeshImportOptions::default()
            },
        ) {
            Ok(mesh) => {
                self.detected_unit = mesh.detected_unit;
                self.preview_mesh = Some(mesh);
                self.preview_error = None;
            }
            Err(err) => {
                self.preview_mesh = None;
                self.preview_error = Some(err.to_string());
            }
        }
        self.dirty_preview = false;
    }

    fn compute_effective_transform_and_aabb(
        &self,
        mesh: &TriangleMeshBody,
    ) -> (ModelTransform, Aabb3) {
        ModelTransform::compose_for_mesh_import(
            &mesh.vertices_mm,
            self.rotate_deg,
            self.uniform_scale,
            false,
            self.center_xy,
            self.drop_z0,
        )
    }

    /// Renders the **Import 3D Mesh (STL / OBJ / 3MF)** preview modal.
    pub fn show(
        &mut self,
        ctx: &egui::Context,
        display_unit: DisplayUnit,
    ) -> Option<MeshImportCommit> {
        if !self.open {
            return None;
        }
        if self.dirty_preview {
            self.rebuild_preview();
        }

        let mut keep_open = self.open;
        let mut close_clicked = false;
        let mut commit: Option<MeshImportCommit> = None;

        egui::Window::new("Import 3D Mesh (STL / OBJ / 3MF)")
            .id(egui::Id::unique("chipcutter_mesh_import_modal"))
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .default_width(540.0)
            .open(&mut keep_open)
            .show(ctx, |ui| {
                let tok = ThemeTokens::from_ui(ui);
                ui.set_min_width(510.0);

                ui.horizontal(|ui| {
                    inline_icon(ui, IconKind::StepModel, tok.accent, 15.0);
                    ui.strong(&self.source_label);
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.small_button("Browse File…").clicked()
                            && let Some(path) = rfd::FileDialog::new()
                                .add_filter(
                                    "3D Triangle Meshes (.stl, .obj, .3mf)",
                                    &["stl", "obj", "3mf", "STL", "OBJ", "3MF"],
                                )
                                .pick_file()
                        {
                            self.load_from_disk_path(path);
                        }
                        if ui
                            .small_button("Sample Wave (OBJ)")
                            .on_hover_text("Preview built-in 80×60×15 mm sinusoidal wave relief OBJ")
                            .clicked()
                        {
                            self.load_builtin_fixture(
                                "relief_wave.obj",
                                sample_relief_wave_obj_text().into_bytes(),
                            );
                        }
                        if ui
                            .small_button("Sample Dome (STL)")
                            .on_hover_text("Preview built-in 70×70×33 mm sculpted hemisphere dome STL")
                            .clicked()
                        {
                            self.load_builtin_fixture(
                                "sculpted_dome.stl",
                                sample_sculpted_dome_stl_bytes(),
                            );
                        }
                    });
                });

                ui.add_space(4.0);

                static_card(ui, Some("Source Coordinate Units"), |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Interpret coordinates as:");
                        let prev = self.selected_unit;
                        egui::ComboBox::from_id_salt("mesh_import_unit_combo")
                            .selected_text(self.selected_unit.label())
                            .show_ui(ui, |ui| {
                                for u in MeshUnitKind::ALL {
                                    ui.selectable_value(&mut self.selected_unit, u, u.label())
                                        .on_hover_text(format!("Interpret raw mesh coordinates in {}", u.label()));
                                }
                            })
                            .response
                            .on_hover_text("Select physical unit of raw mesh coordinates (verify physical size below)");
                        if self.selected_unit != prev {
                            self.dirty_preview = true;
                        }
                    });

                    if let Some(det) = self.detected_unit {
                        ui.colored_label(
                            tok.info_fg,
                            format!(
                                "File metadata declared unit: {} (pre-selected automatically).",
                                det.label()
                            ),
                        );
                    } else if self
                        .preview_mesh
                        .as_ref()
                        .is_some_and(|m| m.format.is_unitless_by_spec())
                    {
                        ui.colored_label(
                            tok.warn_fg,
                            "STL and OBJ files do not store physical units — verify the X × Y × Z dimensions below before importing to avoid a 25.4× scale error.",
                        );
                    }
                });

                ui.add_space(4.0);

                static_card(ui, Some("Initial Orientation, Scale, & Placement"), |ui| {
                    ui.horizontal_wrapped(|ui| {
                        ui.spacing_mut().item_spacing.x = 4.0;
                        if ui
                            .selectable_label(self.rotate_deg == [0.0, 0.0, 0.0], "As-Is")
                            .on_hover_text("Import mesh without any initial axis rotation")
                            .clicked()
                        {
                            self.rotate_deg = [0.0, 0.0, 0.0];
                        }
                        if ui
                            .selectable_label(
                                (self.rotate_deg[0] - 90.0).abs() < 1e-3
                                    && self.rotate_deg[1].abs() < 1e-3
                                    && self.rotate_deg[2].abs() < 1e-3,
                                "Y-Up -> Z-Up (+90° X)",
                            )
                            .on_hover_text("Convert Y-up Blender / 3D-print STLs to CNC Z-up")
                            .clicked()
                        {
                            self.rotate_deg = [90.0, 0.0, 0.0];
                        }
                        if ui
                            .small_button("+90° X")
                            .on_hover_text("Rotate imported mesh +90 degrees around the X axis")
                            .clicked()
                        {
                            self.rotate_deg[0] = (self.rotate_deg[0] + 90.0) % 360.0;
                        }
                        if ui
                            .small_button("+90° Y")
                            .on_hover_text("Rotate imported mesh +90 degrees around the Y axis")
                            .clicked()
                        {
                            self.rotate_deg[1] = (self.rotate_deg[1] + 90.0) % 360.0;
                        }
                        if ui
                            .small_button("+90° Z")
                            .on_hover_text("Rotate imported mesh +90 degrees around the Z axis")
                            .clicked()
                        {
                            self.rotate_deg[2] = (self.rotate_deg[2] + 90.0) % 360.0;
                        }
                    });
                    ui.add_space(4.0);
                    ui.horizontal(|ui| {
                        ui.label("Scale:");
                        ui.add(
                            egui::DragValue::new(&mut self.uniform_scale)
                                .speed(0.01)
                                .range(MIN_MODEL_SCALE..=MAX_MODEL_SCALE)
                                .suffix("×")
                                .max_decimals(4),
                        )
                        .on_hover_text("Uniform scale multiplier applied to the mesh on import");
                        if (self.uniform_scale - 1.0).abs() > 1e-6
                            && ui
                                .small_button("1.0×")
                                .on_hover_text("Reset import scale factor back to 1.0x")
                                .clicked()
                        {
                            self.uniform_scale = 1.0;
                        }
                        ui.add_space(8.0);
                        ui.checkbox(&mut self.center_xy, "Center XY")
                            .on_hover_text("Translate the mesh bounding box center to X = 0, Y = 0");
                        ui.checkbox(&mut self.drop_z0, "Drop Bottom to Z = 0")
                            .on_hover_text("Rest the lowest point of the imported mesh on Z = 0");
                    });
                });

                ui.add_space(4.0);

                static_card(ui, Some("Physical Size & Mesh Diagnostics"), |ui| {
                    if let Some(err) = &self.preview_error {
                        ui.colored_label(tok.error_fg, format!("Parse error: {err}"));
                    } else if let Some(mesh) = &self.preview_mesh {
                        let (_, eff_aabb) = self.compute_effective_transform_and_aabb(mesh);
                        let sz = if eff_aabb.is_valid() {
                            eff_aabb.size()
                        } else {
                            mesh.aabb_mm.size()
                        };
                        let d = &mesh.diagnostics;
                        egui::Grid::new("mesh_import_diag_grid")
                            .num_columns(2)
                            .spacing([12.0, 4.0])
                            .show(ui, |ui| {
                                ui.weak("Format:");
                                ui.strong(mesh.format.label());
                                ui.end_row();

                                ui.weak("Physical Size (mm):");
                                ui.strong(format!(
                                    "{:.2} × {:.2} × {:.2} mm",
                                    sz.x, sz.y, sz.z
                                ));
                                ui.end_row();

                                ui.weak("Physical Size (in):");
                                ui.label(format!(
                                    "{:.3} × {:.3} × {:.3} in",
                                    sz.x / 25.4,
                                    sz.y / 25.4,
                                    sz.z / 25.4
                                ));
                                ui.end_row();

                                ui.weak("Display Size:");
                                ui.label(format!(
                                    "{} × {} × {}",
                                    display_unit.format_length(sz.x),
                                    display_unit.format_length(sz.y),
                                    display_unit.format_length(sz.z)
                                ));
                                ui.end_row();

                                ui.weak("Triangles / Vertices:");
                                ui.label(format!(
                                    "{} triangles · {} welded vertices ({} degenerate removed)",
                                    d.valid_triangle_count,
                                    d.welded_vertex_count,
                                    d.degenerate_triangles_removed
                                ));
                                ui.end_row();

                                ui.weak("Manifold Status:");
                                if d.is_watertight {
                                    ui.colored_label(
                                        tok.clean_fg,
                                        format!(
                                            "Watertight (0 open edges · {:.2} cm³)",
                                            d.signed_volume_mm3 / 1000.0
                                        ),
                                    );
                                } else {
                                    ui.colored_label(
                                        tok.warn_fg,
                                        format!(
                                            "Open / Non-Manifold ({} open · {} non-manifold edges)",
                                            d.open_boundary_edges, d.non_manifold_edges
                                        ),
                                    );
                                }
                                ui.end_row();
                            });
                    } else {
                        ui.weak("Select an .stl, .obj, or .3mf file to preview.");
                    }
                });

                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    if ui
                        .button("Cancel")
                        .on_hover_text("Close the mesh import dialog without modifying the workspace")
                        .clicked()
                    {
                        close_clicked = true;
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        let can_import =
                            self.preview_mesh.is_some() && self.preview_error.is_none();
                        ui.add_enabled_ui(can_import, |ui| {
                            if primary_icon_button(
                                ui,
                                IconKind::StepModel,
                                "Import 3D Mesh",
                                "Load this triangle mesh into the active ChipCutter workspace",
                            )
                            .clicked()
                            {
                                let unit_override = if self.detected_unit == Some(self.selected_unit)
                                {
                                    None
                                } else {
                                    Some(self.selected_unit)
                                };
                                let transform = self
                                    .preview_mesh
                                    .as_ref()
                                    .map_or(ModelTransform::IDENTITY, |m| {
                                        self.compute_effective_transform_and_aabb(m).0
                                    });
                                commit = Some(MeshImportCommit {
                                    source_name: self.source_label.clone(),
                                    source_path: self.source_path.clone(),
                                    raw_bytes: self.raw_bytes.clone(),
                                    unit_override,
                                    transform,
                                });
                            }
                        });
                    });
                });
            });

        if close_clicked || commit.is_some() {
            keep_open = false;
        }
        self.open = keep_open;
        commit
    }
}
