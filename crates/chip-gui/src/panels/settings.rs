//! Unified **Application Settings** modal (`prefs.toml`-backed) consolidating infrequently changed
//! Appearance, Viewport Navigation, Simulation Engine/Resolution/Skinning, and Machine Defaults.

use chip_machines::MachineDefinition;
use chip_math::DisplayUnit;
use chip_sim::{ActiveSimBackendInfo, SimComputeBackendChoice, SimEngineKind, SimResolution};
use egui::{CornerRadius, Stroke};

use crate::{
    camera::{MousePreset, OrbitMode},
    prefs::UserPreferences,
    theme::{AppTheme, ThemeTokens},
    widgets::{INPUT_COL_WIDTH, grid_row, inspector_grid, section_card},
};

const SETTINGS_COMBO_W: f32 = INPUT_COL_WIDTH + 116.0;
const APPLICATION_SETTINGS_WINDOW_ID: &str = "chipcutter_application_settings_modal";

/// Category sections in the Settings sidebar.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SettingsSection {
    #[default]
    General,
    Viewport,
    Simulation,
    Machine,
}

impl SettingsSection {
    pub const ALL: [Self; 4] = [
        Self::General,
        Self::Viewport,
        Self::Simulation,
        Self::Machine,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::General => "General & Appearance",
            Self::Viewport => "Viewport & Navigation",
            Self::Simulation => "Simulation",
            Self::Machine => "Machine Defaults",
        }
    }
}

/// Persistent UI state for the Settings window.
#[derive(Debug, Default)]
pub struct SettingsModalState {
    pub open: bool,
    pub active_section: SettingsSection,
}

impl SettingsModalState {
    pub fn open_section(&mut self, section: SettingsSection) {
        self.active_section = section;
        self.open = true;
    }
}

/// Mutable & read-only references provided to [`show_settings_modal`].
pub struct SettingsModalCtx<'a> {
    pub theme: AppTheme,
    pub project_units: DisplayUnit,
    pub prefs: &'a mut UserPreferences,
    pub mouse_preset: &'a mut MousePreset,
    pub orbit_mode: &'a mut OrbitMode,
    pub orthographic: bool,
    pub show_z_planes: &'a mut bool,
    pub show_all_setup_toolpaths: &'a mut bool,
    pub show_grid: &'a mut bool,
    pub sim_engine: &'a mut SimEngineKind,
    pub sim_resolution: &'a mut SimResolution,
    pub sim_backend: &'a mut SimComputeBackendChoice,
    pub sim_mesh_skinning: &'a mut bool,
    pub gpu_adapter_info: Option<&'a ActiveSimBackendInfo>,
    pub machines: &'a [MachineDefinition],
}

/// Side-effect actions emitted by [`show_settings_modal`] back to `ChipCutterApp`.
#[derive(Debug, Default)]
pub struct SettingsModalActions {
    pub theme_changed: Option<AppTheme>,
    pub project_units_changed: Option<DisplayUnit>,
    pub orbit_mode_changed: Option<OrbitMode>,
    pub projection_changed: Option<bool>,
    pub reset_workspace_layout: bool,
    pub sim_rerun_requested: bool,
    pub sim_skinning_changed: bool,
    pub open_machine_manager: bool,
}

/// Renders the unified **Application Settings** dialog when `state.open == true`.
pub fn show_settings_modal(
    ctx: &egui::Context,
    state: &mut SettingsModalState,
    mut sctx: SettingsModalCtx<'_>,
) -> SettingsModalActions {
    let mut actions = SettingsModalActions::default();
    if !state.open {
        return actions;
    }

    if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
        state.open = false;
        return actions;
    }

    let tok = ThemeTokens::for_theme(sctx.theme);
    let mut keep_open = state.open;

    egui::Window::new("Application Settings")
        .id(egui::Id::unique(APPLICATION_SETTINGS_WINDOW_ID))
        .open(&mut keep_open)
        .default_size([760.0, 520.0])
        .min_size([680.0, 440.0])
        .resizable(true)
        .collapsible(false)
        .show(ctx, |ui| {
            let total_w = ui.available_width().max(660.0);
            let body_h = ui.available_height().max(380.0);
            let rail_w = 184.0_f32;
            let gap_w = 10.0_f32;
            let content_w = (total_w - rail_w - gap_w - 2.0).max(360.0);

            ui.horizontal_top(|ui| {
                ui.spacing_mut().item_spacing.x = gap_w;

                // Left category rail
                ui.allocate_ui_with_layout(
                    egui::vec2(rail_w, body_h),
                    egui::Layout::top_down(egui::Align::LEFT),
                    |ui| {
                        let inner_rail_w = (rail_w - 14.0).max(120.0);
                        let inner_rail_h = (body_h - 14.0).max(200.0);
                        egui::Frame::NONE
                            .fill(tok.panel_bg)
                            .stroke(Stroke::new(1.0, tok.border))
                            .corner_radius(CornerRadius::same(6))
                            .inner_margin(egui::Margin::same(6))
                            .show(ui, |ui| {
                                ui.set_min_width(inner_rail_w);
                                ui.set_max_width(inner_rail_w);
                                ui.set_min_height(inner_rail_h);
                                ui.spacing_mut().item_spacing.y = 4.0;
                                for section in SettingsSection::ALL {
                                    let selected = state.active_section == section;
                                    if ui
                                        .add_sized(
                                            [inner_rail_w, 28.0],
                                            egui::Button::selectable(selected, section.label()),
                                        )
                                        .clicked()
                                    {
                                        state.active_section = section;
                                    }
                                }
                            });
                    },
                );

                // Right scrollable content pane (explicit top-down layout & width so
                // section_card / inspector_grid never collapse or creep inside horizontal_top)
                ui.allocate_ui_with_layout(
                    egui::vec2(content_w, body_h),
                    egui::Layout::top_down(egui::Align::LEFT),
                    |ui| {
                        egui::ScrollArea::vertical()
                            .id_salt(("settings_scroll", state.active_section as u8))
                            .auto_shrink([false, false])
                            .show(ui, |ui| {
                                ui.set_width((content_w - 14.0).max(320.0));
                                match state.active_section {
                                    SettingsSection::General => {
                                        show_general_section(ui, &mut sctx, &mut actions);
                                    }
                                    SettingsSection::Viewport => {
                                        show_viewport_section(ui, &tok, &mut sctx, &mut actions);
                                    }
                                    SettingsSection::Simulation => {
                                        show_simulation_section(ui, &tok, &mut sctx, &mut actions);
                                    }
                                    SettingsSection::Machine => {
                                        show_machine_section(ui, &tok, &mut sctx, &mut actions);
                                    }
                                }
                            });
                    },
                );
            });
        });

    state.open = keep_open;
    actions
}

fn show_general_section(
    ui: &mut egui::Ui,
    sctx: &mut SettingsModalCtx<'_>,
    actions: &mut SettingsModalActions,
) {
    section_card(
        ui,
        "settings_appearance_card",
        "Appearance & Theme",
        true,
        |ui| {
            inspector_grid(ui, "settings_appearance_grid", |ui| {
                grid_row(
                    ui,
                    "Workspace Theme",
                    Some("Switch between Studio Dark and high-contrast Daylight Light palettes"),
                    |ui| {
                        let mut chosen = sctx.theme;
                        egui::ComboBox::from_id_salt("settings_theme_cb")
                            .width(SETTINGS_COMBO_W)
                            .selected_text(chosen.label())
                            .show_ui(ui, |ui| {
                                for t in AppTheme::ALL {
                                    ui.selectable_value(&mut chosen, t, t.label());
                                }
                            });
                        if chosen != sctx.theme {
                            actions.theme_changed = Some(chosen);
                        }
                    },
                );
                grid_row(
                    ui,
                    "Startup Screen",
                    Some(
                        "Show the Welcome & Quick-Start overlay when launching with no file argument",
                    ),
                    |ui| {
                        ui.checkbox(
                            &mut sctx.prefs.show_welcome_on_startup,
                            "Show Welcome Gallery on startup",
                        );
                    },
                );
                grid_row(
                    ui,
                    "Workspace Layout",
                    Some(
                        "Restore default Browser, Viewport, Inspector, and Bottom Dock pane shares",
                    ),
                    |ui| {
                        if ui
                            .button("Reset Default Workspace Layout")
                            .on_hover_text("Restore default panel docking sizes and tab positions")
                            .clicked()
                        {
                            actions.reset_workspace_layout = true;
                        }
                    },
                );
            });
        },
    );

    ui.add_space(6.0);

    section_card(ui, "settings_units_card", "Display Units", true, |ui| {
        inspector_grid(ui, "settings_units_grid", |ui| {
            grid_row(
                ui,
                "Current Project Units",
                Some(
                    "Display lengths and feedrates for the currently open project (stored internally in mm)",
                ),
                |ui| {
                    let mut u_sel = sctx.project_units;
                    ui.horizontal(|ui| {
                        ui.selectable_value(
                            &mut u_sel,
                            DisplayUnit::Millimeter,
                            "Millimeters (mm)",
                        );
                        ui.selectable_value(&mut u_sel, DisplayUnit::Inch, "Inches (in)");
                    });
                    if u_sel != sctx.project_units {
                        actions.project_units_changed = Some(u_sel);
                    }
                },
            );
            grid_row(
                ui,
                "Default for New Projects",
                Some("Preferred display unit applied when creating a new blank project"),
                |ui| {
                    ui.horizontal(|ui| {
                        ui.selectable_value(
                            &mut sctx.prefs.default_units,
                            DisplayUnit::Millimeter,
                            "Millimeters (mm)",
                        );
                        ui.selectable_value(
                            &mut sctx.prefs.default_units,
                            DisplayUnit::Inch,
                            "Inches (in)",
                        );
                    });
                },
            );
        });
    });
}

fn show_viewport_section(
    ui: &mut egui::Ui,
    tok: &ThemeTokens,
    sctx: &mut SettingsModalCtx<'_>,
    actions: &mut SettingsModalActions,
) {
    section_card(
        ui,
        "settings_nav_card",
        "3D Navigation & Camera",
        true,
        |ui| {
            inspector_grid(ui, "settings_nav_grid", |ui| {
                grid_row(
                    ui,
                    "Mouse Preset",
                    Some("Choose CAD/CAM mouse orbit, pan, and zoom bindings"),
                    |ui| {
                        egui::ComboBox::from_id_salt("settings_mouse_preset_cb")
                            .width(SETTINGS_COMBO_W)
                            .selected_text(sctx.mouse_preset.label())
                            .show_ui(ui, |ui| {
                                for preset in MousePreset::ALL {
                                    ui.selectable_value(sctx.mouse_preset, preset, preset.label());
                                }
                            });
                    },
                );
                grid_row(
                    ui,
                    "Orbit Mode",
                    Some(
                        "Keep world +Z upright (Turntable) or allow free 3-axis trackball rotation",
                    ),
                    |ui| {
                        let mut om = *sctx.orbit_mode;
                        egui::ComboBox::from_id_salt("settings_orbit_mode_cb")
                            .width(SETTINGS_COMBO_W)
                            .selected_text(om.label())
                            .show_ui(ui, |ui| {
                                for mode in [OrbitMode::TurntableZUp, OrbitMode::FreeTrackball] {
                                    ui.selectable_value(&mut om, mode, mode.label());
                                }
                            });
                        if om != *sctx.orbit_mode {
                            actions.orbit_mode_changed = Some(om);
                        }
                    },
                );
                grid_row(
                    ui,
                    "Camera Projection",
                    Some(
                        "Parallel orthographic projection or perspective foreshortening (Shortcut: P)",
                    ),
                    |ui| {
                        let mut ortho = sctx.orthographic;
                        ui.horizontal(|ui| {
                            ui.selectable_value(&mut ortho, true, "Orthographic");
                            ui.selectable_value(&mut ortho, false, "Perspective");
                        });
                        if ortho != sctx.orthographic {
                            actions.projection_changed = Some(ortho);
                        }
                    },
                );
            });
            ui.add_space(4.0);
            let hint = match *sctx.mouse_preset {
                MousePreset::Fusion360 => {
                    "Fusion 360: Shift+Middle Orbit · Middle Pan · Scroll Zoom"
                }
                MousePreset::SolidWorks => {
                    "SolidWorks: Middle Orbit · Ctrl+Middle Pan · Scroll Zoom"
                }
                MousePreset::FreeCadBlender => {
                    "FreeCAD / Blender: Middle Orbit · Shift+Middle Pan · Scroll Zoom"
                }
            };
            ui.label(egui::RichText::new(hint).small().color(tok.text_muted));
        },
    );

    ui.add_space(6.0);

    section_card(
        ui,
        "settings_vp_overlays_card",
        "Viewport Display Defaults",
        true,
        |ui| {
            ui.checkbox(
            sctx.show_all_setup_toolpaths,
            "Show all visible operations in active Setup",
        )
        .on_hover_text(
            "When checked, render toolpaths for every visible operation in the active Setup; when unchecked, show only the selected operation",
        );
            ui.checkbox(sctx.show_grid, "Show active Setup WCS XY reference grid")
                .on_hover_text(
                    "Display the coordinate reference grid on the active Setup's Z = 0 plane",
                );
            ui.checkbox(
                sctx.show_z_planes,
                "Show Z-height planes when editing Heights tab",
            )
            .on_hover_text("Render translucent Clearance, Retract, Top, and Bottom Z planes");
            ui.checkbox(
                &mut sctx.prefs.auto_rotate_indexed_op,
                "Auto-rotate workpiece to active Indexed 3+1 angle",
            )
            .on_hover_text(
                "Automatically rotate the displayed 3D model, raw setup stock, chuck jaws, Z planes, and boundary preview to the selected Indexed 3+1 operation's index angle",
            );
        },
    );
}

fn show_simulation_section(
    ui: &mut egui::Ui,
    tok: &ThemeTokens,
    sctx: &mut SettingsModalCtx<'_>,
    actions: &mut SettingsModalActions,
) {
    section_card(
        ui,
        "settings_sim_engine_card",
        "Simulation Engine & Detail Level",
        true,
        |ui| {
            let prev_engine = *sctx.sim_engine;
            let prev_res = *sctx.sim_resolution;
            let prev_backend = *sctx.sim_backend;
            let prev_skinning = *sctx.sim_mesh_skinning;

            inspector_grid(ui, "settings_sim_grid", |ui| {
                grid_row(
                    ui,
                    "Simulation Engine",
                    Some(
                        "Tri-Dexel models full 3D overhangs/undercuts and 2-sided carryover; Heightmap is fastest for 2.5D top-down cuts",
                    ),
                    |ui| {
                        egui::ComboBox::from_id_salt("settings_sim_engine_cb")
                            .width(SETTINGS_COMBO_W)
                            .selected_text(sctx.sim_engine.label())
                            .show_ui(ui, |ui| {
                                for eng in SimEngineKind::ALL {
                                    ui.selectable_value(sctx.sim_engine, eng, eng.label())
                                        .on_hover_text(
                                            crate::panels::simulation::sim_engine_description(eng),
                                        );
                                }
                            });
                    },
                );
                let active_eng = *sctx.sim_engine;
                grid_row(
                    ui,
                    "Detail Level",
                    Some(
                        "Volumetric grid resolution (higher detail yields sharper edges and tighter error metrology)",
                    ),
                    |ui| {
                        egui::ComboBox::from_id_salt("settings_sim_res_cb")
                            .width(SETTINGS_COMBO_W)
                            .selected_text(
                                crate::panels::simulation::sim_resolution_label_for_engine(
                                    *sctx.sim_resolution,
                                    active_eng,
                                ),
                            )
                            .show_ui(ui, |ui| {
                                for r in SimResolution::ALL {
                                    ui.selectable_value(
                                        sctx.sim_resolution,
                                        r,
                                        crate::panels::simulation::sim_resolution_label_for_engine(
                                            r, active_eng,
                                        ),
                                    )
                                    .on_hover_text(
                                        crate::panels::simulation::sim_resolution_description_for_engine(
                                            r, active_eng,
                                        ),
                                    );
                                }
                            });
                    },
                );
                grid_row(
                    ui,
                    "Compute Backend",
                    Some("Choose GPU compute (wgpu) or exact CPU + Rayon execution"),
                    |ui| {
                        let backend_label = |c: SimComputeBackendChoice| match c {
                            SimComputeBackendChoice::Auto => "Auto (GPU with CPU fallback)",
                            SimComputeBackendChoice::Gpu => "GPU (Compute)",
                            SimComputeBackendChoice::Cpu => "CPU (Exact)",
                        };
                        egui::ComboBox::from_id_salt("settings_sim_backend_cb")
                            .width(SETTINGS_COMBO_W)
                            .selected_text(backend_label(*sctx.sim_backend))
                            .show_ui(ui, |ui| {
                                for c in SimComputeBackendChoice::ALL {
                                    ui.selectable_value(sctx.sim_backend, c, backend_label(c));
                                }
                            });
                    },
                );
                grid_row(
                    ui,
                    "Surface Display",
                    Some(
                        "Apply crease-preserving mesh skinning to show smoother curved surfaces while keeping 90° edges sharp",
                    ),
                    |ui| {
                        ui.checkbox(
                            sctx.sim_mesh_skinning,
                            "Smooth Skinned Surfaces (Mesh Skinning)",
                        )
                        .on_hover_text(
                            "Visually relaxes smooth curved/helical simulation surfaces and interpolates crease-aware vertex normals. Strictly visual — never alters volumetric gouge or error metrology.",
                        );
                    },
                );
            });

            if *sctx.sim_engine != prev_engine
                || *sctx.sim_resolution != prev_res
                || *sctx.sim_backend != prev_backend
            {
                actions.sim_rerun_requested = true;
            }
            if *sctx.sim_mesh_skinning != prev_skinning {
                actions.sim_skinning_changed = true;
            }

            ui.add_space(6.0);
            let adapter_line = if let Some(info) = sctx.gpu_adapter_info {
                format!(
                    "Detected GPU adapter: {} ({})",
                    info.adapter_name, info.backend_api
                )
            } else {
                "Detected GPU adapter: none (CPU + Rayon fallback active)".to_string()
            };
            ui.label(
                egui::RichText::new(adapter_line)
                    .small()
                    .color(tok.text_muted),
            );
            ui.label(
                egui::RichText::new(format!(
                    "Active detail: {}",
                    crate::panels::simulation::sim_resolution_description_for_engine(
                        *sctx.sim_resolution,
                        *sctx.sim_engine
                    )
                ))
                .small()
                .color(tok.text_muted),
            );
        },
    );
}

fn show_machine_section(
    ui: &mut egui::Ui,
    tok: &ThemeTokens,
    sctx: &mut SettingsModalCtx<'_>,
    actions: &mut SettingsModalActions,
) {
    section_card(
        ui,
        "settings_machine_card",
        "Default CNC Machine",
        true,
        |ui| {
            let current_machine = sctx
                .machines
                .iter()
                .find(|m| m.id.eq_ignore_ascii_case(&sctx.prefs.default_machine_id))
                .or_else(|| sctx.machines.first());

            inspector_grid(ui, "settings_machine_grid", |ui| {
                grid_row(
                    ui,
                    "Default Machine",
                    Some(
                        "Target CNC machine profile assigned automatically to newly created projects",
                    ),
                    |ui| {
                        let selected_label = current_machine
                            .map_or(sctx.prefs.default_machine_id.as_str(), |m| m.name.as_str());
                        egui::ComboBox::from_id_salt("settings_default_machine_cb")
                            .width(SETTINGS_COMBO_W)
                            .selected_text(selected_label)
                            .show_ui(ui, |ui| {
                                for m in sctx.machines {
                                    let tag = if m.is_builtin {
                                        "[Built-In]"
                                    } else {
                                        "[Custom]"
                                    };
                                    ui.selectable_value(
                                        &mut sctx.prefs.default_machine_id,
                                        m.id.clone(),
                                        format!("{} {}", m.name, tag),
                                    );
                                }
                            });
                    },
                );
            });

            if let Some(m) = current_machine {
                ui.add_space(4.0);
                ui.label(
                    egui::RichText::new(format!(
                        "Travel {:.0}×{:.0}×{:.0} mm · {:.0}–{:.0} RPM · {} tools · Post: {}",
                        m.travel_mm.x_mm,
                        m.travel_mm.y_mm,
                        m.travel_mm.z_mm,
                        m.spindle.min_rpm,
                        m.spindle.max_rpm,
                        m.tools.len(),
                        m.default_post_profile
                    ))
                    .small()
                    .color(tok.text_muted),
                );
            }

            ui.add_space(6.0);
            if ui
                .button("Open Machine Manager…")
                .on_hover_text(
                    "Inspect, clone, edit, import, or export CNC machine profiles and their tool libraries",
                )
                .clicked()
            {
                actions.open_machine_manager = true;
            }
        },
    );
}

#[cfg(test)]
mod tests {
    use chip_math::DisplayUnit;
    use chip_sim::{SimComputeBackendChoice, SimEngineKind, SimResolution};

    use super::{SettingsModalCtx, SettingsModalState, SettingsSection, show_settings_modal};
    use crate::{
        camera::{MousePreset, OrbitMode},
        prefs::UserPreferences,
        theme::AppTheme,
    };

    fn collect_text_shapes(shapes: &[egui::epaint::ClippedShape]) -> Vec<(String, egui::Rect)> {
        let mut out = Vec::new();
        for cs in shapes {
            collect_shape_text(&cs.shape, cs.clip_rect, &mut out);
        }
        out
    }

    fn collect_shape_text(
        shape: &egui::Shape,
        clip: egui::Rect,
        out: &mut Vec<(String, egui::Rect)>,
    ) {
        match shape {
            egui::Shape::Text(ts) => {
                let rect = ts.visual_bounding_rect().intersect(clip);
                out.push((ts.galley.text().to_string(), rect));
            }
            egui::Shape::Vec(v) => {
                for s in v {
                    collect_shape_text(s, clip, out);
                }
            }
            _ => {}
        }
    }

    #[test]
    fn settings_modal_renders_visible_controls_in_all_four_sections() {
        let machines = chip_machines::builtin_machine_definitions();
        let expected = [
            (SettingsSection::General, "Workspace Theme"),
            (SettingsSection::Viewport, "Mouse Preset"),
            (
                SettingsSection::Simulation,
                "Smooth Skinned Surfaces (Mesh Skinning)",
            ),
            (SettingsSection::Machine, "Default Machine"),
        ];

        for (section, needle) in expected {
            let ctx = egui::Context::default();
            let mut state = SettingsModalState::default();
            state.open_section(section);

            let mut prefs = UserPreferences::default();
            let mut mouse_preset = MousePreset::Fusion360;
            let mut orbit_mode = OrbitMode::TurntableZUp;
            let mut show_z_planes = true;
            let mut show_all_setup_toolpaths = true;
            let mut show_grid = true;
            let mut sim_engine = SimEngineKind::TriDexel;
            let mut sim_resolution = SimResolution::Standard;
            let mut sim_backend = SimComputeBackendChoice::Auto;
            let mut sim_mesh_skinning = true;

            let mut last_shapes = Vec::new();
            for _ in 0..2 {
                let input = egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(
                        egui::Pos2::ZERO,
                        egui::Vec2::new(1280.0, 800.0),
                    )),
                    ..Default::default()
                };
                let mut out = ctx.run_ui(input, |ui| {
                    let _ = show_settings_modal(
                        ui.ctx(),
                        &mut state,
                        SettingsModalCtx {
                            theme: AppTheme::Dark,
                            project_units: DisplayUnit::Millimeter,
                            prefs: &mut prefs,
                            mouse_preset: &mut mouse_preset,
                            orbit_mode: &mut orbit_mode,
                            orthographic: true,
                            show_z_planes: &mut show_z_planes,
                            show_all_setup_toolpaths: &mut show_all_setup_toolpaths,
                            show_grid: &mut show_grid,
                            sim_engine: &mut sim_engine,
                            sim_resolution: &mut sim_resolution,
                            sim_backend: &mut sim_backend,
                            sim_mesh_skinning: &mut sim_mesh_skinning,
                            gpu_adapter_info: None,
                            machines: &machines,
                        },
                    );
                });
                out.textures_delta.clear();
                last_shapes = out.shapes;
            }

            let texts = collect_text_shapes(&last_shapes);
            assert!(
                texts
                    .iter()
                    .any(|(t, r)| t.contains(needle) && r.width() > 20.0 && r.height() > 5.0),
                "section {section:?} must render visible control containing '{needle}'; found: {:?}",
                texts.iter().map(|(t, r)| (t, r.size())).collect::<Vec<_>>()
            );
            assert!(
                texts.iter().any(|(t, _)| t == "Application Settings"),
                "dialog title must be 'Application Settings'"
            );
        }
    }

    #[test]
    #[allow(clippy::expect_used)]
    fn application_settings_window_opens_full_width_with_zero_creep_for_30_frames() {
        let machines = chip_machines::builtin_machine_definitions();
        let win_id = egui::Id::unique(super::APPLICATION_SETTINGS_WINDOW_ID);

        for section in SettingsSection::ALL {
            let ctx = egui::Context::default();
            let mut state = SettingsModalState::default();
            state.open_section(section);

            let mut prefs = UserPreferences::default();
            let mut mouse_preset = MousePreset::Fusion360;
            let mut orbit_mode = OrbitMode::TurntableZUp;
            let mut show_z_planes = true;
            let mut show_all_setup_toolpaths = true;
            let mut show_grid = true;
            let mut sim_engine = SimEngineKind::TriDexel;
            let mut sim_resolution = SimResolution::Standard;
            let mut sim_backend = SimComputeBackendChoice::Auto;
            let mut sim_mesh_skinning = true;

            let mut rects = Vec::new();
            for frame in 0..30 {
                let input = egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(
                        egui::Pos2::ZERO,
                        egui::Vec2::new(1400.0, 900.0),
                    )),
                    time: Some(frame as f64 * 0.016),
                    ..Default::default()
                };
                let mut out = ctx.run_ui(input, |ui| {
                    let _ = show_settings_modal(
                        ui.ctx(),
                        &mut state,
                        SettingsModalCtx {
                            theme: AppTheme::Dark,
                            project_units: DisplayUnit::Millimeter,
                            prefs: &mut prefs,
                            mouse_preset: &mut mouse_preset,
                            orbit_mode: &mut orbit_mode,
                            orthographic: true,
                            show_z_planes: &mut show_z_planes,
                            show_all_setup_toolpaths: &mut show_all_setup_toolpaths,
                            show_grid: &mut show_grid,
                            sim_engine: &mut sim_engine,
                            sim_resolution: &mut sim_resolution,
                            sim_backend: &mut sim_backend,
                            sim_mesh_skinning: &mut sim_mesh_skinning,
                            gpu_adapter_info: None,
                            machines: &machines,
                        },
                    );
                });
                out.textures_delta.clear();
                let rect = ctx
                    .memory(|m| m.area_rect(win_id))
                    .expect("Application Settings window rect must exist");
                rects.push(rect);
            }

            assert!(
                rects[0].width() >= 720.0,
                "section {section:?}: Frame 1 Application Settings window must open at full width (>= 720 px), got {:.1} px",
                rects[0].width()
            );
            let base_w = rects[1].width();
            let base_h = rects[1].height();
            for (idx, r) in rects.iter().enumerate().skip(2) {
                assert!(
                    (r.width() - base_w).abs() < 1.0 && (r.height() - base_h).abs() < 1.0,
                    "section {section:?}: Application Settings window crept on frame {}: ({:.2}, {:.2}) vs frame 2 ({:.2}, {:.2})",
                    idx + 1,
                    r.width(),
                    r.height(),
                    base_w,
                    base_h
                );
            }
        }
    }
}
