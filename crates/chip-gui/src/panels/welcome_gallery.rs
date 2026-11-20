//! Welcome / Quick-Start Overlay & Filterable Sample Projects Gallery (`08-ui-viewport.md`).

use std::path::{Path, PathBuf};

use egui::{CornerRadius, Margin, Stroke};

use crate::{
    prefs::UserPreferences,
    theme::ThemeTokens,
    widgets::{IconKind, icon_button, inline_icon, primary_icon_button, static_card},
};

/// Identifiers for all built-in showcase and workflow demo projects.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SampleProjectId {
    CamTortureBlock,
    AdaptiveRest,
    Pure2dPanel,
    EngravedNameplate,
    VCarveSignInlay,
    MultiFeatureBracket,
    Tilted30Bracket,
    CadRevisionV1,
    CadRevisionV2Reload,
    SamplePlate,
    SculptedMeshDome,
}

impl SampleProjectId {
    /// Parse a CLI `--sample <SLUG>` identifier into a built-in [`SampleProjectId`].
    pub fn from_cli_slug(raw: &str) -> Result<Self, String> {
        match raw.trim().to_ascii_lowercase().as_str() {
            "cam-torture" | "torture" => Ok(Self::CamTortureBlock),
            "adaptive-rest" | "adaptive" => Ok(Self::AdaptiveRest),
            "pure-2d" | "pure2d" | "panel" => Ok(Self::Pure2dPanel),
            "nameplate" | "engraved-nameplate" => Ok(Self::EngravedNameplate),
            "vcarve" | "vcarve-sign" => Ok(Self::VCarveSignInlay),
            "multi-feature" | "bracket" => Ok(Self::MultiFeatureBracket),
            "tilted-bracket" | "tilted" => Ok(Self::Tilted30Bracket),
            "revision-v1" => Ok(Self::CadRevisionV1),
            "revision-v2" => Ok(Self::CadRevisionV2Reload),
            "sample-plate" | "plate" => Ok(Self::SamplePlate),
            "mesh-dome" | "sculpted-dome" | "dome" => Ok(Self::SculptedMeshDome),
            other => Err(format!(
                "invalid --sample '{other}'; expected one of: cam-torture, adaptive-rest, pure-2d, nameplate, vcarve, multi-feature, tilted-bracket, revision-v1, revision-v2, sample-plate, mesh-dome"
            )),
        }
    }
}

/// Filter categories in the Sample Projects Gallery modal.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SampleCategory {
    #[default]
    All,
    Step3D,
    Mesh3D,
    Pure2dSketch,
    VCarveTypography,
}

impl SampleCategory {
    pub const ALL: [Self; 5] = [
        Self::All,
        Self::Step3D,
        Self::Mesh3D,
        Self::Pure2dSketch,
        Self::VCarveTypography,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::All => "All Samples",
            Self::Step3D => "3D STEP Solids",
            Self::Mesh3D => "3D Meshes",
            Self::Pure2dSketch => "Pure-2D & DXF",
            Self::VCarveTypography => "V-Carve & Text",
        }
    }
}

/// Rich catalog metadata for a single built-in Sample Project card.
pub struct SampleCardSpec {
    pub id: SampleProjectId,
    pub category: SampleCategory,
    pub icon: IconKind,
    pub title: &'static str,
    pub badge: &'static str,
    pub summary: &'static str,
}

pub const SAMPLE_CATALOG: &[SampleCardSpec] = &[
    SampleCardSpec {
        id: SampleProjectId::CamTortureBlock,
        category: SampleCategory::Step3D,
        icon: IconKind::StepModel,
        title: "CAM Torture Block (8 Operations)",
        badge: "3D STEP · G54",
        summary: "Machined B-Rep block exercising Facing, Pocket + Island, Open Shoulder, Helical Bore, 4x Peck Drills, Profile + Tabs, and 45° Bevel Chamfer.",
    },
    SampleCardSpec {
        id: SampleProjectId::AdaptiveRest,
        category: SampleCategory::Step3D,
        icon: IconKind::Adaptive2D,
        title: "2D Adaptive Clearing + Rest Machining",
        badge: "Trochoidal · Rest DAG",
        summary: "High-speed constant-engagement Ø10mm Adaptive roughing followed by Ø4mm Rest Pocket & Rest Contour corner cleanup.",
    },
    SampleCardSpec {
        id: SampleProjectId::VCarveSignInlay,
        category: SampleCategory::VCarveTypography,
        icon: IconKind::VCarve,
        title: "V-Carve Sign & Mated Male V-Inlay",
        badge: "Two-Stage T1+T8 · G54/G55",
        summary: "Hardwood plaque with Ø6mm Flat Endmill Stage 1 pocket clearing + 60° V-Bit Stage 2 bevels/corners and mirrored Male V-Inlay plug.",
    },
    SampleCardSpec {
        id: SampleProjectId::EngravedNameplate,
        category: SampleCategory::VCarveTypography,
        icon: IconKind::Sketch2D,
        title: "Engraved Nameplate (TrueType + Hershey)",
        badge: "Typography · Bolt Circle",
        summary: "Parametric R10 filleted plate with 6x Bolt Circle, TrueType outline lettering with counter-islands, and 1-pass Hershey serial engraving.",
    },
    SampleCardSpec {
        id: SampleProjectId::Pure2dPanel,
        category: SampleCategory::Pure2dSketch,
        icon: IconKind::VectorImport,
        title: "Pure-2D CNC Router Front Panel (.dxf)",
        badge: "Zero-STEP · DXF",
        summary: "Sheet-stock front panel imported directly from AutoCAD DXF with connector cutouts, mounting holes, and outer perimeter tabs.",
    },
    SampleCardSpec {
        id: SampleProjectId::MultiFeatureBracket,
        category: SampleCategory::Step3D,
        icon: IconKind::Pocket2D,
        title: "Prismatic Multi-Feature Bracket",
        badge: "3D STEP · G54",
        summary: "Clean starter 3D STEP bracket demonstrating planar face pocketing, through-hole drilling, and outer contour profiling.",
    },
    SampleCardSpec {
        id: SampleProjectId::Tilted30Bracket,
        category: SampleCategory::Step3D,
        icon: IconKind::OrthoCube,
        title: "30°-Tilted WCS Alignment Bracket",
        badge: "Tilted SE(3) WCS",
        summary: "Demonstrates aligning Setup WCS +Z/+X directly onto a 30°-tilted STEP solid face with tight rotated 27-point stock bounds.",
    },
    SampleCardSpec {
        id: SampleProjectId::CadRevisionV1,
        category: SampleCategory::Step3D,
        icon: IconKind::Reload,
        title: "Associative CAD Revision Reload (v1 -> v2)",
        badge: "Associative B-Rep",
        summary: "Loads Revision v1; use 'Reload Rev v2' here or in Command Palette to watch fuzzy feature re-binding and orphan detection.",
    },
    SampleCardSpec {
        id: SampleProjectId::SamplePlate,
        category: SampleCategory::Step3D,
        icon: IconKind::StepModel,
        title: "CNC Mounting Plate (Blank STEP Sandbox)",
        badge: "Unseeded 3D STEP",
        summary: "Clean unseeded machined STEP mounting plate for practicing interactive 3D face, loop, and hole picking from scratch.",
    },
    SampleCardSpec {
        id: SampleProjectId::SculptedMeshDome,
        category: SampleCategory::Mesh3D,
        icon: IconKind::StepModel,
        title: "Sculpted Dome (Watertight STL Mesh)",
        badge: "3D STL Mesh · G54",
        summary: "R25mm hemisphere seated on a 70×70×8mm base pedestal demonstrating pure-Rust STL import, crease-angle shading, and 3D stock/WCS setup.",
    },
];

/// High-level action emitted by either the Welcome Overlay or the Sample Projects Gallery.
#[derive(Debug, Clone)]
pub enum WelcomeGalleryAction {
    OpenProjectDialog,
    ImportStepDialog,
    ImportMeshDialog,
    ImportVectorDialog,
    StartPure2dSketchJob,
    OpenSampleGallery,
    LoadSample(SampleProjectId),
    OpenRecentPath(PathBuf),
    ClearRecentFiles,
    PrefsChanged,
}

#[derive(Debug, Default)]
pub struct WelcomeModalState {
    pub open: bool,
}

#[derive(Debug, Default)]
pub struct SampleGalleryModalState {
    pub open: bool,
    pub category: SampleCategory,
    pub search_query: String,
}

/// Renders the **Welcome / Quick-Start Overlay** modal.
pub fn show_welcome_modal(
    ctx: &egui::Context,
    state: &mut WelcomeModalState,
    prefs: &mut UserPreferences,
) -> Option<WelcomeGalleryAction> {
    if !state.open {
        return None;
    }

    let mut action = None;
    let mut keep_open = state.open;
    let mut dismiss_clicked = false;
    let tok = ThemeTokens::for_theme(prefs.theme);

    egui::Window::new("Welcome to ChipCutter")
        .id(egui::Id::unique("chipcutter_welcome_modal"))
        .collapsible(false)
        .resizable(false)
        .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
        .default_width(690.0)
        .open(&mut keep_open)
        .show(ctx, |ui| {
            ui.set_min_width(660.0);
            ui.set_max_width(700.0);

            ui.horizontal(|ui| {
                inline_icon(ui, IconKind::Sparkles, tok.accent, 16.0);
                ui.strong("2.5D Milling, Routing, & 3D V-Carve CAM");
                ui.weak("· Offline STEP, DXF/SVG, Parametric Sketches, Stock Sim & Post");
            });
            ui.add_space(4.0);
            ui.separator();
            ui.add_space(4.0);

            ui.columns(2, |cols| {
                // Left column: Start / Open actions
                static_card(&mut cols[0], Some("Start or Open a Job"), |ui| {
                    ui.spacing_mut().item_spacing.y = 6.0;
                    if welcome_action_row(
                        ui,
                        IconKind::FolderOpen,
                        "Open Project or Portable Bundle…",
                        "Load an existing .chip.toml manifest or self-contained .chipz archive (Ctrl+O)",
                    ) {
                        action = Some(WelcomeGalleryAction::OpenProjectDialog);
                    }
                    if welcome_action_row(
                        ui,
                        IconKind::StepModel,
                        "Import 3D STEP Solid…",
                        "Load an ISO 10303-21 (.step / .stp) CAD part and initialize Stock & G54 WCS",
                    ) {
                        action = Some(WelcomeGalleryAction::ImportStepDialog);
                    }
                    if welcome_action_row(
                        ui,
                        IconKind::StepModel,
                        "Import 3D Mesh (STL / OBJ / 3MF)…",
                        "Preview physical size & units on sculpted .stl, .obj, or .3mf triangle meshes",
                    ) {
                        action = Some(WelcomeGalleryAction::ImportMeshDialog);
                    }
                    if welcome_action_row(
                        ui,
                        IconKind::VectorImport,
                        "Import 2D DXF / SVG Artwork…",
                        "Preview physical scale, heal micro-gaps, and create a Pure-2D router/laser/engraving job",
                    ) {
                        action = Some(WelcomeGalleryAction::ImportVectorDialog);
                    }
                    if welcome_action_row(
                        ui,
                        IconKind::Sketch2D,
                        "New Pure-2D Parametric Sketch",
                        "Start on blank sheet stock with interactive Rectangles, Slots, Bolt Circles & CAM Text",
                    ) {
                        action = Some(WelcomeGalleryAction::StartPure2dSketchJob);
                    }
                    if welcome_action_row(
                        ui,
                        IconKind::Sparkles,
                        "Browse Sample Projects Gallery…",
                        "Explore 8 ready-to-machine 3D STEP, Adaptive+Rest, Typography, and V-Inlay demos",
                    ) {
                        action = Some(WelcomeGalleryAction::OpenSampleGallery);
                    }
                });

                // Right column: Recent Projects + Featured Samples
                static_card(&mut cols[1], Some("Recent Projects"), |ui| {
                    if prefs.recent_files.is_empty() {
                        ui.weak("No recent projects opened yet.");
                    } else {
                        let mut clicked_recent: Option<PathBuf> = None;
                        egui::ScrollArea::vertical()
                            .id_salt("welcome_recent_scroll")
                            .max_height(132.0)
                            .show(ui, |ui| {
                                for p in &prefs.recent_files {
                                    let exists = p.exists();
                                    let fname = p
                                        .file_name()
                                        .and_then(|s| s.to_str())
                                        .unwrap_or("project");
                                    let parent_disp = p
                                        .parent()
                                        .map(Path::to_string_lossy)
                                        .unwrap_or_default();
                                    ui.horizontal(|ui| {
                                        let resp = ui
                                            .add_enabled(
                                                exists,
                                                egui::Button::selectable(false, fname).truncate(),
                                            )
                                            .on_hover_text(format!(
                                                "{}{}",
                                                p.display(),
                                                if exists { "" } else { " (missing on disk)" }
                                            ));
                                        if resp.clicked() {
                                            clicked_recent = Some(p.clone());
                                        }
                                        if !exists {
                                            ui.colored_label(tok.warn_fg, "(missing)");
                                        } else {
                                            ui.weak(truncate_left(&parent_disp, 26));
                                        }
                                    });
                                }
                            });
                        if let Some(p) = clicked_recent {
                            action = Some(WelcomeGalleryAction::OpenRecentPath(p));
                        }
                        if ui.small_button("Clear Recent List").clicked() {
                            action = Some(WelcomeGalleryAction::ClearRecentFiles);
                        }
                    }
                });

                static_card(&mut cols[1], Some("Quick-Launch Featured Demos"), |ui| {
                    ui.spacing_mut().item_spacing.y = 5.0;
                    for (id, icon, lbl) in [
                        (
                            SampleProjectId::CamTortureBlock,
                            IconKind::StepModel,
                            "CAM Torture Block (3D STEP · 8 Ops)",
                        ),
                        (
                            SampleProjectId::AdaptiveRest,
                            IconKind::Adaptive2D,
                            "2D Adaptive Roughing + Rest Machining",
                        ),
                        (
                            SampleProjectId::VCarveSignInlay,
                            IconKind::VCarve,
                            "V-Carve Sign & Mated Male V-Inlay",
                        ),
                    ] {
                        if icon_button(ui, icon, Some(lbl), "Load ready-to-simulate sample job")
                            .clicked()
                        {
                            action = Some(WelcomeGalleryAction::LoadSample(id));
                        }
                    }
                });
            });

            ui.separator();
            ui.horizontal(|ui| {
                if ui
                    .checkbox(
                        &mut prefs.show_welcome_on_startup,
                        "Show Welcome overlay on bare startup",
                    )
                    .changed()
                {
                    action = Some(WelcomeGalleryAction::PrefsChanged);
                }
                ui.weak("· Tip: Ctrl+K Command Palette · F1 Shortcuts");
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.button("Continue to Workspace").clicked() {
                        dismiss_clicked = true;
                    }
                });
            });
        });

    if dismiss_clicked
        || matches!(
            action,
            Some(
                WelcomeGalleryAction::OpenProjectDialog
                    | WelcomeGalleryAction::ImportStepDialog
                    | WelcomeGalleryAction::ImportMeshDialog
                    | WelcomeGalleryAction::ImportVectorDialog
                    | WelcomeGalleryAction::StartPure2dSketchJob
                    | WelcomeGalleryAction::OpenSampleGallery
                    | WelcomeGalleryAction::LoadSample(_)
                    | WelcomeGalleryAction::OpenRecentPath(_)
            )
        )
    {
        keep_open = false;
    }

    state.open = keep_open;
    action
}

/// Renders the filterable **Sample Projects Gallery** modal.
pub fn show_sample_gallery_modal(
    ctx: &egui::Context,
    state: &mut SampleGalleryModalState,
) -> Option<SampleProjectId> {
    if !state.open {
        return None;
    }

    let mut picked: Option<SampleProjectId> = None;
    let mut keep_open = state.open;

    egui::Window::new("Sample Projects Gallery")
        .id(egui::Id::unique("chipcutter_sample_gallery_modal"))
        .collapsible(false)
        .resizable(true)
        .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
        .default_width(720.0)
        .open(&mut keep_open)
        .show(ctx, |ui| {
            let tok = ThemeTokens::from_ui(ui);
            ui.set_min_width(660.0);

            ui.horizontal(|ui| {
                for cat in SampleCategory::ALL {
                    ui.selectable_value(&mut state.category, cat, cat.label());
                }
                ui.separator();
                inline_icon(ui, IconKind::Search, tok.text_muted, 13.0);
                ui.add(
                    egui::TextEdit::singleline(&mut state.search_query)
                        .hint_text("Filter samples…")
                        .desired_width(150.0),
                );
            });
            ui.separator();

            let q = state.search_query.trim().to_ascii_lowercase();
            let filtered: Vec<&SampleCardSpec> = SAMPLE_CATALOG
                .iter()
                .filter(|s| {
                    (state.category == SampleCategory::All || s.category == state.category)
                        && (q.is_empty()
                            || s.title.to_ascii_lowercase().contains(&q)
                            || s.summary.to_ascii_lowercase().contains(&q)
                            || s.badge.to_ascii_lowercase().contains(&q))
                })
                .collect();

            egui::ScrollArea::vertical()
                .id_salt("sample_gallery_scroll")
                .max_height(430.0)
                .show(ui, |ui| {
                    for pair in filtered.chunks(2) {
                        ui.columns(2, |cols| {
                            for (col_idx, spec) in pair.iter().enumerate() {
                                egui::Frame::NONE
                                    .fill(tok.card_bg)
                                    .stroke(Stroke::new(1.0, tok.border))
                                    .corner_radius(CornerRadius::same(6))
                                    .inner_margin(Margin::symmetric(10, 8))
                                    .show(&mut cols[col_idx], |ui| {
                                        ui.horizontal(|ui| {
                                            inline_icon(ui, spec.icon, tok.accent, 15.0);
                                            ui.strong(spec.title);
                                        });
                                        ui.colored_label(tok.info_fg, spec.badge);
                                        ui.add_space(2.0);
                                        ui.label(spec.summary);
                                        ui.add_space(5.0);
                                        ui.horizontal(|ui| {
                                            if primary_icon_button(
                                                ui,
                                                IconKind::FolderOpen,
                                                "Open Sample",
                                                "Load this sample job into the workspace",
                                            )
                                            .clicked()
                                            {
                                                picked = Some(spec.id);
                                            }
                                            if spec.id == SampleProjectId::CadRevisionV1
                                                && ui
                                                    .button("Reload Rev v2")
                                                    .on_hover_text(
                                                        "Associatively reload CAD Revision v2 onto the active v1 project",
                                                    )
                                                    .clicked()
                                            {
                                                picked = Some(SampleProjectId::CadRevisionV2Reload);
                                            }
                                        });
                                    });
                            }
                        });
                        ui.add_space(6.0);
                    }
                });
        });

    if picked.is_some() {
        keep_open = false;
    }
    state.open = keep_open;
    picked
}

fn welcome_action_row(ui: &mut egui::Ui, icon: IconKind, title: &str, subtitle: &str) -> bool {
    let tok = ThemeTokens::from_ui(ui);
    let mut clicked = false;
    egui::Frame::NONE
        .fill(tok.elevated_bg)
        .stroke(Stroke::new(1.0, tok.border))
        .corner_radius(CornerRadius::same(5))
        .inner_margin(Margin::symmetric(8, 6))
        .show(ui, |ui| {
            ui.set_min_width(ui.available_width());
            ui.horizontal(|ui| {
                inline_icon(ui, icon, tok.accent, 15.0);
                if ui.link(egui::RichText::new(title).strong()).clicked() {
                    clicked = true;
                }
            });
            ui.small(subtitle);
        });
    clicked
}

fn truncate_left(s: &str, max_chars: usize) -> String {
    let count = s.chars().count();
    if count <= max_chars {
        s.to_string()
    } else {
        let tail: String = s.chars().skip(count - max_chars).collect();
        format!("…{tail}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_all_sample_cli_slugs() {
        assert_eq!(
            SampleProjectId::from_cli_slug("cam-torture"),
            Ok(SampleProjectId::CamTortureBlock)
        );
        assert_eq!(
            SampleProjectId::from_cli_slug("adaptive-rest"),
            Ok(SampleProjectId::AdaptiveRest)
        );
        assert_eq!(
            SampleProjectId::from_cli_slug("pure-2d"),
            Ok(SampleProjectId::Pure2dPanel)
        );
        assert_eq!(
            SampleProjectId::from_cli_slug("nameplate"),
            Ok(SampleProjectId::EngravedNameplate)
        );
        assert_eq!(
            SampleProjectId::from_cli_slug("vcarve"),
            Ok(SampleProjectId::VCarveSignInlay)
        );
        assert_eq!(
            SampleProjectId::from_cli_slug("multi-feature"),
            Ok(SampleProjectId::MultiFeatureBracket)
        );
        assert_eq!(
            SampleProjectId::from_cli_slug("tilted-bracket"),
            Ok(SampleProjectId::Tilted30Bracket)
        );
        assert_eq!(
            SampleProjectId::from_cli_slug("revision-v1"),
            Ok(SampleProjectId::CadRevisionV1)
        );
        assert_eq!(
            SampleProjectId::from_cli_slug("revision-v2"),
            Ok(SampleProjectId::CadRevisionV2Reload)
        );
        assert_eq!(
            SampleProjectId::from_cli_slug("sample-plate"),
            Ok(SampleProjectId::SamplePlate)
        );
        assert_eq!(
            SampleProjectId::from_cli_slug("mesh-dome"),
            Ok(SampleProjectId::SculptedMeshDome)
        );
        assert!(SampleProjectId::from_cli_slug("nonexistent").is_err());
    }
}
