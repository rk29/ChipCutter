//! **About ChipCutter** modal & centralized application version/build metadata (`08-ui-viewport.md`).

use std::time::{Duration, Instant};

use egui::{CornerRadius, Key, Margin, Stroke, Vec2};

use crate::{
    theme::ThemeTokens,
    widgets::{IconKind, inline_icon, static_card},
};

/// Human-readable application name.
pub const APP_NAME: &str = "ChipCutter";

/// Workspace crate version sourced directly from `Cargo.toml`.
pub const APP_VERSION: &str = env!("CARGO_PKG_VERSION");

/// Package description sourced from `Cargo.toml`.
pub const APP_DESCRIPTION: &str = env!("CARGO_PKG_DESCRIPTION");

/// SPDX license expression sourced from `Cargo.toml`.
pub const APP_LICENSE: &str = env!("CARGO_PKG_LICENSE");

/// Upstream source repository URL sourced from `Cargo.toml`.
pub const APP_REPOSITORY: &str = env!("CARGO_PKG_REPOSITORY");

const APP_ICON_PNG: &[u8] = include_bytes!("../../../../packaging/icons/chipcutter-256.png");

/// Returns `"Debug"` for debug builds and `"Release"` for optimized builds.
pub const fn build_profile() -> &'static str {
    if cfg!(debug_assertions) {
        "Debug"
    } else {
        "Release"
    }
}

/// Returns the target OS and CPU architecture string (e.g. `linux / x86_64`).
pub fn platform_label() -> String {
    format!("{} / {}", std::env::consts::OS, std::env::consts::ARCH)
}

/// Formats the desktop window title with the running application version and project name.
pub fn format_window_title(project_name: &str, is_dirty: bool) -> String {
    format!(
        "{APP_NAME} v{APP_VERSION} — {project_name}{}",
        if is_dirty { "*" } else { "" }
    )
}

/// Formats a plain-text version and environment summary suitable for clipboard copy / issue reports.
pub fn format_copy_summary(graphics_backend: Option<&str>) -> String {
    let gpu = graphics_backend.unwrap_or("wgpu");
    format!(
        "{APP_NAME} v{APP_VERSION} ({profile})\nPlatform: {platform}\nGraphics: {gpu}\nLicense: {APP_LICENSE}\nRepository: {APP_REPOSITORY}",
        profile = build_profile(),
        platform = platform_label(),
    )
}

/// Persistent UI state for the `Help → About ChipCutter…` modal.
#[derive(Default)]
pub struct AboutModalState {
    pub open: bool,
    copied_feedback_until: Option<Instant>,
    icon_texture: Option<egui::TextureHandle>,
    icon_load_attempted: bool,
}

impl std::fmt::Debug for AboutModalState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AboutModalState")
            .field("open", &self.open)
            .field("icon_loaded", &self.icon_texture.is_some())
            .finish()
    }
}

impl AboutModalState {
    fn ensure_icon_texture(&mut self, ctx: &egui::Context) -> Option<&egui::TextureHandle> {
        if !self.icon_load_attempted {
            self.icon_load_attempted = true;
            if let Ok(img) =
                image::load_from_memory_with_format(APP_ICON_PNG, image::ImageFormat::Png)
            {
                let rgba = img.to_rgba8();
                let (w, h) = rgba.dimensions();
                let color_image = egui::ColorImage::from_rgba_unmultiplied(
                    [w as usize, h as usize],
                    rgba.as_raw(),
                );
                let handle = ctx.load_texture(
                    "chipcutter_about_app_icon",
                    color_image,
                    egui::TextureOptions::LINEAR,
                );
                self.icon_texture = Some(handle);
            }
        }
        self.icon_texture.as_ref()
    }
}

/// Renders the **About ChipCutter** modal displaying version, build, platform, and license details.
pub fn show_about_modal(
    ctx: &egui::Context,
    state: &mut AboutModalState,
    graphics_backend: Option<&str>,
) {
    if !state.open {
        return;
    }

    let mut keep_open = state.open;
    let mut close_clicked = false;
    let icon_id = state.ensure_icon_texture(ctx).map(egui::TextureHandle::id);

    egui::Window::new(format!("About {APP_NAME}"))
        .id(egui::Id::unique("chipcutter_about_modal"))
        .collapsible(false)
        .resizable(false)
        .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
        .default_width(450.0)
        .open(&mut keep_open)
        .show(ctx, |ui| {
            let tok = ThemeTokens::from_ui(ui);
            ui.set_min_width(430.0);
            ui.set_max_width(460.0);

            ui.horizontal(|ui| {
                if let Some(tex_id) = icon_id {
                    ui.image((tex_id, Vec2::splat(48.0)));
                } else {
                    inline_icon(ui, IconKind::Sparkles, tok.accent, 36.0);
                }
                ui.add_space(6.0);
                ui.vertical(|ui| {
                    ui.horizontal(|ui| {
                        ui.heading(APP_NAME);
                        egui::Frame::NONE
                            .fill(tok.accent.gamma_multiply(0.18))
                            .stroke(Stroke::new(1.0, tok.accent))
                            .corner_radius(CornerRadius::same(4))
                            .inner_margin(Margin::symmetric(6, 2))
                            .show(ui, |ui| {
                                ui.label(
                                    egui::RichText::new(format!("v{APP_VERSION}"))
                                        .monospace()
                                        .strong()
                                        .size(12.0)
                                        .color(tok.accent),
                                );
                            });
                    });
                    ui.weak(APP_DESCRIPTION);
                });
            });

            ui.add_space(6.0);

            static_card(ui, Some("Version & Build Information"), |ui| {
                egui::Grid::new("chipcutter_about_info_grid")
                    .num_columns(2)
                    .min_col_width(92.0)
                    .spacing([12.0, 6.0])
                    .show(ui, |ui| {
                        ui.weak("Version");
                        ui.colored_label(
                            tok.info_fg,
                            egui::RichText::new(format!("v{APP_VERSION}"))
                                .monospace()
                                .strong(),
                        );
                        ui.end_row();

                        ui.weak("Build");
                        ui.label(build_profile());
                        ui.end_row();

                        ui.weak("Platform");
                        ui.label(egui::RichText::new(platform_label()).monospace());
                        ui.end_row();

                        ui.weak("Graphics");
                        ui.label(graphics_backend.unwrap_or("wgpu"));
                        ui.end_row();

                        ui.weak("License");
                        ui.label(APP_LICENSE);
                        ui.end_row();

                        ui.weak("Repository");
                        ui.hyperlink(APP_REPOSITORY);
                        ui.end_row();
                    });
            });

            ui.add_space(4.0);
            ui.horizontal(|ui| {
                if ui
                    .button("Copy Version Info")
                    .on_hover_text("Copy version, platform, and graphics info to clipboard")
                    .clicked()
                {
                    ctx.copy_text(format_copy_summary(graphics_backend));
                    state.copied_feedback_until = Some(Instant::now() + Duration::from_secs(2));
                }

                if let Some(until) = state.copied_feedback_until {
                    if Instant::now() < until {
                        ui.colored_label(tok.clean_fg, "Copied!");
                        ctx.request_repaint_after(Duration::from_millis(250));
                    } else {
                        state.copied_feedback_until = None;
                    }
                }

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.button("Close").clicked() {
                        close_clicked = true;
                    }
                });
            });
        });

    if close_clicked || ctx.input(|i| i.key_pressed(Key::Escape)) {
        keep_open = false;
    }
    state.open = keep_open;
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn app_version_matches_cargo_pkg_version_and_is_non_empty() {
        assert!(!APP_VERSION.is_empty());
        assert_eq!(APP_VERSION, env!("CARGO_PKG_VERSION"));
        assert!(APP_VERSION.contains('.'));
    }

    #[test]
    fn window_title_includes_version_and_dirty_marker() {
        let clean = format_window_title("Bracket", false);
        assert_eq!(clean, format!("ChipCutter v{APP_VERSION} — Bracket"));

        let dirty = format_window_title("Bracket", true);
        assert_eq!(dirty, format!("ChipCutter v{APP_VERSION} — Bracket*"));
    }

    #[test]
    fn copy_summary_includes_version_platform_and_graphics() {
        let summary = format_copy_summary(Some("Vulkan (Test GPU)"));
        assert!(summary.contains(&format!("ChipCutter v{APP_VERSION}")));
        assert!(summary.contains(&platform_label()));
        assert!(summary.contains("Vulkan (Test GPU)"));
        assert!(summary.contains(APP_LICENSE));
        assert!(summary.contains(APP_REPOSITORY));
    }

    #[test]
    fn embedded_app_icon_decodes_cleanly() {
        let img =
            image::load_from_memory_with_format(APP_ICON_PNG, image::ImageFormat::Png).unwrap();
        let rgba = img.to_rgba8();
        assert_eq!(rgba.dimensions(), (256, 256));
    }
}
