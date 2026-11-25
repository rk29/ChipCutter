use egui::{Color32, CornerRadius, Margin, Stroke, Visuals};
use serde::{Deserialize, Serialize};

/// Application color theme (`08-ui-viewport.md` §1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AppTheme {
    /// Low-glare dark studio workspace optimized for CAD/CAM 3D contrast.
    #[default]
    Dark,
    /// High-contrast daylight shop-floor workspace.
    Light,
}

impl AppTheme {
    pub const ALL: [Self; 2] = [Self::Dark, Self::Light];

    pub const fn label(self) -> &'static str {
        match self {
            Self::Dark => "Studio Dark",
            Self::Light => "Daylight Light",
        }
    }

    pub const fn short_label(self) -> &'static str {
        match self {
            Self::Dark => "Dark",
            Self::Light => "Light",
        }
    }

    pub const fn toggled(self) -> Self {
        match self {
            Self::Dark => Self::Light,
            Self::Light => Self::Dark,
        }
    }

    pub const fn is_light(self) -> bool {
        matches!(self, Self::Light)
    }
}

/// Centralized semantic design-system tokens for `chip-gui` (`08-ui-viewport.md` §Design System).
#[derive(Debug, Clone, Copy)]
pub struct ThemeTokens {
    pub theme: AppTheme,

    // Surface hierarchy
    pub app_bg: Color32,
    pub menubar_bg: Color32,
    pub toolbar_bg: Color32,
    pub panel_bg: Color32,
    pub card_bg: Color32,
    pub elevated_bg: Color32,
    pub input_bg: Color32,
    pub code_bg: Color32,
    pub hud_bg: Color32,
    pub hud_border: Color32,
    pub border: Color32,
    pub border_strong: Color32,

    // Typography colors
    pub text_primary: Color32,
    pub text_secondary: Color32,
    pub text_muted: Color32,

    // Primary brand & interactive accents
    pub accent: Color32,
    pub accent_hover: Color32,
    pub accent_soft: Color32,
    pub accent_on_fg: Color32,

    // Semantic status tokens
    pub clean_fg: Color32,
    pub clean_bg: Color32,
    pub clean_border: Color32,

    pub dirty_fg: Color32,
    pub dirty_bg: Color32,
    pub dirty_border: Color32,

    pub info_fg: Color32,
    pub info_bg: Color32,
    pub info_border: Color32,

    pub warn_fg: Color32,
    pub warn_bg: Color32,
    pub warn_border: Color32,

    pub error_fg: Color32,
    pub error_bg: Color32,
    pub error_border: Color32,

    pub suppressed_fg: Color32,
    pub snap_badge_fg: Color32,
    pub pick_banner_fg: Color32,

    // G-Code syntax highlighting
    pub gcode_comment: Color32,
    pub gcode_rapid: Color32,
    pub gcode_feed: Color32,
    pub gcode_arc: Color32,
    pub gcode_spindle: Color32,
    pub gcode_coord: Color32,

    // 3D `wgpu` Viewport studio background & grid parameters
    pub vp_bg_top_rgb: [f32; 3],
    pub vp_bg_bottom_rgb: [f32; 3],
    pub vp_grid_rgb: [f32; 3],
    pub vp_grid_alpha_scale: f32,
}

impl ThemeTokens {
    pub const fn for_theme(theme: AppTheme) -> Self {
        match theme {
            AppTheme::Dark => Self {
                theme: AppTheme::Dark,
                app_bg: Color32::from_rgb(16, 19, 24),
                menubar_bg: Color32::from_rgb(20, 23, 29),
                toolbar_bg: Color32::from_rgb(24, 28, 35),
                panel_bg: Color32::from_rgb(24, 27, 33),
                card_bg: Color32::from_rgb(30, 35, 43),
                elevated_bg: Color32::from_rgb(34, 40, 50),
                input_bg: Color32::from_rgb(18, 21, 27),
                code_bg: Color32::from_rgb(17, 20, 26),
                hud_bg: Color32::from_rgba_premultiplied(18, 22, 28, 224),
                hud_border: Color32::from_rgb(62, 72, 88),
                border: Color32::from_rgb(46, 53, 65),
                border_strong: Color32::from_rgb(72, 84, 102),

                text_primary: Color32::from_rgb(232, 237, 244),
                text_secondary: Color32::from_rgb(178, 188, 204),
                text_muted: Color32::from_rgb(138, 150, 168),

                accent: Color32::from_rgb(20, 108, 204),
                accent_hover: Color32::from_rgb(32, 126, 224),
                accent_soft: Color32::from_rgb(26, 48, 76),
                accent_on_fg: Color32::from_rgb(255, 255, 255),

                clean_fg: Color32::from_rgb(86, 224, 138),
                clean_bg: Color32::from_rgb(20, 48, 34),
                clean_border: Color32::from_rgb(48, 128, 84),

                dirty_fg: Color32::from_rgb(248, 194, 72),
                dirty_bg: Color32::from_rgb(54, 42, 18),
                dirty_border: Color32::from_rgb(154, 114, 38),

                info_fg: Color32::from_rgb(98, 204, 252),
                info_bg: Color32::from_rgb(20, 38, 54),
                info_border: Color32::from_rgb(54, 108, 152),

                warn_fg: Color32::from_rgb(252, 186, 78),
                warn_bg: Color32::from_rgb(56, 38, 20),
                warn_border: Color32::from_rgb(172, 112, 44),

                error_fg: Color32::from_rgb(255, 108, 108),
                error_bg: Color32::from_rgb(58, 26, 28),
                error_border: Color32::from_rgb(184, 68, 68),

                suppressed_fg: Color32::from_rgb(126, 136, 152),
                snap_badge_fg: Color32::from_rgb(255, 218, 64),
                pick_banner_fg: Color32::from_rgb(108, 224, 255),

                gcode_comment: Color32::from_rgb(132, 146, 166),
                gcode_rapid: Color32::from_rgb(248, 198, 76),
                gcode_feed: Color32::from_rgb(92, 228, 158),
                gcode_arc: Color32::from_rgb(232, 142, 248),
                gcode_spindle: Color32::from_rgb(250, 134, 120),
                gcode_coord: Color32::from_rgb(150, 208, 252),

                vp_bg_top_rgb: [0.155, 0.178, 0.212],
                vp_bg_bottom_rgb: [0.088, 0.098, 0.116],
                vp_grid_rgb: [0.25, 0.29, 0.35],
                vp_grid_alpha_scale: 1.0,
            },
            AppTheme::Light => Self {
                theme: AppTheme::Light,
                app_bg: Color32::from_rgb(228, 233, 240),
                menubar_bg: Color32::from_rgb(242, 245, 250),
                toolbar_bg: Color32::from_rgb(236, 240, 246),
                panel_bg: Color32::from_rgb(245, 247, 251),
                card_bg: Color32::from_rgb(255, 255, 255),
                elevated_bg: Color32::from_rgb(250, 252, 255),
                input_bg: Color32::from_rgb(255, 255, 255),
                code_bg: Color32::from_rgb(248, 250, 253),
                hud_bg: Color32::from_rgba_premultiplied(238, 242, 248, 236),
                hud_border: Color32::from_rgb(176, 188, 204),
                border: Color32::from_rgb(204, 212, 224),
                border_strong: Color32::from_rgb(158, 170, 188),

                text_primary: Color32::from_rgb(22, 28, 38),
                text_secondary: Color32::from_rgb(62, 74, 92),
                text_muted: Color32::from_rgb(94, 106, 124),

                accent: Color32::from_rgb(16, 104, 198),
                accent_hover: Color32::from_rgb(12, 88, 172),
                accent_soft: Color32::from_rgb(220, 234, 252),
                accent_on_fg: Color32::from_rgb(255, 255, 255),

                clean_fg: Color32::from_rgb(14, 118, 62),
                clean_bg: Color32::from_rgb(224, 246, 234),
                clean_border: Color32::from_rgb(112, 196, 148),

                dirty_fg: Color32::from_rgb(148, 86, 6),
                dirty_bg: Color32::from_rgb(254, 243, 216),
                dirty_border: Color32::from_rgb(224, 176, 84),

                info_fg: Color32::from_rgb(12, 96, 168),
                info_bg: Color32::from_rgb(226, 241, 254),
                info_border: Color32::from_rgb(126, 178, 228),

                warn_fg: Color32::from_rgb(154, 78, 8),
                warn_bg: Color32::from_rgb(255, 241, 218),
                warn_border: Color32::from_rgb(228, 168, 86),

                error_fg: Color32::from_rgb(178, 28, 36),
                error_bg: Color32::from_rgb(254, 230, 232),
                error_border: Color32::from_rgb(226, 126, 132),

                suppressed_fg: Color32::from_rgb(114, 124, 140),
                snap_badge_fg: Color32::from_rgb(146, 84, 4),
                pick_banner_fg: Color32::from_rgb(10, 92, 168),

                gcode_comment: Color32::from_rgb(104, 116, 136),
                gcode_rapid: Color32::from_rgb(158, 92, 8),
                gcode_feed: Color32::from_rgb(14, 118, 66),
                gcode_arc: Color32::from_rgb(138, 38, 164),
                gcode_spindle: Color32::from_rgb(176, 42, 34),
                gcode_coord: Color32::from_rgb(16, 94, 172),

                vp_bg_top_rgb: [0.925, 0.942, 0.964],
                vp_bg_bottom_rgb: [0.835, 0.862, 0.902],
                vp_grid_rgb: [0.40, 0.46, 0.55],
                vp_grid_alpha_scale: 1.25,
            },
        }
    }

    #[inline]
    pub fn from_ctx(ctx: &egui::Context) -> Self {
        let theme = if ctx.global_style().visuals.dark_mode {
            AppTheme::Dark
        } else {
            AppTheme::Light
        };
        Self::for_theme(theme)
    }

    #[inline]
    pub fn from_ui(ui: &egui::Ui) -> Self {
        let theme = if ui.visuals().dark_mode {
            AppTheme::Dark
        } else {
            AppTheme::Light
        };
        Self::for_theme(theme)
    }
}

/// Applies `theme`'s typography, spacing, corner radii, and widget palette to `ctx`.
pub fn apply_theme(ctx: &egui::Context, theme: AppTheme) {
    let tok = ThemeTokens::for_theme(theme);
    let mut style = (*ctx.global_style()).clone();

    style.spacing.item_spacing = egui::vec2(6.0, 5.0);
    style.spacing.button_padding = egui::vec2(7.0, 3.5);
    style.spacing.window_margin = Margin::same(10);
    style.spacing.menu_margin = Margin::same(6);
    style.spacing.indent = 14.0;

    let mut visuals = match theme {
        AppTheme::Dark => Visuals::dark(),
        AppTheme::Light => Visuals::light(),
    };

    visuals.panel_fill = tok.panel_bg;
    visuals.window_fill = tok.elevated_bg;
    visuals.faint_bg_color = tok.card_bg;
    visuals.extreme_bg_color = tok.input_bg;
    visuals.code_bg_color = tok.code_bg;
    visuals.override_text_color = None;

    visuals.window_corner_radius = CornerRadius::same(7);
    visuals.menu_corner_radius = CornerRadius::same(6);
    visuals.window_stroke = Stroke::new(1.0, tok.border_strong);

    visuals.widgets.noninteractive.bg_fill = tok.panel_bg;
    visuals.widgets.noninteractive.weak_bg_fill = tok.card_bg;
    visuals.widgets.noninteractive.bg_stroke = Stroke::new(1.0, tok.border);
    visuals.widgets.noninteractive.fg_stroke = Stroke::new(1.0, tok.text_primary);
    visuals.widgets.noninteractive.corner_radius = CornerRadius::same(5);

    visuals.widgets.inactive.bg_fill = tok.card_bg;
    visuals.widgets.inactive.weak_bg_fill = tok.card_bg;
    visuals.widgets.inactive.bg_stroke = Stroke::new(1.0, tok.border);
    visuals.widgets.inactive.fg_stroke = Stroke::new(1.0, tok.text_primary);
    visuals.widgets.inactive.corner_radius = CornerRadius::same(5);

    visuals.widgets.hovered.bg_fill = match theme {
        AppTheme::Dark => Color32::from_rgb(40, 48, 60),
        AppTheme::Light => Color32::from_rgb(232, 238, 248),
    };
    visuals.widgets.hovered.weak_bg_fill = visuals.widgets.hovered.bg_fill;
    visuals.widgets.hovered.bg_stroke = Stroke::new(1.0, tok.accent);
    visuals.widgets.hovered.fg_stroke = Stroke::new(1.2, tok.text_primary);
    visuals.widgets.hovered.corner_radius = CornerRadius::same(5);

    match theme {
        AppTheme::Dark => {
            visuals.widgets.active.bg_fill = tok.accent;
            visuals.widgets.active.weak_bg_fill = tok.accent;
            visuals.widgets.active.bg_stroke = Stroke::new(1.0, tok.accent_hover);
            visuals.widgets.active.fg_stroke = Stroke::new(1.4, Color32::from_rgb(248, 250, 255));
        }
        AppTheme::Light => {
            visuals.widgets.active.bg_fill = Color32::from_rgb(214, 226, 244);
            visuals.widgets.active.weak_bg_fill = Color32::from_rgb(214, 226, 244);
            visuals.widgets.active.bg_stroke = Stroke::new(1.2, tok.accent);
            visuals.widgets.active.fg_stroke = Stroke::new(1.5, Color32::from_rgb(12, 18, 28));
        }
    }
    visuals.widgets.active.corner_radius = CornerRadius::same(5);

    visuals.selection.bg_fill = match theme {
        AppTheme::Dark => Color32::from_rgba_unmultiplied(36, 134, 226, 92),
        AppTheme::Light => Color32::from_rgba_unmultiplied(16, 104, 198, 52),
    };
    visuals.selection.stroke = Stroke::new(1.2, tok.accent);

    visuals.warn_fg_color = tok.warn_fg;
    visuals.error_fg_color = tok.error_fg;

    style.visuals = visuals;
    ctx.set_global_style(style);
}

/// Computes WCAG 2.1 relative luminance (`[0.0, 1.0]`) of an sRGB `Color32`.
pub fn wcag_relative_luminance(c: Color32) -> f64 {
    let channel = |u: u8| -> f64 {
        let s = f64::from(u) / 255.0;
        if s <= 0.04045 {
            s / 12.92
        } else {
            ((s + 0.055) / 1.055).powf(2.4)
        }
    };
    0.2126 * channel(c.r()) + 0.7152 * channel(c.g()) + 0.0722 * channel(c.b())
}

/// Computes WCAG 2.1 contrast ratio (`[1.0, 21.0]`) between two opaque sRGB colors.
pub fn wcag_contrast_ratio(fg: Color32, bg: Color32) -> f64 {
    let l1 = wcag_relative_luminance(fg);
    let l2 = wcag_relative_luminance(bg);
    let (hi, lo) = if l1 >= l2 { (l1, l2) } else { (l2, l1) };
    (hi + 0.05) / (lo + 0.05)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dark_and_light_theme_tokens_satisfy_wcag_aa_contrast() {
        for theme in AppTheme::ALL {
            let tok = ThemeTokens::for_theme(theme);
            for (bg_name, bg) in [
                ("panel_bg", tok.panel_bg),
                ("card_bg", tok.card_bg),
                ("input_bg", tok.input_bg),
            ] {
                let primary_ratio = wcag_contrast_ratio(tok.text_primary, bg);
                assert!(
                    primary_ratio >= 7.0,
                    "{theme:?} text_primary vs {bg_name} contrast {primary_ratio:.2} < 7.0"
                );
                let secondary_ratio = wcag_contrast_ratio(tok.text_secondary, bg);
                assert!(
                    secondary_ratio >= 4.5,
                    "{theme:?} text_secondary vs {bg_name} contrast {secondary_ratio:.2} < 4.5"
                );
                for (fg_name, fg) in [
                    ("clean_fg", tok.clean_fg),
                    ("dirty_fg", tok.dirty_fg),
                    ("info_fg", tok.info_fg),
                    ("warn_fg", tok.warn_fg),
                    ("error_fg", tok.error_fg),
                ] {
                    let ratio = wcag_contrast_ratio(fg, bg);
                    assert!(
                        ratio >= 4.5,
                        "{theme:?} {fg_name} vs {bg_name} contrast {ratio:.2} < 4.5"
                    );
                }
            }
            assert!(
                wcag_contrast_ratio(tok.accent_on_fg, tok.accent) >= 4.5,
                "{theme:?} accent button contrast < 4.5"
            );

            let ctx = egui::Context::default();
            apply_theme(&ctx, theme);
            let style = ctx.global_style();
            let strong_ratio = wcag_contrast_ratio(style.visuals.strong_text_color(), tok.panel_bg);
            assert!(
                strong_ratio >= 7.0,
                "{theme:?} egui strong_text_color vs panel_bg contrast {strong_ratio:.2} < 7.0"
            );
            let body_ratio = wcag_contrast_ratio(style.visuals.text_color(), tok.panel_bg);
            assert!(
                body_ratio >= 7.0,
                "{theme:?} egui text_color vs panel_bg contrast {body_ratio:.2} < 7.0"
            );
        }
    }
}
