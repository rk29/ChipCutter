//! Standardized two-column Inspector layout primitives, collapsible section cards, and semantic
//! validation banners (`08-ui-viewport.md` §Parametric Input Widgets).

use std::hash::Hash;

use egui::{CornerRadius, Margin, Pos2, Rect, Sense, Stroke, StrokeKind, Ui, Vec2};

use crate::{
    theme::ThemeTokens,
    widgets::{IconKind, inline_icon},
};

/// Default left-column label minimum width in two-column Inspector property grids.
pub const LABEL_COL_MIN_WIDTH: f32 = 96.0;

/// Standard single-line input width inside two-column Inspector property grids.
pub const INPUT_COL_WIDTH: f32 = 82.0;

/// Semantic severity/tone for inline validation and status callout banners.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BannerKind {
    Clean,
    Info,
    Warn,
    Error,
}

/// Renders a consistent bordered semantic status/validation banner with a procedural vector icon.
pub fn inline_banner(ui: &mut Ui, kind: BannerKind, message: impl AsRef<str>) {
    let tok = ThemeTokens::from_ui(ui);
    let (bg, border, fg, icon) = match kind {
        BannerKind::Clean => (
            tok.clean_bg,
            tok.clean_border,
            tok.clean_fg,
            IconKind::StatusClean,
        ),
        BannerKind::Info => (
            tok.info_bg,
            tok.info_border,
            tok.info_fg,
            IconKind::InfoCircle,
        ),
        BannerKind::Warn => (
            tok.warn_bg,
            tok.warn_border,
            tok.warn_fg,
            IconKind::StatusNeedsRepick,
        ),
        BannerKind::Error => (
            tok.error_bg,
            tok.error_border,
            tok.error_fg,
            IconKind::StatusError,
        ),
    };

    egui::Frame::NONE
        .fill(bg)
        .stroke(Stroke::new(1.0, border))
        .corner_radius(CornerRadius::same(5))
        .inner_margin(Margin::symmetric(8, 6))
        .show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                inline_icon(ui, icon, fg, 14.0);
                ui.colored_label(fg, message.as_ref());
            });
        });
}

/// Renders a collapsible `ThemeTokens::card_bg` section card with a procedural disclosure chevron.
pub fn section_card<R>(
    ui: &mut Ui,
    id_salt: impl Hash + std::fmt::Debug,
    title: &str,
    default_open: bool,
    add_body: impl FnOnce(&mut Ui) -> R,
) -> Option<R> {
    let tok = ThemeTokens::from_ui(ui);
    let state_id = ui.make_persistent_id(id_salt);
    let mut is_open = ui
        .data_mut(|d| d.get_persisted::<bool>(state_id))
        .unwrap_or(default_open);

    let mut body_out = None;
    egui::Frame::NONE
        .fill(tok.card_bg)
        .stroke(Stroke::new(1.0, tok.border))
        .corner_radius(CornerRadius::same(6))
        .inner_margin(Margin::symmetric(8, 7))
        .show(ui, |ui| {
            ui.set_min_width(ui.available_width());
            let hdr_h = 20.0;
            let hdr_w = ui.available_width().max(64.0);
            let (hdr_rect, hdr_resp) =
                ui.allocate_exact_size(Vec2::new(hdr_w, hdr_h), Sense::click());
            if hdr_resp.clicked() {
                is_open = !is_open;
                ui.data_mut(|d| d.insert_persisted(state_id, is_open));
            }
            if ui.is_rect_visible(hdr_rect) {
                let hovered = hdr_resp.hovered();
                if hovered {
                    ui.painter().rect_filled(
                        hdr_rect.expand2(Vec2::new(3.0, 1.0)),
                        CornerRadius::same(4),
                        ui.visuals().widgets.hovered.bg_fill.gamma_multiply(0.55),
                    );
                }
                let chev_col = if hovered {
                    tok.accent
                } else {
                    tok.text_secondary
                };
                paint_disclosure_chevron(
                    ui.painter(),
                    Rect::from_center_size(
                        Pos2::new(hdr_rect.left() + 6.0, hdr_rect.center().y),
                        Vec2::splat(10.0),
                    ),
                    is_open,
                    chev_col,
                );
                ui.painter().text(
                    Pos2::new(hdr_rect.left() + 16.0, hdr_rect.center().y),
                    egui::Align2::LEFT_CENTER,
                    title,
                    egui::FontId::proportional(12.5),
                    tok.text_primary,
                );
            }

            if is_open {
                ui.add_space(4.0);
                ui.separator();
                ui.add_space(3.0);
                body_out = Some(add_body(ui));
            }
        });
    ui.add_space(5.0);
    body_out
}

/// Renders a non-collapsible `ThemeTokens::card_bg` container card with an optional title header.
pub fn static_card<R>(ui: &mut Ui, title: Option<&str>, add_body: impl FnOnce(&mut Ui) -> R) -> R {
    let tok = ThemeTokens::from_ui(ui);
    let out = egui::Frame::NONE
        .fill(tok.card_bg)
        .stroke(Stroke::new(1.0, tok.border))
        .corner_radius(CornerRadius::same(6))
        .inner_margin(Margin::symmetric(8, 7))
        .show(ui, |ui| {
            ui.set_min_width(ui.available_width());
            if let Some(hdr) = title {
                ui.strong(hdr);
                ui.add_space(3.0);
                ui.separator();
                ui.add_space(3.0);
            }
            add_body(ui)
        })
        .inner;
    ui.add_space(5.0);
    out
}

/// Opens a standardized two-column Inspector property grid (`Column 1 = Label`, `Column 2 = Control`).
pub fn inspector_grid<R>(
    ui: &mut Ui,
    id_salt: impl Hash + std::fmt::Debug,
    add_rows: impl FnOnce(&mut Ui) -> R,
) -> R {
    egui::Grid::new(id_salt)
        .num_columns(2)
        .min_col_width(LABEL_COL_MIN_WIDTH)
        .spacing([10.0, 6.0])
        .show(ui, add_rows)
        .inner
}

/// Emits a single aligned two-column row inside [`inspector_grid`] (`label` + optional `tooltip` on
/// the left, `add_control` horizontally laid out on the right, terminated with `ui.end_row()`).
pub fn grid_row<R>(
    ui: &mut Ui,
    label: &str,
    tooltip: Option<&str>,
    add_control: impl FnOnce(&mut Ui) -> R,
) -> R {
    let tok = ThemeTokens::from_ui(ui);
    let lbl_resp = ui.colored_label(tok.text_secondary, label);
    if let Some(tip) = tooltip {
        lbl_resp.on_hover_text(tip);
    }
    let out = ui
        .horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 4.0;
            add_control(ui)
        })
        .inner;
    ui.end_row();
    out
}

fn paint_disclosure_chevron(painter: &egui::Painter, rect: Rect, open: bool, color: egui::Color32) {
    let stroke = Stroke::new(1.5, color);
    let c = rect.center();
    let r = rect.width() * 0.34;
    if open {
        painter.add(egui::Shape::line(
            vec![
                Pos2::new(c.x - r, c.y - r * 0.55),
                Pos2::new(c.x, c.y + r * 0.55),
                Pos2::new(c.x + r, c.y - r * 0.55),
            ],
            stroke,
        ));
    } else {
        painter.add(egui::Shape::line(
            vec![
                Pos2::new(c.x - r * 0.55, c.y - r),
                Pos2::new(c.x + r * 0.55, c.y),
                Pos2::new(c.x - r * 0.55, c.y + r),
            ],
            stroke,
        ));
    }
    let _ = StrokeKind::Inside;
}
