//! Parametric machinist expression input widgets (`08-ui-viewport.md` §Parametric Input Widgets):
//! - Mixed-unit formulas (`1/4 in`, `6mm + 10 thou`) and scoped CAM variables (`tool_d * 0.4`);
//! - Clean draggable `[mm]` / `[in]` unit pill on plain numbers vs. `= <resolved>` badge on formulas;
//! - `ArrowUp` / `ArrowDown` keyboard step nudging (`0.1` / `Shift: 1.0` / `Alt: 0.01` display units);
//! - Live `fx` scoped-variable reference & prefix-autocomplete popover (`Tab` or click to insert).

use std::hash::Hash;

use chip_math::{DisplayUnit, EvalContext, ParamLength, evaluate_length_mm};
use egui::{CornerRadius, CursorIcon, Key, Order, Pos2, Sense, Stroke, StrokeKind, Vec2};

use crate::{
    theme::ThemeTokens,
    widgets::{StatusBadgeKind, status_badge_icon},
};

/// Metadata for a single scoped CAM variable exposed in the `fx` reference / autocomplete popup.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ScopedVarInfo {
    pub name: &'static str,
    pub description: &'static str,
    pub formatted_value: String,
}

const KNOWN_LENGTH_VARS: &[(&str, &str)] = &[
    ("tool_d", "Active cutter diameter"),
    ("tool_r", "Active cutter radius (tool_d / 2)"),
    ("flute_len", "Cutter flute length"),
    ("hole_d", "Selected hole diameter"),
    ("feature_top_z", "Inferred feature top Z (WCS)"),
    ("feature_bottom_z", "Inferred feature bottom Z (WCS)"),
    ("model_top_z", "CAD model top Z (WCS)"),
    ("model_bottom_z", "CAD model bottom Z (WCS)"),
    ("stock_x", "Setup stock X width"),
    ("stock_y", "Setup stock Y depth"),
    ("stock_z", "Setup stock Z thickness"),
];

const KNOWN_SCALAR_VARS: &[(&str, &str)] = &[
    ("flutes", "Cutter flute count"),
    ("tip_angle", "Cutter tip included angle"),
];

const RESERVED_UNIT_TOKENS: &[&str] = &[
    "mm", "cm", "m", "in", "inch", "inches", "ft", "thou", "mil", "deg", "rad", "pi",
];

/// Collects all scoped variables currently populated in `ctx` in deterministic machinist order.
pub fn collect_scoped_vars(ctx: &EvalContext) -> Vec<ScopedVarInfo> {
    let mut out = Vec::new();
    for &(name, description) in KNOWN_LENGTH_VARS {
        if let Some(val_mm) = ctx.get_var(name) {
            out.push(ScopedVarInfo {
                name,
                description,
                formatted_value: ctx.default_unit.format_length(val_mm),
            });
        }
    }
    for &(name, description) in KNOWN_SCALAR_VARS {
        if let Some(val) = ctx.get_scalar_var(name) {
            let formatted_value = if name == "flutes" {
                format!("{}", val.round() as i64)
            } else if name == "tip_angle" {
                format!("{:.1}° ({val:.4} rad)", val.to_degrees())
            } else {
                format!("{val:.4}")
            };
            out.push(ScopedVarInfo {
                name,
                description,
                formatted_value,
            });
        }
    }
    out
}

/// Extracts a trailing identifier prefix (`>= 2` chars, starting with ASCII alphabetic/`_`) at the
/// end of `draft`, ignoring reserved unit/constant tokens like `mm`, `in`, `thou`.
pub fn extract_trailing_identifier_prefix(draft: &str) -> Option<&str> {
    let trimmed = draft.trim_end_matches(' ');
    if trimmed.len() != draft.len() {
        return None;
    }
    let start = draft
        .rfind(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
        .map_or(0, |idx| idx + 1);
    let tail = &draft[start..];
    if tail.len() < 2 {
        return None;
    }
    let first = tail.as_bytes()[0];
    if !(first.is_ascii_alphabetic() || first == b'_') {
        return None;
    }
    if RESERVED_UNIT_TOKENS
        .iter()
        .any(|u| u.eq_ignore_ascii_case(tail))
    {
        return None;
    }
    Some(tail)
}

/// Replaces a trailing identifier prefix in `draft` with `var_name`, or appends/replaces `draft`
/// cleanly when clicked from the `fx` reference menu.
pub fn replace_trailing_identifier_prefix(draft: &str, var_name: &str) -> String {
    if let Some(prefix) = extract_trailing_identifier_prefix(draft) {
        let keep = draft.len() - prefix.len();
        format!("{}{var_name}", &draft[..keep])
    } else {
        let t = draft.trim();
        if t.is_empty() || is_plain_number_literal(t) {
            var_name.to_string()
        } else if t.ends_with(['+', '-', '*', '/', '(']) {
            format!("{t} {var_name}")
        } else {
            format!("{t} + {var_name}")
        }
    }
}

/// Returns the keyboard `ArrowUp`/`ArrowDown` increment in `unit` (`Shift = 10x`, `Alt = 0.1x`).
pub const fn nudge_step_display_units(unit: DisplayUnit, shift: bool, alt: bool) -> f64 {
    let base = match unit {
        DisplayUnit::Millimeter => 0.1,
        DisplayUnit::Inch => 0.005,
    };
    if shift {
        base * 10.0
    } else if alt {
        match unit {
            DisplayUnit::Millimeter => 0.01,
            DisplayUnit::Inch => 0.001,
        }
    } else {
        base
    }
}

/// Applies `steps` (`+1.0`, `-1.0`, or fractional drag delta) to a plain-numeric [`ParamLength`].
pub fn apply_numeric_nudge(
    param: &ParamLength,
    unit: DisplayUnit,
    steps: f64,
    shift: bool,
    alt: bool,
) -> ParamLength {
    let step_u = nudge_step_display_units(unit, shift, alt);
    let cur_u = unit.from_mm(param.value_mm);
    let next_u = ((cur_u + steps * step_u) * 1e6).round() / 1e6;
    ParamLength::from_mm(unit.to_mm(next_u))
}

fn is_plain_number_literal(s: &str) -> bool {
    let t = s.trim();
    !t.is_empty() && t.parse::<f64>().is_ok()
}

/// Immediate-mode inline expression editor operating directly on a document-owned [`ParamLength`].
///
/// Stores transient keyboard edit buffer in `egui` temporary memory keyed by `id_salt`, so
/// `Ctrl+Z` / `Ctrl+Y` document Undo/Redo immediately reflects updated values with zero stale state.
pub fn edit_param_length(
    ui: &mut egui::Ui,
    id_salt: impl Hash + std::fmt::Debug,
    param: &mut ParamLength,
    ctx: &EvalContext,
    width_px: f32,
) -> bool {
    let widget_id = ui.make_persistent_id(id_salt);
    let draft_id = widget_id.with("draft");
    let fx_open_id = widget_id.with("fx_open");

    let mut draft_text: String = ui
        .data_mut(|d| d.get_temp::<String>(draft_id))
        .unwrap_or_else(|| match &param.expr {
            Some(expr) => expr.clone(),
            None => ctx.default_unit.format_number(param.value_mm),
        });
    let mut fx_open: bool = ui
        .data_mut(|d| d.get_temp::<bool>(fx_open_id))
        .unwrap_or(false);

    let scoped_vars = collect_scoped_vars(ctx);
    let tok = ThemeTokens::from_ui(ui);
    let mut committed = false;

    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 4.0;

        let input_tip = match &param.expr {
            Some(expr) => format!(
                "Parametric length: {expr} = {}\nSupports units (mm, in) and CAM variables (tool_d, stock_z, ...). Click 'fx' to insert variables.",
                ctx.default_unit.format_length(param.value_mm),
            ),
            None => format!(
                "Length value ({}).\nEnter a number, unit (mm / in), or formula (e.g. tool_d * 0.4). Use Up/Down arrows to nudge.",
                ctx.default_unit.format_length(param.value_mm),
            ),
        };
        let resp = ui
            .add(
                egui::TextEdit::singleline(&mut draft_text)
                    .id(widget_id)
                    .desired_width(width_px),
            )
            .on_hover_text(input_tip);

        let prefix_matches: Vec<&ScopedVarInfo> = if resp.has_focus()
            && let Some(prefix) = extract_trailing_identifier_prefix(&draft_text)
        {
            let lower = prefix.to_ascii_lowercase();
            scoped_vars
                .iter()
                .filter(|v| {
                    v.name.to_ascii_lowercase().starts_with(&lower)
                        && !v.name.eq_ignore_ascii_case(prefix)
                })
                .collect()
        } else {
            Vec::new()
        };

        // Tab autocompletion when prefix matches exist
        if resp.has_focus()
            && !prefix_matches.is_empty()
            && ui.input_mut(|i| i.consume_key(egui::Modifiers::NONE, Key::Tab))
        {
            draft_text = replace_trailing_identifier_prefix(&draft_text, prefix_matches[0].name);
            ui.data_mut(|d| d.insert_temp(draft_id, draft_text.clone()));
        }

        // ArrowUp / ArrowDown step nudging on plain numeric literals while focused
        if resp.has_focus() && param.expr.is_none() && is_plain_number_literal(&draft_text) {
            let (up, down, shift, alt) = ui.input(|i| {
                (
                    i.key_pressed(Key::ArrowUp),
                    i.key_pressed(Key::ArrowDown),
                    i.modifiers.shift,
                    i.modifiers.alt,
                )
            });
            let step_dir = if up {
                1.0
            } else if down {
                -1.0
            } else {
                0.0
            };
            if step_dir != 0.0 {
                let nudged = apply_numeric_nudge(param, ctx.default_unit, step_dir, shift, alt);
                if (nudged.value_mm - param.value_mm).abs() > 1e-9 {
                    *param = nudged;
                    draft_text = ctx.default_unit.format_number(param.value_mm);
                    ui.data_mut(|d| d.insert_temp(draft_id, draft_text.clone()));
                    committed = true;
                }
            }
        }

        if !resp.has_focus() && !resp.lost_focus() && !fx_open {
            draft_text = match &param.expr {
                Some(expr) => expr.clone(),
                None => ctx.default_unit.format_number(param.value_mm),
            };
        }

        if resp.has_focus() {
            ui.data_mut(|d| d.insert_temp(draft_id, draft_text.clone()));
        }

        let enter_pressed = resp.lost_focus() && ui.input(|i| i.key_pressed(Key::Enter));
        if resp.lost_focus() || enter_pressed {
            ui.data_mut(|d| d.remove::<String>(draft_id));
            if let Ok(parsed) = ParamLength::parse(&draft_text, ctx)
                && ((parsed.value_mm - param.value_mm).abs() > 1e-9 || parsed.expr != param.expr)
            {
                *param = parsed;
                committed = true;
            }
        }

        // Compact `fx` variable reference button when scoped variables exist
        let mut anchor_pos = Pos2::new(resp.rect.left(), resp.rect.bottom() + 3.0);
        if !scoped_vars.is_empty() {
            let fx_active = fx_open || param.expr.is_some();
            let (fx_rect, fx_resp) = ui.allocate_exact_size(Vec2::new(20.0, 20.0), Sense::click());
            if fx_resp.clicked() {
                fx_open = !fx_open;
                ui.data_mut(|d| d.insert_temp(fx_open_id, fx_open));
            }
            if ui.is_rect_visible(fx_rect) {
                let (bg, border, fg) = if fx_open {
                    (tok.accent, tok.accent_hover, tok.accent_on_fg)
                } else if fx_resp.hovered() {
                    (
                        ui.visuals().widgets.hovered.bg_fill,
                        tok.accent,
                        tok.text_primary,
                    )
                } else if fx_active {
                    (tok.info_bg, tok.info_border, tok.info_fg)
                } else {
                    (tok.elevated_bg, tok.border, tok.text_muted)
                };
                ui.painter().rect(
                    fx_rect,
                    CornerRadius::same(4),
                    bg,
                    Stroke::new(1.0, border),
                    StrokeKind::Inside,
                );
                ui.painter().text(
                    fx_rect.center(),
                    egui::Align2::CENTER_CENTER,
                    "fx",
                    egui::FontId::monospace(10.5),
                    fg,
                );
            }
            fx_resp.on_hover_text(
                "Insert scoped CAM variable (`tool_d`, `stock_z`, `feature_top_z`, …) or unit expression",
            );
            anchor_pos = Pos2::new(resp.rect.left(), fx_rect.bottom().max(resp.rect.bottom()) + 3.0);
        }

        let live_eval = evaluate_length_mm(&draft_text, ctx);
        let is_formula_now = if resp.has_focus() {
            !is_plain_number_literal(&draft_text)
        } else {
            param.expr.is_some()
        };

        match live_eval {
            Ok(mm) if is_formula_now => {
                ui.colored_label(
                    tok.info_fg,
                    format!("= {}", ctx.default_unit.format_length(mm)),
                );
            }
            Ok(_) => {
                // Draggable unit suffix pill (`mm` / `in`) for plain numeric literals
                let unit_txt = ctx.default_unit.suffix();
                let (u_rect, u_resp) =
                    ui.allocate_exact_size(Vec2::new(26.0, 20.0), Sense::drag());
                if u_resp.hovered() || u_resp.dragged() {
                    ui.ctx().set_cursor_icon(CursorIcon::ResizeHorizontal);
                }
                if u_resp.dragged() {
                    let dx = f64::from(u_resp.drag_delta().x) * 0.25;
                    if dx.abs() > 1e-4 {
                        let (shift, alt) =
                            ui.input(|i| (i.modifiers.shift, i.modifiers.alt));
                        let nudged =
                            apply_numeric_nudge(param, ctx.default_unit, dx, shift, alt);
                        if (nudged.value_mm - param.value_mm).abs() > 1e-9 {
                            *param = nudged;
                            committed = true;
                        }
                    }
                }
                if ui.is_rect_visible(u_rect) {
                    let border = if u_resp.hovered() || u_resp.dragged() {
                        tok.accent
                    } else {
                        tok.border
                    };
                    ui.painter().rect(
                        u_rect,
                        CornerRadius::same(4),
                        tok.elevated_bg,
                        Stroke::new(1.0, border),
                        StrokeKind::Inside,
                    );
                    ui.painter().text(
                        u_rect.center(),
                        egui::Align2::CENTER_CENTER,
                        unit_txt,
                        egui::FontId::proportional(11.0),
                        tok.text_secondary,
                    );
                }
                u_resp.on_hover_text(
                    "Drag horizontally to scrub value, or press Up/Down arrows while editing",
                );
            }
            Err(err) if resp.has_focus() => {
                status_badge_icon(ui, StatusBadgeKind::Error, Some(&err.to_string()));
            }
            Err(_) => {
                ui.weak(format!(
                    "= {}",
                    ctx.default_unit.format_length(param.value_mm)
                ));
            }
        }

        // Scoped Variable Reference / Autocomplete Popover
        let show_autocomplete = resp.has_focus() && !prefix_matches.is_empty();
        if fx_open || show_autocomplete {
            let vars_to_show: Vec<&ScopedVarInfo> = if show_autocomplete && !fx_open {
                prefix_matches
            } else {
                scoped_vars.iter().collect()
            };
            let mut picked_token: Option<&'static str> = None;
            let mut close_fx = false;

            let area_resp = egui::Area::new(widget_id.with("fx_popup"))
                .order(Order::Foreground)
                .fixed_pos(anchor_pos)
                .show(ui.ctx(), |ui| {
                    egui::Frame::NONE
                        .fill(tok.elevated_bg)
                        .stroke(Stroke::new(1.0, tok.border_strong))
                        .corner_radius(CornerRadius::same(6))
                        .inner_margin(egui::Margin::symmetric(8, 6))
                        .show(ui, |ui| {
                            ui.set_min_width(248.0);
                            ui.set_max_width(290.0);
                            ui.horizontal(|ui| {
                                ui.strong(if show_autocomplete && !fx_open {
                                    "Autocomplete (Tab to insert)"
                                } else {
                                    "Scoped CAM Variables"
                                });
                                if fx_open {
                                    ui.with_layout(
                                        egui::Layout::right_to_left(egui::Align::Center),
                                        |ui| {
                                            if ui.small_button("Close").clicked() {
                                                close_fx = true;
                                            }
                                        },
                                    );
                                }
                            });
                            ui.separator();
                            for v in &vars_to_show {
                                let row_lbl = format!("{:<16} = {}", v.name, v.formatted_value);
                                if ui
                                    .selectable_label(
                                        false,
                                        egui::RichText::new(row_lbl).monospace().size(11.5),
                                    )
                                    .on_hover_text(v.description)
                                    .clicked()
                                {
                                    picked_token = Some(v.name);
                                }
                            }
                            if fx_open {
                                ui.separator();
                                ui.weak("Examples: tool_d * 0.4, 1/4 in, stock_z + 2mm");
                            }
                        });
                });

            if let Some(var_name) = picked_token {
                let next_expr = replace_trailing_identifier_prefix(&draft_text, var_name);
                if let Ok(parsed) = ParamLength::parse(&next_expr, ctx) {
                    *param = parsed;
                    committed = true;
                }
                draft_text = next_expr;
                ui.data_mut(|d| d.insert_temp(draft_id, draft_text.clone()));
                close_fx = true;
            }

            let clicked_outside = fx_open
                && ui.input(|i| i.pointer.any_pressed())
                && !area_resp.response.hovered()
                && !resp.hovered();
            if close_fx || clicked_outside || ui.input(|i| i.key_pressed(Key::Escape)) {
                ui.data_mut(|d| d.insert_temp(fx_open_id, false));
            }
        }
    });

    committed
}

/// Reusable stateful `egui` parameter input widget supporting mixed units (`1/4 in`, `6mm + 10 thou`)
/// and scoped CAM variables (`tool_d * 0.4`).
#[derive(Debug, Clone)]
pub struct ExpressionInput {
    pub param: ParamLength,
}

impl ExpressionInput {
    pub fn new(initial_mm: f64, initial_expr: Option<&str>) -> Self {
        Self {
            param: ParamLength {
                expr: initial_expr.map(str::to_string),
                value_mm: initial_mm,
            },
        }
    }

    pub fn value_mm(&self) -> f64 {
        self.param.value_mm
    }

    pub fn refresh(&mut self, ctx: &EvalContext) {
        let _ = self.param.reevaluate(ctx);
    }

    pub fn show(&mut self, ui: &mut egui::Ui, id_salt: &str, ctx: &EvalContext) -> bool {
        edit_param_length(ui, id_salt, &mut self.param, ctx, 128.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn collects_scoped_vars_in_deterministic_order_and_formats_units() {
        let ctx_mm = EvalContext::new(DisplayUnit::Millimeter)
            .with_var("stock_x", 100.0)
            .with_var("tool_d", 6.0)
            .with_var("tool_r", 3.0)
            .with_var("feature_bottom_z", -5.0)
            .with_scalar_var("flutes", 3.0);

        let vars = collect_scoped_vars(&ctx_mm);
        let names: Vec<&str> = vars.iter().map(|v| v.name).collect();
        assert_eq!(
            names,
            vec!["tool_d", "tool_r", "feature_bottom_z", "stock_x", "flutes"]
        );
        assert_eq!(vars[0].formatted_value, "6.000 mm");
        assert_eq!(vars[4].formatted_value, "3");

        let ctx_in = EvalContext::new(DisplayUnit::Inch).with_var("tool_d", 6.35);
        let vars_in = collect_scoped_vars(&ctx_in);
        assert_eq!(vars_in[0].formatted_value, "0.2500 in");
    }

    #[test]
    fn extracts_and_replaces_trailing_identifier_prefixes_cleanly() {
        assert_eq!(extract_trailing_identifier_prefix("to"), Some("to"));
        assert_eq!(
            extract_trailing_identifier_prefix("tool_d * 0.5 + st"),
            Some("st")
        );
        assert_eq!(extract_trailing_identifier_prefix("6mm"), None);
        assert_eq!(extract_trailing_identifier_prefix("0.25 in"), None);
        assert_eq!(extract_trailing_identifier_prefix("tool_d + "), None);

        assert_eq!(
            replace_trailing_identifier_prefix("tool_d * 0.5 + st", "stock_z"),
            "tool_d * 0.5 + stock_z"
        );
        assert_eq!(
            replace_trailing_identifier_prefix("3.0", "tool_d"),
            "tool_d"
        );
        assert_eq!(
            replace_trailing_identifier_prefix("tool_d *", "0.4"),
            "tool_d * 0.4"
        );
    }

    #[test]
    fn nudges_plain_numeric_lengths_in_millimeter_and_inch_display_units() {
        let p0 = ParamLength::from_mm(3.0);
        let p_up = apply_numeric_nudge(&p0, DisplayUnit::Millimeter, 1.0, false, false);
        assert!((p_up.value_mm - 3.1).abs() < 1e-9);
        assert!(p_up.expr.is_none());

        let p_shift = apply_numeric_nudge(&p0, DisplayUnit::Millimeter, 1.0, true, false);
        assert!((p_shift.value_mm - 4.0).abs() < 1e-9);

        let p_alt = apply_numeric_nudge(&p0, DisplayUnit::Millimeter, -1.0, false, true);
        assert!((p_alt.value_mm - 2.99).abs() < 1e-9);

        let p_in = ParamLength::from_mm(25.4 * 0.25);
        let p_in_up = apply_numeric_nudge(&p_in, DisplayUnit::Inch, 1.0, false, false);
        assert!((DisplayUnit::Inch.from_mm(p_in_up.value_mm) - 0.255).abs() < 1e-9);
    }
}
