use std::hash::Hash;

use chip_math::{DisplayUnit, EvalContext, ParamLength, evaluate_length_mm};

/// Immediate-mode inline expression editor operating directly on a document-owned [`ParamLength`].
///
/// Stores transient keyboard edit buffer in `egui` temporary memory keyed by `id_salt`, so
/// `Ctrl+Z` / `Ctrl+Y` document Undo/Redo immediately reflects updated values with zero stale state!
pub fn edit_param_length(
    ui: &mut egui::Ui,
    id_salt: impl Hash + std::fmt::Debug,
    param: &mut ParamLength,
    ctx: &EvalContext,
    width_px: f32,
) -> bool {
    let widget_id = ui.make_persistent_id(id_salt);
    let draft_id = widget_id.with("draft");

    let mut draft_text: String = ui
        .data_mut(|d| d.get_temp::<String>(draft_id))
        .unwrap_or_else(|| match &param.expr {
            Some(expr) => expr.clone(),
            None => ctx.default_unit.format_number(param.value_mm),
        });

    let mut committed = false;

    ui.horizontal(|ui| {
        let resp = ui.add(
            egui::TextEdit::singleline(&mut draft_text)
                .id(widget_id)
                .desired_width(width_px),
        );

        if !resp.has_focus() && !resp.lost_focus() {
            draft_text = match &param.expr {
                Some(expr) => expr.clone(),
                None => ctx.default_unit.format_number(param.value_mm),
            };
        }

        let live_eval = evaluate_length_mm(&draft_text, ctx);

        if resp.has_focus() {
            ui.data_mut(|d| d.insert_temp(draft_id, draft_text.clone()));
        }

        let enter_pressed = resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
        if resp.lost_focus() || enter_pressed {
            ui.data_mut(|d| d.remove::<String>(draft_id));
            if let Ok(parsed) = ParamLength::parse(&draft_text, ctx)
                && ((parsed.value_mm - param.value_mm).abs() > 1e-9 || parsed.expr != param.expr)
            {
                *param = parsed;
                committed = true;
            }
        }

        match live_eval {
            Ok(mm) => {
                ui.weak(format!("= {}", ctx.default_unit.format_length(mm)));
            }
            Err(err) if resp.has_focus() => {
                crate::widgets::status_badge_icon(
                    ui,
                    crate::widgets::StatusBadgeKind::Error,
                    Some(&err.to_string()),
                );
            }
            Err(_) => {
                ui.weak(format!(
                    "= {}",
                    ctx.default_unit.format_length(param.value_mm)
                ));
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
    draft_text: String,
    editing: bool,
    last_error: Option<String>,
}

impl ExpressionInput {
    pub fn new(initial_mm: f64, initial_expr: Option<&str>) -> Self {
        let param = ParamLength {
            expr: initial_expr.map(str::to_string),
            value_mm: initial_mm,
        };
        Self {
            draft_text: initial_expr
                .map(str::to_string)
                .unwrap_or_else(|| DisplayUnit::Millimeter.format_number(initial_mm)),
            param,
            editing: false,
            last_error: None,
        }
    }

    pub fn value_mm(&self) -> f64 {
        self.param.value_mm
    }

    pub fn refresh(&mut self, ctx: &EvalContext) {
        let _ = self.param.reevaluate(ctx);
    }

    pub fn show(&mut self, ui: &mut egui::Ui, id_salt: &str, ctx: &EvalContext) -> bool {
        let mut committed = false;

        if !self.editing {
            self.draft_text = match &self.param.expr {
                Some(expr) => expr.clone(),
                None => ctx.default_unit.format_number(self.param.value_mm),
            };
        }

        ui.horizontal(|ui| {
            let text_edit = egui::TextEdit::singleline(&mut self.draft_text)
                .id_salt(id_salt)
                .desired_width(154.0);
            let resp = ui.add(text_edit);

            if resp.gained_focus() {
                self.editing = true;
            }

            let live_eval = evaluate_length_mm(&self.draft_text, ctx);

            if self.editing {
                match &live_eval {
                    Ok(_) => self.last_error = None,
                    Err(e) => self.last_error = Some(e.to_string()),
                }
            }

            let enter_pressed = resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            if resp.lost_focus() || enter_pressed {
                self.editing = false;
                if let Ok(parsed) = ParamLength::parse(&self.draft_text, ctx) {
                    if (parsed.value_mm - self.param.value_mm).abs() > 1e-9
                        || parsed.expr != self.param.expr
                    {
                        self.param = parsed;
                        committed = true;
                    }
                    self.last_error = None;
                }
            }

            let preview_mm = live_eval.unwrap_or(self.param.value_mm);
            if let Some(err) = &self.last_error {
                crate::widgets::status_badge_icon(
                    ui,
                    crate::widgets::StatusBadgeKind::Error,
                    Some(err),
                );
            } else {
                ui.weak(format!("= {}", ctx.default_unit.format_length(preview_mm)));
            }
        });

        committed
    }
}
