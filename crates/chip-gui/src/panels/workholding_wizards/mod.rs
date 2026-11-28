//! Interactive **Two-Sided Flip-Jig Wizard** and **Auto Soft-Jaw Generator Wizard** modals
//! (`chip-gui`, Milestone 63).
//!
//! Strict two-column layout with universal hover tooltips, live exact-geometry preview summary,
//! single-step `< 1 ms` Undo on Apply, and native / XDG-portal STL folder export.

mod flip_jig;
mod soft_jaws;

use chip_cam::WorkOffsetSlot;
use chip_math::DisplayUnit;
use chip_tools::{Tool, ToolId};

pub use flip_jig::FlipJigWizardModalState;
pub use soft_jaws::SoftJawWizardModalState;

use crate::widgets::INPUT_COL_WIDTH;

/// Outcome returned when the user applies the Flip-Jig Wizard.
pub struct FlipJigApplyResult {
    pub new_setup_idx: usize,
    pub log_message: String,
}

/// Outcome returned when the user applies or exports from the Auto Soft-Jaw Wizard.
pub struct SoftJawApplyResult {
    pub new_setup_idx: Option<usize>,
    pub log_message: String,
}

#[allow(clippy::too_many_arguments)]
pub(super) fn edit_length_mm(
    ui: &mut egui::Ui,
    val_mm: &mut f64,
    range_mm: std::ops::RangeInclusive<f64>,
    speed_mm: f64,
    decimals_mm: usize,
    display_unit: DisplayUnit,
    prefix: &str,
    hover: &str,
) -> bool {
    let mut disp = display_unit.from_mm(*val_mm);
    let lo = display_unit.from_mm(*range_mm.start());
    let hi = display_unit.from_mm(*range_mm.end());
    let speed = if display_unit == DisplayUnit::Inch {
        (speed_mm / 25.4).max(0.001)
    } else {
        speed_mm
    };
    let decimals = if display_unit == DisplayUnit::Inch {
        decimals_mm.max(3)
    } else {
        decimals_mm
    };
    let mut dv = egui::DragValue::new(&mut disp)
        .speed(speed)
        .range(lo..=hi)
        .max_decimals(decimals)
        .suffix(format!(" {}", display_unit.suffix()));
    if !prefix.is_empty() {
        dv = dv.prefix(prefix);
    }
    let resp = ui.add(dv).on_hover_text(hover);
    if resp.changed() {
        let new_mm = display_unit
            .to_mm(disp)
            .clamp(*range_mm.start(), *range_mm.end());
        if new_mm.is_finite() {
            *val_mm = new_mm;
            return true;
        }
    }
    false
}

pub(super) fn render_optional_tool_combo(
    ui: &mut egui::Ui,
    id_salt: &'static str,
    selected: &mut Option<ToolId>,
    tools: &[Tool],
    auto_hint: &str,
    hover: &str,
) -> bool {
    let prev = selected.clone();
    let sel_text = match selected {
        None => format!("Auto ({auto_hint})"),
        Some(tid) => tools
            .iter()
            .find(|t| &t.id == tid)
            .map(|t| format!("T{} {}", t.tool_number, t.name))
            .unwrap_or_else(|| tid.0.clone()),
    };
    egui::ComboBox::from_id_salt(id_salt)
        .width(INPUT_COL_WIDTH + 60.0)
        .truncate()
        .selected_text(sel_text)
        .show_ui(ui, |ui| {
            ui.selectable_value(selected, None, format!("Auto ({auto_hint})"))
                .on_hover_text(
                    "Automatically pick the best matching tool from the project library",
                );
            for t in tools {
                let lbl = format!("T{} {}", t.tool_number, t.name);
                ui.selectable_value(selected, Some(t.id.clone()), lbl)
                    .on_hover_text(format!("Force tool T{} ({})", t.tool_number, t.name));
            }
        })
        .response
        .on_hover_text(hover);
    *selected != prev
}

pub(super) fn render_optional_wcs_combo(
    ui: &mut egui::Ui,
    id_salt: &'static str,
    selected: &mut Option<WorkOffsetSlot>,
    auto_slot: WorkOffsetSlot,
    hover: &str,
) -> bool {
    let prev = *selected;
    let auto_lbl = format!("Auto ({})", auto_slot.short_code());
    let sel_text = match selected {
        None => auto_lbl.clone(),
        Some(slot) => slot.label().to_string(),
    };
    egui::ComboBox::from_id_salt(id_salt)
        .width(INPUT_COL_WIDTH + 60.0)
        .truncate()
        .selected_text(sel_text)
        .show_ui(ui, |ui| {
            ui.selectable_value(selected, None, auto_lbl)
                .on_hover_text("Automatically select the next unused work offset slot");
            for slot in WorkOffsetSlot::ALL {
                ui.selectable_value(selected, Some(slot), slot.label())
                    .on_hover_text(format!("Force work offset {}", slot.label()));
            }
        })
        .response
        .on_hover_text(hover);
    *selected != prev
}

#[cfg(test)]
mod tests;
