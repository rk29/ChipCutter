//! Interactive **Swap Machine Tools** dialog shown when switching a project to a CNC machine
//! whose tool library does not contain one or more tools referenced by existing operations.

use std::collections::BTreeMap;

use chip_machines::MachineDefinition;
use chip_project::{MachineToolSwapPlan, ToolSwapChoice};
use chip_tools::ToolId;
use egui::{CornerRadius, Stroke};

use crate::theme::{AppTheme, ThemeTokens};

/// Committed result emitted when the user confirms the machine switch & tool swap.
#[derive(Debug, Clone)]
pub struct MachineToolSwapCommit {
    pub target_machine: MachineDefinition,
    pub sync_setups: bool,
    pub choices: BTreeMap<ToolId, ToolSwapChoice>,
    pub prune_replaced_unused: bool,
}

/// Persistent UI state for the Machine-Switch Tool Replacement modal.
#[derive(Debug, Default)]
pub struct MachineToolSwapModalState {
    pub open: bool,
    source_machine_name: String,
    target_machine: Option<MachineDefinition>,
    plan: Option<MachineToolSwapPlan>,
    choices: BTreeMap<ToolId, ToolSwapChoice>,
    sync_setups: bool,
    prune_replaced_unused: bool,
}

impl MachineToolSwapModalState {
    pub fn open_for_plan(
        &mut self,
        source_machine_name: String,
        target_machine: MachineDefinition,
        plan: MachineToolSwapPlan,
        sync_setups: bool,
    ) {
        let mut choices = BTreeMap::new();
        for entry in &plan.entries {
            choices.insert(entry.current_tool.id.clone(), entry.suggested.clone());
        }
        self.source_machine_name = source_machine_name;
        self.target_machine = Some(target_machine);
        self.plan = Some(plan);
        self.choices = choices;
        self.sync_setups = sync_setups;
        self.prune_replaced_unused = true;
        self.open = true;
    }

    pub fn close(&mut self) {
        self.open = false;
        self.target_machine = None;
        self.plan = None;
        self.choices.clear();
    }

    /// Shows the modal when `self.open == true`, returning `Some(commit)` when confirmed.
    pub fn show(&mut self, ctx: &egui::Context, theme: AppTheme) -> Option<MachineToolSwapCommit> {
        if !self.open {
            return None;
        }
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            self.close();
            return None;
        }

        let Some(target_machine) = self.target_machine.clone() else {
            self.close();
            return None;
        };
        let Some(plan) = self.plan.clone() else {
            self.close();
            return None;
        };

        let tok = ThemeTokens::for_theme(theme);
        let mut keep_open = self.open;
        let mut confirmed = false;
        let mut cancelled = false;

        egui::Window::new(format!("Switch Machine — {}", target_machine.name))
            .open(&mut keep_open)
            .default_size([720.0, 440.0])
            .min_size([560.0, 320.0])
            .resizable(true)
            .collapsible(false)
            .anchor(egui::Align2::CENTER_CENTER, egui::vec2(0.0, 0.0))
            .show(ctx, |ui| {
                // Machine transition summary banner
                egui::Frame::NONE
                    .fill(tok.info_bg)
                    .stroke(Stroke::new(1.0, tok.info_border))
                    .corner_radius(CornerRadius::same(6))
                    .inner_margin(egui::Margin::symmetric(10, 8))
                    .show(ui, |ui| {
                        ui.label(
                            egui::RichText::new(format!(
                                "Switching from '{}' to '{}'",
                                self.source_machine_name, target_machine.name
                            ))
                            .strong()
                            .color(tok.info_fg),
                        );
                        ui.label(
                            egui::RichText::new(format!(
                                "Travel {:.0}×{:.0}×{:.0} mm · {:.0}–{:.0} RPM · {} tools · Post: {}",
                                target_machine.travel_mm.x_mm,
                                target_machine.travel_mm.y_mm,
                                target_machine.travel_mm.z_mm,
                                target_machine.spindle.min_rpm,
                                target_machine.spindle.max_rpm,
                                target_machine.tools.len(),
                                target_machine.default_post_profile
                            ))
                            .small()
                            .color(tok.text_secondary),
                        );
                    });

                ui.add_space(6.0);
                ui.label(
                    egui::RichText::new(format!(
                        "{} tool(s) used by operations are not in '{}'s library. Choose replacement tools below (or keep the current embedded cutter):",
                        plan.entries.len(),
                        target_machine.name
                    ))
                    .color(tok.text_secondary),
                );
                if !plan.exact_matches.is_empty() {
                    ui.label(
                        egui::RichText::new(format!(
                            "{} matching tool(s) already exist on '{}' and will remain unchanged.",
                            plan.exact_matches.len(),
                            target_machine.name
                        ))
                        .small()
                        .color(tok.text_muted),
                    );
                }
                ui.add_space(6.0);

                // Per-tool replacement rows
                egui::ScrollArea::vertical()
                    .id_salt("machine_tool_swap_scroll")
                    .max_height(240.0)
                    .show(ui, |ui| {
                        egui::Grid::new("machine_tool_swap_grid")
                            .num_columns(3)
                            .spacing([12.0, 8.0])
                            .striped(true)
                            .show(ui, |ui| {
                                ui.strong("Current Project Tool");
                                ui.strong("Used By");
                                ui.strong(format!("Replacement on {}", target_machine.name));
                                ui.end_row();

                                for (idx, entry) in plan.entries.iter().enumerate() {
                                    let cur = &entry.current_tool;
                                    ui.vertical(|ui| {
                                        ui.label(
                                            egui::RichText::new(cur.summary_label()).strong(),
                                        );
                                    });

                                    let ops_summary = if entry.usages.len() == 1 {
                                        let u = &entry.usages[0];
                                        if u.is_clearing_tool {
                                            format!("{} (clearing)", u.op_name)
                                        } else {
                                            u.op_name.clone()
                                        }
                                    } else {
                                        let names: Vec<String> = entry
                                            .usages
                                            .iter()
                                            .take(3)
                                            .map(|u| u.op_name.clone())
                                            .collect();
                                        let more = if entry.usages.len() > 3 {
                                            format!(" +{}", entry.usages.len() - 3)
                                        } else {
                                            String::new()
                                        };
                                        format!(
                                            "{} ops ({}{more})",
                                            entry.usages.len(),
                                            names.join(", ")
                                        )
                                    };
                                    ui.label(
                                        egui::RichText::new(ops_summary)
                                            .small()
                                            .color(tok.text_secondary),
                                    );

                                    let cur_choice = self
                                        .choices
                                        .entry(cur.id.clone())
                                        .or_insert_with(|| entry.suggested.clone());
                                    let selected_txt = match cur_choice {
                                        ToolSwapChoice::ReplaceWith(t) => t.summary_label(),
                                        ToolSwapChoice::KeepExisting => {
                                            "Keep current embedded tool".to_string()
                                        }
                                    };
                                    let cur_disc = std::mem::discriminant(&cur.geometry);
                                    egui::ComboBox::from_id_salt(("swap_tool_cb", idx))
                                        .width(260.0)
                                        .truncate()
                                        .selected_text(selected_txt)
                                        .show_ui(ui, |ui| {
                                            let is_keep =
                                                matches!(cur_choice, ToolSwapChoice::KeepExisting);
                                            if ui
                                                .selectable_label(
                                                    is_keep,
                                                    "Keep current embedded tool",
                                                )
                                                .clicked()
                                            {
                                                *cur_choice = ToolSwapChoice::KeepExisting;
                                            }
                                            ui.separator();

                                            let mut shown_compat_hdr = false;
                                            let mut shown_other_hdr = false;
                                            for cand in &entry.candidates {
                                                let same_kind =
                                                    std::mem::discriminant(&cand.geometry)
                                                        == cur_disc;
                                                if same_kind && !shown_compat_hdr {
                                                    ui.label(
                                                        egui::RichText::new("Matching Cutter Type")
                                                            .small()
                                                            .strong()
                                                            .color(tok.text_muted),
                                                    );
                                                    shown_compat_hdr = true;
                                                } else if !same_kind && !shown_other_hdr {
                                                    if shown_compat_hdr {
                                                        ui.separator();
                                                    }
                                                    ui.label(
                                                        egui::RichText::new("Other Machine Tools")
                                                            .small()
                                                            .strong()
                                                            .color(tok.text_muted),
                                                    );
                                                    shown_other_hdr = true;
                                                }
                                                let is_sel = matches!(
                                                    cur_choice,
                                                    ToolSwapChoice::ReplaceWith(t) if t.id == cand.id
                                                );
                                                if ui
                                                    .selectable_label(is_sel, cand.summary_label())
                                                    .clicked()
                                                {
                                                    *cur_choice = ToolSwapChoice::ReplaceWith(
                                                        Box::new(cand.clone()),
                                                    );
                                                }
                                            }
                                        });
                                    ui.end_row();
                                }
                            });
                    });

                ui.add_space(6.0);
                ui.separator();
                ui.add_space(4.0);

                ui.checkbox(
                    &mut self.prune_replaced_unused,
                    "Remove replaced tools that are no longer used by any operation",
                );
                ui.checkbox(
                    &mut self.sync_setups,
                    format!(
                        "Sync setup post-processor & rotary chuck defaults to '{}'",
                        target_machine.name
                    ),
                );

                ui.add_space(8.0);
                ui.horizontal(|ui| {
                    if ui
                        .add(
                            egui::Button::new(
                                egui::RichText::new("Switch Machine & Swap Tools")
                                    .strong()
                                    .color(tok.accent_on_fg),
                            )
                            .fill(tok.accent),
                        )
                        .clicked()
                    {
                        confirmed = true;
                    }
                    if ui.button("Cancel").clicked() {
                        cancelled = true;
                    }
                });
            });

        if confirmed {
            let commit = MachineToolSwapCommit {
                target_machine,
                sync_setups: self.sync_setups,
                choices: self.choices.clone(),
                prune_replaced_unused: self.prune_replaced_unused,
            };
            self.close();
            return Some(commit);
        }
        if cancelled || !keep_open {
            self.close();
        }
        None
    }
}
