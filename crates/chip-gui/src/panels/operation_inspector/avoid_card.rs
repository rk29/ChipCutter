use chip_cam::{DEFAULT_AVOID_CLEARANCE_MM, GeometrySelectionItem, Operation, SharedAvoidRegion};
use chip_math::{EvalContext, ParamLength};
use chip_project::ProjectDocument;
pub use chip_project::{
    create_and_link_shared_avoid, detach_shared_avoid_to_local, mark_ops_dirty_with_rest,
    mark_shared_avoid_dependents_dirty, next_shared_avoid_id_and_name,
    promote_local_avoid_to_shared, set_shared_avoid_for_all_setup_ops, toggle_shared_avoid_link,
};
use chip_sketch::SketchLayer;

use crate::{
    panels::{GeometryPickTarget, ViewportPickMode},
    theme::ThemeTokens,
    widgets::{
        BannerKind, INPUT_COL_WIDTH, IconKind, edit_param_length, grid_row, inline_banner,
        inspector_grid, section_card, small_icon_button,
    },
};

/// Deferred project-level avoid mutations executed atomically after rendering the `[Geometry]` card
/// (so idle frames never clone `ProjectDocument`).
pub enum AvoidCardAction {
    PromoteLocalToShared {
        name: String,
    },
    CreateNewShared,
    ToggleLinkOnOp {
        target_op_idx: usize,
        shared_id: String,
    },
    SetLinkForAllSetupOps {
        shared_id: String,
        link: bool,
    },
    DetachToLocal {
        shared_id: String,
    },
}

/// Read-only Setup context snapshot needed by [`show_avoid_regions_card`] without borrowing `ProjectDocument`.
pub struct AvoidCardSetupView {
    pub setup_fixtures: Vec<(String, String, bool)>,
    pub setup_ops: Vec<(usize, String, Vec<String>)>,
    pub shared_usage_counts: Vec<(String, usize)>,
}

impl AvoidCardSetupView {
    pub fn from_project(project: &ProjectDocument, setup_idx: usize) -> Self {
        let setup_fixtures = project
            .setups
            .get(setup_idx)
            .map(|s| {
                s.fixtures
                    .iter()
                    .map(|f| (f.id.clone(), f.name.clone(), f.enabled))
                    .collect()
            })
            .unwrap_or_default();
        let setup_ops = project
            .setups
            .get(setup_idx)
            .map(|s| {
                s.operations
                    .iter()
                    .enumerate()
                    .map(|(i, o)| (i, o.name.clone(), o.avoid.shared_region_ids.clone()))
                    .collect()
            })
            .unwrap_or_default();
        let shared_usage_counts = project
            .avoid_regions
            .iter()
            .map(|r| (r.id.clone(), project.avoid_region_usage_count(&r.id)))
            .collect();
        Self {
            setup_fixtures,
            setup_ops,
            shared_usage_counts,
        }
    }

    fn usage_count(&self, shared_id: &str) -> usize {
        self.shared_usage_counts
            .iter()
            .find(|(id, _)| id == shared_id)
            .map_or(0, |(_, c)| *c)
    }
}

pub struct AvoidCardCtx<'a> {
    pub op: &'a mut Operation,
    pub shared_regions: &'a mut [SharedAvoidRegion],
    pub sketches: &'a [SketchLayer],
    pub setup_view: &'a AvoidCardSetupView,
    pub setup_idx: usize,
    pub op_idx: usize,
    pub eval_ctx: &'a EvalContext,
    pub pick_mode: &'a mut ViewportPickMode,
    pub pick_target: &'a mut GeometryPickTarget,
    pub editing_shared_avoid_id: &'a mut Option<String>,
    pub mutated_label: &'a mut Option<&'static str>,
    pub shared_mutated_label: &'a mut Option<(&'static str, String)>,
    pub deferred_action: &'a mut Option<AvoidCardAction>,
}

pub fn show_avoid_regions_card(ui: &mut egui::Ui, ctx: AvoidCardCtx<'_>) {
    let AvoidCardCtx {
        op,
        shared_regions,
        sketches,
        setup_view,
        setup_idx,
        op_idx,
        eval_ctx,
        pick_mode,
        pick_target,
        editing_shared_avoid_id,
        mutated_label,
        shared_mutated_label,
        deferred_action,
    } = ctx;

    // Clear stale editing ID if deleted externally (e.g. from CAM Browser or Undo)
    if let Some(eid) = editing_shared_avoid_id.as_deref()
        && !shared_regions.iter().any(|r| r.id == eid)
    {
        *editing_shared_avoid_id = None;
    }

    let active_count = op.avoid.shared_region_ids.len()
        + op.avoid.selections.len()
        + op.avoid.fixture_ids.len()
        + usize::from(op.avoid.avoid_all_fixtures);
    let avoid_default_open = !op.avoid.is_empty() || editing_shared_avoid_id.is_some();
    let card_title = if active_count > 0 {
        format!("Avoid / Keep-Out Regions ({active_count})")
    } else {
        "Avoid / Keep-Out Regions".to_string()
    };

    let tok = ThemeTokens::from_ui(ui);

    section_card(
        ui,
        ("geom_avoid_card", setup_idx, op_idx, avoid_default_open),
        &card_title,
        avoid_default_open,
        |ui| {
            // 1. Active Shared Region Edit Banner (when editing a linked shared region)
            let editing_idx = editing_shared_avoid_id
                .as_deref()
                .and_then(|eid| shared_regions.iter().position(|r| r.id == eid));
            if let Some(s_idx) = editing_idx {
                let s_id = shared_regions[s_idx].id.clone();
                let s_name_snapshot = shared_regions[s_idx].name.clone();
                let usage = setup_view.usage_count(&s_id);
                let mut exit_edit = false;

                egui::Frame::NONE
                    .fill(tok.error_bg)
                    .stroke(egui::Stroke::new(1.2, tok.error_border))
                    .corner_radius(egui::CornerRadius::same(5))
                    .inner_margin(egui::Margin::symmetric(8, 6))
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            ui.colored_label(
                                tok.error_fg,
                                egui::RichText::new(format!(
                                    "Editing Shared Region: {} — linked to {} operation(s)",
                                    s_name_snapshot, usage
                                ))
                                .strong()
                                .small(),
                            );
                            ui.with_layout(
                                egui::Layout::right_to_left(egui::Align::Center),
                                |ui| {
                                    if ui
                                        .small_button("Done")
                                        .on_hover_text("Finish editing this shared keep-out region")
                                        .clicked()
                                    {
                                        exit_edit = true;
                                    }
                                },
                            );
                        });

                        ui.add_space(4.0);
                        let sr = &mut shared_regions[s_idx];

                        inspector_grid(ui, ("geom_shared_edit_grid", setup_idx, op_idx, &s_id), |ui| {
                            grid_row(
                                ui,
                                "Region Name",
                                Some("Display name for this reusable shared keep-out region"),
                                |ui| {
                                    let prev_name = sr.name.clone();
                                    let resp = ui.add(
                                        egui::TextEdit::singleline(&mut sr.name)
                                            .desired_width(INPUT_COL_WIDTH + 30.0),
                                    );
                                    if resp.lost_focus() && sr.name != prev_name {
                                        *shared_mutated_label =
                                            Some(("Rename Shared Keep-Out Region", s_id.clone()));
                                    }
                                },
                            );

                            // Optional Radial Clearance Override
                            grid_row(
                                ui,
                                "Radial Override",
                                Some(
                                    "Optional radial clearance override for this shared region (unchecked inherits the operation's Radial Clearance)",
                                ),
                                |ui| {
                                    ui.horizontal(|ui| {
                                        let mut has_rad = sr.radial_clearance.is_some();
                                        if ui
                                            .checkbox(&mut has_rad, "")
                                            .on_hover_text("Override operation radial clearance")
                                            .changed()
                                        {
                                            sr.radial_clearance = has_rad.then(|| {
                                                ParamLength::from_mm(DEFAULT_AVOID_CLEARANCE_MM)
                                            });
                                            *shared_mutated_label = Some((
                                                "Toggle Shared Radial Clearance Override",
                                                s_id.clone(),
                                            ));
                                        }
                                        if let Some(rad) = &mut sr.radial_clearance {
                                            if edit_param_length(
                                                ui,
                                                ("geom_shared_rad", setup_idx, op_idx, &s_id),
                                                rad,
                                                eval_ctx,
                                                INPUT_COL_WIDTH - 24.0,
                                            ) {
                                                *shared_mutated_label = Some((
                                                    "Edit Shared Keep-Out Radial Clearance",
                                                    s_id.clone(),
                                                ));
                                            }
                                        } else {
                                            ui.weak("Inherit from op");
                                        }
                                    });
                                },
                            );

                            // Optional Axial Clearance Override
                            grid_row(
                                ui,
                                "Axial Override",
                                Some(
                                    "Optional axial clearance override for this shared region (unchecked inherits the operation's Axial Clearance)",
                                ),
                                |ui| {
                                    ui.horizontal(|ui| {
                                        let mut has_ax = sr.axial_clearance.is_some();
                                        if ui
                                            .checkbox(&mut has_ax, "")
                                            .on_hover_text("Override operation axial clearance")
                                            .changed()
                                        {
                                            sr.axial_clearance = has_ax.then(|| {
                                                ParamLength::from_mm(DEFAULT_AVOID_CLEARANCE_MM)
                                            });
                                            *shared_mutated_label = Some((
                                                "Toggle Shared Axial Clearance Override",
                                                s_id.clone(),
                                            ));
                                        }
                                        if let Some(ax) = &mut sr.axial_clearance {
                                            if edit_param_length(
                                                ui,
                                                ("geom_shared_ax", setup_idx, op_idx, &s_id),
                                                ax,
                                                eval_ctx,
                                                INPUT_COL_WIDTH - 24.0,
                                            ) {
                                                *shared_mutated_label = Some((
                                                    "Edit Shared Keep-Out Axial Clearance",
                                                    s_id.clone(),
                                                ));
                                            }
                                        } else {
                                            ui.weak("Inherit from op");
                                        }
                                    });
                                },
                            );
                        });

                        // Fixture chips inside shared region edit
                        if !setup_view.setup_fixtures.is_empty() {
                            ui.add_space(4.0);
                            ui.weak("Shared Fixtures:");
                            ui.horizontal_wrapped(|ui| {
                                for (fid, fname, enabled) in &setup_view.setup_fixtures {
                                    let linked = sr.fixture_ids.contains(fid);
                                    let lbl = if linked {
                                        format!("[x] {fname}")
                                    } else {
                                        format!("[ ] {fname}")
                                    };
                                    let rich = if *enabled {
                                        egui::RichText::new(lbl)
                                    } else {
                                        egui::RichText::new(lbl).weak()
                                    };
                                    if ui
                                        .selectable_label(linked, rich)
                                        .on_hover_text(format!(
                                            "Include/exclude fixture '{fname}' in shared keep-out region"
                                        ))
                                        .clicked()
                                    {
                                        if linked {
                                            sr.fixture_ids.retain(|x| x != fid);
                                        } else {
                                            sr.fixture_ids.push(fid.clone());
                                        }
                                        *shared_mutated_label = Some((
                                            "Toggle Shared Keep-Out Fixture",
                                            s_id.clone(),
                                        ));
                                    }
                                }
                            });
                        }

                        // Sketch + Avoid quick-add directly into this shared region
                        if !sketches.is_empty() {
                            ui.add_space(4.0);
                            show_sketch_avoid_quick_add(
                                ui,
                                sketches,
                                &mut sr.selections,
                                &mut || {
                                    *shared_mutated_label = Some((
                                        "Update Shared Keep-Out Sketch",
                                        s_id.clone(),
                                    ));
                                },
                            );
                        }

                        // Removable list of shared region selections
                        if !sr.selections.is_empty() {
                            ui.add_space(4.0);
                            let mut rm_idx: Option<usize> = None;
                            for (i, item) in sr.selections.iter().enumerate() {
                                ui.horizontal(|ui| {
                                    let w = (ui.available_width() - 28.0).max(56.0);
                                    ui.add_sized(
                                        [w, 18.0],
                                        egui::Label::new(format!("• {}", item.short_label()))
                                            .truncate(),
                                    )
                                    .on_hover_text(item.short_label());
                                    if small_icon_button(
                                        ui,
                                        IconKind::Trash,
                                        "Remove from shared keep-out region",
                                    )
                                    .clicked()
                                    {
                                        rm_idx = Some(i);
                                    }
                                });
                            }
                            if let Some(ri) = rm_idx {
                                sr.selections.remove(ri);
                                *shared_mutated_label = Some((
                                    "Remove Shared Keep-Out Selection",
                                    s_id.clone(),
                                ));
                            }
                        } else {
                            ui.add_space(2.0);
                            ui.weak(
                                "Click 3D faces, closed loops, holes, or sketch buttons above to add keep-out geometry to this shared region.",
                            );
                        }
                    });
                ui.add_space(5.0);

                if exit_edit {
                    *editing_shared_avoid_id = None;
                    *pick_target = GeometryPickTarget::CuttingArea;
                }
            }

            // 2. Shared Keep-Out Regions (`ProjectDocument::avoid_regions`)
            let save_prompt_open_id =
                ui.make_persistent_id(("avoid_save_shared_open", setup_idx, op_idx));
            let save_prompt_name_id =
                ui.make_persistent_id(("avoid_save_shared_name", setup_idx, op_idx));
            let mut save_prompt_open = ui
                .data_mut(|d| d.get_temp::<bool>(save_prompt_open_id))
                .unwrap_or(false);
            let mut save_prompt_name = ui
                .data_mut(|d| d.get_temp::<String>(save_prompt_name_id))
                .unwrap_or_default();

            let has_local_promotable =
                !op.avoid.selections.is_empty() || !op.avoid.fixture_ids.is_empty();

            ui.horizontal_wrapped(|ui| {
                ui.strong("Shared Regions");
                if ui
                    .small_button("+ New Shared")
                    .on_hover_text(
                        "Create a new reusable named Keep-Out region and link it to this operation",
                    )
                    .clicked()
                {
                    *deferred_action = Some(AvoidCardAction::CreateNewShared);
                }
                if has_local_promotable
                    && ui
                        .small_button("Save Local as Shared…")
                        .on_hover_text(
                            "Promote this operation's local keep-out selections & fixtures into a named shared region reusable across operations",
                        )
                        .clicked()
                {
                    save_prompt_open = true;
                    if save_prompt_name.trim().is_empty() {
                        save_prompt_name = format!("Keep-Out {}", shared_regions.len() + 1);
                    }
                }
            });

            if save_prompt_open {
                ui.horizontal(|ui| {
                    let resp = ui.add(
                        egui::TextEdit::singleline(&mut save_prompt_name)
                            .desired_width(140.0)
                            .hint_text("Clamp Zone A"),
                    );
                    let enter = resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                    if (ui.small_button("Save").clicked() || enter)
                        && !save_prompt_name.trim().is_empty()
                    {
                        *deferred_action = Some(AvoidCardAction::PromoteLocalToShared {
                            name: save_prompt_name.trim().to_string(),
                        });
                        save_prompt_open = false;
                        save_prompt_name.clear();
                    }
                    if ui.small_button("Cancel").clicked() {
                        save_prompt_open = false;
                    }
                });
            }
            ui.data_mut(|d| {
                d.insert_temp(save_prompt_open_id, save_prompt_open);
                d.insert_temp(save_prompt_name_id, save_prompt_name);
            });

            if !shared_regions.is_empty() {
                ui.add_space(2.0);
                // Toggleable 1-click chips for every project SharedAvoidRegion
                ui.horizontal_wrapped(|ui| {
                    for sr in shared_regions.iter() {
                        let linked = op.avoid.shared_region_ids.contains(&sr.id);
                        let item_count = sr.selections.len() + sr.fixture_ids.len();
                        let chip_txt = if linked {
                            format!("[x] {} ({item_count})", sr.name)
                        } else {
                            format!("[ ] {} ({item_count})", sr.name)
                        };
                        let rich = if sr.suppressed {
                            egui::RichText::new(chip_txt).weak().strikethrough()
                        } else if linked {
                            egui::RichText::new(chip_txt).color(tok.error_fg)
                        } else {
                            egui::RichText::new(chip_txt)
                        };
                        let tip = if sr.suppressed {
                            format!(
                                "'{}' is currently suppressed in the CAM Browser (click to link/unlink)",
                                sr.name
                            )
                        } else if linked {
                            format!("Linked to '{}' — click to unlink from this operation", sr.name)
                        } else {
                            format!("Click to link shared keep-out '{}' to this operation", sr.name)
                        };
                        if ui
                            .selectable_label(linked, rich)
                            .on_hover_text(tip)
                            .clicked()
                        {
                            *deferred_action = Some(AvoidCardAction::ToggleLinkOnOp {
                                target_op_idx: op_idx,
                                shared_id: sr.id.clone(),
                            });
                        }
                    }
                });

                // Inline actions for every linked shared region
                let linked_ids: Vec<String> = op.avoid.shared_region_ids.clone();
                for sid in &linked_ids {
                    if let Some(sr) = shared_regions.iter().find(|r| r.id == *sid) {
                        let is_editing = editing_shared_avoid_id.as_deref() == Some(sr.id.as_str());
                        let sr_id = sr.id.clone();
                        let sr_name = sr.name.clone();
                        ui.horizontal_wrapped(|ui| {
                            ui.spacing_mut().item_spacing.x = 4.0;
                            ui.colored_label(tok.text_secondary, format!("• {sr_name}:"));
                            if ui
                                .selectable_label(is_editing, "Edit")
                                .on_hover_text(
                                    "Edit this shared keep-out region directly (routes viewport picking & sketch + Avoid into it)",
                                )
                                .clicked()
                            {
                                if is_editing {
                                    *editing_shared_avoid_id = None;
                                    *pick_target = GeometryPickTarget::CuttingArea;
                                } else {
                                    *editing_shared_avoid_id = Some(sr_id.clone());
                                    *pick_target = GeometryPickTarget::AvoidKeepOut;
                                    if *pick_mode == ViewportPickMode::InspectEntity {
                                        *pick_mode =
                                            ViewportPickMode::OpPlanarFaces { setup_idx, op_idx };
                                    }
                                }
                            }
                            if ui
                                .small_button("Detach to Local")
                                .on_hover_text(
                                    "Copy this shared region's selections & fixtures into local Avoid and unlink the shared ID",
                                )
                                .clicked()
                            {
                                if is_editing {
                                    *editing_shared_avoid_id = None;
                                }
                                *deferred_action = Some(AvoidCardAction::DetachToLocal {
                                    shared_id: sr_id.clone(),
                                });
                            }
                            ui.menu_button("Apply to Setup Ops…", |ui| {
                                ui.strong(format!("Link '{sr_name}' to Operations"));
                                ui.separator();
                                for (target_idx, target_name, target_ids) in &setup_view.setup_ops {
                                    let mut has_link = target_ids.contains(&sr_id);
                                    if ui
                                        .checkbox(
                                            &mut has_link,
                                            format!("{}. {target_name}", target_idx + 1),
                                        )
                                        .changed()
                                    {
                                        *deferred_action = Some(AvoidCardAction::ToggleLinkOnOp {
                                            target_op_idx: *target_idx,
                                            shared_id: sr_id.clone(),
                                        });
                                    }
                                }
                                ui.separator();
                                ui.horizontal(|ui| {
                                    if ui.small_button("Link All").clicked() {
                                        *deferred_action =
                                            Some(AvoidCardAction::SetLinkForAllSetupOps {
                                                shared_id: sr_id.clone(),
                                                link: true,
                                            });
                                        ui.close();
                                    }
                                    if ui.small_button("Unlink All").clicked() {
                                        *deferred_action =
                                            Some(AvoidCardAction::SetLinkForAllSetupOps {
                                                shared_id: sr_id.clone(),
                                                link: false,
                                            });
                                        ui.close();
                                    }
                                });
                            })
                            .response
                            .on_hover_text(
                                "Link or unlink this shared keep-out region across multiple operations in the current Setup",
                            );
                        });
                    }
                }
            }

            ui.add_space(4.0);
            ui.separator();
            ui.add_space(4.0);

            // 3. Fixture Keep-Outs
            let enabled_fixture_count = setup_view
                .setup_fixtures
                .iter()
                .filter(|(_, _, en)| *en)
                .count();
            let fix_badge = if enabled_fixture_count == 1 {
                "(1 fixture in Setup)".to_string()
            } else {
                format!("({enabled_fixture_count} fixtures in Setup)")
            };
            ui.horizontal_wrapped(|ui| {
                if ui
                    .checkbox(&mut op.avoid.avoid_all_fixtures, "Avoid All Setup Fixtures")
                    .on_hover_text(
                        "Automatically subtract all enabled Setup workholding fixtures overlapping this operation's Z band",
                    )
                    .changed()
                {
                    *mutated_label = Some("Toggle Avoid All Setup Fixtures");
                }
                ui.weak(fix_badge);
            });

            if !op.avoid.avoid_all_fixtures && !setup_view.setup_fixtures.is_empty() {
                ui.horizontal_wrapped(|ui| {
                    for (fid, fname, enabled) in &setup_view.setup_fixtures {
                        let selected = op.avoid.fixture_ids.contains(fid);
                        let lbl = if selected {
                            format!("[x] {fname}")
                        } else {
                            format!("[ ] {fname}")
                        };
                        let rich = if !*enabled {
                            egui::RichText::new(lbl).weak()
                        } else if selected {
                            egui::RichText::new(lbl).color(tok.error_fg)
                        } else {
                            egui::RichText::new(lbl)
                        };
                        if ui
                            .selectable_label(selected, rich)
                            .on_hover_text(format!(
                                "Exclude fixture '{fname}' from this operation's cutting toolpath"
                            ))
                            .clicked()
                        {
                            if selected {
                                op.avoid.fixture_ids.retain(|x| x != fid);
                            } else {
                                op.avoid.fixture_ids.push(fid.clone());
                            }
                            *mutated_label = Some("Toggle Keep-Out Fixture");
                        }
                    }
                });
            }

            ui.add_space(4.0);
            ui.separator();
            ui.add_space(4.0);

            // 4. Local Geometry & Sketch Keep-Outs
            ui.horizontal_wrapped(|ui| {
                ui.strong(format!(
                    "Local Keep-Out Geometry ({})",
                    op.avoid.selections.len()
                ));
                let is_avoid_picking = *pick_target == GeometryPickTarget::AvoidKeepOut
                    && editing_shared_avoid_id.is_none();
                let pick_btn_lbl = if is_avoid_picking {
                    "[x] Picking 3D Avoid"
                } else {
                    "Pick 3D Avoid"
                };
                if ui
                    .selectable_label(
                        is_avoid_picking,
                        egui::RichText::new(pick_btn_lbl).color(if is_avoid_picking {
                            tok.error_fg
                        } else {
                            tok.text_primary
                        }),
                    )
                    .on_hover_text(
                        "Switch 3D viewport picking to add/remove local Avoid / Keep-Out geometry",
                    )
                    .clicked()
                {
                    *editing_shared_avoid_id = None;
                    if is_avoid_picking {
                        *pick_target = GeometryPickTarget::CuttingArea;
                    } else {
                        *pick_target = GeometryPickTarget::AvoidKeepOut;
                        if *pick_mode == ViewportPickMode::InspectEntity {
                            *pick_mode = ViewportPickMode::OpPlanarFaces { setup_idx, op_idx };
                        }
                    }
                }
                if !op.avoid.selections.is_empty()
                    && ui
                        .small_button("Clear Local Avoid")
                        .on_hover_text(
                            "Remove all local keep-out geometry selections from this operation",
                        )
                        .clicked()
                {
                    op.avoid.selections.clear();
                    *mutated_label = Some("Clear Local Avoid Selections");
                }
            });

            // Compact + Avoid quick-add buttons for 2D Sketch layers/entities (routed to local when not editing shared)
            if !sketches.is_empty() && editing_shared_avoid_id.is_none() {
                ui.add_space(2.0);
                show_sketch_avoid_quick_add(ui, sketches, &mut op.avoid.selections, &mut || {
                    *mutated_label = Some("Update Local Avoid Sketch");
                });
            }

            if !op.avoid.selections.is_empty() {
                ui.add_space(3.0);
                let mut remove_idx: Option<usize> = None;
                for (idx, item) in op.avoid.selections.iter().enumerate() {
                    ui.horizontal(|ui| {
                        let lbl_w = (ui.available_width() - 28.0).max(56.0);
                        ui.add_sized(
                            [lbl_w, 18.0],
                            egui::Label::new(format!("• {}", item.short_label())).truncate(),
                        )
                        .on_hover_text(item.short_label());
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if small_icon_button(
                                ui,
                                IconKind::Trash,
                                "Remove this local keep-out selection",
                            )
                            .clicked()
                            {
                                remove_idx = Some(idx);
                            }
                        });
                    });
                }
                if let Some(ri) = remove_idx {
                    op.avoid.selections.remove(ri);
                    *mutated_label = Some("Remove Local Avoid Selection");
                }
            } else if op.avoid.is_empty() {
                ui.add_space(2.0);
                inline_banner(
                    ui,
                    BannerKind::Info,
                    "No keep-out regions active. Enable fixture avoidance, link a shared region, or pick 3D/sketch keep-outs above.",
                );
            }

            ui.add_space(4.0);
            ui.separator();
            ui.add_space(4.0);

            // 5. Clearances (`Radial Clearance` & `Axial Clearance`)
            let show_axial = op.kind.is_rotary_native()
                || op.avoid.references_fixtures(shared_regions)
                || !setup_view.setup_fixtures.is_empty()
                || (op.avoid.axial_clearance.value_mm - DEFAULT_AVOID_CLEARANCE_MM).abs() > 1e-6;

            inspector_grid(ui, ("geom_avoid_clr_grid", setup_idx, op_idx), |ui| {
                grid_row(
                    ui,
                    "Radial Clearance",
                    Some(
                        "Extra XY radial safety buffer added around Avoid / Keep-Out boundaries (tool_r + Radial Clearance)",
                    ),
                    |ui| {
                        if edit_param_length(
                            ui,
                            ("geom_avoid_rad_clr", setup_idx, op_idx),
                            &mut op.avoid.radial_clearance,
                            eval_ctx,
                            INPUT_COL_WIDTH,
                        ) {
                            *mutated_label = Some("Edit Avoid Radial Clearance");
                        }
                    },
                );
                if show_axial {
                    grid_row(
                        ui,
                        "Axial Clearance",
                        Some(
                            "Extra Z / axial safety buffer for Z-band fixture overlap and 4th-axis rotary axial keep-out bands",
                        ),
                        |ui| {
                            if edit_param_length(
                                ui,
                                ("geom_avoid_ax_clr", setup_idx, op_idx),
                                &mut op.avoid.axial_clearance,
                                eval_ctx,
                                INPUT_COL_WIDTH,
                            ) {
                                *mutated_label = Some("Edit Avoid Axial Clearance");
                            }
                        },
                    );
                }
            });
        },
    );
}

pub(crate) fn show_sketch_avoid_quick_add(
    ui: &mut egui::Ui,
    sketches: &[SketchLayer],
    target_selections: &mut Vec<GeometrySelectionItem>,
    on_mutated: &mut impl FnMut(),
) {
    ui.horizontal_wrapped(|ui| {
        ui.weak("Sketch:");
        const MAX_ENTITIES: usize = 6;
        for sk in sketches {
            let all_item = GeometrySelectionItem::SketchLayerAll {
                sketch_id: sk.id.clone(),
                sub_layer: None,
                reverse: false,
                side_flip: false,
            };
            let has_all = target_selections.contains(&all_item);
            let layer_lbl = if has_all {
                format!("[x] {}", sk.name)
            } else {
                format!("+ Avoid {}", sk.name)
            };
            if ui
                .small_button(layer_lbl)
                .on_hover_text(format!(
                    "Toggle entire 2D sketch layer '{}' as a keep-out region",
                    sk.name
                ))
                .clicked()
            {
                if has_all {
                    target_selections.retain(|s| s != &all_item);
                } else {
                    target_selections.push(all_item);
                }
                on_mutated();
            }

            for (e_idx, ent) in sk
                .entities
                .iter()
                .filter(|e| !e.is_construction)
                .take(MAX_ENTITIES)
                .enumerate()
            {
                let short_name = if ent.name.trim().is_empty() {
                    format!("{} {}", ent.entity.kind_label(), e_idx + 1)
                } else {
                    ent.name.trim().to_string()
                };
                let existing_pos = target_selections.iter().position(|s| {
                    matches!(
                        s,
                        GeometrySelectionItem::SketchEntity {
                            sketch_id,
                            entity_id,
                            ..
                        } if sketch_id == &sk.id && entity_id == &ent.id
                    )
                });
                let chip = if existing_pos.is_some() {
                    format!("[x] {short_name}")
                } else {
                    format!("+ Avoid: {short_name}")
                };
                if ui
                    .small_button(chip)
                    .on_hover_text(format!(
                        "Toggle sketch entity '{short_name}' from '{}' as a keep-out region",
                        sk.name
                    ))
                    .clicked()
                {
                    if let Some(pos) = existing_pos {
                        target_selections.remove(pos);
                    } else {
                        target_selections.push(GeometrySelectionItem::SketchEntity {
                            sketch_id: sk.id.clone(),
                            entity_id: ent.id.clone(),
                            reverse: false,
                            side_flip: false,
                        });
                    }
                    on_mutated();
                }
            }
        }
    });
}
