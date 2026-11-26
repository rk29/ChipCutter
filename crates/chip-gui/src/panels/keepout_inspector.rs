//! Dedicated right-panel **Keep-Out Region Inspector** (`InspectorFocus::KeepOutRegion`) for
//! editing reusable project-level [`SharedAvoidRegion`]s directly from the Project Tree.

use chip_cam::{DEFAULT_AVOID_CLEARANCE_MM, SharedAvoidRegion};
use chip_math::{EvalContext, ParamLength};
use chip_project::{ProjectDocument, UndoHistory};

use crate::{
    panels::{
        InspectorFocus, ViewportPickMode,
        operation_inspector::avoid_card::{
            mark_shared_avoid_dependents_dirty, set_shared_avoid_for_all_setup_ops,
            show_sketch_avoid_quick_add, toggle_shared_avoid_link,
        },
    },
    theme::ThemeTokens,
    widgets::{
        BannerKind, INPUT_COL_WIDTH, IconKind, edit_param_length, grid_row, inline_banner,
        inspector_grid, section_card, small_icon_button, static_card,
    },
};

/// Context passed into [`show_keepout_inspector`].
pub struct KeepOutInspectorCtx<'a> {
    pub project: &'a mut ProjectDocument,
    pub region_idx: usize,
    pub active_setup_idx: usize,
    pub history: &'a mut UndoHistory,
    pub eval_ctx: &'a EvalContext,
    pub pick_mode: &'a mut ViewportPickMode,
    pub editing_shared_avoid_id: &'a mut Option<String>,
    pub focus: &'a mut InspectorFocus,
}

/// High-level action emitted by [`show_keepout_inspector`].
#[derive(Debug, Clone)]
pub enum KeepOutInspectorAction {
    InvalidateFrameCache,
    Log(String),
}

struct SetupFixturesSnapshot {
    setup_label: String,
    fixtures: Vec<(String, String, bool)>,
}

struct SetupOpsSnapshot {
    setup_idx: usize,
    setup_name: String,
    ops: Vec<(usize, String, bool)>,
}

enum DeferredKeepOutAction {
    ToggleOpLink { setup_idx: usize, op_idx: usize },
    SetAllSetupOps { setup_idx: usize, link: bool },
    DeleteRegion,
}

/// Renders the dedicated right-panel **Keep-Out Region Inspector**.
pub fn show_keepout_inspector(
    ui: &mut egui::Ui,
    ctx: KeepOutInspectorCtx<'_>,
) -> Option<KeepOutInspectorAction> {
    let KeepOutInspectorCtx {
        project,
        region_idx,
        active_setup_idx,
        history,
        eval_ctx,
        pick_mode,
        editing_shared_avoid_id,
        focus,
    } = ctx;

    if region_idx >= project.avoid_regions.len() {
        ui.weak("No Keep-Out Region selected.");
        return None;
    }

    let tok = ThemeTokens::from_ui(ui);
    let region_id = project.avoid_regions[region_idx].id.clone();
    let usage_count = project.avoid_region_usage_count(&region_id);
    *editing_shared_avoid_id = Some(region_id.clone());

    // Capture lightweight snapshot of only this region (never cloning the whole document on idle frames!)
    let pre_region: SharedAvoidRegion = project.avoid_regions[region_idx].clone();
    let mut mutated_label: Option<&'static str> = None;
    let mut deferred: Option<DeferredKeepOutAction> = None;
    let mut emitted: Option<KeepOutInspectorAction> = None;

    // Collect read-only snapshots of Setup fixtures & operations to avoid conflicting borrows
    let setup_fixtures: Vec<SetupFixturesSnapshot> = project
        .setups
        .iter()
        .enumerate()
        .filter(|(_, s)| !s.fixtures.is_empty())
        .map(|(si, s)| {
            let setup_label = if si == active_setup_idx {
                format!("{} (Active)", s.name)
            } else {
                s.name.clone()
            };
            let fixtures = s
                .fixtures
                .iter()
                .map(|f| (f.id.clone(), f.name.clone(), f.enabled))
                .collect();
            SetupFixturesSnapshot {
                setup_label,
                fixtures,
            }
        })
        .collect();
    let setups_ops: Vec<SetupOpsSnapshot> = project
        .setups
        .iter()
        .enumerate()
        .map(|(si, s)| {
            let ops = s
                .operations
                .iter()
                .enumerate()
                .map(|(oi, op)| {
                    let linked = op
                        .avoid
                        .shared_region_ids
                        .iter()
                        .any(|rid| rid == &region_id);
                    (oi, op.name.clone(), linked)
                })
                .collect();
            SetupOpsSnapshot {
                setup_idx: si,
                setup_name: s.name.clone(),
                ops,
            }
        })
        .collect();

    static_card(ui, None, |ui| {
        ui.horizontal(|ui| {
            ui.strong("Keep-Out Region");
            let usage_lbl = if usage_count == 1 {
                "1 linked op".to_string()
            } else {
                format!("{usage_count} linked ops")
            };
            ui.colored_label(tok.text_muted, usage_lbl);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui
                    .small_button("Delete")
                    .on_hover_text(
                        "Delete this shared keep-out region and unlink all referencing operations",
                    )
                    .clicked()
                {
                    deferred = Some(DeferredKeepOutAction::DeleteRegion);
                }
            });
        });
    });

    egui::ScrollArea::vertical()
        .id_salt(("keepout_inspector_scroll", region_idx))
        .auto_shrink([false, false])
        .show(ui, |ui| {
            let is_picking_this = matches!(
                *pick_mode,
                ViewportPickMode::PickKeepOutGeometry { region_idx: r } if r == region_idx
            );
            if is_picking_this && let Some(msg) = pick_mode.banner_text() {
                inline_banner(ui, BannerKind::Info, msg);
                ui.add_space(5.0);
            }

            // 1. General & Clearance Overrides
            section_card(
                ui,
                ("keepout_general_card", region_idx),
                "General & Clearances",
                true,
                |ui| {
                    let sr = &mut project.avoid_regions[region_idx];
                    inspector_grid(ui, ("keepout_general_grid", region_idx), |ui| {
                        grid_row(
                            ui,
                            "Name",
                            Some("Display name for this reusable shared keep-out region"),
                            |ui| {
                                let resp = ui.add(
                                    egui::TextEdit::singleline(&mut sr.name)
                                        .desired_width(INPUT_COL_WIDTH + 48.0),
                                );
                                let enter = (resp.has_focus() || resp.lost_focus())
                                    && ui.input(|i| i.key_pressed(egui::Key::Enter));
                                if (resp.lost_focus() || enter) && sr.name != pre_region.name {
                                    let trimmed = sr.name.trim().to_string();
                                    if trimmed.is_empty() {
                                        sr.name = pre_region.name.clone();
                                    } else {
                                        sr.name = trimmed;
                                        mutated_label = Some("Rename Shared Keep-Out Region");
                                    }
                                }
                            },
                        );

                        grid_row(
                            ui,
                            "Suppressed",
                            Some(
                                "Temporarily disable this keep-out region across all linked operations without unlinking it",
                            ),
                            |ui| {
                                if ui
                                    .checkbox(&mut sr.suppressed, "Suppress region")
                                    .on_hover_text(
                                        "Check to suppress this keep-out region from toolpath avoidance",
                                    )
                                    .changed()
                                {
                                    mutated_label = Some("Toggle Shared Keep-Out Suppress");
                                }
                            },
                        );

                        grid_row(
                            ui,
                            "Radial Clearance",
                            Some(
                                "Optional radial clearance override (unchecked inherits each operation's Radial Clearance)",
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
                                        mutated_label =
                                            Some("Toggle Shared Radial Clearance Override");
                                    }
                                    if let Some(rad) = &mut sr.radial_clearance {
                                        if edit_param_length(
                                            ui,
                                            ("keepout_rad_clr", region_idx),
                                            rad,
                                            eval_ctx,
                                            INPUT_COL_WIDTH - 16.0,
                                        ) {
                                            mutated_label =
                                                Some("Edit Shared Keep-Out Radial Clearance");
                                        }
                                    } else {
                                        ui.weak("Inherit from op");
                                    }
                                });
                            },
                        );

                        grid_row(
                            ui,
                            "Axial Clearance",
                            Some(
                                "Optional axial clearance override (unchecked inherits each operation's Axial Clearance)",
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
                                        mutated_label =
                                            Some("Toggle Shared Axial Clearance Override");
                                    }
                                    if let Some(ax) = &mut sr.axial_clearance {
                                        if edit_param_length(
                                            ui,
                                            ("keepout_ax_clr", region_idx),
                                            ax,
                                            eval_ctx,
                                            INPUT_COL_WIDTH - 16.0,
                                        ) {
                                            mutated_label =
                                                Some("Edit Shared Keep-Out Axial Clearance");
                                        }
                                    } else {
                                        ui.weak("Inherit from op");
                                    }
                                });
                            },
                        );
                    });
                },
            );

            // 2. Keep-Out Geometry (3D Picking + 2D Sketch quick-add + selections list)
            let sel_count = project.avoid_regions[region_idx].selections.len();
            section_card(
                ui,
                ("keepout_geom_card", region_idx),
                &format!("Keep-Out Geometry ({sel_count})"),
                true,
                |ui| {
                    ui.horizontal_wrapped(|ui| {
                        let btn_lbl = if is_picking_this {
                            "[x] Picking 3D Avoid Geometry..."
                        } else {
                            "Pick 3D Avoid Geometry..."
                        };
                        if ui
                            .selectable_label(
                                is_picking_this,
                                egui::RichText::new(btn_lbl).color(if is_picking_this {
                                    tok.error_fg
                                } else {
                                    tok.text_primary
                                }),
                            )
                            .on_hover_text(
                                "Click planar faces, closed loops, holes, or 2D sketches in the 3D viewport to add/remove keep-out geometry",
                            )
                            .clicked()
                        {
                            *pick_mode = if is_picking_this {
                                ViewportPickMode::InspectEntity
                            } else {
                                ViewportPickMode::PickKeepOutGeometry { region_idx }
                            };
                        }
                        if sel_count > 0
                            && ui
                                .small_button("Clear")
                                .on_hover_text(
                                    "Remove all 3D and 2D sketch selections from this keep-out region",
                                )
                                .clicked()
                        {
                            project.avoid_regions[region_idx].selections.clear();
                            mutated_label = Some("Clear Shared Keep-Out Selections");
                        }
                    });

                    if !project.sketches.is_empty() {
                        ui.add_space(4.0);
                        let sketches = &project.sketches;
                        let sr = &mut project.avoid_regions[region_idx];
                        show_sketch_avoid_quick_add(ui, sketches, &mut sr.selections, &mut || {
                            mutated_label = Some("Update Shared Keep-Out Sketch");
                        });
                    }

                    let sr = &mut project.avoid_regions[region_idx];
                    if sr.selections.is_empty() {
                        ui.add_space(4.0);
                        inline_banner(
                            ui,
                            BannerKind::Info,
                            "No 3D or 2D sketch keep-out selections yet. Click 'Pick 3D Avoid Geometry...' or a 2D Sketch '+ Avoid' chip above.",
                        );
                    } else {
                        ui.add_space(4.0);
                        let mut remove_idx: Option<usize> = None;
                        for (i, item) in sr.selections.iter().enumerate() {
                            ui.horizontal(|ui| {
                                let w = (ui.available_width() - 28.0).max(56.0);
                                ui.add_sized(
                                    [w, 18.0],
                                    egui::Label::new(format!("- {}", item.short_label()))
                                        .truncate(),
                                )
                                .on_hover_text(item.short_label());
                                ui.with_layout(
                                    egui::Layout::right_to_left(egui::Align::Center),
                                    |ui| {
                                        if small_icon_button(
                                            ui,
                                            IconKind::Trash,
                                            "Remove selection from keep-out region",
                                        )
                                        .clicked()
                                        {
                                            remove_idx = Some(i);
                                        }
                                    },
                                );
                            });
                        }
                        if let Some(ri) = remove_idx {
                            sr.selections.remove(ri);
                            mutated_label = Some("Remove Shared Keep-Out Selection");
                        }
                    }
                },
            );

            // 3. Setup Fixtures chips
            let fix_sel_count = project.avoid_regions[region_idx].fixture_ids.len();
            section_card(
                ui,
                ("keepout_fixtures_card", region_idx),
                &format!("Setup Fixtures ({fix_sel_count})"),
                true,
                |ui| {
                    if setup_fixtures.is_empty() {
                        ui.weak(
                            "No workholding fixtures configured on any Setup yet. Add fixtures in a Setup's Fixtures tab.",
                        );
                    } else {
                        let sr = &mut project.avoid_regions[region_idx];
                        for sf in &setup_fixtures {
                            if setup_fixtures.len() > 1 {
                                ui.colored_label(tok.text_secondary, &sf.setup_label);
                            }
                            ui.horizontal_wrapped(|ui| {
                                for (fid, fname, enabled) in &sf.fixtures {
                                    let linked = sr.fixture_ids.contains(fid);
                                    let lbl = if linked {
                                        format!("[x] {fname}")
                                    } else {
                                        format!("[ ] {fname}")
                                    };
                                    let rich = if !*enabled {
                                        egui::RichText::new(lbl).weak()
                                    } else if linked {
                                        egui::RichText::new(lbl).color(tok.error_fg)
                                    } else {
                                        egui::RichText::new(lbl)
                                    };
                                    if ui
                                        .selectable_label(linked, rich)
                                        .on_hover_text(format!(
                                            "Include or exclude fixture '{fname}' in this keep-out region"
                                        ))
                                        .clicked()
                                    {
                                        if linked {
                                            sr.fixture_ids.retain(|x| x != fid);
                                        } else {
                                            sr.fixture_ids.push(fid.clone());
                                        }
                                        mutated_label = Some("Toggle Shared Keep-Out Fixture");
                                    }
                                }
                            });
                        }
                    }
                },
            );

            // 4. Linked Operations across all Setups
            section_card(
                ui,
                ("keepout_linked_ops_card", region_idx),
                &format!("Linked Operations ({usage_count})"),
                true,
                |ui| {
                    let total_ops: usize = setups_ops.iter().map(|s| s.ops.len()).sum();
                    if total_ops == 0 {
                        ui.weak(
                            "No operations in this project yet. Add an operation to a Setup to link this keep-out region.",
                        );
                    } else {
                        for so in &setups_ops {
                            if so.ops.is_empty() {
                                continue;
                            }
                            ui.horizontal(|ui| {
                                ui.strong(&so.setup_name);
                                ui.with_layout(
                                    egui::Layout::right_to_left(egui::Align::Center),
                                    |ui| {
                                        if ui
                                            .small_button("Unlink All")
                                            .on_hover_text(
                                                "Unlink this keep-out region from every operation in this Setup",
                                            )
                                            .clicked()
                                        {
                                            deferred =
                                                Some(DeferredKeepOutAction::SetAllSetupOps {
                                                    setup_idx: so.setup_idx,
                                                    link: false,
                                                });
                                        }
                                        if ui
                                            .small_button("Link All")
                                            .on_hover_text(
                                                "Link this keep-out region to every operation in this Setup",
                                            )
                                            .clicked()
                                        {
                                            deferred =
                                                Some(DeferredKeepOutAction::SetAllSetupOps {
                                                    setup_idx: so.setup_idx,
                                                    link: true,
                                                });
                                        }
                                    },
                                );
                            });
                            for (oi, op_name, linked) in &so.ops {
                                let mut is_linked = *linked;
                                if ui
                                    .checkbox(&mut is_linked, format!("{}. {op_name}", oi + 1))
                                    .on_hover_text(format!(
                                        "Link or unlink keep-out region on '{op_name}'"
                                    ))
                                    .changed()
                                {
                                    deferred = Some(DeferredKeepOutAction::ToggleOpLink {
                                        setup_idx: so.setup_idx,
                                        op_idx: *oi,
                                    });
                                }
                            }
                            ui.add_space(4.0);
                        }
                    }
                },
            );
        });

    // Commit atomic Undo + dependent dirty propagation for direct region edits
    if let Some(undo_lbl) = mutated_label {
        let updated = project.avoid_regions[region_idx].clone();
        project.avoid_regions[region_idx] = pre_region;
        history.record_before_edit(undo_lbl, None, project);
        project.avoid_regions[region_idx] = updated;
        mark_shared_avoid_dependents_dirty(project, &region_id);
        emitted = Some(KeepOutInspectorAction::InvalidateFrameCache);
    }

    // Apply deferred cross-operation mutations cleanly
    if let Some(act) = deferred {
        match act {
            DeferredKeepOutAction::ToggleOpLink { setup_idx, op_idx } => {
                if toggle_shared_avoid_link(project, history, setup_idx, op_idx, &region_id) {
                    emitted = Some(KeepOutInspectorAction::InvalidateFrameCache);
                }
            }
            DeferredKeepOutAction::SetAllSetupOps { setup_idx, link } => {
                if set_shared_avoid_for_all_setup_ops(project, history, setup_idx, &region_id, link)
                {
                    emitted = Some(KeepOutInspectorAction::InvalidateFrameCache);
                }
            }
            DeferredKeepOutAction::DeleteRegion => {
                history.record_before_edit("Delete Shared Keep-Out Region", None, project);
                mark_shared_avoid_dependents_dirty(project, &region_id);
                let deleted_name = project.avoid_regions[region_idx].name.clone();
                if project.remove_avoid_region(&region_id) {
                    *editing_shared_avoid_id = None;
                    if matches!(*pick_mode, ViewportPickMode::PickKeepOutGeometry { .. }) {
                        *pick_mode = ViewportPickMode::InspectEntity;
                    }
                    *focus = if !project.avoid_regions.is_empty() {
                        InspectorFocus::KeepOutRegion {
                            region_idx: region_idx.min(project.avoid_regions.len() - 1),
                        }
                    } else {
                        InspectorFocus::Setup {
                            setup_idx: active_setup_idx.min(project.setups.len().saturating_sub(1)),
                        }
                    };
                    emitted = Some(KeepOutInspectorAction::Log(format!(
                        "Deleted shared keep-out region '{deleted_name}' and unlinked referencing operations."
                    )));
                }
            }
        }
    }

    emitted
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cam::{GeometrySelectionItem, Operation, OperationKind, SharedAvoidRegion};
    use chip_math::{DisplayUnit, ParamLength};
    use chip_project::ProjectDocument;
    use chip_sketch::{SketchEntity, SketchEntityEntry, SketchLayer};

    use crate::viewport::{BoundaryPreviewCache, BoundaryPreviewOverlay, SharedAvoidPreviewInput};

    #[test]
    fn shared_avoid_region_produces_live_3d_keepout_overlay() {
        let mut project = ProjectDocument::new("Keep-Out Overlay Test");
        let mut sk = SketchLayer::new("sk_1", "Clamp Footprint", "setup_1");
        sk.entities.push(SketchEntityEntry {
            id: "ent_1".to_string(),
            name: "Clamp Circle".to_string(),
            sub_layer: None,
            is_construction: false,
            entity: SketchEntity::new_circle(
                ParamLength::from_mm(0.0),
                ParamLength::from_mm(0.0),
                ParamLength::from_mm(16.0),
            ),
        });
        sk.rebuild_regions(None);
        project.sketches.push(sk);

        let mut sr = SharedAvoidRegion::new("avoid_1", "Clamp Zone");
        sr.selections.push(GeometrySelectionItem::SketchLayerAll {
            sketch_id: "sk_1".to_string(),
            sub_layer: None,
            reverse: false,
            side_flip: false,
        });
        project.avoid_regions.push(sr);
        project.setups[0].operations.push(Operation::new(
            "op_1",
            "Pocket",
            OperationKind::Pocket2D,
        ));

        let empty_doc = chip_cad::CadDocument::empty();
        let rs = project.setups[0]
            .resolve_with_sketches(&empty_doc, &project.sketches, DisplayUnit::Millimeter)
            .expect("setup resolves");
        let mut cache = BoundaryPreviewCache::default();
        let overlay = cache
            .get_or_compute_shared_avoid(&SharedAvoidPreviewInput {
                region_idx: 0,
                cad_rev: 0,
                frame_geo_sig: 1,
                display_unit: DisplayUnit::Millimeter,
                doc: Some(&empty_doc),
                sketches: &project.sketches,
                resolved_setup: &rs,
                resolved_fixtures: &[],
                shared_region: &project.avoid_regions[0],
            })
            .expect("selected Keep-Out Region must produce live coral/red 3D preview overlay");

        match overlay.as_ref() {
            BoundaryPreviewOverlay::Planar2D {
                avoid_nominal_region,
                avoid_effective_region,
                ..
            } => {
                assert!(
                    !avoid_nominal_region.is_empty(),
                    "nominal keep-out footprint must be populated"
                );
                assert!(
                    !avoid_effective_region.is_empty(),
                    "effective clearance-inflated keep-out footprint must be populated"
                );
            }
            other => panic!("expected Planar2D keep-out overlay, got {other:?}"),
        }
    }
}
