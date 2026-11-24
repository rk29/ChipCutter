use chip_cam::{
    FixtureBody, FixtureKind, FixtureZRef, ResolvedFixtureSolid, ResolvedSetup, Setup,
    ViseClampAxis, ViseFixedSide,
};
use chip_math::{DVec2, DVec3, DisplayUnit, EvalContext, ParamLength};
use chip_project::ModelEntry;

use crate::{
    panels::{FixturePlaceKind, ViewportPickMode, setup_inspector::FixtureSafetySummary},
    widgets::{
        BannerKind, INPUT_COL_WIDTH, edit_param_length, grid_row, inline_banner, inspector_grid,
        section_card, static_card,
    },
};

pub(super) struct FixturesTabCtx<'a> {
    pub(super) setup: &'a mut Setup,
    pub(super) setup_idx: usize,
    pub(super) fixture_models: &'a [ModelEntry],
    pub(super) eval_ctx: &'a EvalContext,
    pub(super) display_unit: DisplayUnit,
    pub(super) resolved: Option<Result<&'a ResolvedSetup, &'a str>>,
    pub(super) resolved_fixtures: &'a [ResolvedFixtureSolid],
    pub(super) fixture_safety: FixtureSafetySummary,
    pub(super) selected_fixture_idx: &'a mut Option<usize>,
    pub(super) show_fixtures: &'a mut bool,
    pub(super) pick_mode: &'a mut ViewportPickMode,
    pub(super) import_fixture_requested: &'a mut bool,
    pub(super) open_flip_jig_wizard: &'a mut bool,
    pub(super) open_soft_jaw_wizard: &'a mut bool,
    pub(super) mutated_label: &'a mut Option<&'static str>,
}

fn next_fixture_id(setup: &Setup, prefix: &str) -> String {
    let mut n = setup.fixtures.len() + 1;
    loop {
        let cand = format!("{prefix}_{n}");
        if !setup.fixtures.iter().any(|f| f.id == cand) {
            return cand;
        }
        n += 1;
    }
}

pub(super) fn show_fixtures_tab(ui: &mut egui::Ui, ctx: FixturesTabCtx<'_>) {
    let FixturesTabCtx {
        setup,
        setup_idx,
        fixture_models,
        eval_ctx,
        display_unit,
        resolved,
        resolved_fixtures,
        fixture_safety,
        selected_fixture_idx,
        show_fixtures,
        pick_mode,
        import_fixture_requested,
        open_flip_jig_wizard,
        open_soft_jaw_wizard,
        mutated_label,
    } = ctx;

    // Clamp selection index safely if fixtures were deleted or undone
    if let Some(sel) = *selected_fixture_idx
        && sel >= setup.fixtures.len()
    {
        *selected_fixture_idx = setup.fixtures.len().checked_sub(1);
    }
    if selected_fixture_idx.is_none() && !setup.fixtures.is_empty() {
        *selected_fixture_idx = Some(0);
    }

    let stock_aabb = resolved.and_then(Result::ok).map(|rs| rs.stock.aabb_wcs());

    // 1. Workholding Visibility, Rapid Safety, & Quick-Add Action Bar
    section_card(
        ui,
        ("setup_fixtures_controls", setup_idx),
        "Workholding & Rapid Safety",
        true,
        |ui| {
            inspector_grid(ui, ("setup_fixtures_hdr_grid", setup_idx), |ui| {
                grid_row(
                    ui,
                    "3D Fixtures",
                    Some(
                        "Show translucent 3D vises, clamps, spoilboards, and keep-outs in viewport",
                    ),
                    |ui| {
                        ui.checkbox(show_fixtures, "Show in viewport").on_hover_text(
                            "Toggle translucent 3D workholding hardware shading and silhouette edges in the viewport",
                        );
                    },
                );
                grid_row(
                    ui,
                    "Rapid Safety",
                    Some(
                        "Automatically route G0 rapid links safely over enabled fixtures + clearance margin",
                    ),
                    |ui| {
                        if ui
                            .checkbox(
                                &mut setup.auto_avoid_fixtures_on_rapids,
                                "Auto-avoid on rapids",
                            )
                            .on_hover_text(
                                "Lift rapid traverse moves above enabled fixtures plus clearance margin when linking between cuts",
                            )
                            .changed()
                        {
                            *mutated_label = Some("Toggle Fixture Rapid Avoidance");
                        }
                    },
                );
            });

            ui.add_space(5.0);
            ui.horizontal_wrapped(|ui| {
                if ui
                    .small_button("+ Vise")
                    .on_hover_text(
                        "Add a parametric two-jaw machinist vise clamping the stock with optional step shelves and parallels",
                    )
                    .clicked()
                {
                    let id = next_fixture_id(setup, "vise");
                    let n = setup
                        .fixtures
                        .iter()
                        .filter(|f| matches!(f.kind, FixtureKind::ViseJaws { .. }))
                        .count()
                        + 1;
                    setup
                        .fixtures
                        .push(FixtureBody::default_vise(id, format!("Machinist Vise {n}")));
                    *selected_fixture_idx = Some(setup.fixtures.len() - 1);
                    *show_fixtures = true;
                    *mutated_label = Some("Add Vise Fixture");
                }

                if ui
                    .small_button("+ Toe Clamp")
                    .on_hover_text(
                        "Add a parametric toe / strap clamp with heel riser and T-slot stud tower on the stock edge",
                    )
                    .clicked()
                {
                    let id = next_fixture_id(setup, "clamp");
                    let n = setup
                        .fixtures
                        .iter()
                        .filter(|f| matches!(f.kind, FixtureKind::ToeClamp { .. }))
                        .count()
                        + 1;
                    let (anchor, yaw) = match stock_aabb {
                        Some(b) if b.is_valid() => {
                            let c = b.center();
                            match (n - 1) % 4 {
                                0 => (DVec2::new(b.max.x, c.y), 0.0),
                                1 => (DVec2::new(b.min.x, c.y), 180.0),
                                2 => (DVec2::new(c.x, b.max.y), 90.0),
                                _ => (DVec2::new(c.x, b.min.y), 270.0),
                            }
                        }
                        _ => (DVec2::new(40.0, 0.0), 0.0),
                    };
                    setup.fixtures.push(FixtureBody::default_toe_clamp(
                        id,
                        format!("Toe Clamp {n}"),
                        anchor,
                        yaw,
                    ));
                    *selected_fixture_idx = Some(setup.fixtures.len() - 1);
                    *show_fixtures = true;
                    *mutated_label = Some("Add Toe Clamp Fixture");
                }

                if ui
                    .small_button("+ Spoilboard")
                    .on_hover_text(
                        "Add a sacrificial MDF or aluminum tooling-plate spoilboard beneath the stock bottom",
                    )
                    .clicked()
                {
                    let id = next_fixture_id(setup, "spoilboard");
                    setup
                        .fixtures
                        .push(FixtureBody::default_spoilboard(id, "MDF Spoilboard"));
                    *selected_fixture_idx = Some(setup.fixtures.len() - 1);
                    *show_fixtures = true;
                    *mutated_label = Some("Add Spoilboard Fixture");
                }

                if ui
                    .small_button("+ Keep-Out")
                    .on_hover_text("Add a rectangular 3D box keep-out obstacle zone in Setup WCS")
                    .clicked()
                {
                    let id = next_fixture_id(setup, "keepout");
                    let n = setup
                        .fixtures
                        .iter()
                        .filter(|f| {
                            matches!(
                                f.kind,
                                FixtureKind::BoxKeepOut { .. }
                                    | FixtureKind::CylinderKeepOut { .. }
                            )
                        })
                        .count()
                        + 1;
                    let center = match stock_aabb {
                        Some(b) if b.is_valid() => {
                            DVec3::new(b.max.x + 20.0, b.center().y, b.center().z)
                        }
                        _ => DVec3::new(50.0, 0.0, 0.0),
                    };
                    setup.fixtures.push(FixtureBody::default_box_keepout(
                        id,
                        format!("Keep-Out Box {n}"),
                        center,
                    ));
                    *selected_fixture_idx = Some(setup.fixtures.len() - 1);
                    *show_fixtures = true;
                    *mutated_label = Some("Add Box Keep-Out");
                }

                if ui
                    .small_button("+ Cylinder")
                    .on_hover_text(
                        "Add a vertical cylindrical keep-out pillar (dowel pin, bolt head, or cam clamp)",
                    )
                    .clicked()
                {
                    let id = next_fixture_id(setup, "pin");
                    let n = setup
                        .fixtures
                        .iter()
                        .filter(|f| matches!(f.kind, FixtureKind::CylinderKeepOut { .. }))
                        .count()
                        + 1;
                    let (xy, zb, zt) = match stock_aabb {
                        Some(b) if b.is_valid() => (
                            DVec2::new(b.min.x - 12.0, b.max.y + 12.0),
                            b.min.z,
                            b.max.z + 10.0,
                        ),
                        _ => (DVec2::new(-30.0, 30.0), -15.0, 10.0),
                    };
                    setup.fixtures.push(FixtureBody::default_cylinder_keepout(
                        id,
                        format!("Keep-Out Pin {n}"),
                        xy,
                        zb,
                        zt,
                    ));
                    *selected_fixture_idx = Some(setup.fixtures.len() - 1);
                    *show_fixtures = true;
                    *mutated_label = Some("Add Cylinder Keep-Out");
                }

                if ui
                    .small_button("+ Import Fixture…")
                    .on_hover_text(
                        "Import an external 3D STEP, STL, OBJ, or 3MF vise/clamp model as a non-machined fixture solid",
                    )
                    .clicked()
                {
                    *import_fixture_requested = true;
                }
            });

            ui.add_space(4.0);
            ui.horizontal_wrapped(|ui| {
                let placing_clamp = matches!(
                    *pick_mode,
                    ViewportPickMode::PlaceFixtureClamp {
                        setup_idx: s,
                        kind: FixturePlaceKind::ToeClamp
                    } if s == setup_idx
                );
                let placing_cyl = matches!(
                    *pick_mode,
                    ViewportPickMode::PlaceFixtureClamp {
                        setup_idx: s,
                        kind: FixturePlaceKind::CylinderKeepOut
                    } if s == setup_idx
                );
                if ui
                    .selectable_label(placing_clamp, "Place Clamp in Viewport…")
                    .on_hover_text(
                        "Click directly on a stock edge, corner, or spoilboard in the 3D viewport to drop & auto-orient a Toe Clamp",
                    )
                    .clicked()
                {
                    *show_fixtures = true;
                    *pick_mode = if placing_clamp {
                        ViewportPickMode::InspectEntity
                    } else {
                        ViewportPickMode::PlaceFixtureClamp {
                            setup_idx,
                            kind: FixturePlaceKind::ToeClamp,
                        }
                    };
                }
                if ui
                    .selectable_label(placing_cyl, "Place Pin in Viewport…")
                    .on_hover_text(
                        "Click directly in the 3D viewport to drop a vertical Cylinder Keep-Out pin or bolt head",
                    )
                    .clicked()
                {
                    *show_fixtures = true;
                    *pick_mode = if placing_cyl {
                        ViewportPickMode::InspectEntity
                    } else {
                        ViewportPickMode::PlaceFixtureClamp {
                            setup_idx,
                            kind: FixturePlaceKind::CylinderKeepOut,
                        }
                    };
                }
                if ui
                    .small_button("Flip-Jig Wizard…")
                    .on_hover_text(
                        "Launch the Two-Sided Flip-Jig Dowel-Pin Wizard to generate symmetric registration pins, Op-1 pin holes, and a paired 180° Op-2 Setup",
                    )
                    .clicked()
                {
                    *open_flip_jig_wizard = true;
                }
                if ui
                    .small_button("Auto Soft-Jaw Wizard…")
                    .on_hover_text(
                        "Launch the Auto Soft-Jaw Generator Wizard to create contoured CNC soft-jaw machining ops and/or 3D-printable .stl jaws",
                    )
                    .clicked()
                {
                    *open_soft_jaw_wizard = true;
                }
            });
        },
    );

    ui.add_space(5.0);

    if setup.fixtures.is_empty() {
        inline_banner(
            ui,
            BannerKind::Info,
            "No workholding fixtures configured on this Setup yet. Use + Vise, + Toe Clamp, + Spoilboard, + Keep-Out, or Place Clamp in Viewport… above.",
        );
        return;
    }

    let enabled_count = setup.fixtures.iter().filter(|f| f.enabled).count();
    if fixture_safety.colliding_fixture_count > 0 {
        inline_banner(
            ui,
            BannerKind::Error,
            format!(
                "Fixture collision detected on {} fixture body(s) — colliding hardware highlighted crimson in viewport. Inspect Diagnostics & Log to jump to the collision.",
                fixture_safety.colliding_fixture_count
            ),
        );
        ui.add_space(5.0);
    } else if setup.auto_avoid_fixtures_on_rapids && fixture_safety.rapid_lifts_inserted > 0 {
        let sign = if fixture_safety.max_lifted_z_wcs >= 0.0 {
            "+"
        } else {
            ""
        };
        inline_banner(
            ui,
            BannerKind::Info,
            format!(
                "Auto-Avoid Rapids enabled — {} rapid jump(s) lifted cleanly over fixtures (max Z = {}{})",
                fixture_safety.rapid_lifts_inserted,
                sign,
                display_unit.format_length(fixture_safety.max_lifted_z_wcs),
            ),
        );
        ui.add_space(5.0);
    } else if setup.auto_avoid_fixtures_on_rapids && enabled_count > 0 {
        inline_banner(
            ui,
            BannerKind::Info,
            "Auto-Avoid Rapids enabled — rapid links clear enabled fixtures automatically on toolpath regeneration (Ctrl+G).",
        );
        ui.add_space(5.0);
    } else if !setup.auto_avoid_fixtures_on_rapids && enabled_count > 0 {
        inline_banner(
            ui,
            BannerKind::Warn,
            "Auto-Avoid Rapids disabled — G0 rapid links will NOT lift automatically over fixtures; run Simulation to check for collisions.",
        );
        ui.add_space(5.0);
    }

    // 2. Compact Fixture Bodies List
    let mut dup_idx: Option<usize> = None;
    let mut del_idx: Option<usize> = None;
    section_card(
        ui,
        ("setup_fixtures_list", setup_idx),
        "Fixture Bodies",
        true,
        |ui| {
            for (f_idx, fix) in setup.fixtures.iter_mut().enumerate() {
                ui.horizontal(|ui| {
                    if ui
                        .checkbox(&mut fix.enabled, "")
                        .on_hover_text(
                            "Enable or disable this fixture in viewport and rapid safety",
                        )
                        .changed()
                    {
                        *mutated_label = Some("Toggle Fixture Enabled");
                    }
                    let is_sel = *selected_fixture_idx == Some(f_idx);
                    let row_lbl = format!("{} [{}]", fix.name, fix.kind.short_badge());
                    if ui
                        .selectable_label(is_sel, row_lbl)
                        .on_hover_text("Click to edit this fixture's geometry and safety margins")
                        .clicked()
                    {
                        *selected_fixture_idx = Some(f_idx);
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui
                            .small_button("×")
                            .on_hover_text("Delete this workholding fixture")
                            .clicked()
                        {
                            del_idx = Some(f_idx);
                        }
                        if ui
                            .small_button("Dup")
                            .on_hover_text("Duplicate this workholding fixture")
                            .clicked()
                        {
                            dup_idx = Some(f_idx);
                        }
                    });
                });
            }
        },
    );

    if let Some(idx) = del_idx
        && idx < setup.fixtures.len()
    {
        setup.fixtures.remove(idx);
        *selected_fixture_idx = if setup.fixtures.is_empty() {
            None
        } else {
            Some(idx.min(setup.fixtures.len() - 1))
        };
        *mutated_label = Some("Delete Workholding Fixture");
        return;
    }
    if let Some(idx) = dup_idx
        && let Some(orig) = setup.fixtures.get(idx).cloned()
    {
        let mut copy = orig;
        copy.id = next_fixture_id(setup, "fixture");
        copy.name = format!("{} Copy", copy.name);
        if let FixtureKind::ToeClamp { anchor_xy, .. } = &mut copy.kind {
            anchor_xy[1] = ParamLength::from_mm(anchor_xy[1].value_mm + 20.0);
        } else if let FixtureKind::BoxKeepOut { center_wcs, .. } = &mut copy.kind {
            center_wcs[1] = ParamLength::from_mm(center_wcs[1].value_mm + 20.0);
        } else if let FixtureKind::CylinderKeepOut { center_xy_wcs, .. } = &mut copy.kind {
            center_xy_wcs[1] = ParamLength::from_mm(center_xy_wcs[1].value_mm + 20.0);
        }
        setup.fixtures.push(copy);
        *selected_fixture_idx = Some(setup.fixtures.len() - 1);
        *mutated_label = Some("Duplicate Workholding Fixture");
    }

    let Some(sel_idx) = *selected_fixture_idx else {
        return;
    };
    let Some(fix) = setup.fixtures.get_mut(sel_idx) else {
        return;
    };

    ui.add_space(5.0);

    // 3. Selected Fixture General Properties
    section_card(
        ui,
        ("setup_fixture_general", setup_idx, sel_idx),
        "Selected Fixture Settings",
        true,
        |ui| {
            inspector_grid(ui, ("setup_fix_gen_grid", setup_idx, sel_idx), |ui| {
                grid_row(
                    ui,
                    "Name",
                    Some("Human-readable workholding fixture name"),
                    |ui| {
                        let resp = ui.add(
                            egui::TextEdit::singleline(&mut fix.name)
                                .desired_width(INPUT_COL_WIDTH + 40.0),
                        );
                        if resp.lost_focus() {
                            *mutated_label = Some("Rename Workholding Fixture");
                        }
                    },
                );
                grid_row(
                    ui,
                    "Type",
                    Some("Workholding fixture primitive or imported solid category"),
                    |ui| {
                        ui.monospace(fix.kind.label());
                    },
                );
                grid_row(
                    ui,
                    "Clearance Margin",
                    Some(
                        "Safety halo (mm/in) added around fixture for rapid avoidance & crash checks",
                    ),
                    |ui| {
                        if edit_param_length(
                            ui,
                            ("fix_margin", setup_idx, sel_idx),
                            &mut fix.clearance_margin,
                            eval_ctx,
                            INPUT_COL_WIDTH,
                        ) {
                            *mutated_label = Some("Edit Fixture Clearance Margin");
                        }
                    },
                );
                grid_row(
                    ui,
                    "Allowed Overcut",
                    Some(
                        "Permitted sacrificial cut depth (e.g. 0.5 mm on an MDF spoilboard; 0.0 for vises/clamps)",
                    ),
                    |ui| {
                        if edit_param_length(
                            ui,
                            ("fix_overcut", setup_idx, sel_idx),
                            &mut fix.allowed_overcut,
                            eval_ctx,
                            INPUT_COL_WIDTH,
                        ) {
                            *mutated_label = Some("Edit Fixture Allowed Overcut");
                        }
                    },
                );
                grid_row(
                    ui,
                    "Color",
                    Some(
                        "Optional RGBA viewport color override (uses fixture type default when unset)",
                    ),
                    |ui| {
                        ui.horizontal(|ui| {
                            let mut rgba = fix.effective_color_rgba();
                            if ui
                                .color_edit_button_rgba_unmultiplied(&mut rgba)
                                .on_hover_text(
                                    "Customize translucent 3D viewport RGBA color for this fixture",
                                )
                                .changed()
                            {
                                fix.color_rgba = Some(rgba);
                                *mutated_label = Some("Edit Fixture Color");
                            }
                            if fix.color_rgba.is_some() {
                                if ui
                                    .small_button("Reset")
                                    .on_hover_text(
                                        "Reset fixture color back to its default type color",
                                    )
                                    .clicked()
                                {
                                    fix.color_rgba = None;
                                    *mutated_label = Some("Reset Fixture Color");
                                }
                            } else {
                                ui.weak("Default");
                            }
                        });
                    },
                );
            });
        },
    );

    ui.add_space(5.0);

    // 4. Kind-Specific Parametric Geometry Card (strict 2-column inspector_grid)
    let kind_title = fix.kind.label();
    section_card(
        ui,
        ("setup_fixture_geom", setup_idx, sel_idx),
        kind_title,
        true,
        |ui| {
            inspector_grid(
                ui,
                ("setup_fix_geom_grid", setup_idx, sel_idx),
                |ui| match &mut fix.kind {
                    FixtureKind::ViseJaws {
                        clamp_axis,
                        jaw_width,
                        jaw_thickness,
                        jaw_height,
                        grip_depth,
                        step_shelf_depth,
                        step_shelf_height,
                        include_parallels,
                        parallel_thickness,
                        center_offset_along_jaw,
                        fixed_jaw_side,
                    } => {
                        grid_row(
                            ui,
                            "Clamp Axis",
                            Some("WCS axis along which the vise jaws close on the stock"),
                            |ui| {
                                let prev = *clamp_axis;
                                egui::ComboBox::from_id_salt(("vise_axis", setup_idx, sel_idx))
                                    .width(INPUT_COL_WIDTH + 54.0)
                                    .truncate()
                                    .selected_text(clamp_axis.label())
                                    .show_ui(ui, |ui| {
                                        for a in ViseClampAxis::ALL {
                                            ui.selectable_value(clamp_axis, a, a.label())
                                                .on_hover_text(format!(
                                                    "Clamp stock along {}",
                                                    a.label()
                                                ));
                                        }
                                    })
                                    .response
                                    .on_hover_text("Select vise jaw clamping axis (Y or X)");
                                if *clamp_axis != prev {
                                    *mutated_label = Some("Change Vise Clamp Axis");
                                }
                            },
                        );
                        grid_row(
                            ui,
                            "Fixed Jaw",
                            Some("Which side of the vise holds the stationary fixed jaw"),
                            |ui| {
                                let prev = *fixed_jaw_side;
                                let axis = *clamp_axis;
                                egui::ComboBox::from_id_salt(("vise_fixed", setup_idx, sel_idx))
                                    .width(INPUT_COL_WIDTH + 54.0)
                                    .truncate()
                                    .selected_text(fixed_jaw_side.label(axis))
                                    .show_ui(ui, |ui| {
                                        for s in ViseFixedSide::ALL {
                                            ui.selectable_value(fixed_jaw_side, s, s.label(axis))
                                                .on_hover_text(format!(
                                                    "Place fixed jaw on {}",
                                                    s.label(axis)
                                                ));
                                        }
                                    })
                                    .response
                                    .on_hover_text("Select stationary fixed vise jaw side");
                                if *fixed_jaw_side != prev {
                                    *mutated_label = Some("Change Vise Fixed Jaw Side");
                                }
                            },
                        );
                        len_row(
                            ui,
                            "Jaw Width",
                            "Width of vise jaw plates (expressions like stock_x + 20 mm supported)",
                            ("vj_w", setup_idx, sel_idx),
                            jaw_width,
                            eval_ctx,
                            "Edit Vise Jaw Width",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Jaw Thickness",
                            "Front-to-back thickness of each jaw plate",
                            ("vj_t", setup_idx, sel_idx),
                            jaw_thickness,
                            eval_ctx,
                            "Edit Vise Jaw Thickness",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Jaw Height",
                            "Total vertical height of vise jaw plates",
                            ("vj_h", setup_idx, sel_idx),
                            jaw_height,
                            eval_ctx,
                            "Edit Vise Jaw Height",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Grip Depth",
                            "Height of stock bottom held upward inside the jaws",
                            ("vj_grip", setup_idx, sel_idx),
                            grip_depth,
                            eval_ctx,
                            "Edit Vise Grip Depth",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Step Shelf Depth",
                            "Optional inward stepped soft-jaw shelf depth beneath stock (0 = flat jaw)",
                            ("vj_sd", setup_idx, sel_idx),
                            step_shelf_depth,
                            eval_ctx,
                            "Edit Vise Step Shelf Depth",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Step Shelf Height",
                            "Vertical thickness of stepped soft-jaw shelf below stock bottom",
                            ("vj_sh", setup_idx, sel_idx),
                            step_shelf_height,
                            eval_ctx,
                            "Edit Vise Step Shelf Height",
                            mutated_label,
                        );
                        grid_row(
                            ui,
                            "Parallels",
                            Some(
                                "Include a pair of machinist parallels supporting the stock bottom inside the jaws",
                            ),
                            |ui| {
                                if ui
                                    .checkbox(include_parallels, "Include under-stock parallels")
                                    .on_hover_text("Model parallel bars beneath the workpiece against the inner jaw faces")
                                    .changed()
                                {
                                    *mutated_label = Some("Toggle Vise Parallels");
                                }
                            },
                        );
                        if *include_parallels {
                            len_row(
                                ui,
                                "Parallel Thick.",
                                "Thickness of each machinist parallel bar",
                                ("vj_pt", setup_idx, sel_idx),
                                parallel_thickness,
                                eval_ctx,
                                "Edit Vise Parallel Thickness",
                                mutated_label,
                            );
                        }
                        len_row(
                            ui,
                            "Jaw Slide Offset",
                            "Shift vise jaws left/right along jaw length relative to stock center",
                            ("vj_off", setup_idx, sel_idx),
                            center_offset_along_jaw,
                            eval_ctx,
                            "Edit Vise Slide Offset",
                            mutated_label,
                        );
                    }
                    FixtureKind::ToeClamp {
                        anchor_xy,
                        z_reference,
                        z_offset,
                        yaw_deg,
                        strap_length,
                        strap_width,
                        strap_height,
                        nose_overhang,
                        stud_diameter,
                        stud_height_above_strap,
                    } => {
                        len_row(
                            ui,
                            "Anchor X",
                            "Clamp nose contact X coordinate in Setup WCS",
                            ("tc_ax", setup_idx, sel_idx),
                            &mut anchor_xy[0],
                            eval_ctx,
                            "Edit Clamp Anchor X",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Anchor Y",
                            "Clamp nose contact Y coordinate in Setup WCS",
                            ("tc_ay", setup_idx, sel_idx),
                            &mut anchor_xy[1],
                            eval_ctx,
                            "Edit Clamp Anchor Y",
                            mutated_label,
                        );
                        z_ref_row(
                            ui,
                            "Z Datum",
                            ("tc_zref", setup_idx, sel_idx),
                            z_reference,
                            "Change Clamp Z Datum",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Z Offset",
                            "Signed Z offset from clamp contact datum",
                            ("tc_zoff", setup_idx, sel_idx),
                            z_offset,
                            eval_ctx,
                            "Edit Clamp Z Offset",
                            mutated_label,
                        );
                        grid_row(
                            ui,
                            "Outward Yaw",
                            Some(
                                "Approach direction (deg) along which the clamp body extends outward away from the part",
                            ),
                            |ui| {
                                ui.horizontal(|ui| {
                                    if ui
                                        .add(
                                            egui::DragValue::new(yaw_deg)
                                                .speed(1.0)
                                                .suffix("°")
                                                .range(-360.0..=360.0),
                                        )
                                        .on_hover_text("Drag or type outward clamp direction in degrees (0°=+X, 90°=+Y, 180°=-X, 270°=-Y)")
                                        .changed()
                                    {
                                        *mutated_label = Some("Edit Clamp Yaw Angle");
                                    }
                                    if ui
                                        .small_button("+90°")
                                        .on_hover_text("Rotate clamp orientation by +90° around WCS Z")
                                        .clicked()
                                    {
                                        *yaw_deg = (*yaw_deg + 90.0) % 360.0;
                                        *mutated_label = Some("Rotate Clamp +90°");
                                    }
                                });
                            },
                        );
                        len_row(
                            ui,
                            "Strap Length",
                            "Total length of the clamping strap bar",
                            ("tc_sl", setup_idx, sel_idx),
                            strap_length,
                            eval_ctx,
                            "Edit Clamp Strap Length",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Strap Width",
                            "Width of the clamping strap bar",
                            ("tc_sw", setup_idx, sel_idx),
                            strap_width,
                            eval_ctx,
                            "Edit Clamp Strap Width",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Strap Height",
                            "Thickness of the clamping strap bar above contact Z",
                            ("tc_sh", setup_idx, sel_idx),
                            strap_height,
                            eval_ctx,
                            "Edit Clamp Strap Height",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Nose Overhang",
                            "Distance the clamp nose overlaps inward onto the stock edge",
                            ("tc_no", setup_idx, sel_idx),
                            nose_overhang,
                            eval_ctx,
                            "Edit Clamp Nose Overhang",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Stud Diameter",
                            "Diameter of the T-slot clamping bolt/stud (0 disables stud tower)",
                            ("tc_sd", setup_idx, sel_idx),
                            stud_diameter,
                            eval_ctx,
                            "Edit Clamp Stud Diameter",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Stud Height",
                            "Protrusion height of clamping stud + nut above strap top",
                            ("tc_sth", setup_idx, sel_idx),
                            stud_height_above_strap,
                            eval_ctx,
                            "Edit Clamp Stud Height",
                            mutated_label,
                        );
                    }
                    FixtureKind::Spoilboard {
                        z_top_ref,
                        z_offset,
                        thickness,
                        margin_xy,
                    } => {
                        z_ref_row(
                            ui,
                            "Top Z Datum",
                            ("sb_zref", setup_idx, sel_idx),
                            z_top_ref,
                            "Change Spoilboard Z Datum",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Z Offset",
                            "Signed Z offset applied to spoilboard top face",
                            ("sb_zoff", setup_idx, sel_idx),
                            z_offset,
                            eval_ctx,
                            "Edit Spoilboard Z Offset",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Thickness",
                            "Vertical slab thickness of MDF / tooling plate",
                            ("sb_thk", setup_idx, sel_idx),
                            thickness,
                            eval_ctx,
                            "Edit Spoilboard Thickness",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "XY Margin",
                            "Overhang distance extending outward past stock XY bounds",
                            ("sb_mxy", setup_idx, sel_idx),
                            margin_xy,
                            eval_ctx,
                            "Edit Spoilboard XY Margin",
                            mutated_label,
                        );
                    }
                    FixtureKind::BoxKeepOut {
                        center_wcs,
                        size_wcs,
                        yaw_deg,
                    } => {
                        len_row(
                            ui,
                            "Center X",
                            "Keep-out box center X in Setup WCS",
                            ("bk_cx", setup_idx, sel_idx),
                            &mut center_wcs[0],
                            eval_ctx,
                            "Edit Keep-Out Center X",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Center Y",
                            "Keep-out box center Y in Setup WCS",
                            ("bk_cy", setup_idx, sel_idx),
                            &mut center_wcs[1],
                            eval_ctx,
                            "Edit Keep-Out Center Y",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Center Z",
                            "Keep-out box center Z in Setup WCS",
                            ("bk_cz", setup_idx, sel_idx),
                            &mut center_wcs[2],
                            eval_ctx,
                            "Edit Keep-Out Center Z",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Size X",
                            "Keep-out box X width",
                            ("bk_sx", setup_idx, sel_idx),
                            &mut size_wcs[0],
                            eval_ctx,
                            "Edit Keep-Out Size X",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Size Y",
                            "Keep-out box Y depth",
                            ("bk_sy", setup_idx, sel_idx),
                            &mut size_wcs[1],
                            eval_ctx,
                            "Edit Keep-Out Size Y",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Size Z",
                            "Keep-out box Z height",
                            ("bk_sz", setup_idx, sel_idx),
                            &mut size_wcs[2],
                            eval_ctx,
                            "Edit Keep-Out Size Z",
                            mutated_label,
                        );
                        grid_row(
                            ui,
                            "Yaw Rotation",
                            Some("Rotation of keep-out box around Setup WCS Z axis (deg)"),
                            |ui| {
                                if ui
                                    .add(
                                        egui::DragValue::new(yaw_deg)
                                            .speed(1.0)
                                            .suffix("°")
                                            .range(-360.0..=360.0),
                                    )
                                    .on_hover_text("Rotate keep-out box around WCS Z")
                                    .changed()
                                {
                                    *mutated_label = Some("Edit Keep-Out Yaw");
                                }
                            },
                        );
                    }
                    FixtureKind::CylinderKeepOut {
                        center_xy_wcs,
                        z_bottom,
                        z_top,
                        diameter,
                    } => {
                        len_row(
                            ui,
                            "Center X",
                            "Cylinder center X in Setup WCS",
                            ("ck_cx", setup_idx, sel_idx),
                            &mut center_xy_wcs[0],
                            eval_ctx,
                            "Edit Cylinder Center X",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Center Y",
                            "Cylinder center Y in Setup WCS",
                            ("ck_cy", setup_idx, sel_idx),
                            &mut center_xy_wcs[1],
                            eval_ctx,
                            "Edit Cylinder Center Y",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Bottom Z",
                            "Cylinder base Z height in Setup WCS",
                            ("ck_zb", setup_idx, sel_idx),
                            z_bottom,
                            eval_ctx,
                            "Edit Cylinder Bottom Z",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Top Z",
                            "Cylinder top Z height in Setup WCS",
                            ("ck_zt", setup_idx, sel_idx),
                            z_top,
                            eval_ctx,
                            "Edit Cylinder Top Z",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Diameter",
                            "Outer diameter of cylindrical keep-out pillar",
                            ("ck_d", setup_idx, sel_idx),
                            diameter,
                            eval_ctx,
                            "Edit Cylinder Diameter",
                            mutated_label,
                        );
                    }
                    FixtureKind::ImportedModel {
                        model_id,
                        offset_wcs,
                        rotate_deg,
                    } => {
                        grid_row(
                            ui,
                            "Fixture Model",
                            Some("Imported STEP/STL/OBJ/3MF model with role = fixture"),
                            |ui| {
                                let prev = model_id.clone();
                                let sel_txt = fixture_models
                                    .iter()
                                    .find(|m| &m.id == model_id)
                                    .map_or(model_id.as_str(), |m| m.path.as_str());
                                egui::ComboBox::from_id_salt(("imp_fix_mid", setup_idx, sel_idx))
                                    .width(INPUT_COL_WIDTH + 54.0)
                                    .truncate()
                                    .selected_text(sel_txt)
                                    .show_ui(ui, |ui| {
                                        for fm in fixture_models {
                                            ui.selectable_value(
                                                model_id,
                                                fm.id.clone(),
                                                format!("{} ({})", fm.id, fm.path),
                                            )
                                            .on_hover_text(format!(
                                                "Bind fixture solid to imported file '{}'",
                                                fm.path
                                            ));
                                        }
                                    })
                                    .response
                                    .on_hover_text("Select imported 3D fixture model asset");
                                if *model_id != prev {
                                    *mutated_label = Some("Change Imported Fixture Model");
                                }
                            },
                        );
                        len_row(
                            ui,
                            "Offset X",
                            "WCS X translation applied to imported fixture solid",
                            ("im_ox", setup_idx, sel_idx),
                            &mut offset_wcs[0],
                            eval_ctx,
                            "Edit Imported Fixture Offset X",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Offset Y",
                            "WCS Y translation applied to imported fixture solid",
                            ("im_oy", setup_idx, sel_idx),
                            &mut offset_wcs[1],
                            eval_ctx,
                            "Edit Imported Fixture Offset Y",
                            mutated_label,
                        );
                        len_row(
                            ui,
                            "Offset Z",
                            "WCS Z translation applied to imported fixture solid",
                            ("im_oz", setup_idx, sel_idx),
                            &mut offset_wcs[2],
                            eval_ctx,
                            "Edit Imported Fixture Offset Z",
                            mutated_label,
                        );
                        for (ax_idx, ax_lbl) in [(0, "Rotate X"), (1, "Rotate Y"), (2, "Rotate Z")]
                        {
                            grid_row(
                                ui,
                                ax_lbl,
                                Some(
                                    "Euler rotation (deg) applied to imported fixture solid in Setup WCS",
                                ),
                                |ui| {
                                    if ui
                                        .add(
                                            egui::DragValue::new(&mut rotate_deg[ax_idx])
                                                .speed(1.0)
                                                .suffix("°")
                                                .range(-360.0..=360.0),
                                        )
                                        .on_hover_text(format!("Adjust {ax_lbl} in degrees"))
                                        .changed()
                                    {
                                        *mutated_label = Some("Rotate Imported Fixture");
                                    }
                                },
                            );
                        }
                    }
                },
            );
        },
    );

    // 5. Resolved WCS Bounding Box & Collision Mesh Summary
    if let Some(sol) = resolved_fixtures.iter().find(|s| s.fixture_id == fix.id) {
        ui.add_space(5.0);
        let fmt = |v| display_unit.format_length(v);
        let b = sol.aabb_wcs;
        static_card(ui, Some("Resolved WCS Bounds"), |ui| {
            ui.monospace(format!("X: {} .. {}", fmt(b.min.x), fmt(b.max.x)));
            ui.monospace(format!("Y: {} .. {}", fmt(b.min.y), fmt(b.max.y)));
            ui.monospace(format!("Z: {} .. {}", fmt(b.min.z), fmt(b.max.z)));
            ui.monospace(format!(
                "Mesh: {} tris · Margin: {} · Overcut: {}",
                sol.mesh_wcs.len(),
                fmt(sol.clearance_margin_mm),
                fmt(sol.allowed_overcut_mm)
            ));
        });
    }
}

#[allow(clippy::too_many_arguments)]
fn len_row(
    ui: &mut egui::Ui,
    label: &str,
    tip: &'static str,
    salt: (&'static str, usize, usize),
    param: &mut ParamLength,
    eval_ctx: &EvalContext,
    undo: &'static str,
    mutated: &mut Option<&'static str>,
) {
    let changed = grid_row(ui, label, Some(tip), |ui| {
        edit_param_length(ui, salt, param, eval_ctx, INPUT_COL_WIDTH)
    });
    if changed {
        *mutated = Some(undo);
    }
}

fn z_ref_row(
    ui: &mut egui::Ui,
    label: &str,
    salt: (&'static str, usize, usize),
    val: &mut FixtureZRef,
    undo: &'static str,
    mutated: &mut Option<&'static str>,
) {
    grid_row(
        ui,
        label,
        Some("Setup Z plane datum anchoring this fixture vertically"),
        |ui| {
            let prev = *val;
            egui::ComboBox::from_id_salt(salt)
                .width(INPUT_COL_WIDTH + 30.0)
                .selected_text(val.label())
                .show_ui(ui, |ui| {
                    for r in FixtureZRef::ALL {
                        ui.selectable_value(val, r, r.label())
                            .on_hover_text(format!("Anchor fixture Z relative to {}", r.label()));
                    }
                })
                .response
                .on_hover_text("Select Setup Z datum plane for this fixture");
            if *val != prev {
                *mutated = Some(undo);
            }
        },
    );
}
