use chip_cad::{CadDocument, find_coplanar_faces, find_same_diameter_holes};
use chip_cam::{
    ChamferSourceMode, ChamferTraceMode, ContourCompensation, DrillCycleKind, EntryStrategy,
    ExtractedFeature25D, FacingCutMode, GeometrySelectionItem, HeightReference, HoleMachiningMode,
    OperationStatus, PocketPassOrder, PocketStrategyKind, ResolvedHeights, ResolvedSetup,
    RetractPolicy, StrategyParams, TabPlacementMode, TabProfileKind, extract_selection_features,
};
use chip_math::{EvalContext, ParamLength};
use chip_project::{ProjectDocument, UndoHistory};
use chip_tools::{
    CoolantMode, ToolLibrary, chipload_from_feed, compute_radial_chip_thinning, feed_from_chipload,
    rpm_from_surface_speed, surface_speed_from_rpm,
};

use crate::{
    panels::{OperationTab, ViewportPickMode, browser::default_pick_mode_for_op},
    widgets::edit_param_length,
};

/// Renders the standardized 3-tab **Operation Inspector** (`[Tool]`, `[Geometry]`, `[Heights]`).
#[allow(clippy::too_many_arguments)]
pub fn show_operation_inspector(
    ui: &mut egui::Ui,
    project: &mut ProjectDocument,
    setup_idx: usize,
    op_idx: usize,
    global_tools: &ToolLibrary,
    history: &mut UndoHistory,
    cad_doc: Option<&CadDocument>,
    resolved_setup: Option<&ResolvedSetup>,
    tab: &mut OperationTab,
    pick_mode: &mut ViewportPickMode,
    lock_chipload: &mut bool,
    show_z_planes: &mut bool,
    open_tool_manager: &mut bool,
) {
    if setup_idx >= project.setups.len() || op_idx >= project.setups[setup_idx].operations.len() {
        ui.weak("No Operation selected.");
        return;
    }

    let display_unit = project.display_units;
    let pre_snapshot = project.clone();
    let mut mutated_label: Option<&'static str> = None;

    let op_kind = project.setups[setup_idx].operations[op_idx].kind;
    ui.horizontal(|ui| {
        ui.strong(op_kind.label());
        let name_ref = &mut project.setups[setup_idx].operations[op_idx].name;
        let resp = ui.text_edit_singleline(name_ref);
        if resp.lost_focus()
            && project.setups[setup_idx].operations[op_idx].name
                != pre_snapshot.setups[setup_idx].operations[op_idx].name
        {
            mutated_label = Some("Rename Operation");
        }
    });

    ui.separator();
    ui.horizontal_wrapped(|ui| {
        if ui
            .selectable_value(tab, OperationTab::Tool, "Tool")
            .clicked()
        {
            *pick_mode = ViewportPickMode::InspectEntity;
        }
        if ui
            .selectable_value(tab, OperationTab::Geometry, "Geometry")
            .clicked()
        {
            *pick_mode = default_pick_mode_for_op(setup_idx, op_idx, op_kind, *tab);
        }
        if ui
            .selectable_value(tab, OperationTab::Heights, "Heights")
            .clicked()
        {
            *pick_mode = ViewportPickMode::InspectEntity;
        }
        if ui
            .selectable_value(tab, OperationTab::Passes, "Passes")
            .clicked()
        {
            *pick_mode = ViewportPickMode::InspectEntity;
        }
        if ui
            .selectable_value(tab, OperationTab::Linking, "Linking")
            .clicked()
        {
            *pick_mode = ViewportPickMode::InspectEntity;
        }
    });
    ui.separator();

    if let OperationStatus::NeedsRepick { message } = project.setups[setup_idx].operations[op_idx]
        .runtime_status
        .clone()
    {
        egui::Frame::NONE
            .fill(egui::Color32::from_rgb(56, 30, 24))
            .stroke(egui::Stroke::new(1.0, egui::Color32::from_rgb(235, 118, 68)))
            .inner_margin(8.0)
            .corner_radius(4.0)
            .show(ui, |ui| {
                ui.colored_label(
                    egui::Color32::from_rgb(255, 168, 96),
                    "[!] CAD Feature Needs Re-Pick (Associative Reload)",
                );
                ui.add_space(2.0);
                ui.small(&message);
                ui.add_space(4.0);
                ui.horizontal_wrapped(|ui| {
                    if ui
                        .button("Clear Orphaned & Re-Pick in 3D")
                        .on_hover_text(
                            "Clears orphaned selections and activates 3D picking while preserving tool, speeds/feeds, heights, passes, and tabs.",
                        )
                        .clicked()
                    {
                        project.setups[setup_idx].operations[op_idx]
                            .selections
                            .clear();
                        *tab = OperationTab::Geometry;
                        *pick_mode =
                            default_pick_mode_for_op(setup_idx, op_idx, op_kind, OperationTab::Geometry);
                        mutated_label = Some("Clear Orphaned Feature for Re-Pick");
                    }
                });
            });
        ui.add_space(6.0);
    }

    // Pre-compute extracted selection features + scoped EvalContext
    let extracted = match (cad_doc, resolved_setup) {
        (Some(doc), Some(rs)) => Some(extract_selection_features(
            doc,
            rs,
            &project.setups[setup_idx].operations[op_idx].selections,
        )),
        _ => None,
    };

    let mut eval_ctx = resolved_setup
        .map(|rs| rs.eval_context(display_unit))
        .unwrap_or_else(|| EvalContext::new(display_unit));

    if let Some(tid) = project.setups[setup_idx].operations[op_idx].tool_id.clone()
        && let Some(tool) = project.tool(&tid).or_else(|| global_tools.tool(&tid))
    {
        tool.populate_eval_context(&mut eval_ctx);
    }

    let (inf_top, inf_bot) = match &extracted {
        Some(Ok(sum)) => {
            eval_ctx.set_var("feature_top_z", sum.inferred_top_z_wcs);
            eval_ctx.set_var("feature_bottom_z", sum.inferred_bottom_z_wcs);
            if let Some(hd) = sum.primary_hole_diameter_mm {
                eval_ctx.set_var("hole_d", hd);
            }
            (sum.inferred_top_z_wcs, sum.inferred_bottom_z_wcs)
        }
        _ => {
            let st = resolved_setup.map_or(16.0, |rs| rs.stock.aabb_wcs().max.z);
            let mt = resolved_setup.map_or(15.0, |rs| rs.model_aabb_wcs.max.z);
            (st, mt)
        }
    };

    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| match *tab {
            OperationTab::Tool => {
                // Collect union of Project-Embedded tools and Global Library tools compatible with `op_kind`
                let mut available_tools = project.tools.clone();
                for gt in &global_tools.tools {
                    if !available_tools.iter().any(|pt| pt.id == gt.id) {
                        available_tools.push(gt.clone());
                    }
                }

                let cur_tid = project.setups[setup_idx].operations[op_idx].tool_id.clone();
                let cur_tool_opt = cur_tid
                    .as_ref()
                    .and_then(|id| available_tools.iter().find(|t| &t.id == id))
                    .cloned();

                let selected_label = cur_tool_opt
                    .as_ref()
                    .map(|t| t.summary_label())
                    .unwrap_or_else(|| "Select a Tool…".to_string());

                ui.horizontal(|ui| {
                    egui::ComboBox::from_id_salt(("op_tool_cb", setup_idx, op_idx))
                        .width(210.0)
                        .selected_text(selected_label)
                        .show_ui(ui, |ui| {
                            for candidate in &available_tools {
                                let compat = op_kind.is_compatible_with_tool(candidate);
                                let prefix = if compat { "" } else { "[Incompat] " };
                                let lbl = format!("{prefix}{}", candidate.summary_label());
                                let is_sel = cur_tid.as_ref() == Some(&candidate.id);
                                if ui.selectable_label(is_sel, lbl).clicked() && !is_sel {
                                    project.ensure_tool_embedded(candidate);
                                    let op = &mut project.setups[setup_idx].operations[op_idx];
                                    op.tool_id = Some(candidate.id.clone());
                                    op.preset_id = candidate.presets.first().map(|p| p.id.clone());
                                    op.feed_overrides.clear_all();
                                    mutated_label = Some("Assign Tool to Operation");
                                }
                            }
                        });
                    if ui
                        .small_button("Library…")
                        .on_hover_text("Open Tool Library Manager")
                        .clicked()
                    {
                        *open_tool_manager = true;
                    }
                });

                let Some(active_tool) = project.setups[setup_idx].operations[op_idx]
                    .tool_id
                    .as_ref()
                    .and_then(|id| project.tool(id).cloned())
                else {
                    ui.weak("Assign a tool above to configure cutting presets and coupled SFM/chipload.");
                    return;
                };

                if !op_kind.is_compatible_with_tool(&active_tool) {
                    ui.colored_label(
                        egui::Color32::from_rgb(245, 96, 96),
                        format!(
                            "[!] '{}' is not compatible with {}!",
                            active_tool.geometry.kind_label(),
                            op_kind.label()
                        ),
                    );
                }

                // Material Preset selector
                if !active_tool.presets.is_empty() {
                    let cur_pid = project.setups[setup_idx].operations[op_idx]
                        .preset_id
                        .clone()
                        .unwrap_or_else(|| active_tool.presets[0].id.clone());
                    let cur_pname = active_tool
                        .presets
                        .iter()
                        .find(|p| p.id == cur_pid)
                        .map(|p| p.name.as_str())
                        .unwrap_or("Default");

                    ui.horizontal(|ui| {
                        ui.label("Material Preset:");
                        egui::ComboBox::from_id_salt(("op_preset_cb", setup_idx, op_idx))
                            .selected_text(cur_pname)
                            .show_ui(ui, |ui| {
                                for p in &active_tool.presets {
                                    let is_p = cur_pid == p.id;
                                    if ui.selectable_label(is_p, &p.name).clicked() && !is_p {
                                        let op = &mut project.setups[setup_idx].operations[op_idx];
                                        op.preset_id = Some(p.id.clone());
                                        op.feed_overrides.clear_all();
                                        mutated_label = Some("Change Material Preset");
                                    }
                                }
                            });
                    });
                }

                let op = &mut project.setups[setup_idx].operations[op_idx];
                let eff = op.effective_cutting_params(&active_tool);
                let dia_mm = active_tool.geometry.cutting_diameter_mm();
                let flutes = active_tool.geometry.flutes();

                ui.horizontal(|ui| {
                    ui.checkbox(lock_chipload, "Lock Chipload on RPM/SFM edit");
                    if !op.feed_overrides.is_all_inherited()
                        && ui
                            .small_button("Reset Overrides")
                            .on_hover_text("Revert all locally overridden feeds/speeds to the material preset")
                            .clicked()
                    {
                        op.feed_overrides.clear_all();
                        mutated_label = Some("Reset Feed/Speed Overrides");
                    }
                });
                ui.separator();

                // Coupled Spindle RPM <-> Surface Speed
                ui.strong("Coupled Spindle & Surface Speed");
                let mut rpm_val = eff.spindle_rpm;
                ui.horizontal(|ui| {
                    ui.label("Spindle Speed:");
                    let r = ui.add(
                        egui::DragValue::new(&mut rpm_val)
                            .speed(25.0)
                            .range(100.0..=60_000.0)
                            .suffix(" RPM"),
                    );
                    if op.feed_overrides.spindle_rpm.is_some() {
                        ui.colored_label(egui::Color32::from_rgb(245, 190, 66), "*")
                            .on_hover_text("Overridden from preset");
                    }
                    if r.changed() {
                        op.feed_overrides.spindle_rpm = Some(rpm_val);
                        if *lock_chipload
                            && let Some(fz) =
                                chipload_from_feed(eff.spindle_rpm, flutes, eff.cutting_feed_mm_min)
                        {
                            op.feed_overrides.cutting_feed_mm_min =
                                Some(feed_from_chipload(rpm_val, flutes, fz));
                        }
                        mutated_label = Some("Adjust Spindle RPM");
                    }
                });

                let cur_vc_m_min = surface_speed_from_rpm(rpm_val, dia_mm).unwrap_or(0.0);
                let mut vc_disp = display_unit.surface_speed_from_m_min(cur_vc_m_min);
                ui.horizontal(|ui| {
                    ui.label("Surface Speed:");
                    let r = ui.add(
                        egui::DragValue::new(&mut vc_disp)
                            .speed(1.0)
                            .range(1.0..=5_000.0)
                            .suffix(format!(" {}", display_unit.surface_speed_suffix())),
                    );
                    if r.changed() {
                        let new_vc_m = display_unit.surface_speed_to_m_min(vc_disp);
                        if let Some(new_rpm) = rpm_from_surface_speed(new_vc_m, dia_mm) {
                            op.feed_overrides.spindle_rpm = Some(new_rpm);
                            if *lock_chipload
                                && let Some(fz) = chipload_from_feed(
                                    eff.spindle_rpm,
                                    flutes,
                                    eff.cutting_feed_mm_min,
                                )
                            {
                                op.feed_overrides.cutting_feed_mm_min =
                                    Some(feed_from_chipload(new_rpm, flutes, fz));
                            }
                            mutated_label = Some("Adjust Surface Speed");
                        }
                    }
                });

                ui.add_space(4.0);
                ui.strong("Coupled Feedrate & Chipload");

                let eff2 = op.effective_cutting_params(&active_tool);
                let mut vf_disp = display_unit.feed_from_mm_min(eff2.cutting_feed_mm_min);
                ui.horizontal(|ui| {
                    ui.label("Cutting Feed:");
                    let r = ui.add(
                        egui::DragValue::new(&mut vf_disp)
                            .speed(5.0)
                            .range(1.0..=25_000.0)
                            .suffix(format!(" {}", display_unit.feed_suffix())),
                    );
                    if op.feed_overrides.cutting_feed_mm_min.is_some() {
                        ui.colored_label(egui::Color32::from_rgb(245, 190, 66), "*");
                    }
                    if r.changed() {
                        op.feed_overrides.cutting_feed_mm_min =
                            Some(display_unit.feed_to_mm_min(vf_disp));
                        mutated_label = Some("Adjust Cutting Feedrate");
                    }
                });

                let cur_fz_mm =
                    chipload_from_feed(eff2.spindle_rpm, flutes, eff2.cutting_feed_mm_min)
                        .unwrap_or(0.0);
                let mut fz_disp = display_unit.chipload_from_mm(cur_fz_mm);
                ui.horizontal(|ui| {
                    ui.label(format!("Chipload ({flutes} fl):"));
                    let r = ui.add(
                        egui::DragValue::new(&mut fz_disp)
                            .speed(0.001)
                            .max_decimals(5)
                            .range(0.0001..=5.0)
                            .suffix(format!(" {}", display_unit.chipload_suffix())),
                    );
                    if r.changed() {
                        let new_fz_mm = display_unit.chipload_to_mm(fz_disp);
                        op.feed_overrides.cutting_feed_mm_min =
                            Some(feed_from_chipload(eff2.spindle_rpm, flutes, new_fz_mm));
                        mutated_label = Some("Adjust Chipload");
                    }
                });

                // Stepdown, Stepover, & Coolant
                let mut ae_disp = display_unit.from_mm(eff2.stepover_mm);
                ui.horizontal(|ui| {
                    ui.label("Radial Stepover:");
                    if ui
                        .add(
                            egui::DragValue::new(&mut ae_disp)
                                .speed(0.05)
                                .range(0.01..=500.0)
                                .suffix(format!(" {}", display_unit.suffix())),
                        )
                        .changed()
                    {
                        op.feed_overrides.stepover_mm = Some(display_unit.to_mm(ae_disp));
                        mutated_label = Some("Adjust Stepover");
                    }
                });

                // Radial Chip Thinning live hint (`04-tool-library.md`)
                if let Some(rct) = compute_radial_chip_thinning(
                    dia_mm,
                    eff2.stepover_mm,
                    cur_fz_mm,
                    eff2.cutting_feed_mm_min,
                ) {
                    ui.colored_label(
                        egui::Color32::from_rgb(110, 212, 248),
                        format!(
                            "RCT ({:.0}% engagement): actual hex = {:.4} {} — RCT feed hint: {:.0} {}",
                            (eff2.stepover_mm / dia_mm) * 100.0,
                            display_unit.chipload_from_mm(rct.actual_hex_mm),
                            display_unit.chipload_suffix(),
                            display_unit.feed_from_mm_min(rct.compensated_feed_mm_min),
                            display_unit.feed_suffix(),
                        ),
                    );
                }

                let mut cool = eff2.coolant;
                ui.horizontal(|ui| {
                    ui.label("Coolant:");
                    egui::ComboBox::from_id_salt(("op_cool_cb", setup_idx, op_idx))
                        .selected_text(cool.label())
                        .show_ui(ui, |ui| {
                            for c in CoolantMode::ALL {
                                if ui.selectable_value(&mut cool, c, c.label()).changed() {
                                    op.feed_overrides.coolant = Some(cool);
                                    mutated_label = Some("Change Operation Coolant");
                                }
                            }
                        });
                });
            }
            OperationTab::Geometry => {
                ui.strong("Interactive 3D Pick Mode");
                let is_faces = matches!(pick_mode, ViewportPickMode::OpPlanarFaces { .. });
                let is_loops = matches!(pick_mode, ViewportPickMode::OpClosedLoops { .. });
                let is_chain = matches!(pick_mode, ViewportPickMode::OpOpenChain { .. });
                let is_holes = matches!(pick_mode, ViewportPickMode::OpCylindricalHoles { .. });

                ui.horizontal_wrapped(|ui| {
                    if ui.selectable_label(is_faces, "Planar Faces").clicked() {
                        *pick_mode = ViewportPickMode::OpPlanarFaces { setup_idx, op_idx };
                    }
                    if ui.selectable_label(is_loops, "Closed Loops").clicked() {
                        *pick_mode = ViewportPickMode::OpClosedLoops { setup_idx, op_idx };
                    }
                    if ui.selectable_label(is_chain, "Open Chain").clicked() {
                        *pick_mode = ViewportPickMode::OpOpenChain {
                            setup_idx,
                            op_idx,
                            start_edge: None,
                        };
                    }
                    if ui.selectable_label(is_holes, "Cylindrical Holes").clicked() {
                        *pick_mode = ViewportPickMode::OpCylindricalHoles { setup_idx, op_idx };
                    }
                });

                // Assistive expanders (`Select Same Diameter` & `Select Coplanar Faces`)
                if let Some(doc) = cad_doc {
                    ui.horizontal_wrapped(|ui| {
                        if ui
                            .small_button("+ Same Diameter")
                            .on_hover_text("Expand hole selection to all matching diameter holes on the same body")
                            .clicked()
                        {
                            let op = &mut project.setups[setup_idx].operations[op_idx];
                            let seed_hole = op.selections.iter().find_map(|s| match s {
                                GeometrySelectionItem::CylindricalHole { binding } => {
                                    Some(chip_cad::FaceId(binding.last_index))
                                }
                                _ => None,
                            });
                            if let Some(seed) = seed_hole {
                                for fid in find_same_diameter_holes(doc, seed, 1e-3, 1e-3) {
                                    let already = op.selections.iter().any(|s| {
                                        matches!(s, GeometrySelectionItem::CylindricalHole { binding } if binding.last_index == fid.0)
                                    });
                                    if !already
                                        && let Some(b) = doc.bind_face(fid)
                                    {
                                        op.selections.push(
                                            GeometrySelectionItem::CylindricalHole { binding: b },
                                        );
                                    }
                                }
                                mutated_label = Some("Select Same-Diameter Holes");
                            }
                        }

                        if ui
                            .small_button("+ Coplanar Faces")
                            .on_hover_text("Expand planar face selection to all coplanar pocket floors/shelves on the same body")
                            .clicked()
                        {
                            let op = &mut project.setups[setup_idx].operations[op_idx];
                            let seed_face = op.selections.iter().find_map(|s| match s {
                                GeometrySelectionItem::PlanarFace { binding } => {
                                    Some(chip_cad::FaceId(binding.last_index))
                                }
                                _ => None,
                            });
                            if let Some(seed) = seed_face {
                                for fid in find_coplanar_faces(doc, seed, 1e-3, 1e-2) {
                                    let already = op.selections.iter().any(|s| {
                                        matches!(s, GeometrySelectionItem::PlanarFace { binding } if binding.last_index == fid.0)
                                    });
                                    if !already
                                        && let Some(b) = doc.bind_face(fid)
                                    {
                                        op.selections
                                            .push(GeometrySelectionItem::PlanarFace { binding: b });
                                    }
                                }
                                mutated_label = Some("Select Coplanar Faces");
                            }
                        }

                        if !project.setups[setup_idx].operations[op_idx]
                            .selections
                            .is_empty()
                            && ui.small_button("Clear All").clicked()
                        {
                            project.setups[setup_idx].operations[op_idx]
                                .selections
                                .clear();
                            mutated_label = Some("Clear Operation Selections");
                        }
                    });
                }

                ui.separator();
                let op = &mut project.setups[setup_idx].operations[op_idx];
                ui.strong(format!("Selected Entities ({})", op.selections.len()));

                if op.selections.is_empty() {
                    if op.kind == chip_cam::OperationKind::Facing {
                        ui.weak("Facing defaults automatically to the Setup XY Stock boundary.");
                    } else {
                        ui.weak("Click entities directly in the 3D viewport above to add geometry.");
                    }
                } else {
                    let mut remove_idx: Option<usize> = None;
                    for (idx, item) in op.selections.iter_mut().enumerate() {
                        ui.horizontal(|ui| {
                            ui.label(format!("• {}", item.short_label()));
                            ui.with_layout(
                                egui::Layout::right_to_left(egui::Align::Center),
                                |ui| {
                                    if ui.small_button("×").clicked() {
                                        remove_idx = Some(idx);
                                    }
                                    match item {
                                        GeometrySelectionItem::ClosedLoop {
                                            reverse,
                                            side_flip,
                                            ..
                                        }
                                        | GeometrySelectionItem::OpenChain {
                                            reverse,
                                            side_flip,
                                            ..
                                        } => {
                                            if ui
                                                .small_button("Flip Side")
                                                .on_hover_text("Flip cutter compensation side (Left <-> Right)")
                                                .clicked()
                                            {
                                                *side_flip = !*side_flip;
                                                mutated_label = Some("Flip Chain Offset Side");
                                            }
                                            if ui
                                                .small_button("Rev")
                                                .on_hover_text("Reverse chain travel direction")
                                                .clicked()
                                            {
                                                *reverse = !*reverse;
                                                mutated_label = Some("Reverse Chain Direction");
                                            }
                                        }
                                        _ => {}
                                    }
                                },
                            );
                        });
                    }
                    if let Some(ri) = remove_idx {
                        op.selections.remove(ri);
                        mutated_label = Some("Remove Selected Entity");
                    }
                }

                // Live Extracted 2.5D Feature Summary
                ui.separator();
                match &extracted {
                    Some(Ok(sum)) => {
                        ui.strong("Extracted 2.5D WCS Features");
                        ui.label(format!(
                            "Inferred Top Z:    {}",
                            display_unit.format_length(sum.inferred_top_z_wcs)
                        ));
                        ui.label(format!(
                            "Inferred Bottom Z: {}",
                            display_unit.format_length(sum.inferred_bottom_z_wcs)
                        ));
                        for feat in &sum.features {
                            match feat {
                                ExtractedFeature25D::PlanarFace {
                                    face_id,
                                    face_z_wcs,
                                    inner_islands_wcs,
                                    area_mm2,
                                    ..
                                } => {
                                    ui.weak(format!(
                                        "  Face #{}: Z={}, {} inner loop(s), {:.2} {}²",
                                        face_id.0,
                                        display_unit.format_length(*face_z_wcs),
                                        inner_islands_wcs.len(),
                                        display_unit.area_from_mm2(*area_mm2),
                                        display_unit.suffix()
                                    ));
                                }
                                ExtractedFeature25D::Contour {
                                    polyline_wcs,
                                    min_z_wcs,
                                    ..
                                } => {
                                    let arcs = polyline_wcs
                                        .segments
                                        .iter()
                                        .filter(|s| {
                                            matches!(s, chip_math::ArcSegment2D::Arc { .. })
                                        })
                                        .count();
                                    ui.weak(format!(
                                        "  {} Contour: len={}, {} segs ({} true arcs), Z={}",
                                        if polyline_wcs.closed { "Closed" } else { "Open" },
                                        display_unit.format_length(polyline_wcs.perimeter()),
                                        polyline_wcs.segments.len(),
                                        arcs,
                                        display_unit.format_length(*min_z_wcs),
                                    ));
                                }
                                ExtractedFeature25D::ModeledChamfer {
                                    face_id,
                                    radial_width_mm,
                                    bevel_half_angle_rad,
                                    top_z_wcs,
                                    bottom_z_wcs,
                                    ..
                                } => {
                                    ui.weak(format!(
                                        "  Modeled Chamfer #{}: width={}, {:.1}°, Z={}..{}",
                                        face_id.0,
                                        display_unit.format_length(*radial_width_mm),
                                        bevel_half_angle_rad.to_degrees(),
                                        display_unit.format_length(*bottom_z_wcs),
                                        display_unit.format_length(*top_z_wcs),
                                    ));
                                }
                                ExtractedFeature25D::CylindricalHole {
                                    center_wcs,
                                    diameter_mm,
                                    top_z_wcs,
                                    bottom_z_wcs,
                                    is_through_hole,
                                    ..
                                } => {
                                    ui.weak(format!(
                                        "  Hole Ø{} ({}): XY=({:.2}, {:.2}), Z={}..{}",
                                        display_unit.format_length(*diameter_mm),
                                        if *is_through_hole { "Through" } else { "Blind" },
                                        display_unit.from_mm(center_wcs.x),
                                        display_unit.from_mm(center_wcs.y),
                                        display_unit.format_length(*bottom_z_wcs),
                                        display_unit.format_length(*top_z_wcs),
                                    ));
                                }
                            }
                        }
                    }
                    Some(Err(err)) => {
                        ui.colored_label(
                            egui::Color32::from_rgb(245, 96, 96),
                            format!("[!] {err}"),
                        );
                    }
                    None => {}
                }
            }
            OperationTab::Heights => {
                ui.checkbox(
                    show_z_planes,
                    "Preview translucent Z-height planes in 3D viewport",
                );
                ui.separator();

                let op = &mut project.setups[setup_idx].operations[op_idx];
                let rows = [
                    ("Clearance Height", 0usize),
                    ("Retract Height", 1usize),
                    ("Feed Height", 2usize),
                    ("Top Height", 3usize),
                    ("Bottom Height", 4usize),
                ];

                for (title, which) in rows {
                    ui.strong(title);
                    let spec = match which {
                        0 => &mut op.heights.clearance,
                        1 => &mut op.heights.retract,
                        2 => &mut op.heights.feed,
                        3 => &mut op.heights.top,
                        _ => &mut op.heights.bottom,
                    };
                    egui::ComboBox::from_id_salt(("op_h_ref", setup_idx, op_idx, which))
                        .selected_text(spec.reference.label())
                        .show_ui(ui, |ui| {
                            for r in HeightReference::OPERATION_REFS {
                                if ui.selectable_value(&mut spec.reference, r, r.label()).changed()
                                {
                                    mutated_label = Some("Change Operation Height Reference");
                                }
                            }
                        });
                    if edit_param_length(
                        ui,
                        ("op_h_off", setup_idx, op_idx, which),
                        &mut spec.offset,
                        &eval_ctx,
                        120.0,
                    ) {
                        mutated_label = Some("Edit Operation Height Offset");
                    }
                    ui.add_space(4.0);
                }

                if let Some(rs) = resolved_setup {
                    ui.separator();
                    let mut h_eval = op.heights.clone();
                    h_eval.reevaluate(&eval_ctx);
                    match ResolvedHeights::resolve(
                        &h_eval,
                        &rs.model_aabb_wcs,
                        &rs.stock.aabb_wcs(),
                        rs.clearance_z_wcs,
                        rs.retract_z_wcs,
                        inf_top,
                        inf_bot,
                    ) {
                        Ok(rh) => {
                            ui.colored_label(
                                egui::Color32::from_rgb(92, 218, 128),
                                "[OK] Z-Height Hierarchy Valid",
                            );
                            ui.label(format!(
                                "Clearance: {}  |  Retract: {}",
                                display_unit.format_length(rh.clearance_z_wcs),
                                display_unit.format_length(rh.retract_z_wcs),
                            ));
                            ui.label(format!(
                                "Feed:      {}  |  Top:     {}",
                                display_unit.format_length(rh.feed_z_wcs),
                                display_unit.format_length(rh.top_z_wcs),
                            ));
                            ui.label(format!(
                                "Bottom:    {}",
                                display_unit.format_length(rh.bottom_z_wcs),
                            ));
                        }
                        Err(err) => {
                            ui.colored_label(
                                egui::Color32::from_rgb(248, 92, 92),
                                format!("[!] {err}"),
                            );
                        }
                    }
                }
            }
            OperationTab::Passes => {
                show_passes_tab(
                    ui,
                    &mut project.setups[setup_idx].operations[op_idx],
                    setup_idx,
                    op_idx,
                    &eval_ctx,
                    pick_mode,
                    &mut mutated_label,
                );
            }
            OperationTab::Linking => {
                show_linking_tab(
                    ui,
                    &mut project.setups[setup_idx].operations[op_idx],
                    setup_idx,
                    op_idx,
                    &eval_ctx,
                    &mut mutated_label,
                );
            }
        });

    if let Some(action) = mutated_label {
        history.record_before_edit(action, None, &pre_snapshot);
        project.setups[setup_idx].operations[op_idx].mark_dirty();
    }
}

fn show_passes_tab(
    ui: &mut egui::Ui,
    op: &mut chip_cam::Operation,
    setup_idx: usize,
    op_idx: usize,
    eval_ctx: &EvalContext,
    pick_mode: &mut ViewportPickMode,
    mutated_label: &mut Option<&'static str>,
) {
    ui.strong("Depth Passes & Stock to Leave");
    if ui
        .checkbox(&mut op.passes.multiple_depths, "Multiple Z Stepdowns")
        .changed()
    {
        *mutated_label = Some("Toggle Multiple Depths");
    }
    if op.passes.multiple_depths {
        ui.label("Maximum Stepdown (`ap`):");
        if edit_param_length(
            ui,
            ("op_max_ap", setup_idx, op_idx),
            &mut op.passes.max_stepdown,
            eval_ctx,
            120.0,
        ) {
            *mutated_label = Some("Edit Max Stepdown");
        }
        if ui
            .checkbox(&mut op.passes.even_stepdowns, "Use Even Z Stepdowns")
            .changed()
        {
            *mutated_label = Some("Toggle Even Stepdowns");
        }
    }

    ui.add_space(4.0);
    ui.label("Radial Stock to Leave:");
    if edit_param_length(
        ui,
        ("op_rad_leave", setup_idx, op_idx),
        &mut op.passes.stock_to_leave_radial,
        eval_ctx,
        120.0,
    ) {
        *mutated_label = Some("Edit Radial Stock to Leave");
    }
    ui.label("Axial Stock to Leave:");
    if edit_param_length(
        ui,
        ("op_ax_leave", setup_idx, op_idx),
        &mut op.passes.stock_to_leave_axial,
        eval_ctx,
        120.0,
    ) {
        *mutated_label = Some("Edit Axial Stock to Leave");
    }

    ui.separator();
    ui.strong("Strategy Parameters");

    match op.strategy_mut() {
        StrategyParams::Facing(p) => {
            egui::ComboBox::from_id_salt(("face_cut_mode", setup_idx, op_idx))
                .selected_text(p.cut_mode.label())
                .show_ui(ui, |ui| {
                    for m in FacingCutMode::ALL {
                        if ui.selectable_value(&mut p.cut_mode, m, m.label()).changed() {
                            *mutated_label = Some("Change Facing Cut Mode");
                        }
                    }
                });
            ui.label("Stepover (`ae`):");
            if edit_param_length(
                ui,
                ("face_ae", setup_idx, op_idx),
                &mut p.stepover,
                eval_ctx,
                120.0,
            ) {
                *mutated_label = Some("Edit Facing Stepover");
            }
            ui.label("Pass Extension Outside Stock:");
            if edit_param_length(
                ui,
                ("face_ext", setup_idx, op_idx),
                &mut p.pass_extension,
                eval_ctx,
                120.0,
            ) {
                *mutated_label = Some("Edit Facing Extension");
            }
            if ui
                .checkbox(
                    &mut p.auto_longest_axis,
                    "Auto-align along longest XY stock axis",
                )
                .changed()
            {
                *mutated_label = Some("Toggle Facing Auto Axis");
            }
            if !p.auto_longest_axis {
                ui.horizontal(|ui| {
                    ui.label("Pass Angle (°):");
                    if ui
                        .add(egui::DragValue::new(&mut p.pass_direction_deg).speed(1.0))
                        .changed()
                    {
                        *mutated_label = Some("Edit Facing Angle");
                    }
                });
            }
        }
        StrategyParams::Contour2D(p) => {
            ui.label("Cutter Compensation:");
            egui::ComboBox::from_id_salt(("cnt_comp", setup_idx, op_idx))
                .selected_text(p.compensation.label())
                .show_ui(ui, |ui| {
                    for c in ContourCompensation::ALL {
                        if ui
                            .selectable_value(&mut p.compensation, c, c.label())
                            .changed()
                        {
                            *mutated_label = Some("Change Contour Compensation");
                        }
                    }
                });
            if ui
                .checkbox(&mut p.wear_comp_metadata, "Emit G41/G42 Wear Comp Metadata")
                .changed()
            {
                *mutated_label = Some("Toggle Wear Comp");
            }
            if ui
                .checkbox(&mut p.continuous_spiral_ramp, "Continuous Spiral Ramp")
                .changed()
            {
                *mutated_label = Some("Toggle Spiral Ramp");
            }

            ui.horizontal(|ui| {
                ui.label("Roughing Passes:");
                if ui
                    .add(egui::DragValue::new(&mut p.roughing_passes).range(0..=16))
                    .changed()
                {
                    *mutated_label = Some("Edit Roughing Passes");
                }
                ui.label("Finish Passes:");
                if ui
                    .add(egui::DragValue::new(&mut p.finish_passes).range(0..=8))
                    .changed()
                {
                    *mutated_label = Some("Edit Finish Passes");
                }
            });
            if p.roughing_passes > 1 {
                ui.label("Roughing Stepover:");
                if edit_param_length(
                    ui,
                    ("cnt_r_step", setup_idx, op_idx),
                    &mut p.roughing_stepover,
                    eval_ctx,
                    120.0,
                ) {
                    *mutated_label = Some("Edit Roughing Stepover");
                }
            }
            if p.finish_passes > 0 {
                ui.label("Finishing Stepover:");
                if edit_param_length(
                    ui,
                    ("cnt_f_step", setup_idx, op_idx),
                    &mut p.finish_stepover,
                    eval_ctx,
                    120.0,
                ) {
                    *mutated_label = Some("Edit Finish Stepover");
                }
                if ui
                    .checkbox(&mut p.repeat_finish_pass, "Repeat Final Pass (Spring Pass)")
                    .changed()
                {
                    *mutated_label = Some("Toggle Spring Pass");
                }
            }

            ui.separator();
            if ui
                .checkbox(&mut p.tabs.enabled, "Enable Holding Tabs (Bridges)")
                .changed()
            {
                *mutated_label = Some("Toggle Holding Tabs");
            }
            if p.tabs.enabled {
                let mut is_tri = matches!(p.tabs.profile, TabProfileKind::Triangular { .. });
                ui.horizontal(|ui| {
                    ui.label("Shape:");
                    if ui
                        .selectable_value(&mut is_tri, true, "Ramped Triangular")
                        .clicked()
                    {
                        p.tabs.profile = TabProfileKind::Triangular {
                            ramp_angle_deg: 25.0,
                        };
                        *mutated_label = Some("Change Tab Profile");
                    }
                    if ui
                        .selectable_value(&mut is_tri, false, "Rectangular")
                        .clicked()
                    {
                        p.tabs.profile = TabProfileKind::Rectangular;
                        *mutated_label = Some("Change Tab Profile");
                    }
                });
                if let TabProfileKind::Triangular { ramp_angle_deg } = &mut p.tabs.profile {
                    ui.horizontal(|ui| {
                        ui.label("Tab Ramp Angle (°):");
                        if ui
                            .add(egui::DragValue::new(ramp_angle_deg).range(5.0..=75.0))
                            .changed()
                        {
                            *mutated_label = Some("Edit Tab Ramp Angle");
                        }
                    });
                }

                ui.label("Tab Width:");
                if edit_param_length(
                    ui,
                    ("tab_w", setup_idx, op_idx),
                    &mut p.tabs.width,
                    eval_ctx,
                    110.0,
                ) {
                    *mutated_label = Some("Edit Tab Width");
                }
                ui.label("Tab Height (from Bottom Z):");
                if edit_param_length(
                    ui,
                    ("tab_h", setup_idx, op_idx),
                    &mut p.tabs.height,
                    eval_ctx,
                    110.0,
                ) {
                    *mutated_label = Some("Edit Tab Height");
                }

                let mut is_manual = matches!(p.tabs.placement, TabPlacementMode::Manual);
                ui.horizontal(|ui| {
                    ui.label("Placement:");
                    if ui
                        .selectable_value(&mut is_manual, false, "Automatic")
                        .clicked()
                    {
                        p.tabs.placement = TabPlacementMode::default();
                        if matches!(pick_mode, ViewportPickMode::PlaceContourTab { .. }) {
                            *pick_mode = ViewportPickMode::InspectEntity;
                        }
                        *mutated_label = Some("Set Automatic Tabs");
                    }
                    if ui
                        .selectable_value(&mut is_manual, true, "Manual Interactive")
                        .clicked()
                    {
                        p.tabs.placement = TabPlacementMode::Manual;
                        *pick_mode = ViewportPickMode::PlaceContourTab { setup_idx, op_idx };
                        *mutated_label = Some("Set Manual Tabs");
                    }
                });

                match &mut p.tabs.placement {
                    TabPlacementMode::Automatic {
                        count,
                        by_spacing,
                        target_spacing,
                    } => {
                        if ui
                            .checkbox(by_spacing, "Place by target perimeter spacing")
                            .changed()
                        {
                            *mutated_label = Some("Toggle Tab Spacing Mode");
                        }
                        if *by_spacing {
                            if edit_param_length(
                                ui,
                                ("tab_sp", setup_idx, op_idx),
                                target_spacing,
                                eval_ctx,
                                110.0,
                            ) {
                                *mutated_label = Some("Edit Tab Spacing");
                            }
                        } else {
                            ui.horizontal(|ui| {
                                ui.label("Tab Count:");
                                if ui.add(egui::DragValue::new(count).range(1..=32)).changed() {
                                    *mutated_label = Some("Edit Tab Count");
                                }
                            });
                        }
                    }
                    TabPlacementMode::Manual => {
                        let picking_tabs =
                            *pick_mode == ViewportPickMode::PlaceContourTab { setup_idx, op_idx };
                        let btn_txt = if picking_tabs {
                            "● Click Contour in 3D Viewport (Active)"
                        } else {
                            "Click Contour in 3D Viewport to Add Tabs"
                        };
                        if ui.button(btn_txt).clicked() {
                            *pick_mode = if picking_tabs {
                                ViewportPickMode::InspectEntity
                            } else {
                                ViewportPickMode::PlaceContourTab { setup_idx, op_idx }
                            };
                        }
                        let mut del_anchor = None;
                        for (a_idx, anchor) in p.tabs.manual_anchors.iter().enumerate() {
                            ui.horizontal(|ui| {
                                ui.weak(format!(
                                    "Tab #{} (C{}, s={:.1}%)",
                                    a_idx + 1,
                                    anchor.contour_index + 1,
                                    anchor.s_norm * 100.0
                                ));
                                if ui.small_button("×").clicked() {
                                    del_anchor = Some(a_idx);
                                }
                            });
                        }
                        if let Some(rm) = del_anchor {
                            p.tabs.manual_anchors.remove(rm);
                            *mutated_label = Some("Remove Manual Tab");
                        }
                    }
                }
            }
        }
        StrategyParams::Pocket2D(p) => {
            let mut is_zigzag = matches!(p.strategy, PocketStrategyKind::ZigZag { .. });
            ui.horizontal(|ui| {
                ui.label("Clearing:");
                if ui
                    .selectable_value(&mut is_zigzag, false, "Contour-Parallel")
                    .clicked()
                {
                    p.strategy = PocketStrategyKind::ContourParallel;
                    *mutated_label = Some("Select Contour-Parallel Pocket");
                }
                if ui
                    .selectable_value(&mut is_zigzag, true, "Zig-Zag Raster")
                    .clicked()
                {
                    p.strategy = PocketStrategyKind::ZigZag {
                        pass_angle_deg: 0.0,
                    };
                    *mutated_label = Some("Select Zig-Zag Pocket");
                }
            });
            if let PocketStrategyKind::ZigZag { pass_angle_deg } = &mut p.strategy {
                ui.horizontal(|ui| {
                    ui.label("Raster Angle (°):");
                    if ui
                        .add(egui::DragValue::new(pass_angle_deg).speed(1.0))
                        .changed()
                    {
                        *mutated_label = Some("Edit Pocket Raster Angle");
                    }
                });
            }

            ui.label("Stepover (`ae`):");
            if edit_param_length(
                ui,
                ("pkt_ae", setup_idx, op_idx),
                &mut p.stepover,
                eval_ctx,
                120.0,
            ) {
                *mutated_label = Some("Edit Pocket Stepover");
            }

            ui.label("Pass Ordering:");
            egui::ComboBox::from_id_salt(("pkt_ord", setup_idx, op_idx))
                .selected_text(p.pass_order.label())
                .show_ui(ui, |ui| {
                    for o in PocketPassOrder::ALL {
                        if ui
                            .selectable_value(&mut p.pass_order, o, o.label())
                            .changed()
                        {
                            *mutated_label = Some("Change Pocket Pass Order");
                        }
                    }
                });

            ui.label("Open-Edge Overhang:");
            if edit_param_length(
                ui,
                ("pkt_ovh", setup_idx, op_idx),
                &mut p.open_edge_overhang,
                eval_ctx,
                120.0,
            ) {
                *mutated_label = Some("Edit Open-Edge Overhang");
            }

            if ui
                .checkbox(&mut p.finish_wall_pass, "Dedicated Wall Finishing Pass")
                .changed()
            {
                *mutated_label = Some("Toggle Pocket Finish Wall Pass");
            }
            if p.finish_wall_pass {
                ui.label("Finish Wall Allowance:");
                if edit_param_length(
                    ui,
                    ("pkt_fin_ae", setup_idx, op_idx),
                    &mut p.finish_wall_stepover,
                    eval_ctx,
                    120.0,
                ) {
                    *mutated_label = Some("Edit Pocket Finish Allowance");
                }
            }
        }
        StrategyParams::DrillBore(p) => {
            ui.label("Machining Mode:");
            egui::ComboBox::from_id_salt(("db_mode", setup_idx, op_idx))
                .selected_text(p.mode.label())
                .show_ui(ui, |ui| {
                    for m in HoleMachiningMode::ALL {
                        if ui.selectable_value(&mut p.mode, m, m.label()).changed() {
                            *mutated_label = Some("Change Hole Machining Mode");
                        }
                    }
                });
            if ui
                .checkbox(
                    &mut p.optimize_hole_order,
                    "Nearest-Neighbor + 2-Opt Hole Sorting",
                )
                .changed()
            {
                *mutated_label = Some("Toggle Hole Order Optimization");
            }

            ui.separator();
            ui.strong("Drilling Cycle Settings");
            egui::ComboBox::from_id_salt(("db_cyc", setup_idx, op_idx))
                .selected_text(p.cycle_kind.label())
                .show_ui(ui, |ui| {
                    for ck in DrillCycleKind::ALL {
                        if ui
                            .selectable_value(&mut p.cycle_kind, ck, ck.label())
                            .changed()
                        {
                            *mutated_label = Some("Change Drill Cycle Kind");
                        }
                    }
                });
            if matches!(
                p.cycle_kind,
                DrillCycleKind::ChipBreak | DrillCycleKind::DeepPeck
            ) {
                ui.label("Peck Depth (`Q`):");
                if edit_param_length(
                    ui,
                    ("db_peck", setup_idx, op_idx),
                    &mut p.peck_depth,
                    eval_ctx,
                    110.0,
                ) {
                    *mutated_label = Some("Edit Peck Depth");
                }
            }
            if ui
                .checkbox(
                    &mut p.tip_breakthrough,
                    "Drill Tip Breakthrough Compensation",
                )
                .changed()
            {
                *mutated_label = Some("Toggle Drill Breakthrough");
            }
            if p.tip_breakthrough {
                ui.label("Extra Breakthrough Past Bottom:");
                if edit_param_length(
                    ui,
                    ("db_bt", setup_idx, op_idx),
                    &mut p.breakthrough_extra,
                    eval_ctx,
                    110.0,
                ) {
                    *mutated_label = Some("Edit Breakthrough Extra");
                }
            }

            ui.separator();
            ui.strong("Helical Bore Settings (when D_tool < D_hole)");
            ui.horizontal(|ui| {
                ui.label("Helix Ramp Angle (°):");
                if ui
                    .add(
                        egui::DragValue::new(&mut p.bore_ramp_angle_deg)
                            .range(0.5..=15.0)
                            .speed(0.2),
                    )
                    .changed()
                {
                    *mutated_label = Some("Edit Bore Ramp Angle");
                }
            });
            if ui
                .checkbox(&mut p.bore_finish_pass, "Radial Finish Pass")
                .changed()
            {
                *mutated_label = Some("Toggle Bore Finish Pass");
            }
            if ui
                .checkbox(&mut p.bore_spring_pass, "Repeat Finish Spring Pass")
                .changed()
            {
                *mutated_label = Some("Toggle Bore Spring Pass");
            }
        }
        StrategyParams::ChamferTrace(p) => {
            egui::ComboBox::from_id_salt(("ct_mode", setup_idx, op_idx))
                .selected_text(p.mode.label())
                .show_ui(ui, |ui| {
                    for m in ChamferTraceMode::ALL {
                        if ui.selectable_value(&mut p.mode, m, m.label()).changed() {
                            *mutated_label = Some("Change Chamfer/Trace Mode");
                        }
                    }
                });

            match p.mode {
                ChamferTraceMode::EdgeChamfer => {
                    let mut is_modeled = matches!(p.chamfer_source, ChamferSourceMode::ModeledFace);
                    ui.horizontal(|ui| {
                        ui.label("CAD Source:");
                        if ui
                            .selectable_value(&mut is_modeled, false, "Sharp 90° Edge")
                            .clicked()
                        {
                            p.chamfer_source = ChamferSourceMode::SharpEdge {
                                width: ParamLength::from_mm(0.5),
                            };
                            *mutated_label = Some("Select Sharp Edge Chamfer");
                        }
                        if ui
                            .selectable_value(&mut is_modeled, true, "Modeled Bevel Face")
                            .clicked()
                        {
                            p.chamfer_source = ChamferSourceMode::ModeledFace;
                            *mutated_label = Some("Select Modeled Chamfer Face");
                        }
                    });
                    if let ChamferSourceMode::SharpEdge { width } = &mut p.chamfer_source {
                        ui.label("Chamfer Width (`w`):");
                        if edit_param_length(
                            ui,
                            ("chamf_w", setup_idx, op_idx),
                            width,
                            eval_ctx,
                            110.0,
                        ) {
                            *mutated_label = Some("Edit Chamfer Width");
                        }
                    }
                    ui.label("Tip Offset Past Lower Edge (`d_tip`):");
                    if edit_param_length(
                        ui,
                        ("chamf_tip", setup_idx, op_idx),
                        &mut p.tip_offset,
                        eval_ctx,
                        110.0,
                    ) {
                        *mutated_label = Some("Edit Chamfer Tip Offset");
                    }
                    if ui
                        .checkbox(
                            &mut p.check_adjacent_walls,
                            "Guard against adjacent lower floor gouges",
                        )
                        .changed()
                    {
                        *mutated_label = Some("Toggle Chamfer Floor Guard");
                    }
                }
                ChamferTraceMode::CurveTrace => {
                    egui::ComboBox::from_id_salt(("tr_comp", setup_idx, op_idx))
                        .selected_text(p.trace_compensation.label())
                        .show_ui(ui, |ui| {
                            for c in ContourCompensation::ALL {
                                if ui
                                    .selectable_value(&mut p.trace_compensation, c, c.label())
                                    .changed()
                                {
                                    *mutated_label = Some("Change Trace Compensation");
                                }
                            }
                        });
                    ui.label("Axial Depth Below Curve:");
                    if edit_param_length(
                        ui,
                        ("tr_ax", setup_idx, op_idx),
                        &mut p.trace_axial_offset,
                        eval_ctx,
                        110.0,
                    ) {
                        *mutated_label = Some("Edit Trace Axial Depth");
                    }
                }
            }
        }
    }
}

fn show_linking_tab(
    ui: &mut egui::Ui,
    op: &mut chip_cam::Operation,
    setup_idx: usize,
    op_idx: usize,
    eval_ctx: &EvalContext,
    mutated_label: &mut Option<&'static str>,
) {
    ui.strong("Retract & Stay-Down Policy");
    egui::ComboBox::from_id_salt(("lnk_ret", setup_idx, op_idx))
        .selected_text(op.linking.retract_policy.label())
        .show_ui(ui, |ui| {
            for r in RetractPolicy::ALL {
                if ui
                    .selectable_value(&mut op.linking.retract_policy, r, r.label())
                    .changed()
                {
                    *mutated_label = Some("Change Retract Policy");
                }
            }
        });
    ui.label("Max Stay-Down Link Distance:");
    if edit_param_length(
        ui,
        ("lnk_stay", setup_idx, op_idx),
        &mut op.linking.safe_stay_down_distance,
        eval_ctx,
        120.0,
    ) {
        *mutated_label = Some("Edit Stay-Down Distance");
    }

    ui.separator();
    ui.strong("Ramp & Cavity Entry");
    let cur_lbl = op.linking.entry_mode.label();
    egui::ComboBox::from_id_salt(("lnk_ent", setup_idx, op_idx))
        .selected_text(cur_lbl)
        .show_ui(ui, |ui| {
            for (lbl, mode) in [
                ("Helical Ramp", EntryStrategy::HelicalRamp),
                ("Linear Profile Ramp", EntryStrategy::LinearRamp),
                ("Direct Plunge", EntryStrategy::Plunge),
            ] {
                if ui
                    .selectable_label(op.linking.entry_mode == mode, lbl)
                    .clicked()
                {
                    op.linking.entry_mode = mode;
                    *mutated_label = Some("Change Entry Strategy");
                }
            }
        });
    ui.horizontal(|ui| {
        ui.label("Ramp Angle (°):");
        if ui
            .add(
                egui::DragValue::new(&mut op.linking.ramp_angle_deg)
                    .range(0.5..=30.0)
                    .speed(0.25),
            )
            .changed()
        {
            *mutated_label = Some("Edit Ramp Angle");
        }
    });
    ui.label("Minimum Helix Path Diameter:");
    if edit_param_length(
        ui,
        ("lnk_min_hd", setup_idx, op_idx),
        &mut op.linking.min_ramp_diameter,
        eval_ctx,
        120.0,
    ) {
        *mutated_label = Some("Edit Min Helix Diameter");
    }

    ui.separator();
    ui.strong("Lead-In / Lead-Out & Seam Overlap");
    if ui
        .checkbox(&mut op.linking.lead_in.enabled, "Enable Tangent Lead-In")
        .changed()
    {
        *mutated_label = Some("Toggle Lead-In");
    }
    if op.linking.lead_in.enabled {
        ui.label("Horizontal Lead Arc Radius:");
        if edit_param_length(
            ui,
            ("lnk_lhr", setup_idx, op_idx),
            &mut op.linking.lead_in.horizontal_radius,
            eval_ctx,
            110.0,
        ) {
            *mutated_label = Some("Edit Lead-In Radius");
        }
        ui.horizontal(|ui| {
            ui.label("Horizontal Sweep (°):");
            if ui
                .add(
                    egui::DragValue::new(&mut op.linking.lead_in.horizontal_sweep_deg)
                        .range(0.0..=180.0),
                )
                .changed()
            {
                *mutated_label = Some("Edit Lead-In Sweep");
            }
        });
        ui.label("Linear Tangent Lead Length:");
        if edit_param_length(
            ui,
            ("lnk_llin", setup_idx, op_idx),
            &mut op.linking.lead_in.linear_length,
            eval_ctx,
            110.0,
        ) {
            *mutated_label = Some("Edit Lead-In Linear");
        }
        ui.label("Vertical Lead Arc Radius:");
        if edit_param_length(
            ui,
            ("lnk_lvr", setup_idx, op_idx),
            &mut op.linking.lead_in.vertical_radius,
            eval_ctx,
            110.0,
        ) {
            *mutated_label = Some("Edit Vertical Lead Radius");
        }
        if ui
            .checkbox(
                &mut op.linking.lead_out_same_as_in,
                "Lead-Out Mirrors Lead-In",
            )
            .changed()
        {
            *mutated_label = Some("Toggle Mirror Lead-Out");
        }
    }
    ui.label("Closed Contour Seam Overlap:");
    if edit_param_length(
        ui,
        ("lnk_ovl", setup_idx, op_idx),
        &mut op.linking.seam_overlap,
        eval_ctx,
        110.0,
    ) {
        *mutated_label = Some("Edit Seam Overlap");
    }
}
