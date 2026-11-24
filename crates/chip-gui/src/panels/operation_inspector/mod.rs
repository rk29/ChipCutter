pub mod geometry_tab;
pub mod heights_tab;
pub mod linking_tab;
pub mod passes_tab;
pub mod tool_tab;

use chip_cad::CadDocument;
use chip_cam::{
    ExtractedSelectionSummary, OperationStatus, ResolvedHeights, ResolvedSetup, SurfaceMesh3D,
};
use chip_math::{DVec3, EvalContext};
use chip_project::{ProjectDocument, UndoHistory};
use chip_tools::{ToolId, ToolLibrary, ToolOperationClass};

use crate::{
    panels::{OperationTab, ViewportPickMode, browser::default_pick_mode_for_op},
    theme::ThemeTokens,
    viewport::BoundaryPreviewOverlay,
    widgets::{BannerKind, IconKind, inline_banner, inline_icon, static_card},
};

/// Context passed to [`show_operation_inspector`].
pub struct OperationInspectorCtx<'a> {
    pub project: &'a mut ProjectDocument,
    pub setup_idx: usize,
    pub op_idx: usize,
    pub global_tools: &'a ToolLibrary,
    pub history: &'a mut UndoHistory,
    pub cad_doc: Option<&'a CadDocument>,
    pub resolved_setup: Option<&'a ResolvedSetup>,
    pub surface_mesh: Option<&'a SurfaceMesh3D>,
    pub boundary_preview: Option<&'a BoundaryPreviewOverlay>,
    pub selected_face_normal_wcs: Option<DVec3>,
    pub extracted: Option<Result<&'a ExtractedSelectionSummary, &'a str>>,
    pub tab: &'a mut OperationTab,
    pub pick_mode: &'a mut ViewportPickMode,
    pub lock_chipload: &'a mut bool,
    pub show_z_planes: &'a mut bool,
    pub open_tool_manager: &'a mut bool,
    pub sim_engine: chip_sim::SimEngineKind,
}

/// Renders the standardized 5-tab **Operation Inspector** (`[Tool]`, `[Geometry]`, `[Heights]`, `[Passes]`, `[Linking]`).
pub fn show_operation_inspector(ui: &mut egui::Ui, ctx: OperationInspectorCtx<'_>) {
    let OperationInspectorCtx {
        project,
        setup_idx,
        op_idx,
        global_tools,
        history,
        cad_doc,
        resolved_setup,
        surface_mesh,
        boundary_preview,
        selected_face_normal_wcs,
        extracted,
        tab,
        pick_mode,
        lock_chipload,
        show_z_planes,
        open_tool_manager,
        sim_engine,
    } = ctx;

    if setup_idx >= project.setups.len() || op_idx >= project.setups[setup_idx].operations.len() {
        inline_banner(ui, BannerKind::Info, "No Operation selected.");
        return;
    }

    let display_unit = project.display_units;
    // Snapshot only the single active Operation (never cloning `project.tools` or other Setups/Operations on idle frames!)
    let pre_op = project.setups[setup_idx].operations[op_idx].clone();
    let pre_tools_len = project.tools.len();
    let mut mutated_label: Option<&'static str> = None;

    let op_kind = project.setups[setup_idx].operations[op_idx].kind;
    let op_is_adaptive = project.setups[setup_idx].operations[op_idx].is_adaptive_2d();
    let tok = ThemeTokens::from_ui(ui);
    static_card(ui, None, |ui| {
        ui.horizontal(|ui| {
            inline_icon(
                ui,
                IconKind::for_operation(op_kind, op_is_adaptive),
                tok.accent,
                15.0,
            );
            ui.strong(op_kind.label());
            let name_ref = &mut project.setups[setup_idx].operations[op_idx].name;
            let resp = ui
                .add(
                    egui::TextEdit::singleline(name_ref)
                        .desired_width(ui.available_width().max(80.0)),
                )
                .on_hover_text(
                    "Operation display name shown in the CAM Browser and posted G-code comments",
                );
            if resp.lost_focus() && project.setups[setup_idx].operations[op_idx].name != pre_op.name
            {
                mutated_label = Some("Rename Operation");
            }
        });
        ui.add_space(4.0);
        ui.separator();
        ui.add_space(2.0);
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 4.0;
            if ui
                .selectable_value(tab, OperationTab::Tool, "Tool")
                .on_hover_text("Select cutter, material preset, spindle RPM, surface speed, feedrate, and coolant")
                .clicked()
            {
                *pick_mode = ViewportPickMode::InspectEntity;
            }
            if ui
                .selectable_value(tab, OperationTab::Geometry, "Geometry")
                .on_hover_text("Pick 3D CAD faces, edges, holes, or 2D sketch regions machined by this operation")
                .clicked()
            {
                *pick_mode = default_pick_mode_for_op(setup_idx, op_idx, op_kind, *tab);
            }
            if ui
                .selectable_value(tab, OperationTab::Heights, "Heights")
                .on_hover_text("Configure Clearance, Retract, Feed, Top, and Bottom Z planes for this operation")
                .clicked()
            {
                *pick_mode = ViewportPickMode::InspectEntity;
            }
            if ui
                .selectable_value(tab, OperationTab::Passes, "Passes")
                .on_hover_text("Configure cutting strategy parameters, stepover, stepdown, stock-to-leave, and tabs")
                .clicked()
            {
                *pick_mode = ViewportPickMode::InspectEntity;
            }
            if ui
                .selectable_value(tab, OperationTab::Linking, "Linking")
                .on_hover_text("Configure retract policy, stay-down distance, helical/linear ramps, and tangent lead-in/out")
                .clicked()
            {
                *pick_mode = ViewportPickMode::InspectEntity;
            }
        });
    });

    if let OperationStatus::NeedsRepick { message } = project.setups[setup_idx].operations[op_idx]
        .runtime_status
        .clone()
    {
        static_card(ui, Some("CAD Feature Needs Re-Pick"), |ui| {
            inline_banner(ui, BannerKind::Warn, &message);
            ui.add_space(4.0);
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
                *pick_mode = default_pick_mode_for_op(
                    setup_idx,
                    op_idx,
                    op_kind,
                    OperationTab::Geometry,
                );
                mutated_label = Some("Clear Orphaned Feature for Re-Pick");
            }
        });
    }

    let mut eval_ctx = resolved_setup
        .map(|rs| rs.eval_context(display_unit))
        .unwrap_or_else(|| EvalContext::new(display_unit));

    if let Some(tool) = project.setups[setup_idx].operations[op_idx]
        .tool_id
        .as_ref()
        .and_then(|tid| project.tool(tid).or_else(|| global_tools.tool(tid)))
    {
        tool.populate_eval_context(&mut eval_ctx);
    }

    let (inf_top, inf_bot) = match extracted {
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

    let resolved_heights = resolved_setup.and_then(|rs| {
        let mut h_eval = project.setups[setup_idx].operations[op_idx].heights.clone();
        h_eval.reevaluate(&eval_ctx);
        ResolvedHeights::resolve(
            &h_eval,
            &rs.model_aabb_wcs,
            &rs.stock.aabb_wcs(),
            rs.clearance_z_wcs,
            rs.retract_z_wcs,
            inf_top,
            inf_bot,
        )
        .ok()
    });

    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| match *tab {
            OperationTab::Tool => tool_tab::show_tool_tab(
                ui,
                tool_tab::ToolTabCtx {
                    project,
                    setup_idx,
                    op_idx,
                    global_tools,
                    display_unit,
                    lock_chipload,
                    open_tool_manager,
                    mutated_label: &mut mutated_label,
                },
            ),
            OperationTab::Geometry => {
                let active_tool = project.setups[setup_idx].operations[op_idx]
                    .tool_id
                    .as_ref()
                    .and_then(|tid| project.tool(tid).or_else(|| global_tools.tool(tid)))
                    .cloned();
                geometry_tab::show_geometry_tab(
                    ui,
                    geometry_tab::GeometryTabCtx {
                        op: &mut project.setups[setup_idx].operations[op_idx],
                        sketches: &project.sketches,
                        setup_idx,
                        op_idx,
                        display_unit,
                        eval_ctx: &eval_ctx,
                        cad_doc,
                        resolved_setup,
                        active_tool: active_tool.as_ref(),
                        surface_mesh,
                        resolved_heights: resolved_heights.as_ref(),
                        boundary_preview,
                        extracted,
                        pick_mode,
                        mutated_label: &mut mutated_label,
                    },
                );
            }
            OperationTab::Heights => heights_tab::show_heights_tab(
                ui,
                heights_tab::HeightsTabCtx {
                    op: &mut project.setups[setup_idx].operations[op_idx],
                    setup_idx,
                    op_idx,
                    display_unit,
                    eval_ctx: &eval_ctx,
                    resolved_setup,
                    inf_top,
                    inf_bot,
                    show_z_planes,
                    mutated_label: &mut mutated_label,
                },
            ),
            OperationTab::Passes => {
                let preceding_ops: Vec<(String, String)> = project.setups[setup_idx].operations
                    [..op_idx]
                    .iter()
                    .filter(|o| !o.suppressed)
                    .map(|o| (o.id.clone(), o.name.clone()))
                    .collect();
                let assigned_tool_geom = project.setups[setup_idx].operations[op_idx]
                    .tool_id
                    .as_ref()
                    .and_then(|tid| project.tool(tid).or_else(|| global_tools.tool(tid)))
                    .map(|t| t.geometry.clone());
                let tool_feed_preview = project.setups[setup_idx].operations[op_idx]
                    .tool_id
                    .as_ref()
                    .and_then(|tid| project.tool(tid).or_else(|| global_tools.tool(tid)))
                    .map(|t| {
                        let eff = project.setups[setup_idx].operations[op_idx]
                            .effective_cutting_params(t);
                        (t.geometry.cutting_diameter_mm(), eff.cutting_feed_mm_min)
                    });
                let resolved_thread_depth_mm = resolved_heights
                    .as_ref()
                    .map(|rh| {
                        let axial_stock = project.setups[setup_idx].operations[op_idx]
                            .passes
                            .stock_to_leave_axial
                            .value_mm;
                        rh.top_z_wcs - (rh.bottom_z_wcs + axial_stock)
                    })
                    .or_else(|| {
                        extracted.and_then(|res| {
                            res.ok().map(|sum| {
                                let axial_stock = project.setups[setup_idx].operations[op_idx]
                                    .passes
                                    .stock_to_leave_axial
                                    .value_mm;
                                sum.inferred_top_z_wcs - (sum.inferred_bottom_z_wcs + axial_stock)
                            })
                        })
                    });
                let mut clearing_tool_candidates: Vec<(ToolId, String, Option<String>)> =
                    Vec::new();
                for t in project.tools.iter().chain(global_tools.tools.iter()) {
                    if t.geometry
                        .supports_operation(ToolOperationClass::VCarveFlatClearance)
                        && !clearing_tool_candidates
                            .iter()
                            .any(|(id, _, _)| *id == t.id)
                    {
                        let preset_id = t.presets.first().map(|p| p.id.clone());
                        clearing_tool_candidates.push((
                            t.id.clone(),
                            format!(
                                "T{} — {} (Ø{:.2} mm)",
                                t.tool_number,
                                t.name,
                                t.geometry.cutting_diameter_mm()
                            ),
                            preset_id,
                        ));
                    }
                }
                passes_tab::show_passes_tab(
                    ui,
                    passes_tab::PassesTabCtx {
                        op: &mut project.setups[setup_idx].operations[op_idx],
                        has_3d_model: cad_doc.is_some_and(CadDocument::has_3d_geometry),
                        preceding_ops: &preceding_ops,
                        clearing_tool_candidates: &clearing_tool_candidates,
                        assigned_tool_geom: assigned_tool_geom.as_ref(),
                        tool_feed_preview,
                        resolved_thread_depth_mm,
                        sim_engine,
                        display_unit,
                        setup_idx,
                        op_idx,
                        eval_ctx: &eval_ctx,
                        resolved_rotary: resolved_setup.and_then(|rs| rs.rotary),
                        selected_face_normal_wcs,
                        pick_mode,
                        mutated_label: &mut mutated_label,
                    },
                );
            }
            OperationTab::Linking => linking_tab::show_linking_tab(
                ui,
                &mut project.setups[setup_idx].operations[op_idx],
                setup_idx,
                op_idx,
                &eval_ctx,
                &mut mutated_label,
            ),
        });

    if let Some(action) = mutated_label {
        let maybe_embed = project.setups[setup_idx].operations[op_idx]
            .clearing_tool_id()
            .filter(|ctid| project.tool(ctid).is_none())
            .and_then(|ctid| global_tools.tool(ctid))
            .cloned();
        if let Some(gt) = maybe_embed {
            project.ensure_tool_embedded(&gt);
        }
        let mut pre_doc = project.clone();
        pre_doc.tools.truncate(pre_tools_len);
        pre_doc.setups[setup_idx].operations[op_idx] = pre_op;
        history.record_before_edit(action, None, &pre_doc);
        project.setups[setup_idx].operations[op_idx].mark_dirty();
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cam::{
        AdaptiveClearingParams, MachiningBoundaryMode, Operation, OperationKind,
        PocketStrategyKind, ResolvedRotaryConfig, ResolvedStockGeometry, RotaryAxialBoundaryMode,
        RotaryAxisKind, RotaryDirection, RotaryDriveMode, RotaryUnwindMode, Setup, StockAxis,
        StockConfig, StrategyParams, WorkOffsetSlot,
    };
    use chip_math::{Aabb3, DVec3, DisplayUnit, ParamLength};
    use chip_project::{ProjectDocument, UndoHistory};
    use chip_tools::{ToolLibrary, default_starter_tool_library};

    use super::*;

    fn collect_shape_texts(shapes: &[egui::epaint::ClippedShape], out: &mut Vec<String>) {
        fn visit(shape: &egui::Shape, out: &mut Vec<String>) {
            match shape {
                egui::Shape::Text(t) => out.push(t.galley.text().to_string()),
                egui::Shape::Vec(v) => {
                    for s in v {
                        visit(s, out);
                    }
                }
                _ => {}
            }
        }
        for cs in shapes {
            visit(&cs.shape, out);
        }
    }

    fn make_resolved_box_setup() -> ResolvedSetup {
        let doc = CadDocument::empty();
        let mut setup = Setup::new("s1", "Setup 1", WorkOffsetSlot::G54);
        setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(120.0),
            depth_y: ParamLength::from_mm(120.0),
            height_z: ParamLength::from_mm(20.0),
            xy_align: Default::default(),
            z_align: Default::default(),
        };
        setup
            .resolve_with_sketches(&doc, &[], DisplayUnit::Millimeter)
            .unwrap()
    }

    fn make_resolved_rotary_setup() -> ResolvedSetup {
        let mut rs = make_resolved_box_setup();
        rs.model_aabb_wcs = Aabb3::new(
            DVec3::new(-30.0, -18.0, -18.0),
            DVec3::new(30.0, 18.0, 18.0),
        );
        rs.stock = ResolvedStockGeometry::Cylinder {
            axis: StockAxis::X,
            center_wcs: DVec3::ZERO,
            radius_mm: 21.0,
            min_axial_mm: -40.0,
            max_axial_mm: 40.0,
            aabb_wcs: Aabb3::new(
                DVec3::new(-40.0, -21.0, -21.0),
                DVec3::new(40.0, 21.0, 21.0),
            ),
        };
        rs.rotary = Some(ResolvedRotaryConfig {
            mode: RotaryDriveMode::Simultaneous4Axis,
            axis: RotaryAxisKind::XAxis,
            direction: RotaryDirection::RightHandCcw,
            center_wcs: DVec3::ZERO,
            stock_outer_radius_mm: 21.0,
            stock_u_min: -40.0,
            stock_u_max: 40.0,
            clearance_radius_mm: 33.0,
            chuck_u_min: -60.0,
            chuck_u_max: -40.0,
            chuck_radius_mm: 32.5,
            chuck_end_positive: false,
            axis_letter: 'A',
            unwind_mode: RotaryUnwindMode::ContinuousMultiTurn,
        });
        rs
    }

    fn render_inspector_texts(
        project: &mut ProjectDocument,
        resolved_setup: &ResolvedSetup,
        tools: &ToolLibrary,
        tab: OperationTab,
    ) -> Vec<String> {
        let ctx = egui::Context::default();
        let mut history = UndoHistory::default();
        let mut cur_tab = tab;
        let mut pick_mode = ViewportPickMode::InspectEntity;
        let mut lock_chipload = true;
        let mut show_z_planes = false;
        let mut open_tool_manager = false;

        let mut out = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(420.0, 1400.0),
                )),
                ..Default::default()
            },
            |ui| {
                egui::CentralPanel::default().show(ui, |ui| {
                    show_operation_inspector(
                        ui,
                        OperationInspectorCtx {
                            project,
                            setup_idx: 0,
                            op_idx: 0,
                            global_tools: tools,
                            history: &mut history,
                            cad_doc: None,
                            resolved_setup: Some(resolved_setup),
                            surface_mesh: None,
                            boundary_preview: None,
                            selected_face_normal_wcs: None,
                            extracted: None,
                            tab: &mut cur_tab,
                            pick_mode: &mut pick_mode,
                            lock_chipload: &mut lock_chipload,
                            show_z_planes: &mut show_z_planes,
                            open_tool_manager: &mut open_tool_manager,
                            sim_engine: chip_sim::SimEngineKind::TriDexel,
                        },
                    );
                });
            },
        );
        out.textures_delta.clear();
        let mut texts = Vec::new();
        collect_shape_texts(&out.shapes, &mut texts);
        texts
    }

    #[test]
    fn geometry_tab_shows_cutting_area_cards_and_passes_removes_duplicates() {
        let tools = default_starter_tool_library();
        let tool_id = tools.tools[0].id.clone();
        let rs_box = make_resolved_box_setup();
        let rs_rot = make_resolved_rotary_setup();

        // 1. 3-Axis + Undercut3D + Deburr3D + Facing -> Machining Boundary in [Geometry], NOT in [Passes]
        for kind in [
            OperationKind::Adaptive3D,
            OperationKind::Waterline3D,
            OperationKind::Parallel3D,
            OperationKind::Scallop3D,
            OperationKind::Pencil3D,
            OperationKind::Undercut3D,
            OperationKind::Deburr3D,
            OperationKind::Facing,
        ] {
            let mut proj = ProjectDocument::new("test");
            proj.tools = tools.tools.clone();
            let mut op = Operation::new("op1", "Op", kind);
            op.tool_id = Some(tool_id.clone());
            proj.setups[0].operations = vec![op];

            let geo_texts =
                render_inspector_texts(&mut proj, &rs_box, &tools, OperationTab::Geometry);
            let geo_joined = geo_texts.join("\n");
            assert!(
                geo_joined.contains("Cutting Area / Machining Boundary"),
                "{kind:?} [Geometry] must render Cutting Area / Machining Boundary card"
            );
            assert!(
                geo_joined.contains("Boundary Source")
                    && geo_joined.contains("Tool Containment")
                    && geo_joined.contains("Boundary Offset"),
                "{kind:?} [Geometry] must render Boundary Source, Tool Containment, and Boundary Offset"
            );
            if kind == OperationKind::Deburr3D {
                assert!(
                    geo_joined.contains("Auto-Detect Edges"),
                    "Deburr3D [Geometry] must note Auto-Detect Edges constraint"
                );
            }

            let pass_texts =
                render_inspector_texts(&mut proj, &rs_box, &tools, OperationTab::Passes);
            let pass_joined = pass_texts.join("\n");
            assert!(
                !pass_joined.contains("3D Machining Boundary")
                    && !pass_joined.contains("Facing Machining Boundary")
                    && !pass_joined.contains("Boundary Source")
                    && !pass_joined.contains("Tool Containment"),
                "{kind:?} [Passes] must not duplicate Machining Boundary controls"
            );
        }

        // 2. 4th-Axis Rotary -> Rotary Cutting Area (-U / +U) in [Geometry], NOT in [Passes]
        for kind in [OperationKind::Rotary2Linear, OperationKind::Rotary4AxisSim] {
            let mut proj = ProjectDocument::new("test_rot");
            proj.tools = tools.tools.clone();
            let mut op = Operation::new("op_rot", "Rotary", kind);
            op.tool_id = Some(tool_id.clone());
            proj.setups[0].operations = vec![op];

            let geo_texts =
                render_inspector_texts(&mut proj, &rs_rot, &tools, OperationTab::Geometry);
            let geo_joined = geo_texts.join("\n");
            assert!(
                geo_joined.contains("4th-Axis Rotary Cutting Area (-U / +U)"),
                "{kind:?} [Geometry] must render rotary cutting area card"
            );
            assert!(
                geo_joined.contains("Axial Source")
                    && geo_joined.contains("Axial Containment")
                    && geo_joined.contains("Start Margin (-U)")
                    && geo_joined.contains("End Margin (+U)")
                    && geo_joined.contains("Full 360°")
                    && geo_joined.contains("Effective U:")
                    && geo_joined.contains("Chuck Guard:"),
                "{kind:?} [Geometry] must render axial bounds, presets, and live Effective U / Chuck Guard readout"
            );
            assert!(
                !geo_joined.contains("Selected 2D Contours' in Passes"),
                "{kind:?} [Geometry] must not show misleading generic 3D banner"
            );

            let pass_texts =
                render_inspector_texts(&mut proj, &rs_rot, &tools, OperationTab::Passes);
            let pass_joined = pass_texts.join("\n");
            assert!(
                !pass_joined.contains("Start Margin (-U)")
                    && !pass_joined.contains("End Margin (+U)")
                    && !pass_joined.contains("Start Angle (°)")
                    && !pass_joined.contains("End Angle (°)"),
                "{kind:?} [Passes] must not duplicate rotary axial/angular boundary fields"
            );
        }

        // 3. Pocket2D & Adaptive2D -> Pocket Area Mode + Open Outer Boundary in [Geometry], NOT in [Passes]
        for is_adaptive in [false, true] {
            let mut proj = ProjectDocument::new("test_pkt");
            proj.tools = tools.tools.clone();
            let mut op = Operation::new("op_pkt", "Pocket", OperationKind::Pocket2D);
            op.tool_id = Some(tool_id.clone());
            if is_adaptive && let StrategyParams::Pocket2D(p) = op.strategy_mut() {
                p.strategy = PocketStrategyKind::Adaptive(AdaptiveClearingParams::default());
            }
            proj.setups[0].operations = vec![op];

            let geo_joined =
                render_inspector_texts(&mut proj, &rs_box, &tools, OperationTab::Geometry)
                    .join("\n");
            assert!(
                geo_joined.contains("Pocket Area Mode")
                    && geo_joined.contains("Open Outer Boundary"),
                "Pocket2D (adaptive={is_adaptive}) [Geometry] must show Pocket Area Mode and Open Outer Boundary"
            );

            let pass_joined =
                render_inspector_texts(&mut proj, &rs_box, &tools, OperationTab::Passes).join("\n");
            assert!(
                !pass_joined.contains("Boundary Mode")
                    && !pass_joined.contains("Treat Outer Boundary as Open"),
                "Pocket2D (adaptive={is_adaptive}) [Passes] must not duplicate pocket boundary controls"
            );
        }

        // 4. Contour2D -> Clip to Stock Bounds & Open Chain Extension in [Geometry], NOT in [Passes]
        {
            let mut proj = ProjectDocument::new("test_cnt");
            proj.tools = tools.tools.clone();
            let mut op = Operation::new("op_cnt", "Contour", OperationKind::Contour2D);
            op.tool_id = Some(tool_id);
            proj.setups[0].operations = vec![op];

            let geo_joined =
                render_inspector_texts(&mut proj, &rs_box, &tools, OperationTab::Geometry)
                    .join("\n");
            assert!(
                geo_joined.contains("Clip to Stock Bounds")
                    && geo_joined.contains("Open Chain Extension"),
                "Contour2D [Geometry] must show Stock Clip and Open Chain Extension"
            );

            let pass_joined =
                render_inspector_texts(&mut proj, &rs_box, &tools, OperationTab::Passes).join("\n");
            assert!(
                !pass_joined.contains("Clip to Stock XY Bounds")
                    && !pass_joined.contains("Open Extension"),
                "Contour2D [Passes] must not duplicate Stock Clip / Open Extension"
            );
        }
    }

    #[test]
    fn pick_card_default_open_collapses_on_automatic_boundaries() {
        // 3D surface & Facing collapse when StockBox or ModelSilhouette, open on SelectedRegions
        for kind in [
            OperationKind::Facing,
            OperationKind::Adaptive3D,
            OperationKind::Waterline3D,
            OperationKind::Parallel3D,
            OperationKind::Scallop3D,
            OperationKind::Pencil3D,
            OperationKind::Undercut3D,
        ] {
            let mut op = Operation::new("op", "Test", kind);
            op.machining_boundary_mut().unwrap().boundary_mode = MachiningBoundaryMode::StockBox;
            assert!(!geometry_tab::pick_card_default_open(&op));
            op.machining_boundary_mut().unwrap().boundary_mode =
                MachiningBoundaryMode::ModelSilhouette;
            assert!(!geometry_tab::pick_card_default_open(&op));
            op.machining_boundary_mut().unwrap().boundary_mode =
                MachiningBoundaryMode::SelectedRegions;
            assert!(geometry_tab::pick_card_default_open(&op));
        }

        // Rotary collapses unless SelectedGeometry is active
        for kind in [OperationKind::Rotary2Linear, OperationKind::Rotary4AxisSim] {
            let mut op = Operation::new("op_r", "Rotary", kind);
            op.rotary_axial_boundary_mut().unwrap().axial_boundary_mode =
                RotaryAxialBoundaryMode::ModelBounds;
            assert!(!geometry_tab::pick_card_default_open(&op));
            op.rotary_axial_boundary_mut().unwrap().axial_boundary_mode =
                RotaryAxialBoundaryMode::ExplicitRange;
            assert!(!geometry_tab::pick_card_default_open(&op));
            op.rotary_axial_boundary_mut().unwrap().axial_boundary_mode =
                RotaryAxialBoundaryMode::SelectedGeometry;
            assert!(geometry_tab::pick_card_default_open(&op));
        }
    }

    #[test]
    fn geometry_tab_mutation_records_undo_and_marks_operation_dirty() {
        let tools = default_starter_tool_library();
        let rs_rot = make_resolved_rotary_setup();
        let mut proj = ProjectDocument::new("undo_test");
        proj.tools = tools.tools.clone();
        let mut op = Operation::new("op_rot", "Rotary Finish", OperationKind::Rotary4AxisSim);
        op.tool_id = Some(tools.tools[0].id.clone());
        op.runtime_status = OperationStatus::Clean;
        proj.setups[0].operations = vec![op];

        let ctx = egui::Context::default();
        let mut history = UndoHistory::default();
        let mut tab = OperationTab::Geometry;
        let mut pick_mode = ViewportPickMode::InspectEntity;
        let mut lock_chipload = true;
        let mut show_z_planes = false;
        let mut open_tool_manager = false;

        // First frame: initialize layout, then probe exact screen center of the "0–180°" preset chip
        let mut chip_pos: Option<egui::Pos2> = None;
        let mut init_out = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(420.0, 900.0),
                )),
                ..Default::default()
            },
            |ui| {
                egui::CentralPanel::default().show(ui, |ui| {
                    show_operation_inspector(
                        ui,
                        OperationInspectorCtx {
                            project: &mut proj,
                            setup_idx: 0,
                            op_idx: 0,
                            global_tools: &tools,
                            history: &mut history,
                            cad_doc: None,
                            resolved_setup: Some(&rs_rot),
                            surface_mesh: None,
                            boundary_preview: None,
                            selected_face_normal_wcs: None,
                            extracted: None,
                            tab: &mut tab,
                            pick_mode: &mut pick_mode,
                            lock_chipload: &mut lock_chipload,
                            show_z_planes: &mut show_z_planes,
                            open_tool_manager: &mut open_tool_manager,
                            sim_engine: chip_sim::SimEngineKind::TriDexel,
                        },
                    );
                });
            },
        );
        init_out.textures_delta.clear();
        // Locate "0–180°" text position in painted shapes
        let mut probe = ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(420.0, 900.0),
                )),
                ..Default::default()
            },
            |ui| {
                egui::CentralPanel::default().show(ui, |ui| {
                    show_operation_inspector(
                        ui,
                        OperationInspectorCtx {
                            project: &mut proj,
                            setup_idx: 0,
                            op_idx: 0,
                            global_tools: &tools,
                            history: &mut history,
                            cad_doc: None,
                            resolved_setup: Some(&rs_rot),
                            surface_mesh: None,
                            boundary_preview: None,
                            selected_face_normal_wcs: None,
                            extracted: None,
                            tab: &mut tab,
                            pick_mode: &mut pick_mode,
                            lock_chipload: &mut lock_chipload,
                            show_z_planes: &mut show_z_planes,
                            open_tool_manager: &mut open_tool_manager,
                            sim_engine: chip_sim::SimEngineKind::TriDexel,
                        },
                    );
                });
            },
        );
        probe.textures_delta.clear();
        fn find_text_center(shape: &egui::Shape, needle: &str) -> Option<egui::Pos2> {
            match shape {
                egui::Shape::Text(t) if t.galley.text() == needle => {
                    Some(t.pos + t.galley.size() * 0.5)
                }
                egui::Shape::Vec(v) => v.iter().find_map(|s| find_text_center(s, needle)),
                _ => None,
            }
        }
        for cs in &probe.shapes {
            if let Some(p) = find_text_center(&cs.shape, "0–180°") {
                chip_pos = Some(p);
                break;
            }
        }
        let click_pt = chip_pos.expect("0–180° preset chip must be rendered");

        // Second frame: click the "0–180°" preset chip
        let click_input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(420.0, 900.0),
            )),
            events: vec![
                egui::Event::PointerMoved(click_pt),
                egui::Event::PointerButton {
                    pos: click_pt,
                    button: egui::PointerButton::Primary,
                    pressed: true,
                    modifiers: egui::Modifiers::NONE,
                },
                egui::Event::PointerButton {
                    pos: click_pt,
                    button: egui::PointerButton::Primary,
                    pressed: false,
                    modifiers: egui::Modifiers::NONE,
                },
            ],
            ..Default::default()
        };
        let mut click_out = ctx.run_ui(click_input, |ui| {
            egui::CentralPanel::default().show(ui, |ui| {
                show_operation_inspector(
                    ui,
                    OperationInspectorCtx {
                        project: &mut proj,
                        setup_idx: 0,
                        op_idx: 0,
                        global_tools: &tools,
                        history: &mut history,
                        cad_doc: None,
                        resolved_setup: Some(&rs_rot),
                        surface_mesh: None,
                        boundary_preview: None,
                        selected_face_normal_wcs: None,
                        extracted: None,
                        tab: &mut tab,
                        pick_mode: &mut pick_mode,
                        lock_chipload: &mut lock_chipload,
                        show_z_planes: &mut show_z_planes,
                        open_tool_manager: &mut open_tool_manager,
                        sim_engine: chip_sim::SimEngineKind::TriDexel,
                    },
                );
            });
        });
        click_out.textures_delta.clear();

        let rb = proj.setups[0].operations[0]
            .rotary_axial_boundary()
            .unwrap();
        assert!((rb.start_angle_deg - 0.0).abs() < 1e-6);
        assert!((rb.end_angle_deg - 180.0).abs() < 1e-6);
        assert_eq!(
            proj.setups[0].operations[0].runtime_status,
            OperationStatus::Dirty,
            "Geometry tab boundary edit must mark operation Dirty"
        );
        assert!(
            history.can_undo(),
            "Geometry tab boundary edit must record an Undo step"
        );

        // Undo restores the original 360° sector
        let undone = history.undo(&mut proj);
        assert!(undone.is_some());
        let rb_restored = proj.setups[0].operations[0]
            .rotary_axial_boundary()
            .unwrap();
        assert!((rb_restored.end_angle_deg - 360.0).abs() < 1e-6);
    }
}
