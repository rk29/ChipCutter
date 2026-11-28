#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::*;
use chip_cam::{FixtureBody, Operation, OperationKind, RestMachiningMode, Setup, WorkOffsetSlot};
use chip_math::{EvalContext, ParamLength};
use chip_project::ModelEntry;
use chip_sketch::{SketchEntity, SketchEntityEntry, SketchLayer};
use egui::{Pos2, Rect};

use crate::{
    panels::{
        keepout_inspector::{KeepOutInspectorCtx, show_keepout_inspector},
        model_inspector::{ModelInspectorCtx, show_model_inspector},
        operation_inspector::{OperationInspectorCtx, show_operation_inspector},
        project_properties::{ProjectPropertiesCtx, show_project_inspector},
        project_tool_inspector::{ProjectToolInspectorCtx, show_project_tool_inspector},
        setup_inspector::{FixtureSafetySummary, SetupInspectorCtx, show_setup_inspector},
        sketch_inspector::{SketchInspectorCtx, show_sketch_inspector},
    },
    theme::{AppTheme, apply_theme},
};

fn collect_texts(shape: &egui::Shape, out: &mut Vec<String>) {
    match shape {
        egui::Shape::Text(t) => out.push(t.galley.text().to_string()),
        egui::Shape::Vec(v) => {
            for s in v {
                collect_texts(s, out);
            }
        }
        _ => {}
    }
}

fn find_text_rect(shapes: &[egui::epaint::ClippedShape], needle: &str) -> Option<Rect> {
    fn visit(shape: &egui::Shape, needle: &str) -> Option<Rect> {
        match shape {
            egui::Shape::Text(t) if t.galley.text() == needle => Some(t.visual_bounding_rect()),
            egui::Shape::Vec(v) => v.iter().find_map(|s| visit(s, needle)),
            _ => None,
        }
    }
    shapes.iter().find_map(|cs| visit(&cs.shape, needle))
}

fn find_text_rect_containing(shapes: &[egui::epaint::ClippedShape], needle: &str) -> Option<Rect> {
    fn visit(shape: &egui::Shape, needle: &str) -> Option<Rect> {
        match shape {
            egui::Shape::Text(t) if t.galley.text().contains(needle) => {
                Some(t.visual_bounding_rect())
            }
            egui::Shape::Vec(v) => v.iter().find_map(|s| visit(s, needle)),
            _ => None,
        }
    }
    shapes.iter().find_map(|cs| visit(&cs.shape, needle))
}

fn click_btn(pos: Pos2, button: egui::PointerButton) -> Vec<egui::Event> {
    vec![
        egui::Event::PointerMoved(pos),
        egui::Event::PointerButton {
            pos,
            button,
            pressed: true,
            modifiers: egui::Modifiers::NONE,
        },
        egui::Event::PointerButton {
            pos,
            button,
            pressed: false,
            modifiers: egui::Modifiers::NONE,
        },
    ]
}

fn build_rich_test_project() -> ProjectDocument {
    let mut project = ProjectDocument::new("Aluminum Manifold Rev B");
    project
        .models
        .push(ModelEntry::new("m_step", "parts/manifold.step", "sha_step"));
    project
        .models
        .push(ModelEntry::new("m_mesh", "parts/relief.stl", "sha_stl"));
    project.models.push(ModelEntry::new_fixture(
        "m_fix",
        "fixtures/kurt_vise.step",
        "sha_fix",
    ));

    let mut sk = SketchLayer::new("sk_1", "Mounting Holes", "setup_1");
    sk.entities.push(SketchEntityEntry {
        id: "ent_1".to_string(),
        name: "Outer Pocket".to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::new_circle(
            ParamLength::from_mm(0.0),
            ParamLength::from_mm(0.0),
            ParamLength::from_mm(20.0),
        ),
    });
    project.sketches.push(sk);

    project
        .avoid_regions
        .push(SharedAvoidRegion::new("avoid_1", "Strap Clamps"));

    let lib = chip_tools::default_starter_tool_library();
    if let Some(t) = lib.tools.first() {
        project.ensure_tool_embedded(t);
    }

    project.setups[0]
        .fixtures
        .push(FixtureBody::default_vise("fix_1", "Kurt 4in Vise"));
    let mut op = Operation::new("op_1", "Adaptive Rough", OperationKind::Adaptive3D);
    op.tool_id = project.tools.first().map(|t| t.id.clone());
    project.setups[0].operations.push(op);
    project
}

struct BrowserTestHarness {
    ctx: egui::Context,
    inspector_ctx: egui::Context,
    project: ProjectDocument,
    global_tools: ToolLibrary,
    machines: Vec<chip_machines::MachineDefinition>,
    history: UndoHistory,
    cache: ToolpathCache,
    active_setup_idx: usize,
    focus: InspectorFocus,
    setup_tab: SetupTab,
    op_tab: OperationTab,
    pick_mode: ViewportPickMode,
    pick_target: crate::panels::GeometryPickTarget,
    hidden: std::collections::HashSet<String>,
    editing_avoid: Option<String>,
    sel_ent: Option<usize>,
    sel_fix: Option<usize>,
    time: f64,
}

impl BrowserTestHarness {
    fn new(project: ProjectDocument) -> Self {
        Self {
            ctx: egui::Context::default(),
            inspector_ctx: egui::Context::default(),
            project,
            global_tools: chip_tools::default_starter_tool_library(),
            machines: chip_machines::builtin_machine_definitions(),
            history: UndoHistory::default(),
            cache: ToolpathCache::new(4),
            active_setup_idx: 0,
            focus: InspectorFocus::Setup { setup_idx: 0 },
            setup_tab: SetupTab::Wcs,
            op_tab: OperationTab::Geometry,
            pick_mode: ViewportPickMode::InspectEntity,
            pick_target: crate::panels::GeometryPickTarget::CuttingArea,
            hidden: std::collections::HashSet::new(),
            editing_avoid: None,
            sel_ent: None,
            sel_fix: None,
            time: 1.0,
        }
    }

    fn step(
        &mut self,
        events: Vec<egui::Event>,
    ) -> (Vec<egui::epaint::ClippedShape>, Option<BrowserAction>) {
        self.time += 0.1;
        let mut action = None;
        let mut out = self.ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(360.0, 960.0))),
                time: Some(self.time),
                events,
                ..Default::default()
            },
            |ui| {
                egui::CentralPanel::default().show(ui, |ui| {
                    action = show_cam_browser(
                        ui,
                        CamBrowserCtx {
                            project: &mut self.project,
                            global_tools: &self.global_tools,
                            active_machine: None,
                            resolved_active_setup: None,
                            history: &mut self.history,
                            toolpath_cache: &self.cache,
                            cad_doc: None,
                            cad_revision: 0,
                            active_setup_idx: &mut self.active_setup_idx,
                            focus: &mut self.focus,
                            setup_tab: &mut self.setup_tab,
                            op_tab: &mut self.op_tab,
                            pick_mode: &mut self.pick_mode,
                            hidden_operation_ids: &mut self.hidden,
                            editing_shared_avoid_id: &mut self.editing_avoid,
                            selected_sketch_entity_idx: &mut self.sel_ent,
                            selected_fixture_idx: &mut self.sel_fix,
                        },
                    );
                });
            },
        );
        out.textures_delta.clear();
        (out.shapes, action)
    }

    fn render_active_inspector(&mut self) -> Vec<String> {
        self.time += 0.1;
        let eval_ctx = EvalContext::new(self.project.display_units);
        let mut out = self.inspector_ctx.run_ui(
            egui::RawInput {
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(420.0, 900.0))),
                time: Some(self.time),
                ..Default::default()
            },
            |ui| {
                egui::CentralPanel::default().show(ui, |ui| match self.focus {
                    InspectorFocus::Project => {
                        let _ = show_project_inspector(
                            ui,
                            ProjectPropertiesCtx {
                                project: &self.project,
                                project_file_path: None,
                                machines: &self.machines,
                            },
                        );
                    }
                    InspectorFocus::Model { model_idx } => {
                        let mut lock = true;
                        let _ = show_model_inspector(
                            ui,
                            ModelInspectorCtx {
                                project: &mut self.project,
                                model_idx,
                                history: &mut self.history,
                                cad_doc: None,
                                lock_uniform_scale: &mut lock,
                                pick_mode: &mut self.pick_mode,
                                focus: &mut self.focus,
                            },
                        );
                    }
                    InspectorFocus::Sketch { sketch_idx } => {
                        let mut snap = None;
                        let mut start_xy = None;
                        let _ = show_sketch_inspector(
                            ui,
                            SketchInspectorCtx {
                                project: &mut self.project,
                                sketch_idx,
                                active_setup_idx: self.active_setup_idx,
                                selected_entity_idx: &mut self.sel_ent,
                                snap_grid_mm: &mut snap,
                                author_start_xy: &mut start_xy,
                                history: &mut self.history,
                                pick_mode: &mut self.pick_mode,
                                eval_ctx: &eval_ctx,
                                has_cad_model: false,
                                project_file: None,
                            },
                        );
                    }
                    InspectorFocus::KeepOutRegion { region_idx } => {
                        let _ = show_keepout_inspector(
                            ui,
                            KeepOutInspectorCtx {
                                project: &mut self.project,
                                region_idx,
                                active_setup_idx: self.active_setup_idx,
                                history: &mut self.history,
                                eval_ctx: &eval_ctx,
                                pick_mode: &mut self.pick_mode,
                                editing_shared_avoid_id: &mut self.editing_avoid,
                                focus: &mut self.focus,
                            },
                        );
                    }
                    InspectorFocus::ProjectTool { tool_idx } => {
                        let _ = show_project_tool_inspector(
                            ui,
                            ProjectToolInspectorCtx {
                                project: &self.project,
                                global_tools: &self.global_tools,
                                tool_idx,
                            },
                        );
                    }
                    InspectorFocus::Setup { setup_idx } => {
                        let mut show_fix = true;
                        let mut show_dots = true;
                        let mut show_z = true;
                        let mut open_mm = false;
                        let mut open_post = None;
                        let mut m_ch = false;
                        let mut sw_m = None;
                        let mut imp_f = false;
                        let mut fj = false;
                        let mut sj = false;
                        let _ = show_setup_inspector(
                            ui,
                            SetupInspectorCtx {
                                project: &mut self.project,
                                setup_idx,
                                history: &mut self.history,
                                project_dir: None,
                                cad_doc: None,
                                resolved: None,
                                resolved_fixtures: &[],
                                fixture_safety: FixtureSafetySummary::default(),
                                tab: &mut self.setup_tab,
                                pick_mode: &mut self.pick_mode,
                                selected_fixture_idx: &mut self.sel_fix,
                                show_fixtures: &mut show_fix,
                                show_27pt_dots: &mut show_dots,
                                show_z_planes: &mut show_z,
                                machines: &self.machines,
                                open_machine_manager: &mut open_mm,
                                open_post_inspector: &mut open_post,
                                machine_changed: &mut m_ch,
                                switch_machine_id: &mut sw_m,
                                import_fixture_requested: &mut imp_f,
                                open_flip_jig_wizard: &mut fj,
                                open_soft_jaw_wizard: &mut sj,
                            },
                        );
                    }
                    InspectorFocus::Operation { setup_idx, op_idx } => {
                        let mut lock_cl = true;
                        let mut show_z = true;
                        let mut show_reach = false;
                        let mut auto_rot = true;
                        let mut open_tm = false;
                        let _ = show_operation_inspector(
                            ui,
                            OperationInspectorCtx {
                                project: &mut self.project,
                                setup_idx,
                                op_idx,
                                global_tools: &self.global_tools,
                                active_machine: None,
                                history: &mut self.history,
                                cad_doc: None,
                                resolved_setup: None,
                                surface_mesh: None,
                                boundary_preview: None,
                                selected_face_normal_wcs: None,
                                extracted: None,
                                tab: &mut self.op_tab,
                                pick_mode: &mut self.pick_mode,
                                pick_target: &mut self.pick_target,
                                editing_shared_avoid_id: &mut self.editing_avoid,
                                lock_chipload: &mut lock_cl,
                                show_z_planes: &mut show_z,
                                show_reach_heatmap: &mut show_reach,
                                force_expand_machinability: false,
                                auto_rotate_indexed_op: &mut auto_rot,
                                open_tool_manager: &mut open_tm,
                                sim_engine: chip_sim::SimEngineKind::Heightmap,
                            },
                        );
                    }
                    InspectorFocus::CadEntity => {}
                });
            },
        );
        out.textures_delta.clear();
        let mut texts = Vec::new();
        for cs in &out.shapes {
            collect_texts(&cs.shape, &mut texts);
        }
        texts
    }
}

#[test]
fn move_duplicate_and_delete_propagate_rest_machining_dirty_state() {
    let mut setup = Setup::new("s1", "Setup 1", WorkOffsetSlot::G54);
    let mut op1 = Operation::new("op_1", "Rough 1", OperationKind::Pocket2D);
    op1.runtime_status = OperationStatus::Clean;
    let mut op2 = Operation::new("op_2", "Rough 2", OperationKind::Pocket2D);
    op2.runtime_status = OperationStatus::Clean;
    let mut op3 = Operation::new("op_3", "Rest Finish", OperationKind::Contour2D);
    op3.passes.rest_machining.mode = RestMachiningMode::FromPrecedingOperations {
        source_op_ids: Vec::new(),
    };
    op3.runtime_status = OperationStatus::Clean;

    setup.operations = vec![op1, op2, op3];

    assert_eq!(move_operation_to_slot(&mut setup, 1, 1), None);
    assert_eq!(move_operation_to_slot(&mut setup, 1, 2), None);
    assert_eq!(setup.operations[2].runtime_status, OperationStatus::Clean);

    let dst = move_operation_to_slot(&mut setup, 1, 3);
    assert_eq!(dst, Some(2));
    assert_eq!(setup.operations[0].id, "op_1");
    assert_eq!(setup.operations[1].id, "op_3");
    assert_eq!(setup.operations[2].id, "op_2");
    assert_eq!(setup.operations[1].runtime_status, OperationStatus::Dirty);

    setup.operations[1].runtime_status = OperationStatus::Clean;
    let dup_idx = duplicate_operation_in_setup(&mut setup, 0);
    assert_eq!(dup_idx, Some(1));
    assert_eq!(setup.operations.len(), 4);
    assert_eq!(setup.operations[2].id, "op_3");
    assert_eq!(setup.operations[2].runtime_status, OperationStatus::Dirty);

    setup.operations[2].runtime_status = OperationStatus::Clean;
    assert!(delete_operation_in_setup(&mut setup, 1));
    assert_eq!(setup.operations[1].id, "op_3");
    assert_eq!(setup.operations[1].runtime_status, OperationStatus::Dirty);
}

#[test]
fn all_six_drag_reorder_helpers_preserve_focus_propagate_dirty_and_reject_cross_section() {
    let mut project = build_rich_test_project();

    // 1. Setups + carryover dirty propagation
    let mut s2 = Setup::new("setup_2", "Setup 2 - Op 2", WorkOffsetSlot::G55);
    s2.carryover_from_setup_id = Some("setup_1".to_string());
    let mut s2_op = Operation::new("s2_op1", "Op 2 Finish", OperationKind::Contour2D);
    s2_op.runtime_status = OperationStatus::Clean;
    s2.operations.push(s2_op);
    let s3 = Setup::new("setup_3", "Setup 3 - Op 3", WorkOffsetSlot::G56);
    project.setups.push(s2);
    project.setups.push(s3);

    let mut active_setup = 1usize;
    let mut focus = InspectorFocus::Setup { setup_idx: 1 };
    // Move setup_3 (idx 2) before setup_2 (slot 1) -> setup_2's upstream ordering changed!
    let dst = move_setup_to_slot(&mut project, 2, 1, &mut active_setup, &mut focus);
    assert_eq!(dst, Some(1));
    assert_eq!(active_setup, 2, "active_setup_idx must track setup_2");
    assert_eq!(focus, InspectorFocus::Setup { setup_idx: 2 });
    assert_eq!(
        project.setups[2].operations[0].runtime_status,
        OperationStatus::Dirty,
        "carryover-dependent setup must be marked Dirty when upstream ordering changes"
    );

    // 2. Sketches
    project
        .sketches
        .push(SketchLayer::new("sk_2", "Text Engraving", "setup_1"));
    focus = InspectorFocus::Sketch { sketch_idx: 0 };
    assert_eq!(move_sketch_to_slot(&mut project, 0, 2, &mut focus), Some(1));
    assert_eq!(project.sketches[0].id, "sk_2");
    assert_eq!(project.sketches[1].id, "sk_1");
    assert_eq!(focus, InspectorFocus::Sketch { sketch_idx: 1 });

    // 3. Sketch Entities
    project.sketches[1].entities.push(SketchEntityEntry {
        id: "ent_2".to_string(),
        name: "Inner Hole".to_string(),
        sub_layer: None,
        is_construction: false,
        entity: SketchEntity::new_circle(
            ParamLength::from_mm(0.0),
            ParamLength::from_mm(0.0),
            ParamLength::from_mm(6.0),
        ),
    });
    let mut sel_ent = Some(1);
    assert_eq!(
        move_sketch_entity_to_slot(&mut project, 1, 1, 0, &mut sel_ent),
        Some(0)
    );
    assert_eq!(project.sketches[1].entities[0].id, "ent_2");
    assert_eq!(sel_ent, Some(0));

    // 4. Keep-Out Regions
    project
        .avoid_regions
        .push(SharedAvoidRegion::new("avoid_2", "Toe Clamp North"));
    focus = InspectorFocus::KeepOutRegion { region_idx: 1 };
    assert_eq!(
        move_avoid_region_to_slot(&mut project, 1, 0, &mut focus),
        Some(0)
    );
    assert_eq!(project.avoid_regions[0].id, "avoid_2");
    assert_eq!(focus, InspectorFocus::KeepOutRegion { region_idx: 0 });

    // 5. Fixtures
    project.setups[0]
        .fixtures
        .push(FixtureBody::default_spoilboard("fix_2", "MDF Spoilboard"));
    let mut sel_fix = Some(0);
    assert_eq!(
        move_fixture_to_slot(&mut project, 0, 0, 2, &mut sel_fix),
        Some(1)
    );
    assert_eq!(project.setups[0].fixtures[1].id, "fix_1");
    assert_eq!(sel_fix, Some(1));

    // 6. Strict cross-section boundary rejection
    let rects = vec![
        Rect::from_min_size(Pos2::new(0.0, 0.0), egui::vec2(200.0, 24.0)),
        Rect::from_min_size(Pos2::new(0.0, 25.0), egui::vec2(200.0, 24.0)),
    ];
    let dragged_op_s0 = DraggedTreeItem {
        section: DragSection::Operations { setup_idx: 0 },
        item_idx: 0,
    };
    assert_eq!(
        resolve_drop_slot_for_section(
            dragged_op_s0,
            DragSection::Operations { setup_idx: 1 },
            &rects,
            Pos2::new(50.0, 10.0)
        ),
        None,
        "Operation from Setup 0 must never drop into Setup 1"
    );
    assert_eq!(
        resolve_drop_slot_for_section(
            dragged_op_s0,
            DragSection::Fixtures { setup_idx: 0 },
            &rects,
            Pos2::new(50.0, 10.0)
        ),
        None,
        "Operation must never drop into Fixtures section"
    );
    assert_eq!(
        resolve_drop_slot_for_section(
            dragged_op_s0,
            DragSection::Operations { setup_idx: 0 },
            &rects,
            Pos2::new(50.0, 45.0)
        ),
        Some(2),
        "Same-section drop resolves cleanly"
    );
}

#[test]
fn selecting_all_nine_tree_node_types_activates_exact_focus_and_renders_right_panel() {
    let mut h = BrowserTestHarness::new(build_rich_test_project());
    let (shapes, _) = h.step(Vec::new());

    // Expand Sketch and Project Tools so child entities and tools are visible too
    let sk_rect = find_text_rect(&shapes, "Mounting Holes").expect("sketch label");
    let _ = h.step(click_btn(
        Pos2::new(sk_rect.left() - 26.5, sk_rect.center().y),
        egui::PointerButton::Primary,
    ));
    let (shapes, _) = h.step(Vec::new());
    let tools_rect =
        find_text_rect_containing(&shapes, "Project Tools (").expect("tools folder label");
    let _ = h.step(click_btn(
        Pos2::new(tools_rect.left() - 26.5, tools_rect.center().y),
        egui::PointerButton::Primary,
    ));
    let (shapes, _) = h.step(Vec::new());

    // 1. Project root -> InspectorFocus::Project
    let pos = find_text_rect(&shapes, "Aluminum Manifold Rev B")
        .expect("project root")
        .center();
    let _ = h.step(click_btn(pos, egui::PointerButton::Primary));
    assert_eq!(h.focus, InspectorFocus::Project);
    let texts = h.render_active_inspector();
    assert!(texts.iter().any(|t| t == "Project Properties"));
    assert!(texts.iter().any(|t| t == "Project Summary"));

    // 2. Model -> InspectorFocus::Model
    let pos = find_text_rect(&shapes, "manifold.step")
        .expect("model row")
        .center();
    let _ = h.step(click_btn(pos, egui::PointerButton::Primary));
    assert_eq!(h.focus, InspectorFocus::Model { model_idx: 0 });
    let _ = h.render_active_inspector();

    // 3. Sketch -> InspectorFocus::Sketch (entity None)
    let pos = find_text_rect(&shapes, "Mounting Holes")
        .expect("sketch row")
        .center();
    let _ = h.step(click_btn(pos, egui::PointerButton::Primary));
    assert_eq!(h.focus, InspectorFocus::Sketch { sketch_idx: 0 });
    assert_eq!(h.sel_ent, None);
    let texts = h.render_active_inspector();
    assert!(texts.iter().any(|t| t == "2D Sketch Layer"));

    // 4. SketchEntity -> InspectorFocus::Sketch + selected_sketch_entity_idx = Some(0)
    let pos = find_text_rect(&shapes, "Outer Pocket")
        .expect("sketch entity row")
        .center();
    let _ = h.step(click_btn(pos, egui::PointerButton::Primary));
    assert_eq!(h.focus, InspectorFocus::Sketch { sketch_idx: 0 });
    assert_eq!(h.sel_ent, Some(0));
    let texts = h.render_active_inspector();
    assert!(texts.iter().any(|t| t.contains("Outer Pocket")));

    // 5. KeepOutRegion -> InspectorFocus::KeepOutRegion
    let pos = find_text_rect(&shapes, "Strap Clamps")
        .expect("keep-out row")
        .center();
    let _ = h.step(click_btn(pos, egui::PointerButton::Primary));
    assert_eq!(h.focus, InspectorFocus::KeepOutRegion { region_idx: 0 });
    let texts = h.render_active_inspector();
    assert!(texts.iter().any(|t| t == "Keep-Out Region"));
    assert!(texts.iter().any(|t| t.contains("Pick 3D Avoid Geometry")));
    assert!(texts.iter().any(|t| t.contains("Linked Operations")));

    // 6. ProjectTool -> InspectorFocus::ProjectTool
    let pos = find_text_rect_containing(&shapes, "T1 -")
        .expect("project tool row")
        .center();
    let _ = h.step(click_btn(pos, egui::PointerButton::Primary));
    assert_eq!(h.focus, InspectorFocus::ProjectTool { tool_idx: 0 });
    let texts = h.render_active_inspector();
    assert!(texts.iter().any(|t| t == "Cutter Profile"));
    assert!(texts.iter().any(|t| t == "Dimensions & Offsets"));
    assert!(texts.iter().any(|t| t == "Edit in Tool Library..."));

    // 7. Setup -> InspectorFocus::Setup
    let pos = find_text_rect(&shapes, "Setup 1 — Top Op")
        .expect("setup row")
        .center();
    let _ = h.step(click_btn(pos, egui::PointerButton::Primary));
    assert_eq!(h.focus, InspectorFocus::Setup { setup_idx: 0 });
    let texts = h.render_active_inspector();
    assert!(texts.iter().any(|t| t == "Setup Name"));

    // 8. Fixture -> InspectorFocus::Setup + SetupTab::Fixtures + selected_fixture_idx = Some(0)
    let pos = find_text_rect(&shapes, "Kurt 4in Vise")
        .expect("fixture row")
        .center();
    let _ = h.step(click_btn(pos, egui::PointerButton::Primary));
    assert_eq!(h.focus, InspectorFocus::Setup { setup_idx: 0 });
    assert_eq!(h.setup_tab, SetupTab::Fixtures);
    assert_eq!(h.sel_fix, Some(0));
    let texts = h.render_active_inspector();
    assert!(texts.iter().any(|t| t == "Selected Fixture Settings"));

    // 9. Operation -> InspectorFocus::Operation
    let pos = find_text_rect(&shapes, "1. Adaptive Rough")
        .expect("operation row")
        .center();
    let _ = h.step(click_btn(pos, egui::PointerButton::Primary));
    assert_eq!(
        h.focus,
        InspectorFocus::Operation {
            setup_idx: 0,
            op_idx: 0
        }
    );
    let _ = h.render_active_inspector();
}

#[test]
fn context_menu_actions_inline_f2_rename_undo_and_dirty_propagation() {
    let mut project = build_rich_test_project();
    project.setups[0].operations[0].runtime_status = OperationStatus::Clean;
    let mut h = BrowserTestHarness::new(project);
    let (shapes, _) = h.step(Vec::new());

    // 1. Right-click Keep-Out Region -> Apply to Active Setup Ops -> links op + dirties + Undo
    let avoid_pos = find_text_rect(&shapes, "Strap Clamps")
        .expect("keep-out row")
        .center();
    let _ = h.step(click_btn(avoid_pos, egui::PointerButton::Secondary));
    let (menu_shapes, _) = h.step(Vec::new());
    let apply_pos = find_text_rect(&menu_shapes, "Apply to Active Setup Ops")
        .expect("Apply to Active Setup Ops context item")
        .center();
    let _ = h.step(click_btn(apply_pos, egui::PointerButton::Primary));
    assert!(
        h.project.setups[0].operations[0]
            .avoid
            .shared_region_ids
            .contains(&"avoid_1".to_string())
    );
    assert_eq!(
        h.project.setups[0].operations[0].runtime_status,
        OperationStatus::Dirty
    );

    // Mark op Clean again, then right-click Keep-Out Region -> Suppress -> dirties + atomic Undo
    h.project.setups[0].operations[0].runtime_status = OperationStatus::Clean;
    let (shapes, _) = h.step(Vec::new());
    let avoid_pos = find_text_rect(&shapes, "Strap Clamps")
        .expect("keep-out row")
        .center();
    let _ = h.step(click_btn(avoid_pos, egui::PointerButton::Secondary));
    let (menu_shapes, _) = h.step(Vec::new());
    let sup_pos = find_text_rect(&menu_shapes, "Suppress")
        .expect("Suppress context item")
        .center();
    let _ = h.step(click_btn(sup_pos, egui::PointerButton::Primary));
    assert!(h.project.avoid_regions[0].suppressed);
    assert_eq!(
        h.project.setups[0].operations[0].runtime_status,
        OperationStatus::Dirty,
        "suppressing linked Keep-Out Region must dirty referencing operations"
    );
    assert_eq!(
        h.history.undo(&mut h.project).as_deref(),
        Some("Toggle Shared Keep-Out Suppress")
    );
    assert!(!h.project.avoid_regions[0].suppressed);

    // 2. Inline F2 rename on Operation -> Escape cancels without mutating or pushing Undo
    let (shapes, _) = h.step(Vec::new());
    let op_pos = find_text_rect(&shapes, "1. Adaptive Rough")
        .expect("operation row")
        .center();
    let _ = h.step(click_btn(op_pos, egui::PointerButton::Primary));
    let _ = h.step(vec![egui::Event::Key {
        key: egui::Key::F2,
        physical_key: None,
        pressed: true,
        repeat: false,
        modifiers: egui::Modifiers::NONE,
    }]);
    let _ = h.step(vec![
        egui::Event::Text("Cancelled Name".to_string()),
        egui::Event::Key {
            key: egui::Key::Escape,
            physical_key: None,
            pressed: true,
            repeat: false,
            modifiers: egui::Modifiers::NONE,
        },
    ]);
    assert_eq!(h.project.setups[0].operations[0].name, "Adaptive Rough");

    // 3. Inline F2 rename on Operation -> type new name + Enter -> atomic Undo
    let _ = h.step(Vec::new());
    let _ = h.step(vec![egui::Event::Key {
        key: egui::Key::F2,
        physical_key: None,
        pressed: true,
        repeat: false,
        modifiers: egui::Modifiers::NONE,
    }]);
    let _ = h.step(Vec::new());
    let _ = h.step(vec![
        egui::Event::Key {
            key: egui::Key::A,
            physical_key: None,
            pressed: true,
            repeat: false,
            modifiers: egui::Modifiers::COMMAND,
        },
        egui::Event::Text("Roughing Pocket Pass".to_string()),
        egui::Event::Key {
            key: egui::Key::Enter,
            physical_key: None,
            pressed: true,
            repeat: false,
            modifiers: egui::Modifiers::NONE,
        },
    ]);
    assert_eq!(
        h.project.setups[0].operations[0].name,
        "Roughing Pocket Pass"
    );
    assert_eq!(
        h.history.undo(&mut h.project).as_deref(),
        Some("Rename Operation")
    );
    assert_eq!(h.project.setups[0].operations[0].name, "Adaptive Rough");

    // 4. Right-click Setup -> Duplicate Setup -> atomic Undo
    let (shapes, _) = h.step(Vec::new());
    let setup_pos = find_text_rect(&shapes, "Setup 1 — Top Op")
        .expect("setup row")
        .center();
    let _ = h.step(click_btn(setup_pos, egui::PointerButton::Secondary));
    let (menu_shapes, _) = h.step(Vec::new());
    let dup_pos = find_text_rect(&menu_shapes, "Duplicate Setup")
        .expect("Duplicate Setup item")
        .center();
    let _ = h.step(click_btn(dup_pos, egui::PointerButton::Primary));
    assert_eq!(h.project.setups.len(), 2);
    assert_eq!(
        h.history.undo(&mut h.project).as_deref(),
        Some("Duplicate Setup")
    );
    assert_eq!(h.project.setups.len(), 1);

    // 5. Expand Project Tools, then right-click Project Tool -> Sync from Machine Library -> marks referencing op Dirty + Undo
    h.project.setups[0].operations[0].runtime_status = OperationStatus::Clean;
    let (shapes, _) = h.step(Vec::new());
    let tools_rect =
        find_text_rect_containing(&shapes, "Project Tools (").expect("tools folder label");
    let _ = h.step(click_btn(
        Pos2::new(tools_rect.left() - 26.5, tools_rect.center().y),
        egui::PointerButton::Primary,
    ));
    let (shapes, _) = h.step(Vec::new());
    let tool_pos = find_text_rect_containing(&shapes, "T1 -")
        .expect("tool row")
        .center();
    let _ = h.step(click_btn(tool_pos, egui::PointerButton::Secondary));
    let (menu_shapes, _) = h.step(Vec::new());
    let sync_pos = find_text_rect(&menu_shapes, "Sync from Machine Library")
        .expect("Sync from Machine Library item")
        .center();
    let _ = h.step(click_btn(sync_pos, egui::PointerButton::Primary));
    assert_eq!(
        h.project.setups[0].operations[0].runtime_status,
        OperationStatus::Dirty
    );
    assert_eq!(
        h.history.undo(&mut h.project).as_deref(),
        Some("Sync Tool from Machine Library")
    );

    // 6. Right-click Fixture -> Duplicate -> atomic Undo + dirties setup ops
    h.project.setups[0].operations[0].runtime_status = OperationStatus::Clean;
    let (shapes, _) = h.step(Vec::new());
    let fix_pos = find_text_rect(&shapes, "Kurt 4in Vise")
        .expect("fixture row")
        .center();
    let _ = h.step(click_btn(fix_pos, egui::PointerButton::Secondary));
    let (menu_shapes, _) = h.step(Vec::new());
    let dup_fix_pos = find_text_rect(&menu_shapes, "Duplicate")
        .expect("Fixture Duplicate item")
        .center();
    let _ = h.step(click_btn(dup_fix_pos, egui::PointerButton::Primary));
    assert_eq!(h.project.setups[0].fixtures.len(), 2);
    assert_eq!(
        h.project.setups[0].operations[0].runtime_status,
        OperationStatus::Dirty
    );
    assert_eq!(
        h.history.undo(&mut h.project).as_deref(),
        Some("Duplicate Fixture")
    );
    assert_eq!(h.project.setups[0].fixtures.len(), 1);

    // 7. Right-click Project Root -> Collapse All hides child nodes; Expand All restores them
    let (shapes, _) = h.step(Vec::new());
    let root_pos = find_text_rect(&shapes, "Aluminum Manifold Rev B")
        .expect("root row")
        .center();
    let _ = h.step(click_btn(root_pos, egui::PointerButton::Secondary));
    let (menu_shapes, _) = h.step(Vec::new());
    let collapse_pos = find_text_rect(&menu_shapes, "Collapse All")
        .expect("Collapse All item")
        .center();
    let _ = h.step(click_btn(collapse_pos, egui::PointerButton::Primary));
    let (collapsed_shapes, _) = h.step(Vec::new());
    assert!(
        find_text_rect(&collapsed_shapes, "1. Adaptive Rough").is_none(),
        "Collapse All must collapse folder children"
    );
    let _ = h.step(click_btn(root_pos, egui::PointerButton::Secondary));
    let (menu_shapes, _) = h.step(Vec::new());
    let expand_pos = find_text_rect(&menu_shapes, "Expand All")
        .expect("Expand All item")
        .center();
    let _ = h.step(click_btn(expand_pos, egui::PointerButton::Primary));
    let (expanded_shapes, _) = h.step(Vec::new());
    assert!(
        find_text_rect(&expanded_shapes, "Outer Pocket").is_some(),
        "Expand All must expand nested Sketch children"
    );
}

#[test]
fn formats_compact_cycle_time_cleanly() {
    assert_eq!(format_compact_cycle_time(0.2), "1s");
    assert_eq!(format_compact_cycle_time(42.4), "42s");
    assert_eq!(format_compact_cycle_time(72.0), "1m12s");
    assert_eq!(format_compact_cycle_time(3_840.0), "1h04m");
}

#[test]
fn creating_default_thread_mill_op_auto_selects_compatible_thread_mill_cutter() {
    use chip_math::DisplayUnit;
    use chip_tools::{CoolantMode, CutterGeometry, SpindleDirection, Tool, ToolId};

    let mut project = ProjectDocument::new("thread_mill_test");
    let mut global_lib = chip_tools::default_starter_tool_library();
    let expected_tm_id = if let Some(existing) = global_lib
        .tools
        .iter()
        .find(|t| matches!(t.geometry, CutterGeometry::ThreadMill { .. }))
    {
        existing.id.clone()
    } else {
        let tid = ToolId("test_thread_mill_475".to_string());
        global_lib.tools.push(Tool {
            id: tid.clone(),
            tool_number: 15,
            length_offset_number: 15,
            diameter_offset_number: 15,
            name: "4.75mm Single-Point Thread Mill".to_string(),
            vendor: None,
            product_id: None,
            native_unit: DisplayUnit::Millimeter,
            spindle_direction: SpindleDirection::Cw,
            default_coolant: CoolantMode::AirBlast,
            stickout_length_mm: 22.0,
            geometry: CutterGeometry::ThreadMill {
                diameter_mm: 4.75,
                neck_diameter_mm: 2.85,
                neck_length_mm: 16.0,
                pitch_mm: 1.0,
                teeth_count: 1,
                thread_angle_deg: 60.0,
                flute_length_mm: 1.2,
                overall_length_mm: 50.0,
                shank_diameter_mm: 6.0,
                flutes: 3,
            },
            presets: Vec::new(),
        });
        tid
    };

    let o_idx =
        create_default_operation_in_setup(&mut project, &global_lib, 0, OperationKind::ThreadMill);
    assert_eq!(o_idx, Some(0));

    let op = &project.setups[0].operations[0];
    assert_eq!(op.kind, OperationKind::ThreadMill);
    assert_eq!(op.tool_id.as_ref(), Some(&expected_tm_id));
    let embedded = project.tool(&expected_tm_id);
    assert!(
        embedded.is_some_and(|t| matches!(t.geometry, CutterGeometry::ThreadMill { .. })),
        "expected compatible ThreadMill cutter to be embedded in project"
    );
    assert_eq!(
        default_pick_mode_for_op(0, 0, OperationKind::ThreadMill, OperationTab::Geometry),
        ViewportPickMode::OpCylindricalHoles {
            setup_idx: 0,
            op_idx: 0
        }
    );
}

#[test]
fn browser_header_shows_project_title_without_redundant_root_buttons() {
    let mut h = BrowserTestHarness::new(ProjectDocument::new("Aluminum Manifold Rev B"));
    let (shapes, _) = h.step(Vec::new());

    let mut texts = Vec::new();
    for cs in &shapes {
        collect_texts(&cs.shape, &mut texts);
    }
    assert!(
        texts.iter().any(|t| t == "Aluminum Manifold Rev B"),
        "CAM Browser root must display the project title; got {texts:?}"
    );
    assert!(
        !texts.iter().any(|t| t == "+ Setup"),
        "CAM Browser root must not show redundant + Setup button (Setups folder '+' handles adding setups)"
    );
    assert!(
        !texts.iter().any(|t| t == "Rename"),
        "Inline Rename button must not appear in CAM Browser tree"
    );
}

#[test]
fn project_tree_renders_full_hierarchy_and_persists_expand_collapse_across_frames() {
    for theme in AppTheme::ALL {
        let mut h = BrowserTestHarness::new(build_rich_test_project());
        apply_theme(&h.ctx, theme);

        let (shapes1, _) = h.step(Vec::new());
        let mut texts1 = Vec::new();
        for cs in &shapes1 {
            collect_texts(&cs.shape, &mut texts1);
        }
        let joined1 = texts1.join("\n");
        for expected in [
            "Aluminum Manifold Rev B",
            "Models (3)",
            "manifold.step",
            "relief.stl",
            "kurt_vise.step",
            "Sketches (1)",
            "Mounting Holes",
            "Keep-Out Regions (1)",
            "Strap Clamps",
            "Project Tools (1)",
            "Setups (1)",
            "Setup 1 — Top Op",
            "G54",
            "Active",
            "Fixtures (1)",
            "Kurt 4in Vise",
            "Operations (1)",
            "1. Adaptive Rough",
        ] {
            assert!(
                joined1.contains(expected),
                "{theme:?}: expected '{expected}' in tree output:\n{joined1}"
            );
        }
        assert!(
            !joined1.contains("Outer Pocket"),
            "{theme:?}: expected sketch entities collapsed by default"
        );

        let sk_rect = find_text_rect(&shapes1, "Mounting Holes").expect("sketch label rect");
        let chev_click_pos = Pos2::new(sk_rect.left() - 28.0, sk_rect.center().y);
        let _ = h.step(click_btn(chev_click_pos, egui::PointerButton::Primary));

        let (shapes2, _) = h.step(Vec::new());
        let mut texts2 = Vec::new();
        for cs in &shapes2 {
            collect_texts(&cs.shape, &mut texts2);
        }
        assert!(
            texts2.iter().any(|t| t == "Outer Pocket"),
            "{theme:?}: expanded Sketch entity must persist across frames; got {texts2:?}"
        );

        let ent_rect = find_text_rect(&shapes2, "Outer Pocket").expect("entity label rect");
        let _ = h.step(click_btn(ent_rect.center(), egui::PointerButton::Primary));
        assert_eq!(h.focus, InspectorFocus::Sketch { sketch_idx: 0 });
        assert_eq!(h.sel_ent, Some(0));

        let fix_rect = find_text_rect(&shapes2, "Kurt 4in Vise").expect("fixture label rect");
        let _ = h.step(click_btn(fix_rect.center(), egui::PointerButton::Primary));
        assert_eq!(h.focus, InspectorFocus::Setup { setup_idx: 0 });
        assert_eq!(h.setup_tab, SetupTab::Fixtures);
        assert_eq!(h.sel_fix, Some(0));

        // Collapsing Operations (1) hides child operations and persists across frames
        let ops_rect =
            find_text_rect(&shapes2, "Operations (1)").expect("operations folder label rect");
        let ops_chev_pos = Pos2::new(ops_rect.left() - 28.0, ops_rect.center().y);
        let _ = h.step(click_btn(ops_chev_pos, egui::PointerButton::Primary));
        let (shapes3, _) = h.step(Vec::new());
        assert!(
            find_text_rect(&shapes3, "1. Adaptive Rough").is_none(),
            "{theme:?}: collapsing Operations (1) must hide child operations"
        );
        let _ = h.step(click_btn(ops_chev_pos, egui::PointerButton::Primary));
        let (shapes4, _) = h.step(Vec::new());
        assert!(
            find_text_rect(&shapes4, "1. Adaptive Rough").is_some(),
            "{theme:?}: re-expanding Operations (1) must restore child operations"
        );
    }
}

#[test]
fn keyboard_navigation_steps_expands_collapses_and_jumps_to_parent() {
    let root_id = Id::unique("n_root");
    let sk_folder_id = Id::unique("n_sk_folder");
    let sk_id = Id::unique("n_sk0");
    let ent_id = Id::unique("n_ent0");

    let collapsed_tree = vec![
        VisibleTreeNode {
            key: TreeNodeKey::Project,
            node_id: root_id,
            parent: None,
            has_children: true,
            is_open: true,
        },
        VisibleTreeNode {
            key: TreeNodeKey::SketchesFolder,
            node_id: sk_folder_id,
            parent: Some(TreeNodeKey::Project),
            has_children: true,
            is_open: true,
        },
        VisibleTreeNode {
            key: TreeNodeKey::Sketch(0),
            node_id: sk_id,
            parent: Some(TreeNodeKey::SketchesFolder),
            has_children: true,
            is_open: false,
        },
    ];

    assert_eq!(
        resolve_tree_keyboard_nav(
            &collapsed_tree,
            Some(&TreeNodeKey::Project),
            TreeNavKey::Down
        ),
        vec![TreeNavAction::Select(TreeNodeKey::SketchesFolder)]
    );
    assert_eq!(
        resolve_tree_keyboard_nav(
            &collapsed_tree,
            Some(&TreeNodeKey::Sketch(0)),
            TreeNavKey::Up
        ),
        vec![TreeNavAction::Select(TreeNodeKey::SketchesFolder)]
    );
    assert_eq!(
        resolve_tree_keyboard_nav(
            &collapsed_tree,
            Some(&TreeNodeKey::Sketch(0)),
            TreeNavKey::Right
        ),
        vec![TreeNavAction::SetOpen {
            node_id: sk_id,
            open: true
        }]
    );

    let expanded_tree = vec![
        collapsed_tree[0].clone(),
        collapsed_tree[1].clone(),
        VisibleTreeNode {
            key: TreeNodeKey::Sketch(0),
            node_id: sk_id,
            parent: Some(TreeNodeKey::SketchesFolder),
            has_children: true,
            is_open: true,
        },
        VisibleTreeNode {
            key: TreeNodeKey::SketchEntity {
                sketch_idx: 0,
                entity_idx: 0,
            },
            node_id: ent_id,
            parent: Some(TreeNodeKey::Sketch(0)),
            has_children: false,
            is_open: false,
        },
    ];

    assert_eq!(
        resolve_tree_keyboard_nav(
            &expanded_tree,
            Some(&TreeNodeKey::Sketch(0)),
            TreeNavKey::Right
        ),
        vec![TreeNavAction::Select(TreeNodeKey::SketchEntity {
            sketch_idx: 0,
            entity_idx: 0
        })]
    );
    assert_eq!(
        resolve_tree_keyboard_nav(
            &expanded_tree,
            Some(&TreeNodeKey::SketchEntity {
                sketch_idx: 0,
                entity_idx: 0
            }),
            TreeNavKey::Left
        ),
        vec![TreeNavAction::Select(TreeNodeKey::Sketch(0))]
    );
    assert_eq!(
        resolve_tree_keyboard_nav(
            &expanded_tree,
            Some(&TreeNodeKey::Sketch(0)),
            TreeNavKey::Left
        ),
        vec![TreeNavAction::SetOpen {
            node_id: sk_id,
            open: false
        }]
    );
    assert_eq!(
        resolve_tree_keyboard_nav(
            &expanded_tree,
            Some(&TreeNodeKey::Sketch(0)),
            TreeNavKey::Rename
        ),
        vec![TreeNavAction::Rename(TreeNodeKey::Sketch(0))]
    );
    assert_eq!(
        resolve_tree_keyboard_nav(
            &expanded_tree,
            Some(&TreeNodeKey::Sketch(0)),
            TreeNavKey::Delete
        ),
        vec![TreeNavAction::Delete(TreeNodeKey::Sketch(0))]
    );
}

#[test]
fn headless_egui_keyboard_navigation_expands_and_selects_in_tree() {
    let mut h = BrowserTestHarness::new(build_rich_test_project());
    let (shapes, _) = h.step(Vec::new());
    let sk_pos = find_text_rect(&shapes, "Mounting Holes")
        .expect("sketch row")
        .center();
    let _ = h.step(click_btn(sk_pos, egui::PointerButton::Primary));
    assert_eq!(h.focus, InspectorFocus::Sketch { sketch_idx: 0 });
    assert_eq!(h.sel_ent, None);

    let key_evt = |key: egui::Key| {
        vec![egui::Event::Key {
            key,
            physical_key: None,
            pressed: true,
            repeat: false,
            modifiers: egui::Modifiers::NONE,
        }]
    };

    let _ = h.step(key_evt(egui::Key::ArrowRight));
    let _ = h.step(key_evt(egui::Key::ArrowRight));
    assert_eq!(h.focus, InspectorFocus::Sketch { sketch_idx: 0 });
    assert_eq!(h.sel_ent, Some(0));

    let _ = h.step(key_evt(egui::Key::ArrowLeft));
    assert_eq!(h.focus, InspectorFocus::Sketch { sketch_idx: 0 });
    assert_eq!(h.sel_ent, None);

    let _ = h.step(key_evt(egui::Key::ArrowLeft));
    let (shapes_after, _) = h.step(Vec::new());
    let mut texts = Vec::new();
    for cs in &shapes_after {
        collect_texts(&cs.shape, &mut texts);
    }
    assert!(
        !texts.iter().any(|t| t == "Outer Pocket"),
        "Left arrow on open Sketch must collapse its children"
    );
}
