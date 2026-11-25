use std::path::Path;

use chip_cam::{Operation, OperationKind, Setup, WorkOffsetSlot};
use chip_math::DisplayUnit;
use chip_project::ProjectDocument;

use crate::{
    app::ChipCutterApp,
    layout::{WorkspacePane, create_default_workspace_tree, focus_workspace_pane},
    panels::{
        InspectorFocus, OperationTab, ViewportPickMode,
        browser::{
            create_default_operation_in_setup, default_pick_mode_for_op, delete_operation_in_setup,
            duplicate_operation_in_setup, move_operation_to_slot,
        },
        command_palette::CommandId,
    },
};

impl ChipCutterApp {
    pub(crate) fn duplicate_focused_operation(&mut self) {
        if let InspectorFocus::Operation { setup_idx, op_idx } = self.inspector_focus
            && setup_idx < self.project.setups.len()
            && op_idx < self.project.setups[setup_idx].operations.len()
        {
            self.history
                .record_before_edit("Duplicate Operation", None, &self.project);
            if let Some(new_idx) =
                duplicate_operation_in_setup(&mut self.project.setups[setup_idx], op_idx)
            {
                self.inspector_focus = InspectorFocus::Operation {
                    setup_idx,
                    op_idx: new_idx,
                };
            }
        }
    }

    pub(crate) fn move_focused_operation(&mut self, delta: isize) {
        if let InspectorFocus::Operation { setup_idx, op_idx } = self.inspector_focus
            && setup_idx < self.project.setups.len()
        {
            let len = self.project.setups[setup_idx].operations.len();
            let slot_opt = if delta < 0 && op_idx > 0 {
                Some(op_idx - 1)
            } else if delta > 0 && op_idx + 1 < len {
                Some(op_idx + 2)
            } else {
                None
            };
            if let Some(slot) = slot_opt {
                self.history
                    .record_before_edit("Reorder Operation", None, &self.project);
                if let Some(dst_idx) =
                    move_operation_to_slot(&mut self.project.setups[setup_idx], op_idx, slot)
                {
                    self.inspector_focus = InspectorFocus::Operation {
                        setup_idx,
                        op_idx: dst_idx,
                    };
                }
            }
        }
    }

    pub(crate) fn delete_focused_item(&mut self) {
        match self.inspector_focus {
            InspectorFocus::Operation { setup_idx, op_idx }
                if setup_idx < self.project.setups.len()
                    && op_idx < self.project.setups[setup_idx].operations.len() =>
            {
                self.history
                    .record_before_edit("Delete Operation", None, &self.project);
                if delete_operation_in_setup(&mut self.project.setups[setup_idx], op_idx) {
                    let rem = self.project.setups[setup_idx].operations.len();
                    self.inspector_focus = if rem > 0 {
                        InspectorFocus::Operation {
                            setup_idx,
                            op_idx: op_idx.min(rem - 1),
                        }
                    } else {
                        InspectorFocus::Setup { setup_idx }
                    };
                    self.pick_mode = ViewportPickMode::InspectEntity;
                }
            }
            InspectorFocus::Sketch { sketch_idx } if sketch_idx < self.project.sketches.len() => {
                if let Some(e_idx) = self.selected_sketch_entity_idx
                    && e_idx < self.project.sketches[sketch_idx].entities.len()
                {
                    self.history
                        .record_before_edit("Delete Sketch Entity", None, &self.project);
                    let eval_ctx = self.eval_context();
                    let project_dir = self.active_project_dir();
                    let font_map = self.project.load_custom_fonts(project_dir);
                    let fonts_opt = (!font_map.is_empty()).then_some(&font_map);
                    let sk = &mut self.project.sketches[sketch_idx];
                    sk.entities.remove(e_idx);
                    sk.reevaluate(&eval_ctx);
                    sk.rebuild_regions(fonts_opt);
                    let rem = sk.entities.len();
                    self.selected_sketch_entity_idx = if rem > 0 {
                        Some(e_idx.min(rem - 1))
                    } else {
                        None
                    };
                    self.cached_frame_geo = None;
                }
            }
            _ => {}
        }
    }

    pub(crate) fn create_new_adaptive_operation(&mut self) {
        if self.active_setup_idx >= self.project.setups.len() {
            return;
        }
        self.history
            .record_before_edit("Add 2D Adaptive Clearing", None, &self.project);
        let s_idx = self.active_setup_idx;
        let n = self.project.setups[s_idx].operations.len() + 1;
        let mut op = Operation::new_adaptive_2d(
            format!("op_{s_idx}_{n}_adaptive"),
            format!("2D Adaptive {n}"),
        );
        if let Some(t) = self
            .project
            .tools
            .iter()
            .chain(self.global_tools.tools.iter())
            .find(|t| OperationKind::Pocket2D.is_compatible_with_tool(t))
            .cloned()
        {
            let material = self.project.effective_workpiece_material();
            self.project.ensure_tool_embedded(&t);
            op.preset_id =
                chip_tools::match_best_preset_for_material(&t, &material).map(|p| p.id.clone());
            op.tool_id = Some(t.id);
        }
        self.project.setups[s_idx].operations.push(op);
        let o_idx = self.project.setups[s_idx].operations.len() - 1;
        self.inspector_focus = InspectorFocus::Operation {
            setup_idx: s_idx,
            op_idx: o_idx,
        };
        self.op_tab = OperationTab::Geometry;
        self.pick_mode =
            default_pick_mode_for_op(s_idx, o_idx, OperationKind::Pocket2D, self.op_tab);
    }

    pub(crate) fn reset_blank_project(&mut self, name: &str) {
        self.prepare_for_project_replacement(None, None);
        let mut doc = ProjectDocument::new(name);
        doc.display_units = self.prefs.default_units;
        if self.prefs.default_machine_id != chip_tools::DEFAULT_MACHINE_ID
            && let Some(m) = self
                .machines
                .iter()
                .find(|m| m.id.eq_ignore_ascii_case(&self.prefs.default_machine_id))
                .cloned()
        {
            doc.apply_machine_defaults(&m, true);
        }
        self.project = doc;
        self.project_file_path = None;
        self.sync_active_machine_tools();
        self.refresh_dependent_expressions();
    }

    pub(crate) fn add_standard_operation(&mut self, kind: OperationKind) {
        if self.active_setup_idx >= self.project.setups.len() {
            return;
        }
        self.history
            .record_before_edit(format!("Add {}", kind.label()), None, &self.project);
        let s_idx = self.active_setup_idx;
        if let Some(o_idx) =
            create_default_operation_in_setup(&mut self.project, &self.global_tools, s_idx, kind)
        {
            self.inspector_focus = InspectorFocus::Operation {
                setup_idx: s_idx,
                op_idx: o_idx,
            };
            self.op_tab = if kind == OperationKind::Facing || kind.is_3d_surface() {
                OperationTab::Passes
            } else {
                OperationTab::Geometry
            };
            self.pick_mode = default_pick_mode_for_op(s_idx, o_idx, kind, self.op_tab);
        }
    }

    pub(crate) fn execute_palette_command(&mut self, cmd: CommandId, ctx: &egui::Context) {
        if let Some(sample_id) = cmd.as_sample() {
            self.load_sample_project(sample_id, ctx);
            return;
        }
        match cmd {
            CommandId::NewProject => self.reset_blank_project("Untitled Job"),
            CommandId::OpenProject => self.open_project_dialog(ctx),
            CommandId::SaveProject => {
                let _ = self.save_project_dialog();
            }
            CommandId::OpenProjectProperties => {
                self.modals
                    .project_properties
                    .open_for_project(&self.project.project_name);
            }
            CommandId::ExportChipzBundle => self.export_chipz_dialog(),
            CommandId::ImportStepModel => self.open_step_dialog(ctx),
            CommandId::ImportMesh3D => self.modals.mesh_import.open_with_file_picker(),
            CommandId::ReloadStepModel => self.reload_active_step_model(ctx),
            CommandId::ImportVector2D => self.modals.vector_import.open_with_file_picker(),
            CommandId::OpenWelcomeScreen => self.modals.welcome.open = true,
            CommandId::OpenSampleGallery => self.modals.sample_gallery.open = true,
            CommandId::NewSetup => {
                self.history
                    .record_before_edit("Add Setup", None, &self.project);
                let n = self.project.setups.len() + 1;
                self.project.setups.push(Setup::new(
                    format!("setup_{n}"),
                    format!("Setup {n} — Op {n}"),
                    WorkOffsetSlot::G55,
                ));
                self.active_setup_idx = self.project.setups.len() - 1;
                self.inspector_focus = InspectorFocus::Setup {
                    setup_idx: self.active_setup_idx,
                };
            }
            CommandId::OpenFlipJigWizard => {
                self.modals
                    .flip_jig_wizard
                    .open_for_setup(self.active_setup_idx);
            }
            CommandId::OpenSoftJawWizard => {
                self.modals
                    .soft_jaw_wizard
                    .open_for_setup(self.active_setup_idx);
            }
            CommandId::NewSketchSetupXy => self.create_new_setup_xy_sketch(),
            CommandId::NewSketchOn3dFace => {
                if self.model.is_some() {
                    self.pick_mode = ViewportPickMode::PickSketchHostFace {
                        setup_idx: self.active_setup_idx,
                    };
                }
            }
            CommandId::AddOpAdaptive2D => self.create_new_adaptive_operation(),
            CommandId::AddOpFacing => self.add_standard_operation(OperationKind::Facing),
            CommandId::AddOpContour2D => self.add_standard_operation(OperationKind::Contour2D),
            CommandId::AddOpPocket2D => self.add_standard_operation(OperationKind::Pocket2D),
            CommandId::AddOpDrillBore => self.add_standard_operation(OperationKind::DrillBore),
            CommandId::AddOpThreadMill => self.add_standard_operation(OperationKind::ThreadMill),
            CommandId::AddOpSlotUndercut => {
                self.add_standard_operation(OperationKind::SlotUndercut);
            }
            CommandId::AddOpDovetail => self.add_standard_operation(OperationKind::Dovetail),
            CommandId::AddOpChamferTrace => {
                self.add_standard_operation(OperationKind::ChamferTrace)
            }
            CommandId::AddOpVCarve => self.create_new_vcarve_operation(),
            CommandId::AddOpAdaptive3D => self.add_standard_operation(OperationKind::Adaptive3D),
            CommandId::AddOpWaterline3D => self.add_standard_operation(OperationKind::Waterline3D),
            CommandId::AddOpParallel3D => self.add_standard_operation(OperationKind::Parallel3D),
            CommandId::AddOpScallop3D => self.add_standard_operation(OperationKind::Scallop3D),
            CommandId::AddOpPencil3D => self.add_standard_operation(OperationKind::Pencil3D),
            CommandId::AddOpDeburr3D => self.add_standard_operation(OperationKind::Deburr3D),
            CommandId::AddOpUndercut3D => self.add_standard_operation(OperationKind::Undercut3D),
            CommandId::AddOpRotary2Linear => {
                self.add_standard_operation(OperationKind::Rotary2Linear);
            }
            CommandId::AddOpRotary4AxisSim => {
                self.add_standard_operation(OperationKind::Rotary4AxisSim);
            }
            CommandId::DuplicateOperation => self.duplicate_focused_operation(),
            CommandId::MoveOperationUp => self.move_focused_operation(-1),
            CommandId::MoveOperationDown => self.move_focused_operation(1),
            CommandId::DeleteSelected => self.delete_focused_item(),
            CommandId::Undo => self.perform_undo(),
            CommandId::Redo => self.perform_redo(),
            CommandId::GenerateActiveSetup => {
                self.trigger_generate_dirty(Some(self.active_setup_idx), None);
            }
            CommandId::GenerateAllSetups => self.trigger_generate_dirty(None, None),
            CommandId::RunSimulation => {
                focus_workspace_pane(&mut self.tree, WorkspacePane::SimulationTimeline);
                self.trigger_active_setup_simulation(ctx);
            }
            CommandId::OpenGCodePreview => {
                focus_workspace_pane(&mut self.tree, WorkspacePane::GCodePreview);
            }
            CommandId::ExportGCode => self.trigger_gcode_export_from_menu(),
            CommandId::OpenToolLibrary => self.modals.tool_library.open = true,
            CommandId::OpenMachineManager => {
                let active_mid = self.project.effective_machine_id().to_string();
                self.modals
                    .machine_manager
                    .open_selecting(&self.machines, &active_mid);
            }
            CommandId::OpenDiagnosticsLog => {
                focus_workspace_pane(&mut self.tree, WorkspacePane::DiagnosticsLog);
            }
            CommandId::ToggleMeasureTool => self.toggle_measure_mode(),
            CommandId::ToggleReachHeatmap => {
                self.highlight.show_reach_heatmap = !self.highlight.show_reach_heatmap;
            }
            CommandId::FitToViewport => self.fit_scene_to_viewport(),
            CommandId::ToggleOrthoPerspective => self.camera.toggle_projection(),
            CommandId::AlignViewSetupTop => self.align_camera_to_active_setup(),
            CommandId::ResetWorkspaceLayout => {
                self.tree = create_default_workspace_tree();
                self.prefs.layout_shares = Default::default();
            }
            CommandId::ToggleThemeDarkLight => self.set_theme(ctx, self.theme.toggled()),
            CommandId::SwitchUnitsMillimeter => {
                if self.project.display_units != DisplayUnit::Millimeter {
                    self.history
                        .record_before_edit("Set Units to mm", None, &self.project);
                    self.project.display_units = DisplayUnit::Millimeter;
                    self.prefs.default_units = DisplayUnit::Millimeter;
                    self.refresh_dependent_expressions();
                }
            }
            CommandId::SwitchUnitsInch => {
                if self.project.display_units != DisplayUnit::Inch {
                    self.history
                        .record_before_edit("Set Units to in", None, &self.project);
                    self.project.display_units = DisplayUnit::Inch;
                    self.prefs.default_units = DisplayUnit::Inch;
                    self.refresh_dependent_expressions();
                }
            }
            CommandId::OpenShortcutCheatsheet => self.modals.shortcut_cheatsheet.open = true,
            CommandId::OpenSettings => self.modals.settings.open = true,
            CommandId::OpenAboutModal => self.modals.about.open = true,
            CommandId::SampleCamTorture
            | CommandId::SampleAdaptiveRest
            | CommandId::SampleVCarveInlay
            | CommandId::SampleEngravedNameplate
            | CommandId::SamplePure2dPanel
            | CommandId::SampleSculptedMeshDome
            | CommandId::SampleReliefWaveMesh
            | CommandId::SampleRotaryIndexedShaft
            | CommandId::SampleRotaryWrappedEngraving
            | CommandId::SampleRotaryHelicalFlute
            | CommandId::SampleThreadMillManifold
            | CommandId::SampleTwoSidedVise
            | CommandId::SampleRevisionReloadV2 => {}
        }
    }

    pub(crate) fn create_new_vcarve_operation(&mut self) {
        if self.active_setup_idx >= self.project.setups.len() {
            return;
        }
        self.history
            .record_before_edit("Add V-Carve", None, &self.project);
        let s_idx = self.active_setup_idx;
        let n = self.project.setups[s_idx].operations.len() + 1;
        let mut op = Operation::new(
            format!("op_{s_idx}_{n}_vcarve"),
            format!("V-Carve {n}"),
            OperationKind::VCarve,
        );
        if let Some(t) = self
            .project
            .tools
            .iter()
            .chain(self.global_tools.tools.iter())
            .find(|t| OperationKind::VCarve.is_compatible_with_tool(t))
            .cloned()
        {
            let material = self.project.effective_workpiece_material();
            self.project.ensure_tool_embedded(&t);
            op.preset_id =
                chip_tools::match_best_preset_for_material(&t, &material).map(|p| p.id.clone());
            op.tool_id = Some(t.id);
        }
        self.project.setups[s_idx].operations.push(op);
        let o_idx = self.project.setups[s_idx].operations.len() - 1;
        self.inspector_focus = InspectorFocus::Operation {
            setup_idx: s_idx,
            op_idx: o_idx,
        };
        self.op_tab = OperationTab::Geometry;
        self.pick_mode = default_pick_mode_for_op(s_idx, o_idx, OperationKind::VCarve, self.op_tab);
    }

    pub(crate) fn trigger_gcode_export_from_menu(&mut self) {
        focus_workspace_pane(&mut self.tree, WorkspacePane::GCodePreview);
        let eff = self.effective_cad_arc();
        let cad_doc = eff.as_ref().map(|(d, _)| d);
        let cad_rev = eff.as_ref().map_or(0, |(_, r)| *r);
        let virtual_proj_path = self.virtual_project_file_path();
        let proj_dir = virtual_proj_path.as_deref().and_then(Path::parent);
        self.gcode_panel.ensure_profiles_loaded(proj_dir);
        self.gcode_panel.rebuild_preview_if_needed(
            &self.project,
            self.active_setup_idx,
            cad_doc,
            cad_rev,
            &self.toolpath_cache,
            virtual_proj_path.as_deref(),
        );
        self.gcode_panel.trigger_export_action(
            &self.project,
            self.active_setup_idx,
            &self.toolpath_cache,
            cad_rev,
            &self.sim_panel,
            &mut self.logs,
        );
    }

    pub(crate) fn trigger_export_all_setups_from_menu(&mut self) {
        focus_workspace_pane(&mut self.tree, WorkspacePane::GCodePreview);
        let eff = self.effective_cad_arc();
        let Some((cad_doc, cad_rev)) = eff else {
            self.logs
                .push("Cannot batch-export: load a model or 2D Sketch first.".to_string());
            return;
        };
        let virtual_proj_path = self.virtual_project_file_path();
        let proj_dir = virtual_proj_path.as_deref().and_then(Path::parent);
        self.gcode_panel.ensure_profiles_loaded(proj_dir);
        self.gcode_panel.export_all_setups_with_dialog(
            &self.project,
            cad_doc.as_ref(),
            cad_rev,
            &self.toolpath_cache,
            virtual_proj_path.as_deref(),
            &mut self.logs,
        );
    }

    pub(crate) fn open_post_profile_inspector(&mut self, post_id: Option<&str>) {
        let proj_dir = self.active_project_dir().map(std::path::Path::to_path_buf);
        self.gcode_panel.refresh_profiles(proj_dir.as_deref());
        let target_id = post_id.map(str::to_string).unwrap_or_else(|| {
            self.project
                .setups
                .get(self.active_setup_idx)
                .map_or_else(|| "grbl".to_string(), |s| s.post_profile.clone())
        });
        self.modals
            .post_profile_inspector
            .open_selecting(self.gcode_panel.available_profiles(), &target_id);
    }
}
