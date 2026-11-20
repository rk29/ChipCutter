use std::{
    hash::{Hash as _, Hasher as _},
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    time::Instant,
};

use chip_cad::CadDocument;
use chip_cam::{GeneratedToolpath, Operation};
use chip_math::{DVec3, DisplayUnit, MM_PER_INCH};
use chip_project::{
    ProjectDocument, ToolpathCache, collect_ready_setup_operations, resolve_kinematics_for_key,
};
use chip_sim::{
    CylindricalStock, HeightfieldStock, ScrubSnapshot, SetupSimulationInput, SetupSimulationResult,
    SimError, SimOperationInput, SimResolution, SimStockMesh, SimToolMesh, SimVerificationConfig,
    StockPlaybackCursor, build_cutter_assembly_mesh, format_cycle_time_hms, run_setup_simulation,
    tessellate_cylindrical_stock_mesh, tessellate_stock_mesh,
};
use chip_tools::{Tool, ToolId};
use crossbeam_channel::{Receiver, Sender, unbounded};

struct OwnedSimOp {
    op: Operation,
    tool: Tool,
    toolpath: Arc<GeneratedToolpath>,
}

enum SimWorkerMsg {
    Finished {
        run_id: u64,
        setup_idx: usize,
        signature: u64,
        elapsed_ms: u128,
        result: Result<Arc<SetupSimulationResult>, String>,
    },
}

/// Interactive UI & background worker state for `[Simulation Timeline]` (`06-simulation.md`).
pub struct SimulationPanelState {
    pub resolution: SimResolution,
    pub show_machined_stock: bool,
    pub color_by_operation: bool,
    pub show_cutter_assembly: bool,
    pub trail_up_to_cutter: bool,
    pub scrub_by_distance: bool,
    pub is_playing: bool,
    pub speed_multiplier: f64,
    pub current_time_sec: f64,

    next_run_id: u64,
    in_flight: Option<(u64, Arc<AtomicBool>)>,
    tx: Sender<SimWorkerMsg>,
    rx: Receiver<SimWorkerMsg>,

    active_run_id: u64,
    active_setup_idx: usize,
    active_signature: u64,
    pub active_result: Option<Arc<SetupSimulationResult>>,

    working_stock_identity: Option<(u64, usize)>,
    working_stock_cursor: Option<StockPlaybackCursor>,
    working_heightfield: Option<HeightfieldStock>,
    working_cylindrical: Option<CylindricalStock>,

    cached_stock_meta: Option<(u64, usize, bool, i32)>,
    cached_stock_rev: u64,
    cached_stock_mesh: Option<Arc<SimStockMesh>>,

    cached_tool_key: Option<(ToolId, bool)>,
    cached_tool_rev: u64,
    cached_tool_mesh: Option<Arc<SimToolMesh>>,
    next_gpu_mesh_rev: u64,
}

/// Arguments for spawning a background heightmap simulation run.
pub struct SimTriggerArgs<'a> {
    pub project: &'a ProjectDocument,
    pub setup_idx: usize,
    pub cad_doc: Arc<CadDocument>,
    pub cad_revision: u64,
    pub cache: &'a ToolpathCache,
    pub project_dir: Option<&'a std::path::Path>,
    pub egui_ctx: egui::Context,
    pub logs: &'a mut Vec<String>,
}

/// Pass #6 GPU stock, revolved cutter assembly, and trail segment cap returned by [`SimulationPanelState::resolve_viewport_visuals`].
pub struct SimViewportVisuals {
    pub stock: Option<(u64, Arc<SimStockMesh>)>,
    pub cutter: Option<(u64, Arc<SimToolMesh>, DVec3)>,
    pub trail_limit: Option<usize>,
    pub rotary_deg: f64,
}

/// Context references passed to [`SimulationPanelState::show_ui`].
pub struct SimPanelCtx<'a> {
    pub project: &'a ProjectDocument,
    pub setup_idx: usize,
    pub cad_doc: Option<&'a Arc<CadDocument>>,
    pub cad_revision: u64,
    pub cache: &'a ToolpathCache,
    pub project_dir: Option<&'a std::path::Path>,
    pub logs: &'a mut Vec<String>,
}
///Context structs above; actions emitted by `[Simulation Timeline]` back to `ChipCutterApp`.
#[derive(Debug, Clone)]
pub enum SimulationUiAction {
    FocusDiagnosticsTab,
    CenterCameraWcs(DVec3),
}

impl Default for SimulationPanelState {
    fn default() -> Self {
        let (tx, rx) = unbounded();
        Self {
            resolution: SimResolution::Standard,
            show_machined_stock: true,
            color_by_operation: true,
            show_cutter_assembly: true,
            trail_up_to_cutter: false,
            scrub_by_distance: false,
            is_playing: false,
            speed_multiplier: 1.0,
            current_time_sec: 0.0,
            next_run_id: 1,
            in_flight: None,
            tx,
            rx,
            active_run_id: 0,
            active_setup_idx: 0,
            active_signature: 0,
            active_result: None,
            working_stock_identity: None,
            working_stock_cursor: None,
            working_heightfield: None,
            working_cylindrical: None,
            cached_stock_meta: None,
            cached_stock_rev: 0,
            cached_stock_mesh: None,
            cached_tool_key: None,
            cached_tool_rev: 0,
            cached_tool_mesh: None,
            next_gpu_mesh_rev: 1,
        }
    }
}

impl SimulationPanelState {
    pub fn is_busy(&self) -> bool {
        self.in_flight.is_some()
    }

    pub fn cancel_in_flight(&mut self) {
        if let Some((_, flag)) = self.in_flight.take() {
            flag.store(true, Ordering::Relaxed);
        }
    }

    /// Computes a deterministic signature of `(setup_idx, resolution, clean toolpath hashes)`
    /// so we know immediately whether `active_result` matches the current Setup toolpaths!
    pub fn compute_setup_sim_signature(
        &self,
        project: &ProjectDocument,
        setup_idx: usize,
        cache: &ToolpathCache,
        cad_revision: u64,
    ) -> u64 {
        let mut h = std::collections::hash_map::DefaultHasher::new();
        cad_revision.hash(&mut h);
        setup_idx.hash(&mut h);
        self.resolution.hash(&mut h);
        if let Some(setup) = project.setups.get(setup_idx) {
            setup.post_profile.hash(&mut h);
            for (o_idx, op) in setup.operations.iter().enumerate() {
                op.suppressed.hash(&mut h);
                if !op.suppressed
                    && let Some((hash_key, _)) =
                        cache.get_for_operation(project, setup_idx, o_idx, cad_revision)
                {
                    hash_key.0.hash(&mut h);
                }
            }
        }
        h.finish()
    }

    pub fn is_verified_up_to_date(
        &self,
        project: &ProjectDocument,
        setup_idx: usize,
        cache: &ToolpathCache,
        cad_revision: u64,
    ) -> bool {
        self.active_result.is_some()
            && self.active_setup_idx == setup_idx
            && self.active_signature
                == self.compute_setup_sim_signature(project, setup_idx, cache, cad_revision)
    }

    /// Spawns a non-blocking `rayon` background job to run `HeightfieldStock` simulation & crash checks.
    pub fn trigger_simulation(&mut self, args: SimTriggerArgs<'_>) {
        let SimTriggerArgs {
            project,
            setup_idx,
            cad_doc,
            cad_revision,
            cache,
            project_dir,
            egui_ctx,
            logs,
        } = args;
        let Some(setup) = project.setups.get(setup_idx) else {
            return;
        };
        let resolved =
            match setup.resolve_with_sketches(&cad_doc, &project.sketches, project.display_units) {
                Ok(r) => r,
                Err(e) => {
                    logs.push(format!("Cannot simulate Setup '{}': {e}", setup.name));
                    return;
                }
            };

        let ready = collect_ready_setup_operations(project, setup_idx, cache, cad_revision);
        let owned_ops: Vec<OwnedSimOp> = ready
            .into_iter()
            .map(|r| OwnedSimOp {
                op: r.op.clone(),
                tool: r.tool.clone(),
                toolpath: r.toolpath,
            })
            .collect();

        if owned_ops.is_empty() {
            logs.push(format!(
                "Cannot simulate Setup '{}': generate clean toolpaths first (Ctrl+G).",
                setup.name
            ));
            return;
        }

        self.cancel_in_flight();
        let run_id = self.next_run_id;
        self.next_run_id += 1;
        let cancel_flag = Arc::new(AtomicBool::new(false));
        self.in_flight = Some((run_id, Arc::clone(&cancel_flag)));

        let signature = self.compute_setup_sim_signature(project, setup_idx, cache, cad_revision);
        let resolution = self.resolution;
        let target_bodies = setup.target_bodies.clone();
        let kin = if let Some(mid) = project.machine_id.as_deref()
            && let Ok(machine) = chip_tools::resolve_machine_definition(mid, project_dir)
        {
            chip_sim::MachineKinematicsConfig::from_machine_spec(&machine.kinematics)
        } else {
            resolve_kinematics_for_key(&setup.post_profile, project_dir)
        };

        logs.push(format!(
            "Simulating Setup '{}' at {} resolution on background worker…",
            setup.name,
            resolution.label()
        ));

        let tx = self.tx.clone();
        rayon::spawn(move || {
            let t0 = Instant::now();
            let refs: Vec<SimOperationInput<'_>> = owned_ops
                .iter()
                .map(|item| {
                    SimOperationInput::from_operation(&item.op, &item.tool, item.toolpath.as_ref())
                })
                .collect();

            let res = run_setup_simulation(&SetupSimulationInput {
                doc: &cad_doc,
                setup: &resolved,
                target_bodies: &target_bodies,
                operations: &refs,
                resolution,
                kinematics: kin,
                verify_config: SimVerificationConfig::default(),
                cancel: Some(&cancel_flag),
            });
            let mapped = match res {
                Ok(sim) => Ok(Arc::new(sim)),
                Err(SimError::Cancelled) => return,
                Err(e) => Err(e.to_string()),
            };
            let _ = tx.send(SimWorkerMsg::Finished {
                run_id,
                setup_idx,
                signature,
                elapsed_ms: t0.elapsed().as_millis(),
                result: mapped,
            });
            egui_ctx.request_repaint();
        });
    }

    /// Polls background simulation completion messages without blocking.
    pub fn poll_worker(&mut self, logs: &mut Vec<String>) -> bool {
        let mut completed_new = false;
        while let Ok(msg) = self.rx.try_recv() {
            match msg {
                SimWorkerMsg::Finished {
                    run_id,
                    setup_idx,
                    signature,
                    elapsed_ms,
                    result,
                } => {
                    if self.in_flight.as_ref().is_some_and(|(id, _)| *id == run_id) {
                        self.in_flight = None;
                    }
                    match result {
                        Ok(sim) => {
                            let crit = sim.critical_error_count();
                            let warn = sim.warning_count();
                            logs.push(format!(
                                "Simulation completed in {elapsed_ms} ms — Cycle {}, Removed {:.0} mm^3 ({crit} critical error(s), {warn} warning(s)).",
                                format_cycle_time_hms(sim.timeline.total_time_sec),
                                sim.removed_volume_mm3
                            ));
                            self.current_time_sec = sim.timeline.total_time_sec;
                            self.is_playing = false;
                            self.active_run_id = run_id;
                            self.active_setup_idx = setup_idx;
                            self.active_signature = signature;
                            self.active_result = Some(sim);
                            self.clear_stock_display_cache();
                            completed_new = true;
                        }
                        Err(err) => {
                            logs.push(format!("ERROR during heightmap simulation: {err}"));
                        }
                    }
                }
            }
        }
        completed_new
    }

    /// Jumps timeline scrubber to `time_sec` (pausing playback and enabling 3D stock & cutter display).
    pub fn jump_to_time(&mut self, time_sec: f64) {
        self.is_playing = false;
        self.show_machined_stock = true;
        self.show_cutter_assembly = true;
        if let Some(res) = &self.active_result {
            self.current_time_sec = time_sec.clamp(0.0, res.timeline.total_time_sec);
        } else {
            self.current_time_sec = time_sec.max(0.0);
        }
    }

    /// Advances playback clock when `is_playing` and handles `Space` / `←` / `→` shortcuts when no text field has focus.
    pub fn update_playback_and_shortcuts(&mut self, ctx: &egui::Context) {
        let Some(res) = self.active_result.clone() else {
            self.is_playing = false;
            return;
        };

        if !ctx.egui_wants_keyboard_input() {
            ctx.input(|inp| {
                if inp.key_pressed(egui::Key::Space) && !inp.modifiers.any() {
                    if !self.is_playing
                        && self.current_time_sec >= res.timeline.total_time_sec - 1e-4
                    {
                        self.current_time_sec = 0.0;
                    }
                    self.is_playing = !self.is_playing;
                }
                if inp.key_pressed(egui::Key::ArrowRight) && !inp.modifiers.any() {
                    self.is_playing = false;
                    let cur_seg = res
                        .timeline
                        .sample_at_time(self.current_time_sec)
                        .map_or(0, |s| s.global_seg_idx);
                    self.current_time_sec = res.timeline.step_segment_time(cur_seg, 1);
                }
                if inp.key_pressed(egui::Key::ArrowLeft) && !inp.modifiers.any() {
                    self.is_playing = false;
                    let cur_seg = res
                        .timeline
                        .sample_at_time(self.current_time_sec)
                        .map_or(0, |s| s.global_seg_idx);
                    self.current_time_sec = res.timeline.step_segment_time(cur_seg, -1);
                }
            });
        }

        if self.is_playing {
            let dt = f64::from(ctx.input(|i| i.stable_dt.clamp(0.001, 0.1)));
            // Fast-forward long M6 tool-change pauses to ~1.5 s visual duration while preserving
            // exact shop-floor cycle time and smooth animated tool retract/swap/descent.
            let tc_end = res
                .timeline
                .sample_at_time(self.current_time_sec)
                .and_then(|s| s.tool_change.map(|tc| tc.end_time_sec));
            if let Some(end_t) = tc_end {
                let tc_speed = self.speed_multiplier.max(10.0);
                let next_t = self.current_time_sec + dt * tc_speed;
                self.current_time_sec = if self.speed_multiplier < 10.0 {
                    next_t.min(end_t)
                } else {
                    next_t
                };
            } else {
                self.current_time_sec += dt * self.speed_multiplier;
            }
            if self.current_time_sec >= res.timeline.total_time_sec {
                self.current_time_sec = res.timeline.total_time_sec;
                self.is_playing = false;
            } else {
                ctx.request_repaint();
            }
        }
    }

    fn clear_stock_display_cache(&mut self) {
        self.working_stock_identity = None;
        self.working_stock_cursor = None;
        self.working_heightfield = None;
        self.working_cylindrical = None;
        self.cached_stock_meta = None;
        self.cached_stock_mesh = None;
    }

    /// Produces the cached Pass #6 GPU stock mesh, 3D revolved cutter mesh + tip position, and trail limit.
    pub fn resolve_viewport_visuals(
        &mut self,
        project: &ProjectDocument,
        setup_idx: usize,
    ) -> SimViewportVisuals {
        let Some(res) = self.active_result.as_ref().map(Arc::clone) else {
            if self.working_stock_identity.is_some() || self.cached_stock_mesh.is_some() {
                self.clear_stock_display_cache();
            }
            return SimViewportVisuals {
                stock: None,
                cutter: None,
                trail_limit: None,
                rotary_deg: 0.0,
            };
        };
        if self.active_setup_idx != setup_idx {
            return SimViewportVisuals {
                stock: None,
                cutter: None,
                trail_limit: None,
                rotary_deg: 0.0,
            };
        }

        let snap = res.timeline.sample_at_time(self.current_time_sec);
        let result_ptr = Arc::as_ptr(&res) as usize;
        let run_identity = (self.active_run_id, result_ptr);

        // 1. 3D Machined Stock Mesh (synchronized to exact cutter timestamp on every frame)
        let stock_out = if self.show_machined_stock {
            let rotary_deg = snap.as_ref().map_or(0.0, |s| s.rotary_deg);
            let rotary_deg_hundredths = if res.final_cylindrical_stock.is_some() {
                (rotary_deg * 100.0).round() as i32
            } else {
                0
            };
            let desired_meta = (
                self.active_run_id,
                result_ptr,
                self.color_by_operation,
                rotary_deg_hundredths,
            );

            if self.working_stock_identity != Some(run_identity) {
                self.working_stock_identity = Some(run_identity);
                self.working_stock_cursor = Some(StockPlaybackCursor {
                    seg_idx: res.timeline.motion_spans.len(),
                    seg_u: 0.0,
                });
                if let Some(cyl_final) = &res.final_cylindrical_stock {
                    self.working_cylindrical = Some(cyl_final.clone());
                    self.working_heightfield = None;
                } else {
                    self.working_heightfield = Some(res.final_stock.clone());
                    self.working_cylindrical = None;
                }
                self.cached_stock_mesh = None;
                self.cached_stock_meta = None;
            }

            let stock_changed = if let Some(cyl_work) = self.working_cylindrical.as_mut() {
                res.sync_cylindrical_stock_to_time(
                    cyl_work,
                    &mut self.working_stock_cursor,
                    self.current_time_sec,
                )
            } else if let Some(hf_work) = self.working_heightfield.as_mut() {
                res.sync_stock_to_time(
                    hf_work,
                    &mut self.working_stock_cursor,
                    self.current_time_sec,
                )
            } else {
                false
            };

            let needs_remesh = stock_changed
                || self.cached_stock_mesh.is_none()
                || self.cached_stock_meta != Some(desired_meta);

            if needs_remesh {
                let mesh = if let Some(cyl_work) = self.working_cylindrical.as_ref() {
                    tessellate_cylindrical_stock_mesh(cyl_work, self.color_by_operation, rotary_deg)
                } else if let Some(hf_work) = self.working_heightfield.as_ref() {
                    tessellate_stock_mesh(hf_work, self.color_by_operation)
                } else {
                    SimStockMesh::default()
                };

                let rev = self.next_gpu_mesh_rev;
                self.next_gpu_mesh_rev += 1;
                self.cached_stock_meta = Some(desired_meta);
                self.cached_stock_rev = rev;
                self.cached_stock_mesh = Some(Arc::new(mesh));
            }

            self.cached_stock_mesh
                .as_ref()
                .map(|m| (self.cached_stock_rev, Arc::clone(m)))
        } else {
            None
        };

        // 2. 3D Revolved Cutter + Shank + Holder Mesh
        let tool_out = if self.show_cutter_assembly
            && let Some(s) = &snap
            && let Some(tool) = project.tools.iter().find(|t| t.id == s.tool_id)
        {
            let has_crash_here = res.diagnostics.iter().any(|d| {
                d.global_seg_idx.abs_diff(s.global_seg_idx) <= 2
                    && d.severity == chip_sim::SimSeverity::CriticalError
            });
            let tkey = (tool.id.clone(), has_crash_here);
            if self.cached_tool_key.as_ref() != Some(&tkey) {
                let rev = self.next_gpu_mesh_rev;
                self.next_gpu_mesh_rev += 1;
                self.cached_tool_key = Some(tkey);
                self.cached_tool_rev = rev;
                self.cached_tool_mesh = Some(Arc::new(build_cutter_assembly_mesh(
                    tool,
                    28,
                    has_crash_here,
                )));
            }
            self.cached_tool_mesh
                .as_ref()
                .map(|m| (self.cached_tool_rev, Arc::clone(m), s.tip_pos_wcs))
        } else {
            None
        };

        // 3. Optional progressive toolpath trail limit
        let trail_out = if self.trail_up_to_cutter {
            snap.as_ref().map(|s| s.global_seg_idx)
        } else {
            None
        };

        let sim_rot_deg = if self.show_machined_stock && res.final_cylindrical_stock.is_some() {
            snap.as_ref().map_or(0.0, |s| s.rotary_deg)
        } else {
            0.0
        };

        SimViewportVisuals {
            stock: stock_out,
            cutter: tool_out,
            trail_limit: trail_out,
            rotary_deg: sim_rot_deg,
        }
    }

    /// Renders the bottom `[Simulation Timeline]` workspace tab (`06-simulation.md` §1–4).
    pub fn show_ui(
        &mut self,
        ui: &mut egui::Ui,
        ctx: SimPanelCtx<'_>,
    ) -> Option<SimulationUiAction> {
        let SimPanelCtx {
            project,
            setup_idx,
            cad_doc,
            cad_revision,
            cache,
            project_dir,
            logs,
        } = ctx;
        let mut action: Option<SimulationUiAction> = None;
        let display_unit = project.display_units;
        let prev_time_sec = self.current_time_sec;
        let prev_vis_flags = (
            self.show_machined_stock,
            self.color_by_operation,
            self.show_cutter_assembly,
            self.trail_up_to_cutter,
        );

        // Top Control Bar: Run Simulation, Resolution, Visual Toggles, Verification Status
        ui.horizontal_wrapped(|ui| {
            let busy = self.is_busy();
            let btn_lbl = if busy {
                "Simulating…"
            } else if self.active_result.is_some() {
                "Refresh Simulation"
            } else {
                "Run Simulation"
            };
            if ui
                .add_enabled(!busy && cad_doc.is_some(), egui::Button::new(btn_lbl))
                .clicked()
                && let Some(doc) = cad_doc
            {
                self.trigger_simulation(SimTriggerArgs {
                    project,
                    setup_idx,
                    cad_doc: Arc::clone(doc),
                    cad_revision,
                    cache,
                    project_dir,
                    egui_ctx: ui.ctx().clone(),
                    logs,
                });
            }
            if busy {
                ui.spinner();
                if ui.small_button("Cancel").clicked() {
                    self.cancel_in_flight();
                }
            }

            ui.separator();
            egui::ComboBox::from_id_salt("sim_res_combo")
                .selected_text(self.resolution.label())
                .width(128.0)
                .show_ui(ui, |ui| {
                    for r in SimResolution::ALL {
                        ui.selectable_value(&mut self.resolution, r, r.label());
                    }
                });

            ui.checkbox(&mut self.show_machined_stock, "3D Stock");
            ui.checkbox(&mut self.color_by_operation, "Color by Op");
            ui.checkbox(&mut self.show_cutter_assembly, "3D Cutter");
            ui.checkbox(&mut self.trail_up_to_cutter, "Progressive Trail");

            ui.separator();
            let tok = crate::theme::ThemeTokens::from_ui(ui);
            let up_to_date = self.is_verified_up_to_date(project, setup_idx, cache, cad_revision);
            if let Some(res) = &self.active_result
                && self.active_setup_idx == setup_idx
            {
                let crit = res.critical_error_count();
                let warn = res.warning_count();
                if crit > 0 {
                    crate::widgets::status_badge_icon(
                        ui,
                        crate::widgets::StatusBadgeKind::Error,
                        None,
                    );
                    if ui
                        .button(
                            egui::RichText::new(format!("{crit} CRASH/GOUGE(S) — Inspect"))
                                .color(tok.error_fg)
                                .strong(),
                        )
                        .clicked()
                    {
                        action = Some(SimulationUiAction::FocusDiagnosticsTab);
                    }
                } else if !up_to_date {
                    crate::widgets::status_badge_icon(
                        ui,
                        crate::widgets::StatusBadgeKind::Dirty,
                        None,
                    );
                    ui.colored_label(tok.dirty_fg, "Toolpaths changed — Refresh Simulation");
                } else if warn > 0 {
                    crate::widgets::status_badge_icon(
                        ui,
                        crate::widgets::StatusBadgeKind::NeedsRepick,
                        None,
                    );
                    if ui
                        .button(
                            egui::RichText::new(format!("0 Crashes ({warn} Warning)"))
                                .color(tok.warn_fg),
                        )
                        .clicked()
                    {
                        action = Some(SimulationUiAction::FocusDiagnosticsTab);
                    }
                } else {
                    crate::widgets::status_badge_icon(
                        ui,
                        crate::widgets::StatusBadgeKind::Clean,
                        None,
                    );
                    ui.colored_label(tok.clean_fg, "Verified Crash-Free");
                }
            } else {
                ui.weak("Click 'Run Simulation' to compute 3D material removal & crash checks.");
            }
        });

        ui.separator();

        let Some(res) = self.active_result.clone() else {
            ui.weak("No heightmap simulation active for this Setup yet.");
            return action;
        };
        if self.active_setup_idx != setup_idx || res.timeline.is_empty() {
            ui.weak("Click 'Run Simulation' to simulate the currently selected Setup.");
            return action;
        }

        let snap: Option<ScrubSnapshot> = res.timeline.sample_at_time(self.current_time_sec);
        let cur_seg = snap.as_ref().map_or(0, |s| s.global_seg_idx);

        // Middle Row: Vector Transport Controls + Speed + Continuous Scrubber Slider
        ui.horizontal_wrapped(|ui| {
            use crate::widgets::{IconKind, icon_button};
            if icon_button(ui, IconKind::SkipStart, None, "Jump to Start").clicked() {
                self.is_playing = false;
                self.current_time_sec = 0.0;
            }
            if icon_button(
                ui,
                IconKind::StepBack,
                Some("Seg"),
                "Step Backward 1 Segment (Left Arrow)",
            )
            .clicked()
            {
                self.is_playing = false;
                self.current_time_sec = res.timeline.step_segment_time(cur_seg, -1);
            }
            let (play_icon, play_txt) = if self.is_playing {
                (IconKind::Pause, "Pause")
            } else {
                (IconKind::Play, "Play")
            };
            if icon_button(
                ui,
                play_icon,
                Some(play_txt),
                "Play / Pause Scrubbing (Space)",
            )
            .clicked()
            {
                if !self.is_playing && self.current_time_sec >= res.timeline.total_time_sec - 1e-4 {
                    self.current_time_sec = 0.0;
                }
                self.is_playing = !self.is_playing;
            }
            if icon_button(
                ui,
                IconKind::StepForward,
                Some("Seg"),
                "Step Forward 1 Segment (Right Arrow)",
            )
            .clicked()
            {
                self.is_playing = false;
                self.current_time_sec = res.timeline.step_segment_time(cur_seg, 1);
            }
            if icon_button(
                ui,
                IconKind::SkipNextOp,
                Some("Next Op"),
                "Jump to Next Operation",
            )
            .clicked()
            {
                self.is_playing = false;
                self.current_time_sec = res.timeline.next_operation_time(self.current_time_sec);
            }
            if icon_button(
                ui,
                IconKind::Tool,
                Some("Next Tool"),
                "Jump to Next Tool Change",
            )
            .clicked()
            {
                self.is_playing = false;
                self.current_time_sec = res.timeline.next_tool_change_time(self.current_time_sec);
            }
            if icon_button(
                ui,
                IconKind::SkipEnd,
                Some("End"),
                "Instant Jump to Final Machined Stock",
            )
            .clicked()
            {
                self.is_playing = false;
                self.current_time_sec = res.timeline.total_time_sec;
            }

            ui.separator();
            ui.label("Speed:");
            egui::ComboBox::from_id_salt("sim_speed_combo")
                .selected_text(format!("{}x", self.speed_multiplier))
                .width(64.0)
                .show_ui(ui, |ui| {
                    for spd in [0.1, 0.5, 1.0, 5.0, 10.0, 25.0, 50.0] {
                        ui.selectable_value(&mut self.speed_multiplier, spd, format!("{spd}x"));
                    }
                });

            ui.selectable_value(&mut self.scrub_by_distance, false, "Time");
            ui.selectable_value(&mut self.scrub_by_distance, true, "Dist");
        });

        // Continuous Timeline / Distance Slider
        ui.horizontal(|ui| {
            ui.spacing_mut().slider_width = (ui.available_width() - 180.0).max(120.0);
            if self.scrub_by_distance {
                let mut cur_d = snap.as_ref().map_or(0.0, |s| s.distance_mm);
                let max_d = res.timeline.total_distance_mm.max(0.001);
                let resp = ui.add(
                    egui::Slider::new(&mut cur_d, 0.0..=max_d)
                        .show_value(false)
                        .trailing_fill(true),
                );
                if resp.changed() {
                    self.is_playing = false;
                    if let Some(ds) = res.timeline.sample_at_distance(cur_d) {
                        self.current_time_sec = ds.time_sec;
                    }
                }
                ui.monospace(format!(
                    "{} / {}",
                    display_unit.format_length(cur_d),
                    display_unit.format_length(max_d)
                ));
            } else {
                let max_t = res.timeline.total_time_sec.max(0.001);
                let resp = ui.add(
                    egui::Slider::new(&mut self.current_time_sec, 0.0..=max_t)
                        .show_value(false)
                        .trailing_fill(true),
                );
                if resp.changed() {
                    self.is_playing = false;
                }
                ui.monospace(format!(
                    "{} / {}",
                    format_cycle_time_hms(self.current_time_sec),
                    format_cycle_time_hms(res.timeline.total_time_sec)
                ));
            }
        });

        // Bottom Live Controller HUD Readout (`Op`, `Tool`, `WCS X/Y/Z`, `G0..G3`, `Feed`, `RPM`, `Seg`)
        if let Some(s) = snap {
            if let Some(tc) = &s.tool_change {
                let tok = crate::theme::ThemeTokens::from_ui(ui);
                ui.horizontal_wrapped(|ui| {
                    ui.colored_label(
                        tok.warn_fg,
                        egui::RichText::new(format!(
                            "M6 Tool Change: T{} -> T{} ({}) · {:.1}s left",
                            tc.from_tool_number,
                            tc.to_tool_number,
                            tc.to_tool_name,
                            tc.remaining_sec
                        ))
                        .strong(),
                    );
                    ui.separator();
                    ui.label(format!(
                        "Next: Op {}: {}",
                        tc.next_op_idx + 1,
                        tc.next_op_name
                    ));
                    ui.separator();
                    if let Some(cyl) = &res.final_cylindrical_stock {
                        ui.monospace(format!(
                            "WCS X={} Y={} Z={} {}={:.1}°",
                            display_unit.format_length(s.tip_pos_wcs.x),
                            display_unit.format_length(s.tip_pos_wcs.y),
                            display_unit.format_length(s.tip_pos_wcs.z),
                            cyl.rotary.axis_letter,
                            s.rotary_deg,
                        ));
                    } else {
                        ui.monospace(format!(
                            "WCS X={} Y={} Z={}",
                            display_unit.format_length(s.tip_pos_wcs.x),
                            display_unit.format_length(s.tip_pos_wcs.y),
                            display_unit.format_length(s.tip_pos_wcs.z),
                        ));
                    }
                    if ui
                        .small_button("Skip Tool Change")
                        .on_hover_text("Jump immediately to the first move of the next operation")
                        .clicked()
                    {
                        self.current_time_sec = tc.end_time_sec;
                    }
                });
            } else {
                let feed_disp = match display_unit {
                    DisplayUnit::Millimeter => format!("{:.0} mm/min", s.commanded_feed_mm_min),
                    DisplayUnit::Inch => {
                        format!("{:.1} ipm", s.commanded_feed_mm_min / MM_PER_INCH)
                    }
                };
                ui.horizontal_wrapped(|ui| {
                    ui.strong(format!("Op {}: {}", s.op_idx + 1, s.op_name));
                    ui.separator();
                    ui.monospace(format!("T{} ({})", s.tool_number, s.tool_name));
                    ui.separator();
                    if let Some(cyl) = &res.final_cylindrical_stock {
                        ui.monospace(format!(
                            "WCS X={} Y={} Z={} {}={:.1}°",
                            display_unit.format_length(s.tip_pos_wcs.x),
                            display_unit.format_length(s.tip_pos_wcs.y),
                            display_unit.format_length(s.tip_pos_wcs.z),
                            cyl.rotary.axis_letter,
                            s.rotary_deg,
                        ));
                    } else {
                        ui.monospace(format!(
                            "WCS X={} Y={} Z={}",
                            display_unit.format_length(s.tip_pos_wcs.x),
                            display_unit.format_length(s.tip_pos_wcs.y),
                            display_unit.format_length(s.tip_pos_wcs.z),
                        ));
                    }
                    ui.separator();
                    ui.monospace(format!(
                        "{} | F={} | S={:.0} RPM | Seg {}/{}",
                        s.motion_code,
                        feed_disp,
                        s.spindle_rpm,
                        s.global_seg_idx + 1,
                        res.timeline.motion_spans.len()
                    ));
                    if ui.small_button("Center on Tool").clicked() {
                        action = Some(SimulationUiAction::CenterCameraWcs(s.tip_pos_wcs));
                    }
                });
            }
        }

        if (self.current_time_sec - prev_time_sec).abs() > 1e-9
            || (
                self.show_machined_stock,
                self.color_by_operation,
                self.show_cutter_assembly,
                self.trail_up_to_cutter,
            ) != prev_vis_flags
        {
            ui.ctx().request_repaint();
        }

        action
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::sync::Arc;

    use chip_cad::CadDocument;
    use chip_cam::{
        AxisAnchor, BoxAnchor27, MotionRole, RotaryAxisKind, RotaryDriveMode, Setup, StockAxis,
        StockConfig, ToolpathBuilder, WcsOriginMode, WorkOffsetSlot,
    };
    use chip_math::{DVec3, DisplayUnit, ParamLength};
    use chip_project::ProjectDocument;
    use chip_sim::{
        MachineKinematicsConfig, SetupSimulationInput, SimOperationInput, SimResolution,
        SimVerificationConfig, run_setup_simulation,
    };
    use chip_tools::default_starter_tool_library;

    use super::SimulationPanelState;

    #[test]
    fn playback_updates_stock_every_frame_within_single_cutting_segment_without_lag() {
        let mut project = ProjectDocument::new("Sim Sync Test");
        let lib = default_starter_tool_library();
        project.tools = lib.tools.clone();
        let flat_6mm = &lib.tools[0];

        let mut setup = Setup::new("s1", "Setup 1", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        };
        setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(100.0),
            depth_y: ParamLength::from_mm(100.0),
            height_z: ParamLength::from_mm(20.0),
            xy_align: chip_cam::FixedBoxXyAlign::CenterXy,
            z_align: chip_cam::FixedBoxZAlign::CenterZ,
        };
        let doc = CadDocument::empty();
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

        // Seg 0: air rapid above stock; Seg 1: plunge; Seg 2: single long 80 mm cut across X; Seg 3: retract in air.
        let mut b = ToolpathBuilder::new("op_slot");
        b.rapid_to(DVec3::new(-40.0, 0.0, 10.0));
        b.rapid_to(DVec3::new(-40.0, 0.0, 2.0));
        b.linear_to(DVec3::new(-40.0, 0.0, -5.0), 300.0, MotionRole::Plunge);
        b.linear_to(DVec3::new(40.0, 0.0, -5.0), 600.0, MotionRole::Cut);
        b.rapid_to(DVec3::new(40.0, 0.0, 10.0));
        b.rapid_to(DVec3::new(-40.0, 0.0, 10.0));
        let tp = b.finish();

        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            operations: &[SimOperationInput {
                op_id: "op_slot",
                op_name: "Long Slot Cut",
                tool: flat_6mm,
                fallback_spindle_rpm: 12_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp,
            }],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            cancel: None,
        })
        .unwrap();

        let cut_start = sim.timeline.motion_spans[2].t_start_sec;
        let cut_dur = sim.timeline.motion_spans[2].t_end_sec - cut_start;
        assert!(cut_dur > 5.0);
        let air_start = sim.timeline.motion_spans[4].t_start_sec;
        let air_end = sim.timeline.motion_spans[4].t_end_sec;

        let mut panel = SimulationPanelState {
            active_run_id: 1,
            active_setup_idx: 0,
            active_result: Some(Arc::new(sim)),
            is_playing: true,
            ..Default::default()
        };

        // Advance through 4 consecutive frames inside the SAME cutting segment (0 ms wall clock apart!)
        let mut prev_rev = 0u64;
        for frac in [0.15_f64, 0.40, 0.65, 0.90] {
            panel.current_time_sec = cut_start + cut_dur * frac;
            let vis = panel.resolve_viewport_visuals(&project, 0);
            let (rev, _mesh) = vis.stock.expect("stock mesh should be present");
            let (_, _, tip_wcs) = vis.cutter.expect("cutter visual should be present");

            assert!(
                rev > prev_rev,
                "stock mesh revision must advance immediately at frac={frac} (prev={prev_rev}, got={rev})"
            );
            prev_rev = rev;

            // Working heightfield must already be cut directly beneath the cutter tip,
            // and remain uncut well ahead of the cutter!
            let hf = panel.working_heightfield.as_ref().unwrap();
            let z_at_tip = hf.sample_stock_z(tip_wcs.x, 0.0).unwrap();
            assert!(
                (z_at_tip - (-5.0)).abs() < 0.05,
                "stock directly beneath cutter at x={:.2} must be cut to -5.0, got {z_at_tip:.2}",
                tip_wcs.x
            );
            if tip_wcs.x + 8.0 < 48.0 {
                let z_ahead = hf.sample_stock_z(tip_wcs.x + 8.0, 0.0).unwrap();
                assert!(
                    z_ahead > -0.1,
                    "stock ahead of cutter at x={:.2} must still be uncut, got {z_ahead:.2}",
                    tip_wcs.x + 8.0
                );
            }
        }

        // Subsequent air-only rapid move must NOT trigger unnecessary stock remeshing
        panel.current_time_sec = 0.5 * (air_start + air_end);
        let vis_air1 = panel.resolve_viewport_visuals(&project, 0);
        let rev_after_cut = vis_air1.stock.unwrap().0;
        panel.current_time_sec = air_start + 0.85 * (air_end - air_start);
        let vis_air2 = panel.resolve_viewport_visuals(&project, 0);
        assert_eq!(
            vis_air2.stock.unwrap().0,
            rev_after_cut,
            "air rapid move should not unnecessarily remesh unchanged stock"
        );
    }

    #[test]
    fn playback_updates_4th_axis_cylindrical_stock_between_checkpoints_without_lag() {
        let mut project = ProjectDocument::new("Rotary Sync Test");
        let lib = default_starter_tool_library();
        project.tools = lib.tools.clone();
        let flat_6mm = &lib.tools[0];

        let mut setup = Setup::new("s_rot", "Rotary A", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27 {
                x: AxisAnchor::Center,
                y: AxisAnchor::Center,
                z: AxisAnchor::Center,
            },
        };
        setup.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(40.0),
            length: ParamLength::from_mm(80.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup.rotary.mode = RotaryDriveMode::TwoLinearPlusRotary;
        setup.rotary.axis = RotaryAxisKind::XAxis;

        let doc = CadDocument::empty();
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

        let mut b = ToolpathBuilder::new("op_rot");
        b.rotary_rapid_to(DVec3::new(-25.0, 0.0, 28.0), 0.0);
        b.rotary_linear_to(
            DVec3::new(-25.0, 0.0, 16.0),
            0.0,
            400.0,
            16.0,
            MotionRole::Plunge,
        );
        b.rotary_linear_to(
            DVec3::new(25.0, 0.0, 16.0),
            0.0,
            600.0,
            16.0,
            MotionRole::Cut,
        );
        let tp = b.finish();

        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            operations: &[SimOperationInput {
                op_id: "op_rot",
                op_name: "Rotary Axial Flute",
                tool: flat_6mm,
                fallback_spindle_rpm: 12_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp,
            }],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            cancel: None,
        })
        .unwrap();

        let cut_start = sim.timeline.motion_spans[1].t_start_sec;
        let cut_dur = sim.timeline.motion_spans[1].t_end_sec - cut_start;

        let mut panel = SimulationPanelState {
            active_run_id: 1,
            active_setup_idx: 0,
            active_result: Some(Arc::new(sim)),
            is_playing: true,
            ..Default::default()
        };

        panel.current_time_sec = cut_start + cut_dur * 0.25;
        let rev_25 = panel.resolve_viewport_visuals(&project, 0).stock.unwrap().0;
        let vol_25 = panel
            .working_cylindrical
            .as_ref()
            .unwrap()
            .current_volume_mm3();

        panel.current_time_sec = cut_start + cut_dur * 0.75;
        let rev_75 = panel.resolve_viewport_visuals(&project, 0).stock.unwrap().0;
        let vol_75 = panel
            .working_cylindrical
            .as_ref()
            .unwrap()
            .current_volume_mm3();

        assert!(
            rev_75 > rev_25,
            "cylindrical stock mesh must update smoothly mid-segment (rev_25={rev_25}, rev_75={rev_75})"
        );
        assert!(
            vol_75 < vol_25 - 100.0,
            "cylindrical stock volume must decrease as cutter advances mid-segment (vol_25={vol_25:.1}, vol_75={vol_75:.1})"
        );
    }
}
