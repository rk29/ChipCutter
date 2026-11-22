use std::sync::Arc;

use chip_math::{DMat4, DVec3};
use chip_project::ProjectDocument;
use chip_sim::{ScrubSnapshot, SetupSimulationResult, build_cutter_assembly_mesh};

use crate::viewport::SimStockRenderSource;

use super::{
    SimViewportVisuals, SimulationPanelState,
    worker::{StockMeshMeta, StockRemeshKey, StockRemeshPayload, StockRemeshRequest},
};

impl SimulationPanelState {
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
        self.repaint_ctx = Some(ctx.clone());
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

    /// Drains any completed stock-remesh meshes from the background worker without blocking.
    /// Returns `true` if a new stock mesh revision was installed.
    pub fn poll_remesh_outputs(&mut self) -> bool {
        let active_identity = self.active_run_identity();
        let mut updated = false;
        while let Some(out) = self.remesh_worker.try_recv() {
            if out.request_id < self.min_accept_remesh_request_id
                || out.request_id < self.last_applied_remesh_request_id
                || active_identity != Some((out.run_id, out.result_ptr))
            {
                continue;
            }
            self.last_applied_remesh_request_id = out.request_id;
            if let Some(payload) = out.payload {
                let rev = self.next_gpu_mesh_rev;
                self.next_gpu_mesh_rev += 1;
                self.cached_stock_meta = Some(out.meta);
                self.cached_stock_rev = rev;
                match payload {
                    StockRemeshPayload::CpuMesh(mesh) => {
                        self.cached_stock_mesh = Some(mesh);
                        #[cfg(feature = "gpu-compute")]
                        {
                            self.cached_gpu_handle = None;
                        }
                    }
                    #[cfg(feature = "gpu-compute")]
                    StockRemeshPayload::GpuStockSnapshot(stock) => {
                        if let Some(render_ctx) = self.gpu_ctx.as_ref() {
                            let target_td = self
                                .active_result
                                .as_ref()
                                .and_then(|r| r.target_tridexel_stock.as_ref());
                            let color_mode = self.active_stock_color_mode();
                            let handle = chip_sim::GpuStockRenderHandle::from_simulated_stock(
                                render_ctx, &stock, target_td, color_mode,
                            );
                            self.cached_gpu_handle = Some(handle);
                            self.cached_stock_mesh = None;
                        } else if let Some(res) = self.active_result.as_ref() {
                            let color_mode = self.active_stock_color_mode();
                            let mesh = res.extract_stock_mesh(&stock, color_mode);
                            self.cached_stock_mesh = Some(Arc::new(mesh));
                            self.cached_gpu_handle = None;
                        }
                    }
                }
                updated = true;
            }
        }
        updated
    }

    /// Waits deterministically for any in-flight background stock remesh to finish and applies
    /// its result. Used by headless `--capture-*` screenshots and deterministic tests.
    pub fn flush_remesh_sync(&mut self) {
        self.remesh_worker.wait_idle();
        let _ = self.poll_remesh_outputs();
    }

    /// Non-blocking interactive viewport visual resolver (`egui` UI thread).
    ///
    /// Dispatches stock scrub synchronization and mesh extraction onto the background remesh
    /// worker (coalescing rapid scrubs) and immediately returns the latest ready
    /// [`SimStockRenderSource`] with up-to-date GPU `stock_model` transform.
    pub fn resolve_viewport_visuals(
        &mut self,
        project: &ProjectDocument,
        setup_idx: usize,
    ) -> SimViewportVisuals {
        self.resolve_viewport_visuals_impl(project, setup_idx, false)
    }

    /// Deterministic synchronous viewport visual resolver for headless `--capture-*` screenshots
    /// and deterministic tests. Guarantees the returned [`SimStockRenderSource`] matches the exact
    /// requested timestamp and shading mode with zero blank or lagged frames.
    pub fn resolve_viewport_visuals_sync(
        &mut self,
        project: &ProjectDocument,
        setup_idx: usize,
    ) -> SimViewportVisuals {
        self.resolve_viewport_visuals_impl(project, setup_idx, true)
    }

    fn resolve_viewport_visuals_impl(
        &mut self,
        project: &ProjectDocument,
        setup_idx: usize,
        synchronous: bool,
    ) -> SimViewportVisuals {
        let Some(res) = self.active_result.as_ref().map(Arc::clone) else {
            if self.cached_stock_run_identity.is_some() || self.cached_stock_mesh.is_some() {
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

        if self.cached_stock_run_identity != Some(run_identity) {
            self.clear_stock_display_cache();
            self.cached_stock_run_identity = Some(run_identity);
        }

        let _ = self.poll_remesh_outputs();

        let sim_rot_deg = if self.show_machined_stock && res.is_cylindrical() {
            snap.as_ref().map_or(0.0, |s| s.rotary_deg)
        } else {
            0.0
        };
        let stock_model = compute_stock_model_wcs(&res, sim_rot_deg);

        // 1. 3D Machined Stock Render Source (off-thread coalescing interactively; synchronous in headless capture)
        let stock_out = if self.show_machined_stock {
            let color_mode = self.active_stock_color_mode();
            let is_gpu_direct = self.is_gpu_direct_active();
            let desired_meta = if is_gpu_direct {
                // On GPU direct rendering, color mode / heatmap tolerance changes update GPU uniforms
                // or dispatch < 0.3 ms tridexel_emit directly—NEVER re-sweeping or remeshing on CPU!
                StockMeshMeta {
                    run_id: self.active_run_id,
                    result_ptr,
                    color_by_operation: false,
                    heatmap_active: false,
                    tol_key: 0,
                    scale_key: 0,
                }
            } else {
                let tol_key =
                    (self.heatmap_tolerance_mm.clamp(1e-4, 50.0) * 10_000.0).round() as u32;
                let scale_key =
                    (self.heatmap_max_scale_mm.clamp(0.01, 100.0) * 1000.0).round() as u32;
                StockMeshMeta {
                    run_id: self.active_run_id,
                    result_ptr,
                    color_by_operation: self.color_by_operation,
                    heatmap_active: self.show_error_heatmap && !self.is_playing,
                    tol_key,
                    scale_key,
                }
            };
            let desired_key = StockRemeshKey {
                meta: desired_meta,
                seg_idx: snap.as_ref().map_or(usize::MAX, |s| s.global_seg_idx),
                seg_u_bits: snap.as_ref().map_or(0, |s| s.seg_progress_t.to_bits()),
                time_bits: self.current_time_sec.to_bits(),
            };

            if self.last_submitted_remesh_key != Some(desired_key) {
                let req_id = self.next_remesh_request_id;
                self.next_remesh_request_id += 1;
                self.last_submitted_remesh_key = Some(desired_key);
                self.remesh_worker.submit(StockRemeshRequest {
                    request_id: req_id,
                    run_id: self.active_run_id,
                    result_ptr,
                    sim_result: Arc::clone(&res),
                    target_time_sec: self.current_time_sec,
                    scrub_snapshot: snap.clone(),
                    color_mode,
                    meta: desired_meta,
                    egui_ctx: if synchronous {
                        None
                    } else {
                        self.repaint_ctx.clone()
                    },
                });
            }

            if synchronous {
                self.flush_remesh_sync();
                #[cfg(feature = "gpu-compute")]
                if let Some(ctx) = &self.gpu_ctx {
                    let _ = ctx.device().poll(wgpu::PollType::wait_indefinitely());
                }
            }

            #[cfg(feature = "gpu-compute")]
            let gpu_source =
                self.cached_gpu_handle
                    .as_ref()
                    .map(|handle| SimStockRenderSource::GpuGrid {
                        revision: self.cached_stock_rev,
                        handle: Arc::clone(handle),
                        stock_model,
                        color_mode,
                    });
            #[cfg(not(feature = "gpu-compute"))]
            let gpu_source: Option<SimStockRenderSource> = None;

            gpu_source.or_else(|| {
                self.cached_stock_mesh
                    .as_ref()
                    .map(|m| SimStockRenderSource::CpuMesh {
                        revision: self.cached_stock_rev,
                        mesh: Arc::clone(m),
                        stock_model,
                    })
            })
        } else {
            None
        };

        // 2. 3D Revolved Cutter + Shank + Holder Mesh
        let tool_out = self.resolve_cutter_visual(project, &res, snap.as_ref());

        // 3. Optional progressive toolpath trail limit
        let trail_out = if self.trail_up_to_cutter {
            snap.as_ref().map(|s| s.global_seg_idx)
        } else {
            None
        };

        SimViewportVisuals {
            stock: stock_out,
            cutter: tool_out,
            trail_limit: trail_out,
            rotary_deg: sim_rot_deg,
        }
    }

    fn resolve_cutter_visual(
        &mut self,
        project: &ProjectDocument,
        res: &SetupSimulationResult,
        snap: Option<&ScrubSnapshot>,
    ) -> Option<(u64, Arc<chip_sim::SimToolMesh>, DVec3)> {
        if !self.show_cutter_assembly {
            return None;
        }
        let s = snap?;
        let tool = project.tools.iter().find(|t| t.id == s.tool_id)?;
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
    }
}

fn compute_stock_model_wcs(res: &SetupSimulationResult, rotary_deg: f64) -> DMat4 {
    if let Some(cyl) = res.as_cylindrical()
        && rotary_deg.abs() > 1e-6
    {
        let rot = cyl.rotary;
        let angle_rad = (rotary_deg * rot.direction_sign).to_radians();
        DMat4::from_translation(rot.center_wcs)
            * DMat4::from_axis_angle(rot.axis_unit(), angle_rad)
            * DMat4::from_translation(-rot.center_wcs)
    } else {
        DMat4::IDENTITY
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

    use crate::{panels::simulation::SimulationPanelState, viewport::SimStockRenderSource};

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
            let vis = panel.resolve_viewport_visuals_sync(&project, 0);
            let rev = vis
                .stock
                .expect("stock render source should be present")
                .revision();
            let (_, _, tip_wcs) = vis.cutter.expect("cutter visual should be present");

            assert!(
                rev > prev_rev,
                "stock mesh revision must advance immediately at frac={frac} (prev={prev_rev}, got={rev})"
            );
            prev_rev = rev;

            // Working heightfield must already be cut directly beneath the cutter tip,
            // and remain uncut well ahead of the cutter!
            panel.remesh_worker.inspect_working_stock(|stock| {
                let hf = stock.unwrap();
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
            });
        }

        // Subsequent air-only rapid move must NOT trigger unnecessary stock remeshing
        panel.current_time_sec = 0.5 * (air_start + air_end);
        let vis_air1 = panel.resolve_viewport_visuals_sync(&project, 0);
        let rev_after_cut = vis_air1.stock.unwrap().revision();
        panel.current_time_sec = air_start + 0.85 * (air_end - air_start);
        let vis_air2 = panel.resolve_viewport_visuals_sync(&project, 0);
        assert_eq!(
            vis_air2.stock.unwrap().revision(),
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
        let rev_25 = panel
            .resolve_viewport_visuals_sync(&project, 0)
            .stock
            .unwrap()
            .revision();
        let vol_25 = panel
            .remesh_worker
            .inspect_working_stock(|s| s.unwrap().compute_volume_mm3());

        panel.current_time_sec = cut_start + cut_dur * 0.75;
        let rev_75 = panel
            .resolve_viewport_visuals_sync(&project, 0)
            .stock
            .unwrap()
            .revision();
        let vol_75 = panel
            .remesh_worker
            .inspect_working_stock(|s| s.unwrap().compute_volume_mm3());

        assert!(
            rev_75 > rev_25,
            "cylindrical stock mesh must update smoothly mid-segment (rev_25={rev_25}, rev_75={rev_75})"
        );
        assert!(
            vol_75 < vol_25 - 100.0,
            "cylindrical stock volume must decrease as cutter advances mid-segment (vol_25={vol_25:.1}, vol_75={vol_75:.1})"
        );
    }

    #[test]
    fn tridexel_playback_and_error_heatmap_update_smoothly() {
        let mut project = ProjectDocument::new("TriDexel Playback Test");
        let lib = default_starter_tool_library();
        project.tools = lib.tools.clone();
        let flat_6mm = &lib.tools[0];

        let mut setup = Setup::new("s_td", "TriDexel Setup", WorkOffsetSlot::G54);
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

        let mut b = ToolpathBuilder::new("op_td_slot");
        b.rapid_to(DVec3::new(-30.0, 0.0, 10.0));
        b.linear_to(DVec3::new(-30.0, 0.0, -6.0), 300.0, MotionRole::Plunge);
        b.linear_to(DVec3::new(30.0, 0.0, -6.0), 600.0, MotionRole::Cut);
        let tp = b.finish();

        let sim = chip_sim::run_setup_simulation_with_engine(
            &SetupSimulationInput {
                doc: &doc,
                setup: &resolved,
                target_bodies: &[],
                operations: &[SimOperationInput {
                    op_id: "op_td_slot",
                    op_name: "TriDexel Slot",
                    tool: flat_6mm,
                    fallback_spindle_rpm: 12_000.0,
                    intentional_edge_break_mm: 0.0,
                    toolpath: &tp,
                }],
                resolution: SimResolution::Standard,
                kinematics: MachineKinematicsConfig::default(),
                verify_config: SimVerificationConfig::default(),
                cancel: None,
            },
            chip_sim::SimEngineKind::TriDexel,
        )
        .unwrap();
        assert_eq!(sim.engine, chip_sim::SimEngineKind::TriDexel);
        assert!(sim.as_tridexel().is_some());

        let cut_start = sim.timeline.motion_spans[1].t_start_sec;
        let cut_dur = sim.timeline.motion_spans[1].t_end_sec - cut_start;
        let mut panel = SimulationPanelState {
            engine: chip_sim::SimEngineKind::TriDexel,
            active_run_id: 1,
            active_setup_idx: 0,
            active_result: Some(Arc::new(sim)),
            ..Default::default()
        };

        panel.current_time_sec = cut_start + cut_dur * 0.25;
        let SimStockRenderSource::CpuMesh {
            revision: rev_25,
            mesh: mesh_25,
            ..
        } = panel
            .resolve_viewport_visuals_sync(&project, 0)
            .stock
            .unwrap()
        else {
            panic!("expected CpuMesh");
        };
        assert!(!mesh_25.vertices.is_empty());
        let vol_25 = panel
            .remesh_worker
            .inspect_working_stock(|s| s.unwrap().compute_volume_mm3());

        panel.current_time_sec = cut_start + cut_dur * 0.75;
        let rev_75 = panel
            .resolve_viewport_visuals_sync(&project, 0)
            .stock
            .unwrap()
            .revision();
        let vol_75 = panel
            .remesh_worker
            .inspect_working_stock(|s| s.unwrap().compute_volume_mm3());
        assert!(rev_75 > rev_25);
        assert!(
            vol_75 < vol_25 - 100.0,
            "vol_25={vol_25:.1}, vol_75={vol_75:.1}"
        );

        // Toggling Error Heatmap must trigger a clean remesh at the same timestamp
        panel.show_error_heatmap = true;
        assert!(matches!(
            panel.active_stock_color_mode(),
            chip_sim::SimStockColorMode::ErrorHeatmap { .. }
        ));
        let SimStockRenderSource::CpuMesh {
            revision: rev_hm,
            mesh: mesh_hm,
            ..
        } = panel
            .resolve_viewport_visuals_sync(&project, 0)
            .stock
            .unwrap()
        else {
            panic!("expected CpuMesh");
        };
        assert!(rev_hm > rev_75);
        assert!(!mesh_hm.vertices.is_empty());

        // During live playback, Error Heatmap is deferred for smooth FPS and restored on pause.
        panel.is_playing = true;
        assert!(!matches!(
            panel.active_stock_color_mode(),
            chip_sim::SimStockColorMode::ErrorHeatmap { .. }
        ));
        let rev_play = panel
            .resolve_viewport_visuals_sync(&project, 0)
            .stock
            .unwrap()
            .revision();
        assert!(rev_play > rev_hm);

        panel.is_playing = false;
        assert!(matches!(
            panel.active_stock_color_mode(),
            chip_sim::SimStockColorMode::ErrorHeatmap { .. }
        ));
        let rev_pause = panel
            .resolve_viewport_visuals_sync(&project, 0)
            .stock
            .unwrap()
            .revision();
        assert!(rev_pause > rev_play);

        // Unchecking Error Heatmap immediately disables heatmap shading and remeshes cleanly.
        panel.show_error_heatmap = false;
        assert!(!matches!(
            panel.active_stock_color_mode(),
            chip_sim::SimStockColorMode::ErrorHeatmap { .. }
        ));
        let rev_off = panel
            .resolve_viewport_visuals_sync(&project, 0)
            .stock
            .unwrap()
            .revision();
        assert!(rev_off > rev_pause);
    }

    #[test]
    fn rotary_simulation_panel_exposes_active_error_report_and_tolerance_updates() {
        let stl = chip_cad::sample_rotary_shaft_stl_bytes();
        let doc = chip_cad::load_mesh_cad_document(
            "rotary_shaft.stl",
            &stl,
            &chip_cad::MeshImportOptions::default(),
        )
        .unwrap();
        let lib = default_starter_tool_library();
        let flat_6mm = &lib.tools[0];

        let mut setup = Setup::new("s_rot_err", "Rotary A", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
            anchor: BoxAnchor27::VOLUMETRIC_CENTER,
        };
        setup.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(44.0),
            length: ParamLength::from_mm(80.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
        setup.rotary.axis = RotaryAxisKind::XAxis;
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

        let mut b = ToolpathBuilder::new("op_rot_err");
        b.rotary_rapid_to(DVec3::new(-15.0, 0.0, 28.0), 0.0);
        b.rotary_linear_to(
            DVec3::new(-15.0, 0.0, 19.0),
            0.0,
            400.0,
            19.0,
            MotionRole::Plunge,
        );
        b.rotary_linear_to(
            DVec3::new(15.0, 0.0, 19.0),
            360.0,
            600.0,
            19.0,
            MotionRole::Cut,
        );
        let tp = b.finish();

        let sim = chip_sim::run_setup_simulation_with_engine(
            &SetupSimulationInput {
                doc: &doc,
                setup: &resolved,
                target_bodies: &[],
                operations: &[SimOperationInput {
                    op_id: "op_rot_err",
                    op_name: "Rotary Cut",
                    tool: flat_6mm,
                    fallback_spindle_rpm: 12_000.0,
                    intentional_edge_break_mm: 0.0,
                    toolpath: &tp,
                }],
                resolution: SimResolution::Standard,
                kinematics: MachineKinematicsConfig::default(),
                verify_config: SimVerificationConfig::default(),
                cancel: None,
            },
            chip_sim::SimEngineKind::TriDexel,
        )
        .unwrap();

        let mut panel = SimulationPanelState {
            engine: chip_sim::SimEngineKind::TriDexel,
            active_run_id: 1,
            active_setup_idx: 0,
            active_result: Some(Arc::new(sim)),
            ..Default::default()
        };

        let rep_default = panel
            .active_error_report()
            .expect("4th-axis rotary simulation must expose Error Metrics in SimulationPanelState")
            .clone();
        assert!(rep_default.sample_count > 1_000);
        assert!(rep_default.target_volume_mm3 > 10_000.0);

        panel.heatmap_tolerance_mm = 2.0;
        let rep_loose = panel
            .active_error_report()
            .expect("recomputed tolerance error report")
            .clone();
        assert!((rep_loose.tolerance_mm - 2.0).abs() < 1e-6);
        assert!(rep_loose.in_tolerance_surface_pct >= rep_default.in_tolerance_surface_pct);
    }

    #[test]
    fn rotary_simulation_panel_renders_error_heatmap_even_when_color_by_op_disabled() {
        let mut project = ProjectDocument::new("4th Axis Heatmap Test");
        let lib = default_starter_tool_library();
        project.tools = lib.tools.clone();
        let ball_6mm = &lib.tools[3];

        let stl = chip_cad::sample_helical_flute_shaft_stl_bytes();
        let doc = chip_cad::load_mesh_cad_document(
            "helical_flute_shaft.stl",
            &stl,
            &chip_cad::MeshImportOptions::default(),
        )
        .unwrap();

        let mut setup = Setup::new("s_helical_hm", "Helical A", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
            anchor: BoxAnchor27::VOLUMETRIC_CENTER,
        };
        setup.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(42.0),
            length: ParamLength::from_mm(80.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
        setup.rotary.axis = RotaryAxisKind::XAxis;
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

        let mut b = ToolpathBuilder::new("op_helical");
        b.rotary_rapid_to(DVec3::new(-20.0, 0.0, 28.0), 0.0);
        b.rotary_linear_to(
            DVec3::new(-20.0, 0.0, 18.0),
            0.0,
            400.0,
            18.0,
            MotionRole::Plunge,
        );
        b.rotary_linear_to(
            DVec3::new(20.0, 0.0, 18.0),
            360.0,
            800.0,
            18.0,
            MotionRole::Cut,
        );
        let tp = b.finish();

        let sim = chip_sim::run_setup_simulation_with_engine(
            &SetupSimulationInput {
                doc: &doc,
                setup: &resolved,
                target_bodies: &[],
                operations: &[SimOperationInput {
                    op_id: "op_helical",
                    op_name: "4-Axis Helical",
                    tool: ball_6mm,
                    fallback_spindle_rpm: 12_000.0,
                    intentional_edge_break_mm: 0.0,
                    toolpath: &tp,
                }],
                resolution: SimResolution::Standard,
                kinematics: MachineKinematicsConfig::default(),
                verify_config: SimVerificationConfig::default(),
                cancel: None,
            },
            chip_sim::SimEngineKind::TriDexel,
        )
        .unwrap();

        let mut panel = SimulationPanelState {
            engine: chip_sim::SimEngineKind::TriDexel,
            color_by_operation: false,
            show_error_heatmap: false,
            active_run_id: 1,
            active_setup_idx: 0,
            active_result: Some(Arc::new(sim)),
            ..Default::default()
        };

        // Baseline metallic mesh with Color by Op disabled & Error Heatmap disabled
        let SimStockRenderSource::CpuMesh {
            revision: rev_metal,
            mesh: mesh_metal,
            ..
        } = panel
            .resolve_viewport_visuals_sync(&project, 0)
            .stock
            .unwrap()
        else {
            panic!("expected CpuMesh");
        };
        assert!(
            mesh_metal.vertices.iter().all(|v| {
                let max_c = v.color_rgba[0].max(v.color_rgba[1]).max(v.color_rgba[2]);
                let min_c = v.color_rgba[0].min(v.color_rgba[1]).min(v.color_rgba[2]);
                (max_c - min_c) < 0.12
            }),
            "baseline metallic cylindrical stock should have neutral metallic gray vertex colors"
        );

        // Enable Error Heatmap (keeping Color by Op = false, matching user repro in Issue #13)
        panel.show_error_heatmap = true;
        let SimStockRenderSource::CpuMesh {
            revision: rev_hm,
            mesh: mesh_hm,
            ..
        } = panel
            .resolve_viewport_visuals_sync(&project, 0)
            .stock
            .unwrap()
        else {
            panic!("expected CpuMesh");
        };
        assert!(rev_hm > rev_metal);
        let has_vivid_heatmap_color = mesh_hm.vertices.iter().any(|v| {
            let max_c = v.color_rgba[0].max(v.color_rgba[1]).max(v.color_rgba[2]);
            let min_c = v.color_rgba[0].min(v.color_rgba[1]).min(v.color_rgba[2]);
            (max_c - min_c) > 0.25
        });
        assert!(
            has_vivid_heatmap_color,
            "4th-axis cylindrical stock must display vivid heatmap colors when Error Heatmap is enabled"
        );
    }

    #[test]
    fn rotary_angle_only_scrub_updates_stock_model_without_remeshing_cylindrical_stock() {
        let mut project = ProjectDocument::new("Rotary GPU Matrix Test");
        let lib = default_starter_tool_library();
        project.tools = lib.tools.clone();
        let flat_6mm = &lib.tools[0];

        let mut setup = Setup::new("s_rot_gpu", "Rotary A", WorkOffsetSlot::G54);
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

        // Seg 0: rapid to plunge; Seg 1: plunge; Seg 2: axial flute cut at A=0°;
        // Seg 3: retract to safe clearance radius Z=28mm; Seg 4: rapid rotary indexing slew A=0° -> A=180° in air!
        let mut b = ToolpathBuilder::new("op_rot_index");
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
        b.rotary_rapid_to(DVec3::new(25.0, 0.0, 28.0), 0.0);
        b.rotary_rapid_to(DVec3::new(25.0, 0.0, 28.0), 180.0);
        let tp = b.finish();

        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            operations: &[SimOperationInput {
                op_id: "op_rot_index",
                op_name: "Rotary Flute + Index",
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

        let index_span = sim
            .timeline
            .motion_spans
            .iter()
            .find(|sp| (sp.segment.end_rotary_deg - sp.segment.start_rotary_deg).abs() > 100.0)
            .expect("expected rotary indexing span A=0°->180°");
        let idx_start = index_span.t_start_sec;
        let idx_dur = index_span.t_end_sec - idx_start;
        assert!(idx_dur > 0.01);

        let mut panel = SimulationPanelState {
            active_run_id: 1,
            active_setup_idx: 0,
            active_result: Some(Arc::new(sim)),
            is_playing: true,
            ..Default::default()
        };

        // Prime working cylindrical stock at the start of the air indexing slew
        panel.current_time_sec = idx_start + idx_dur * 0.10;
        let vis_10 = panel.resolve_viewport_visuals_sync(&project, 0);
        let src_10 = vis_10.stock.expect("stock render source present");
        let rev_10 = src_10.revision();
        let mat_10 = src_10.stock_model();
        let rot_10 = vis_10.rotary_deg;

        let hl_10 = crate::viewport::HighlightState {
            wcs_frame: resolved.frame,
            rotary: resolved.rotary,
            sim_rotary_deg: rot_10,
            ..Default::default()
        };
        assert!(
            mat_10.abs_diff_eq(hl_10.stock_model_wcs_transform(), 1e-9),
            "SimStockRenderSource::stock_model must match HighlightState::stock_model_wcs_transform"
        );

        // Advance through multiple rotary-only ticks where `r_stock` does NOT mutate:
        // `vis.rotary_deg` and GPU `stock_model` MUST advance while CPU stock revision stays constant!
        let mut prev_rot = rot_10;
        for frac in [0.35_f64, 0.65, 0.90] {
            panel.current_time_sec = idx_start + idx_dur * frac;
            let vis = panel.resolve_viewport_visuals_sync(&project, 0);
            let src = vis.stock.expect("stock render source present");
            assert_eq!(
                src.revision(),
                rev_10,
                "rotary angle-only tick at frac={frac} must NOT re-tessellate CylindricalStock on CPU"
            );
            assert!(
                vis.rotary_deg > prev_rot + 10.0,
                "expected rotary angle to advance smoothly (prev={prev_rot:.1}, got={:.1})",
                vis.rotary_deg
            );
            assert!(
                !src.stock_model().abs_diff_eq(mat_10, 1e-4),
                "GPU stock_model transform must update as rotary angle advances"
            );
            prev_rot = vis.rotary_deg;
        }
    }

    #[cfg(feature = "gpu-compute")]
    #[test]
    fn gpu_playback_toggling_heatmap_and_tolerance_updates_without_resweep_or_cpu_mesh() {
        let presentation_ctx = chip_sim::GpuSimContext::try_new_headless()
            .expect("expected Vulkan/wgpu compute adapter");
        let mut project = ProjectDocument::new("GPU Instant Heatmap Test");
        let lib = default_starter_tool_library();
        project.tools = lib.tools.clone();
        let flat_6mm = &lib.tools[0];

        let mut setup = Setup::new("s_gpu_hm", "GPU HF", WorkOffsetSlot::G54);
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

        let mut b = ToolpathBuilder::new("op_gpu_slot");
        b.rapid_to(DVec3::new(-30.0, 0.0, 10.0));
        b.linear_to(DVec3::new(-30.0, 0.0, -5.0), 300.0, MotionRole::Plunge);
        b.linear_to(DVec3::new(30.0, 0.0, -5.0), 600.0, MotionRole::Cut);
        let tp = b.finish();

        let sim = chip_sim::run_setup_simulation_with_choice(
            &SetupSimulationInput {
                doc: &doc,
                setup: &resolved,
                target_bodies: &[],
                operations: &[SimOperationInput {
                    op_id: "op_gpu_slot",
                    op_name: "GPU Slot",
                    tool: flat_6mm,
                    fallback_spindle_rpm: 12_000.0,
                    intentional_edge_break_mm: 0.0,
                    toolpath: &tp,
                }],
                resolution: SimResolution::Standard,
                kinematics: MachineKinematicsConfig::default(),
                verify_config: SimVerificationConfig::default(),
                cancel: None,
            },
            chip_sim::SimEngineKind::Heightmap,
            chip_sim::SimComputeBackendChoice::Gpu,
        )
        .unwrap();
        assert!(sim.active_backend.is_gpu);

        let cut_start = sim.timeline.motion_spans[1].t_start_sec;
        let cut_dur = sim.timeline.motion_spans[1].t_end_sec - cut_start;

        let mut panel = SimulationPanelState::with_engine_and_gpu(
            chip_sim::SimEngineKind::Heightmap,
            Some(Arc::clone(&presentation_ctx)),
        );
        panel.active_run_id = 1;
        panel.active_setup_idx = 0;
        panel.active_result = Some(Arc::new(sim));
        panel.current_time_sec = cut_start + cut_dur * 0.35;

        let vis0 = panel.resolve_viewport_visuals_sync(&project, 0);
        let src0 = vis0.stock.expect("GPU stock render source present");
        assert!(
            src0.cpu_mesh().is_none(),
            "GPU direct path must not allocate a CPU SimStockMesh"
        );
        let SimStockRenderSource::GpuGrid {
            revision: rev0,
            color_mode: mode0,
            ..
        } = src0
        else {
            panic!("expected SimStockRenderSource::GpuGrid");
        };
        assert_eq!(mode0, chip_sim::SimStockColorMode::ByOperation);
        let reqs_after_init = panel.remesh_worker.processed_request_count();
        assert_eq!(reqs_after_init, 1);

        // Toggle Error Heatmap & drag ±tol slider multiple times: ZERO re-sweeps, ZERO CPU meshes!
        panel.show_error_heatmap = true;
        panel.heatmap_tolerance_mm = 0.05;
        let vis_hm1 = panel.resolve_viewport_visuals_sync(&project, 0);
        let SimStockRenderSource::GpuGrid {
            revision: rev_hm1,
            color_mode: mode_hm1,
            ..
        } = vis_hm1.stock.expect("GpuGrid present")
        else {
            panic!("expected GpuGrid");
        };
        assert_eq!(rev_hm1, rev0);
        assert!(matches!(
            mode_hm1,
            chip_sim::SimStockColorMode::ErrorHeatmap { tolerance_mm, .. }
            if (tolerance_mm - 0.05).abs() < 1e-6
        ));
        assert_eq!(
            panel.remesh_worker.processed_request_count(),
            reqs_after_init
        );

        panel.heatmap_tolerance_mm = 0.25;
        let vis_hm2 = panel.resolve_viewport_visuals_sync(&project, 0);
        let SimStockRenderSource::GpuGrid {
            revision: rev_hm2,
            color_mode: mode_hm2,
            ..
        } = vis_hm2.stock.expect("GpuGrid present")
        else {
            panic!("expected GpuGrid");
        };
        assert_eq!(rev_hm2, rev0);
        assert!(matches!(
            mode_hm2,
            chip_sim::SimStockColorMode::ErrorHeatmap { tolerance_mm, .. }
            if (tolerance_mm - 0.25).abs() < 1e-6
        ));
        assert_eq!(
            panel.remesh_worker.processed_request_count(),
            reqs_after_init
        );

        panel.show_error_heatmap = false;
        panel.color_by_operation = false;
        let vis_metal = panel.resolve_viewport_visuals_sync(&project, 0);
        let SimStockRenderSource::GpuGrid {
            revision: rev_metal,
            color_mode: mode_metal,
            ..
        } = vis_metal.stock.expect("GpuGrid present")
        else {
            panic!("expected GpuGrid");
        };
        assert_eq!(rev_metal, rev0);
        assert_eq!(mode_metal, chip_sim::SimStockColorMode::Metallic);
        assert_eq!(
            panel.remesh_worker.processed_request_count(),
            reqs_after_init
        );
        assert!(panel.cached_stock_mesh.is_none());

        // Scrubbing forward along the cutting segment dispatches GPU delta sweep & advances revision!
        let vol_before = panel
            .remesh_worker
            .inspect_working_stock(|s| s.unwrap().compute_volume_mm3());
        panel.current_time_sec = cut_start + cut_dur * 0.85;
        let vis_later = panel.resolve_viewport_visuals_sync(&project, 0);
        let rev_later = vis_later.stock.expect("GpuGrid present").revision();
        assert!(rev_later > rev0);
        assert_eq!(
            panel.remesh_worker.processed_request_count(),
            reqs_after_init + 1
        );
        let vol_after = panel
            .remesh_worker
            .inspect_working_stock(|s| s.unwrap().compute_volume_mm3());
        assert!(
            vol_after < vol_before - 100.0,
            "GPU scrub must cut additional stock (before={vol_before:.1}, after={vol_after:.1})"
        );

        // Regression test: background remesh worker MUST use an isolated compute `wgpu::Device`
        // distinct from `eframe`'s presentation device so `Surface::configure` on window resize
        // can never panic with `ConfigureSurfaceError::GpuWaitTimeout`.
        let presentation_ctx_ptr = Arc::as_ptr(&presentation_ctx);
        let worker_ctx_ptr = panel
            .remesh_worker
            .compute_gpu_ctx_ptr()
            .expect("background remesh worker initialized isolated compute GPU context");
        assert_ne!(
            worker_ctx_ptr, presentation_ctx_ptr,
            "background simulation/remesh worker must never submit to eframe's presentation wgpu::Device"
        );
    }
}
