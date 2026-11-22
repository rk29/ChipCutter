//! Unified setup simulation orchestrator, [`SetupSimulationResult`], single polymorphic
//! [`run_setup_simulation_with_backend`] driver, and single [`SimulatedStock`] playback cursor
//! state machine covering `Heightfield`, `TriDexel`, and 4th-axis `Cylindrical` engines.

#[cfg(feature = "gpu-compute")]
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use chip_cad::{BodyId, CadDocument};
use chip_cam::ResolvedSetup;
use chip_stock::{
    ActiveSimBackendInfo, CpuExactBackend, CylindricalStock, HeightfieldStock,
    OperationErrorSnapshot, RadialCutterProfile, SimComputeBackendChoice, SimEngineKind, SimError,
    SimResolution, SimStockBackend, SimStockColorMode, SimStockMesh, SimulatedStock,
    StockCheckpointSnapshot, StockErrorReport, StockModelKind, StockPlaybackCursor,
    StockVerifyTolerances, SweepChord, SweepToolSpec, TriDexelStock,
};
#[cfg(feature = "gpu-compute")]
use chip_stock::{GpuSimContext, WgpuComputeBackend};
use chip_tools::CutterGeometry;

use crate::{
    cad_adapter::{
        extract_cylindrical_target_mesh_from_cad, extract_heightfield_target_from_cad,
        extract_tridexel_target_from_cad, rotary_stock_spec_from_cam, stock_shape_spec_from_cam,
    },
    timeline::{MachineKinematicsConfig, ScrubSnapshot, SimOperationInput, SimulationTimeline},
    verifier::{SimDiagnostic, SimSeverity, SimVerificationConfig},
    verify_common::{
        ViolationBatchCoalescer, evaluate_vertical_plunge_violation, push_segment_sweep_chords,
        slice_segment_range,
    },
};

/// Borrowed input bundle passed to [`run_setup_simulation`] and [`run_setup_simulation_with_engine`].
#[derive(Clone, Copy)]
pub struct SetupSimulationInput<'a> {
    pub doc: &'a CadDocument,
    pub setup: &'a ResolvedSetup,
    pub target_bodies: &'a [BodyId],
    pub operations: &'a [SimOperationInput<'a>],
    pub resolution: SimResolution,
    pub kinematics: MachineKinematicsConfig,
    pub verify_config: SimVerificationConfig,
    pub cancel: Option<&'a AtomicBool>,
}

/// Complete output of simulating a Setup (`timeline`, unified `final_stock`, unified `checkpoints`,
/// `diagnostics`, and quantitative `error_report`).
#[derive(Debug, Clone)]
pub struct SetupSimulationResult {
    pub engine: SimEngineKind,
    pub resolution: SimResolution,
    pub active_backend: ActiveSimBackendInfo,
    pub timeline: SimulationTimeline,
    pub initial_stock_volume_mm3: f64,
    pub remaining_stock_volume_mm3: f64,
    pub removed_volume_mm3: f64,
    /// Unified polymorphic final simulated stock (`Heightfield`, `TriDexel`, or `Cylindrical`).
    pub final_stock: SimulatedStock,
    /// Optional aligned 3D target solid when running the volumetric `TriDexel` engine.
    pub target_tridexel_stock: Option<TriDexelStock>,
    /// Unified periodic stock checkpoints for `< 4 ms` interactive timeline scrubbing.
    pub checkpoints: Vec<StockCheckpointSnapshot>,
    /// Per-operation cutter specs captured during simulation for exact interactive timeline
    /// reconstruction even if project operations are later edited.
    pub operation_profiles: Vec<SweepToolSpec>,
    pub diagnostics: Vec<SimDiagnostic>,
    /// Quantitative stock-vs-target error volume & surface deviation metrics (when 3D target model exists).
    pub error_report: Option<StockErrorReport>,
}

impl SetupSimulationResult {
    #[inline]
    pub fn stock_kind(&self) -> StockModelKind {
        self.final_stock.kind()
    }

    #[inline]
    pub const fn is_cylindrical(&self) -> bool {
        self.final_stock.is_cylindrical()
    }

    #[inline]
    pub fn as_heightfield(&self) -> Option<&HeightfieldStock> {
        self.final_stock.as_heightfield()
    }

    #[inline]
    pub fn as_tridexel(&self) -> Option<&TriDexelStock> {
        self.final_stock.as_tridexel()
    }

    #[inline]
    pub fn as_cylindrical(&self) -> Option<&CylindricalStock> {
        self.final_stock.as_cylindrical()
    }

    pub fn critical_error_count(&self) -> usize {
        self.diagnostics
            .iter()
            .filter(|d| d.severity == SimSeverity::CriticalError)
            .count()
    }

    pub fn warning_count(&self) -> usize {
        self.diagnostics
            .iter()
            .filter(|d| d.severity == SimSeverity::Warning)
            .count()
    }

    pub fn has_critical_errors(&self) -> bool {
        self.critical_error_count() > 0
    }

    /// Recomputes [`StockErrorReport`] for an arbitrary `tolerance_mm` band (`in_tolerance_surface_pct`)
    /// without re-running the toolpath sweep.
    pub fn compute_error_report_at_tolerance(&self, tolerance_mm: f64) -> Option<StockErrorReport> {
        let per_op = self
            .error_report
            .as_ref()
            .map_or_else(Vec::new, |r| r.per_operation.clone());
        self.final_stock.compute_error_report(
            self.target_tridexel_stock.as_ref(),
            self.initial_stock_volume_mm3,
            tolerance_mm,
            per_op,
        )
    }

    /// Tessellates `stock` into a GPU-ready [`SimStockMesh`] using this result's target solid (when needed
    /// for `SimStockColorMode::ErrorHeatmap`).
    pub fn extract_stock_mesh(
        &self,
        stock: &SimulatedStock,
        color_mode: SimStockColorMode,
    ) -> SimStockMesh {
        stock.extract_stock_mesh(self.target_tridexel_stock.as_ref(), color_mode)
    }

    /// Reconstructs the exact [`SimulatedStock`] state at `time_sec` by restoring the nearest
    /// preceding [`StockCheckpointSnapshot`] and sweeping only the short tail delta up to `time_sec`.
    pub fn reconstruct_stock_at_time(&self, time_sec: f64) -> SimulatedStock {
        let mut stock = self.final_stock.clone();
        let mut cursor = None;
        self.sync_stock_to_time(&mut stock, &mut cursor, time_sec);
        stock
    }

    /// Synchronizes a reusable working [`SimulatedStock`] to the exact timeline state at `time_sec`.
    /// Returns `true` if `stock` mutated (indicating any cached display mesh should be re-tessellated).
    pub fn sync_stock_to_time(
        &self,
        stock: &mut SimulatedStock,
        cursor: &mut Option<StockPlaybackCursor>,
        time_sec: f64,
    ) -> bool {
        let total_spans = self.timeline.motion_spans.len();
        let target = self.timeline.sample_at_time(time_sec).map_or(
            StockPlaybackCursor {
                seg_idx: total_spans,
                seg_u: 0.0,
            },
            |s| target_cursor_from_snap(&s, total_spans),
        );
        self.sync_stock_to_target_cursor(stock, cursor, target)
    }

    /// Synchronizes a reusable working [`SimulatedStock`] to an already-sampled [`ScrubSnapshot`].
    /// Returns `true` if `stock` mutated (indicating any cached display mesh should be re-tessellated).
    pub fn sync_stock_to_scrub_snapshot(
        &self,
        stock: &mut SimulatedStock,
        cursor: &mut Option<StockPlaybackCursor>,
        snap: &ScrubSnapshot,
    ) -> bool {
        let target = target_cursor_from_snap(snap, self.timeline.motion_spans.len());
        self.sync_stock_to_target_cursor(stock, cursor, target)
    }

    fn sync_stock_to_target_cursor(
        &self,
        stock: &mut SimulatedStock,
        cursor: &mut Option<StockPlaybackCursor>,
        target: StockPlaybackCursor,
    ) -> bool {
        let total_spans = self.timeline.motion_spans.len();
        let spans = &self.timeline.motion_spans;
        let default_tol = StockVerifyTolerances::default();
        stock.sync_to_cursor(
            cursor,
            target,
            total_spans,
            &self.final_stock,
            &self.checkpoints,
            &self.operation_profiles,
            |seg_idx, u_start, u_end, out| {
                let Some(span) = spans.get(seg_idx) else {
                    return;
                };
                if u_start <= 1e-6 && u_end >= 1.0 - 1e-6 {
                    push_segment_sweep_chords(
                        &span.segment,
                        span.op_idx as u32,
                        span.op_idx,
                        default_tol,
                        out,
                    );
                } else if let Some(sub_seg) = slice_segment_range(&span.segment, u_start, u_end) {
                    push_segment_sweep_chords(
                        &sub_seg,
                        span.op_idx as u32,
                        span.op_idx,
                        default_tol,
                        out,
                    );
                }
            },
        )
    }
}

/// Extension trait adding ergonomic playback cursor sync directly onto [`SimulatedStock`] for
/// [`SetupSimulationResult`] timelines across all 3 volumetric stock engines.
pub trait SimulatedStockPlaybackExt {
    fn sync_to_scrub_snapshot(
        &mut self,
        cursor: &mut Option<StockPlaybackCursor>,
        result: &SetupSimulationResult,
        snap: &ScrubSnapshot,
    ) -> bool;

    fn sync_to_time(
        &mut self,
        cursor: &mut Option<StockPlaybackCursor>,
        result: &SetupSimulationResult,
        time_sec: f64,
    ) -> bool;
}

impl SimulatedStockPlaybackExt for SimulatedStock {
    fn sync_to_scrub_snapshot(
        &mut self,
        cursor: &mut Option<StockPlaybackCursor>,
        result: &SetupSimulationResult,
        snap: &ScrubSnapshot,
    ) -> bool {
        result.sync_stock_to_scrub_snapshot(self, cursor, snap)
    }

    fn sync_to_time(
        &mut self,
        cursor: &mut Option<StockPlaybackCursor>,
        result: &SetupSimulationResult,
        time_sec: f64,
    ) -> bool {
        result.sync_stock_to_time(self, cursor, time_sec)
    }
}

pub(crate) fn target_cursor_from_snap(
    snap: &ScrubSnapshot,
    total_spans: usize,
) -> StockPlaybackCursor {
    if snap.completed_segments >= total_spans {
        return StockPlaybackCursor {
            seg_idx: total_spans,
            seg_u: 0.0,
        };
    }
    let u = snap.seg_progress_t.clamp(0.0, 1.0);
    if u >= 1.0 - 1e-9 {
        StockPlaybackCursor {
            seg_idx: (snap.global_seg_idx + 1).min(total_spans),
            seg_u: 0.0,
        }
    } else if u <= 1e-6 {
        StockPlaybackCursor {
            seg_idx: snap.global_seg_idx.min(total_spans),
            seg_u: 0.0,
        }
    } else {
        StockPlaybackCursor {
            seg_idx: snap.global_seg_idx,
            seg_u: u,
        }
    }
}

/// Executes full heightmap (or 4th-axis cylindrical) stock removal, periodic checkpoint capture,
/// and 5-rule crash/gouge verification across all active Setup operations (`06-simulation.md`).
pub fn run_setup_simulation(
    input: &SetupSimulationInput<'_>,
) -> Result<SetupSimulationResult, SimError> {
    run_setup_simulation_with_engine(input, SimEngineKind::Heightmap)
}

/// Executes stock simulation & 5-rule crash/gouge verification using the selected [`SimEngineKind`]
/// (`Heightmap` or volumetric `TriDexel`, automatically selecting 4th-axis `Cylindrical` on rotary setups).
pub fn run_setup_simulation_with_engine(
    input: &SetupSimulationInput<'_>,
    engine: SimEngineKind,
) -> Result<SetupSimulationResult, SimError> {
    run_setup_simulation_with_choice(input, engine, SimComputeBackendChoice::Cpu)
}

/// Executes stock simulation & 5-rule crash/gouge verification using the selected [`SimEngineKind`]
/// and [`SimComputeBackendChoice`] (`Auto` / `Gpu` / `Cpu`), falling back cleanly to
/// [`CpuExactBackend`] whenever `gpu-compute` is disabled, no GPU adapter is available, or the
/// stock model is `TriDexel` (until Milestone 53).
pub fn run_setup_simulation_with_choice(
    input: &SetupSimulationInput<'_>,
    engine: SimEngineKind,
    choice: SimComputeBackendChoice,
) -> Result<SetupSimulationResult, SimError> {
    #[cfg(feature = "gpu-compute")]
    {
        let gpu_ctx = match choice {
            SimComputeBackendChoice::Cpu => None,
            SimComputeBackendChoice::Auto | SimComputeBackendChoice::Gpu => {
                GpuSimContext::shared_headless()
            }
        };
        run_setup_simulation_with_gpu_context(input, engine, choice, gpu_ctx)
    }
    #[cfg(not(feature = "gpu-compute"))]
    {
        let _ = choice;
        let (effective_engine, stock, target_td) = prepare_initial_stock(input, engine)?;
        let backend = CpuExactBackend::new(stock, target_td);
        run_setup_simulation_with_backend(input, effective_engine, backend)
    }
}

/// Executes stock simulation with an explicitly provided shared [`GpuSimContext`] (e.g., from
/// `eframe`/`ViewportRenderer`), transparently falling back to [`CpuExactBackend`] when
/// `choice == SimComputeBackendChoice::Cpu`, `gpu_ctx` is `None`, or `TriDexel` is selected.
#[cfg(feature = "gpu-compute")]
pub fn run_setup_simulation_with_gpu_context(
    input: &SetupSimulationInput<'_>,
    engine: SimEngineKind,
    choice: SimComputeBackendChoice,
    gpu_ctx: Option<Arc<GpuSimContext>>,
) -> Result<SetupSimulationResult, SimError> {
    let (effective_engine, stock, target_td) = prepare_initial_stock(input, engine)?;
    let use_gpu = choice != SimComputeBackendChoice::Cpu
        && matches!(
            stock,
            SimulatedStock::Heightfield(_) | SimulatedStock::Cylindrical(_)
        );

    if use_gpu && let Some(ctx) = gpu_ctx {
        let backend = WgpuComputeBackend::new(ctx, stock, target_td);
        run_setup_simulation_with_backend(input, effective_engine, backend)
    } else {
        let backend = CpuExactBackend::new(stock, target_td);
        run_setup_simulation_with_backend(input, effective_engine, backend)
    }
}

fn prepare_initial_stock(
    input: &SetupSimulationInput<'_>,
    engine: SimEngineKind,
) -> Result<(SimEngineKind, SimulatedStock, Option<TriDexelStock>), SimError> {
    check_cancelled(input.cancel)?;
    let stock_spec = stock_shape_spec_from_cam(&input.setup.stock);

    if let Some(rotary) = input.setup.rotary {
        let rotary_spec = rotary_stock_spec_from_cam(rotary);
        let mut cyl =
            CylindricalStock::from_stock_spec(&stock_spec, rotary_spec, input.resolution)?;
        check_cancelled(input.cancel)?;
        if let Some(target_mesh) =
            extract_cylindrical_target_mesh_from_cad(input.doc, input.setup, input.target_bodies)
        {
            cyl.rasterize_target_mesh(&target_mesh);
        }
        Ok((
            SimEngineKind::Heightmap,
            SimulatedStock::Cylindrical(cyl),
            None,
        ))
    } else if engine == SimEngineKind::TriDexel {
        let td = TriDexelStock::from_stock_spec(&stock_spec, input.resolution)?;
        check_cancelled(input.cancel)?;
        let target_td =
            extract_tridexel_target_from_cad(input.doc, input.setup, input.target_bodies, &td);
        Ok((
            SimEngineKind::TriDexel,
            SimulatedStock::TriDexel(Box::new(td)),
            target_td,
        ))
    } else {
        let mut hf = HeightfieldStock::from_stock_spec(&stock_spec, input.resolution)?;
        check_cancelled(input.cancel)?;
        if input.doc.has_3d_geometry() {
            let (horiz_up, tris) =
                extract_heightfield_target_from_cad(input.doc, input.setup, input.target_bodies);
            hf.rasterize_target(&horiz_up, &tris);
        }
        Ok((
            SimEngineKind::Heightmap,
            SimulatedStock::Heightfield(hf),
            None,
        ))
    }
}

/// Single unified polymorphic setup simulation loop driving any [`SimStockBackend`]
/// (`CpuExactBackend` today; future `WgpuComputeBackend` plugs in here directly).
pub fn run_setup_simulation_with_backend<B: SimStockBackend>(
    input: &SetupSimulationInput<'_>,
    engine: SimEngineKind,
    mut backend: B,
) -> Result<SetupSimulationResult, SimError> {
    check_cancelled(input.cancel)?;
    let timeline = SimulationTimeline::from_operations(input.operations, input.kinematics);
    let initial_stock_volume_mm3 = backend.compute_volume_mm3();

    let total_spans = timeline.motion_spans.len();
    let checkpoint_stride = backend.kind().checkpoint_stride(total_spans);
    let mut checkpoints = vec![backend.capture_checkpoint(0)];

    let operation_profiles: Vec<SweepToolSpec> = input
        .operations
        .iter()
        .map(|op| SweepToolSpec {
            profile: RadialCutterProfile::from_tool(op.tool),
            is_flat_endmill: matches!(op.tool.geometry, CutterGeometry::FlatEndmill { .. }),
        })
        .collect();

    let mut coalescer = ViolationBatchCoalescer::default();
    let mut per_op_errors: Vec<OperationErrorSnapshot> = Vec::new();
    let mut prev_vol = initial_stock_volume_mm3;
    let mut span_chords: Vec<SweepChord> = Vec::with_capacity(16);

    for (span_idx, span) in timeline.motion_spans.iter().enumerate() {
        check_cancelled(input.cancel)?;

        let op_in = &input.operations[span.op_idx];
        let tool_spec = &operation_profiles[span.op_idx];
        let mut op_verify = input.verify_config;
        op_verify.gouge_tol_mm += op_in.intentional_edge_break_mm.max(0.0);

        // Rule 5: Vertical Plunge check (shared across 3-axis Heightfield & Tri-Dexel; 0.0 on Cylindrical)
        if let Some(hit) = evaluate_vertical_plunge_violation(
            span,
            op_in.op_id,
            op_in.op_name,
            op_in.tool.tool_number,
            &tool_spec.profile,
            &op_verify,
            |x, y, z_start, z_end, thread_check_r| {
                backend.query_vertical_plunge_depth(x, y, z_start, z_end, thread_check_r)
            },
        ) {
            coalescer.push(hit);
        }

        span_chords.clear();
        push_segment_sweep_chords(
            &span.segment,
            span.op_idx as u32,
            span.op_idx,
            StockVerifyTolerances {
                gouge_tol_mm: op_verify.gouge_tol_mm,
                rapid_crash_tol_mm: op_verify.rapid_crash_tol_mm,
                radial_wall_skin_mm: op_verify.radial_wall_skin_mm,
            },
            &mut span_chords,
        );

        let sweep_out = backend.sweep_batch(&span_chords, &operation_profiles, true);
        coalescer.push_span_violations(
            &sweep_out.violations,
            span.global_seg_idx,
            span.op_idx,
            op_in.op_id,
            op_in.op_name,
            op_in.tool.tool_number,
            span.t_start_sec,
            &tool_spec.profile,
        );

        let completed = span_idx + 1;
        let is_op_end = timeline.operations[span.op_idx].end_global_seg == completed;
        if is_op_end {
            let cur_vol = backend.compute_volume_mm3();
            let op_removed = (prev_vol - cur_vol).max(0.0);
            prev_vol = cur_vol;
            if let Some((ex_v, g_v, rms)) = backend.compute_quick_error_summary() {
                per_op_errors.push(OperationErrorSnapshot {
                    op_idx: span.op_idx,
                    op_id: op_in.op_id.to_string(),
                    op_name: op_in.op_name.to_string(),
                    removed_volume_mm3: op_removed,
                    excess_volume_mm3: ex_v,
                    gouge_volume_mm3: g_v,
                    total_error_volume_mm3: ex_v + g_v,
                    rms_error_mm: rms,
                });
            }
        }
        if (completed.is_multiple_of(checkpoint_stride) || is_op_end) && completed < total_spans {
            checkpoints.push(backend.capture_checkpoint(completed));
        }
    }

    let remaining_stock_volume_mm3 = backend.compute_volume_mm3();
    let removed_volume_mm3 = (initial_stock_volume_mm3 - remaining_stock_volume_mm3).max(0.0);
    let error_report = backend.compute_error_report(initial_stock_volume_mm3, 0.05, per_op_errors);
    let active_backend = backend.backend_info();
    let (final_stock, target_tridexel_stock) = backend.into_parts();
    let diagnostics = coalescer.into_diagnostics();

    Ok(SetupSimulationResult {
        engine,
        resolution: input.resolution,
        active_backend,
        timeline,
        initial_stock_volume_mm3,
        remaining_stock_volume_mm3,
        removed_volume_mm3,
        final_stock,
        target_tridexel_stock,
        checkpoints,
        operation_profiles,
        diagnostics,
        error_report,
    })
}

#[inline]
fn check_cancelled(cancel: Option<&AtomicBool>) -> Result<(), SimError> {
    if cancel.is_some_and(|flag| flag.load(Ordering::Relaxed)) {
        return Err(SimError::Cancelled);
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::f64::consts::PI;

    use chip_cad::{
        CadDocument, CadKernel, MeshImportOptions, MonstertruckKernel, load_mesh_cad_document,
        sample_machined_plate_step, sample_rotary_shaft_stl_bytes,
    };
    use chip_cam::{
        AxisAnchor, BoxAnchor27, ClEvent, MotionRole, RotaryAxisKind, RotaryDriveMode, Setup,
        StockAxis, StockConfig, ToolpathBuilder, WcsOriginMode, WorkOffsetSlot, XyArcMove,
    };
    use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DVec2, DVec3, DisplayUnit, ParamLength, approx_eq};
    use chip_stock::{
        DexelSpan, RAW_STOCK_SIDE_COLOR, RAW_STOCK_TOP_COLOR, UNCUT_OP_IDX,
        deviation_to_heatmap_rgba, subtract_dexel_interval, tessellate_cylindrical_stock_mesh,
        tessellate_cylindrical_stock_mesh_colored, tessellate_tridexel_stock_mesh,
    };
    use chip_tools::default_starter_tool_library;

    use crate::{SimIssueKind, SimVerificationConfig};

    use super::*;

    #[test]
    fn tridexel_slot_cut_undercut_split_and_watertight_meshing_are_accurate() {
        let lib = default_starter_tool_library();
        let flat_6 = &lib.tools[0];

        let mut setup = Setup::new("s1", "Slot Test", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        };
        setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(100.0),
            depth_y: ParamLength::from_mm(100.0),
            height_z: ParamLength::from_mm(20.0),
            xy_align: chip_cam::FixedBoxXyAlign::CenterXy,
            z_align: chip_cam::FixedBoxZAlign::default(),
        };
        let empty_doc = CadDocument::empty();
        let resolved = setup.resolve(&empty_doc, DisplayUnit::Millimeter).unwrap();

        let mut b = ToolpathBuilder::new("op_slot");
        b.push_event(ClEvent::SpindleStart {
            rpm: 12_000.0,
            direction: chip_tools::SpindleDirection::Cw,
        });
        b.rapid_to(DVec3::new(-10.0, 0.0, 5.0));
        b.linear_to(DVec3::new(-10.0, 0.0, -5.0), 300.0, MotionRole::Plunge);
        b.linear_to(DVec3::new(10.0, 0.0, -5.0), 1000.0, MotionRole::Cut);
        b.rapid_to(DVec3::new(10.0, 0.0, 5.0));
        let tp = b.finish();

        let sim = run_setup_simulation_with_engine(
            &SetupSimulationInput {
                doc: &empty_doc,
                setup: &resolved,
                target_bodies: &[],
                operations: &[SimOperationInput {
                    op_id: "op_slot",
                    op_name: "Slot",
                    tool: flat_6,
                    fallback_spindle_rpm: 12_000.0,
                    toolpath: &tp,
                    intentional_edge_break_mm: 0.0,
                }],
                resolution: SimResolution::Standard,
                kinematics: MachineKinematicsConfig::default(),
                verify_config: SimVerificationConfig::default(),
                cancel: None,
            },
            SimEngineKind::TriDexel,
        )
        .unwrap();

        assert_eq!(sim.engine, SimEngineKind::TriDexel);
        assert!(!sim.has_critical_errors(), "diags: {:?}", sim.diagnostics);

        let expected_removed = (20.0 * 6.0 + PI * 9.0) * 5.0;
        let rel_err = (sim.removed_volume_mm3 - expected_removed).abs() / expected_removed;
        assert!(
            rel_err < 0.035,
            "Standard removed vol {:.2} vs expected {:.2} (err {:.2}%)",
            sim.removed_volume_mm3,
            expected_removed,
            rel_err * 100.0
        );

        let sim_fine = run_setup_simulation_with_engine(
            &SetupSimulationInput {
                doc: &empty_doc,
                setup: &resolved,
                target_bodies: &[],
                operations: &[SimOperationInput {
                    op_id: "op_slot",
                    op_name: "Slot",
                    tool: flat_6,
                    fallback_spindle_rpm: 12_000.0,
                    toolpath: &tp,
                    intentional_edge_break_mm: 0.0,
                }],
                resolution: SimResolution::Fine,
                kinematics: MachineKinematicsConfig::default(),
                verify_config: SimVerificationConfig::default(),
                cancel: None,
            },
            SimEngineKind::TriDexel,
        )
        .unwrap();
        let fine_err = (sim_fine.removed_volume_mm3 - expected_removed).abs() / expected_removed;
        assert!(
            fine_err < 0.015,
            "Fine removed vol {:.2} vs expected {:.2} (err {:.2}%)",
            sim_fine.removed_volume_mm3,
            expected_removed,
            fine_err * 100.0
        );

        let td = sim.as_tridexel().unwrap();
        let mesh = tessellate_tridexel_stock_mesh(td, true);
        assert!(mesh.vertices.len() > 100);
        assert_eq!(mesh.indices.len() % 3, 0);

        let mut ray = vec![DexelSpan::new(-20.0, 0.0, UNCUT_OP_IDX, UNCUT_OP_IDX)];
        let rem = subtract_dexel_interval(&mut ray, -12.0, -8.0, 0);
        assert!((rem - 4.0).abs() < 1e-12);
        assert_eq!(ray.len(), 2, "undercut must split span into 2 intervals");
        assert!((ray[0].max_mm - (-12.0)).abs() < 1e-12);
        assert!((ray[1].min_mm - (-8.0)).abs() < 1e-12);
    }

    #[test]
    fn tridexel_fault_injection_catches_rapid_crash_and_vertical_wall_target_gouge() {
        let step = sample_machined_plate_step().unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("plate.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let lib = default_starter_tool_library();
        let flat_6 = &lib.tools[0];

        let mut setup = Setup::new("s1", "Fault Test", WorkOffsetSlot::G54);
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
        let maabb = resolved.model_aabb_wcs;
        let center = maabb.center();

        let mut b = ToolpathBuilder::new("op_bad");
        b.push_event(ClEvent::SpindleStart {
            rpm: 12_000.0,
            direction: chip_tools::SpindleDirection::Cw,
        });
        b.rapid_to(DVec3::new(center.x, center.y, maabb.max.z + 15.0));
        b.rapid_to(DVec3::new(center.x, center.y, maabb.max.z - 5.0));
        b.linear_to(
            DVec3::new(center.x + 15.0, center.y, maabb.max.z - 5.0),
            800.0,
            MotionRole::Cut,
        );
        let tp = b.finish();

        let sim = run_setup_simulation_with_engine(
            &SetupSimulationInput {
                doc: &doc,
                setup: &resolved,
                target_bodies: &[],
                operations: &[SimOperationInput {
                    op_id: "op_bad",
                    op_name: "Bad Cut",
                    tool: flat_6,
                    fallback_spindle_rpm: 12_000.0,
                    toolpath: &tp,
                    intentional_edge_break_mm: 0.0,
                }],
                resolution: SimResolution::Standard,
                kinematics: MachineKinematicsConfig::default(),
                verify_config: SimVerificationConfig::default(),
                cancel: None,
            },
            SimEngineKind::TriDexel,
        )
        .unwrap();

        assert!(sim.has_critical_errors());
        assert!(
            sim.diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::RapidIntoStock),
            "expected RapidIntoStock diagnostic: {:?}",
            sim.diagnostics
        );
        assert!(
            sim.diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::TargetModelGouge),
            "expected TargetModelGouge diagnostic: {:?}",
            sim.diagnostics
        );
    }

    #[test]
    fn cylindrical_stock_turning_volume_and_chucked_square_bar_are_exact() {
        let lib = default_starter_tool_library();
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
            diameter: ParamLength::from_mm(44.0),
            length: ParamLength::from_mm(80.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup.rotary.mode = RotaryDriveMode::TwoLinearPlusRotary;
        setup.rotary.axis = RotaryAxisKind::XAxis;

        let doc = CadDocument::empty();
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
        assert!(resolved.rotary.is_some());

        let mut b = ToolpathBuilder::new("op_turn");
        b.rapid_to(DVec3::new(-10.0, 0.0, 30.0));
        b.linear_to(DVec3::new(-10.0, 0.0, 18.0), 400.0, MotionRole::Plunge);
        b.rotary_linear_to(
            DVec3::new(-10.0, 0.0, 18.0),
            360.0,
            800.0,
            18.0,
            MotionRole::Cut,
        );
        let n_turns = 10usize;
        let steps = n_turns * 36;
        for s in 1..=steps {
            let frac = s as f64 / steps as f64;
            let x = -10.0 + 20.0 * frac;
            let deg = 360.0 + 360.0 * n_turns as f64 * frac;
            b.rotary_linear_to(DVec3::new(x, 0.0, 18.0), deg, 800.0, 18.0, MotionRole::Cut);
        }
        b.rotary_linear_to(
            DVec3::new(10.0, 0.0, 18.0),
            360.0 * (n_turns as f64 + 2.0),
            800.0,
            18.0,
            MotionRole::FinishCut,
        );
        b.rapid_to(DVec3::new(10.0, 0.0, 30.0));
        let tp = b.finish();

        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            operations: &[SimOperationInput {
                op_id: "op_turn",
                op_name: "Rotary Turn Groove",
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

        assert_eq!(
            sim.critical_error_count(),
            0,
            "Clean rotary turning groove should have 0 crashes, got: {:#?}",
            sim.diagnostics
        );
        let exact_v = PI * (22.0_f64.powi(2) - 18.0_f64.powi(2)) * 26.0;
        let err_pct = ((sim.removed_volume_mm3 - exact_v).abs() / exact_v) * 100.0;
        assert!(
            err_pct < 3.0,
            "Removed volume {:.1} mm³ should match analytical {:.1} mm³ within 3% (err={err_pct:.2}%)",
            sim.removed_volume_mm3,
            exact_v
        );

        let cyl_stock = sim.as_cylindrical().unwrap();
        let mesh = tessellate_cylindrical_stock_mesh(cyl_stock, true, 45.0);
        assert!(mesh.vertices.len() > 1_000);
        assert_eq!(mesh.indices.len() % 3, 0);

        let mut sq_setup = Setup::new("s_sq", "Square Bar", WorkOffsetSlot::G54);
        sq_setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27 {
                x: AxisAnchor::Center,
                y: AxisAnchor::Center,
                z: AxisAnchor::Center,
            },
        };
        sq_setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(120.0),
            depth_y: ParamLength::from_mm(100.0),
            height_z: ParamLength::from_mm(100.0),
            xy_align: chip_cam::FixedBoxXyAlign::CenterXy,
            z_align: chip_cam::FixedBoxZAlign::CenterZ,
        };
        sq_setup.rotary.mode = RotaryDriveMode::Indexed3Plus1;
        sq_setup.rotary.axis = RotaryAxisKind::XAxis;
        let sq_res = sq_setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
        let sq_cyl = CylindricalStock::from_stock_spec(
            &stock_shape_spec_from_cam(&sq_res.stock),
            rotary_stock_spec_from_cam(sq_res.rotary.unwrap()),
            SimResolution::Standard,
        )
        .unwrap();
        assert!(
            approx_eq(sq_cyl.initial_volume_mm3(), 1_200_000.0, 2_000.0),
            "Square bar polar volume should be ~1,200,000 mm³, got {:.1}",
            sq_cyl.initial_volume_mm3()
        );
    }

    #[test]
    fn fault_injection_catches_rotary_rapid_crash_target_gouge_and_chuck_collision() {
        let stl = sample_rotary_shaft_stl_bytes();
        let doc = load_mesh_cad_document("rotary_shaft.stl", &stl, &MeshImportOptions::default())
            .unwrap();
        let lib = default_starter_tool_library();
        let flat_6mm = &lib.tools[0];

        let mut setup = Setup::new("s_shaft", "Rotary Shaft", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
            anchor: BoxAnchor27 {
                x: AxisAnchor::Center,
                y: AxisAnchor::Center,
                z: AxisAnchor::Center,
            },
        };
        setup.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(44.0),
            length: ParamLength::from_mm(80.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
        setup.rotary.axis = RotaryAxisKind::XAxis;
        setup.rotary.chuck_length = ParamLength::from_mm(20.0);
        setup.rotary.chuck_diameter = ParamLength::from_mm(65.0);
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

        let mut b = ToolpathBuilder::new("op_fault");
        b.rapid_to(DVec3::new(0.0, 0.0, 35.0));
        b.rapid_to(DVec3::new(0.0, 0.0, 17.0));
        b.linear_to(DVec3::new(0.0, 0.0, 10.0), 400.0, MotionRole::Cut);
        b.rapid_to(DVec3::new(0.0, 0.0, 35.0));
        b.rapid_to(DVec3::new(-48.0, 0.0, 25.0));
        let tp = b.finish();

        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            operations: &[SimOperationInput {
                op_id: "op_fault",
                op_name: "Fault Injection",
                tool: flat_6mm,
                fallback_spindle_rpm: 10_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp,
            }],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            cancel: None,
        })
        .unwrap();

        assert!(
            sim.diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::RapidIntoStock),
            "Must detect rotary RapidIntoStock, got: {:#?}",
            sim.diagnostics
        );
        assert!(
            sim.diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::TargetModelGouge),
            "Must detect rotary TargetModelGouge on shaft STL, got: {:#?}",
            sim.diagnostics
        );
        assert!(
            sim.diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::HolderCollision),
            "Must detect rotary chuck HolderCollision, got: {:#?}",
            sim.diagnostics
        );
    }

    #[test]
    fn off_centerline_simultaneous_4axis_fault_injection_catches_flank_gouge_and_holder_crash() {
        let stl = chip_cad::sample_helical_flute_shaft_stl_bytes();
        let doc = load_mesh_cad_document(
            "helical_flute_shaft.stl",
            &stl,
            &MeshImportOptions::default(),
        )
        .unwrap();
        assert!(doc.meshes[0].diagnostics.is_watertight);
        let lib = default_starter_tool_library();
        let flat_6mm = &lib.tools[0];

        let mut setup = Setup::new("s_helical", "Helical Shaft", WorkOffsetSlot::G54);
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

        let mut b = ToolpathBuilder::new("op_4ax_fault");
        b.rotary_rapid_to(DVec3::new(0.0, 8.0, 35.0), 0.0);
        b.rotary_linear_to(DVec3::new(0.0, 8.0, 9.0), 0.0, 400.0, 12.0, MotionRole::Cut);
        b.rotary_linear_to(
            DVec3::new(10.0, 8.0, 9.0),
            45.0,
            400.0,
            12.0,
            MotionRole::Cut,
        );
        b.rotary_rapid_to(DVec3::new(10.0, 8.0, 35.0), 45.0);
        b.rotary_rapid_to(DVec3::new(-32.0, -7.0, 35.0), 45.0);
        b.rotary_linear_to(
            DVec3::new(-32.0, -7.0, -25.0),
            45.0,
            300.0,
            20.0,
            MotionRole::Plunge,
        );
        let tp = b.finish();

        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            operations: &[SimOperationInput {
                op_id: "op_4ax_fault",
                op_name: "Off-Centerline Faults",
                tool: flat_6mm,
                fallback_spindle_rpm: 10_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp,
            }],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            cancel: None,
        })
        .unwrap();

        assert!(
            sim.diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::TargetModelGouge),
            "Off-centerline 4-axis verifier must catch helical flank gouge, got: {:#?}",
            sim.diagnostics
        );
        assert!(
            sim.diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::HolderCollision),
            "Off-centerline 4-axis verifier must catch deep holder crash, got: {:#?}",
            sim.diagnostics
        );
    }

    #[test]
    fn rotary_setup_simulation_populates_error_report_per_op_and_tolerance_recompute() {
        let stl = sample_rotary_shaft_stl_bytes();
        let doc = load_mesh_cad_document("rotary_shaft.stl", &stl, &MeshImportOptions::default())
            .unwrap();
        let lib = default_starter_tool_library();
        let flat_6mm = &lib.tools[0];

        let mut setup = Setup::new("s_shaft_err", "Rotary Shaft Error", WorkOffsetSlot::G54);
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

        let mut b = ToolpathBuilder::new("op_turn");
        b.rotary_rapid_to(DVec3::new(-10.0, 0.0, 30.0), 0.0);
        b.rotary_linear_to(
            DVec3::new(-10.0, 0.0, 19.0),
            0.0,
            400.0,
            19.0,
            MotionRole::Plunge,
        );
        b.rotary_linear_to(
            DVec3::new(10.0, 0.0, 19.0),
            360.0,
            800.0,
            19.0,
            MotionRole::Cut,
        );
        b.rotary_rapid_to(DVec3::new(10.0, 0.0, 30.0), 360.0);
        let tp = b.finish();

        let sim = run_setup_simulation_with_engine(
            &SetupSimulationInput {
                doc: &doc,
                setup: &resolved,
                target_bodies: &[],
                operations: &[SimOperationInput {
                    op_id: "op_turn",
                    op_name: "Rotary Rough",
                    tool: flat_6mm,
                    fallback_spindle_rpm: 10_000.0,
                    intentional_edge_break_mm: 0.0,
                    toolpath: &tp,
                }],
                resolution: SimResolution::Standard,
                kinematics: MachineKinematicsConfig::default(),
                verify_config: SimVerificationConfig::default(),
                cancel: None,
            },
            SimEngineKind::TriDexel,
        )
        .unwrap();

        let rep = sim
            .error_report
            .as_ref()
            .expect("4th-axis rotary simulation with 3D target must populate error_report");
        assert!(rep.target_volume_mm3 > 10_000.0);
        assert!(rep.sample_count > 1_000);
        assert!(rep.rms_error_mm.is_finite() && rep.rms_error_mm >= 0.0);
        assert!(rep.total_error_volume_mm3 > 0.0);
        assert_eq!(rep.per_operation.len(), 1);
        assert_eq!(rep.per_operation[0].op_name, "Rotary Rough");

        let tight = sim
            .compute_error_report_at_tolerance(0.01)
            .expect("recompute_at_tolerance must succeed for cylindrical stock");
        let loose = sim
            .compute_error_report_at_tolerance(5.0)
            .expect("recompute_at_tolerance must succeed for cylindrical stock");
        assert!(
            loose.in_tolerance_surface_pct >= tight.in_tolerance_surface_pct,
            "loose tol ({:.2}%) should be >= tight tol ({:.2}%)",
            loose.in_tolerance_surface_pct,
            tight.in_tolerance_surface_pct
        );
    }

    #[test]
    fn cylindrical_error_heatmap_tessellation_colors_in_tol_excess_and_gouge_bands() {
        let mut setup = Setup::new("s_hm", "Rotary Heatmap", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::VOLUMETRIC_CENTER,
        };
        setup.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(40.0),
            length: ParamLength::from_mm(60.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
        setup.rotary.axis = RotaryAxisKind::XAxis;

        let doc = CadDocument::empty();
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
        let mut stock = CylindricalStock::from_stock_spec(
            &stock_shape_spec_from_cam(&resolved.stock),
            rotary_stock_spec_from_cam(resolved.rotary.unwrap()),
            SimResolution::Standard,
        )
        .unwrap();

        let nu = stock.nu;
        let nt = stock.ntheta;
        for iu in 0..nu {
            for it in 0..nt {
                let idx = iu * nt + it;
                stock.r_target[idx] = 18.0_f32;
                stock.r_stock[idx] = if iu < nu / 3 {
                    19.0_f32
                } else if iu < (2 * nu) / 3 {
                    18.0_f32
                } else {
                    17.0_f32
                };
            }
        }

        let hm_mesh = tessellate_cylindrical_stock_mesh_colored(
            &stock,
            SimStockColorMode::ErrorHeatmap {
                tolerance_mm: 0.05,
                max_scale_mm: 1.0,
            },
            0.0,
        );
        let green = deviation_to_heatmap_rgba(0.0, 0.05, 1.0);
        let blue = deviation_to_heatmap_rgba(1.0, 0.05, 1.0);
        let red = deviation_to_heatmap_rgba(-1.0, 0.05, 1.0);
        let close = |a: [f32; 4], b: [f32; 4]| {
            (a[0] - b[0]).abs() < 1e-3 && (a[1] - b[1]).abs() < 1e-3 && (a[2] - b[2]).abs() < 1e-3
        };
        assert!(hm_mesh.vertices.iter().any(|v| close(v.color_rgba, green)));
        assert!(hm_mesh.vertices.iter().any(|v| close(v.color_rgba, blue)));
        assert!(hm_mesh.vertices.iter().any(|v| close(v.color_rgba, red)));

        let metal_mesh =
            tessellate_cylindrical_stock_mesh_colored(&stock, SimStockColorMode::Metallic, 0.0);
        assert!(metal_mesh.vertices.iter().all(|v| {
            close(v.color_rgba, RAW_STOCK_TOP_COLOR) || close(v.color_rgba, RAW_STOCK_SIDE_COLOR)
        }));
    }

    #[test]
    fn unified_playback_cursor_matches_full_reconstruction_across_all_three_stock_engines() {
        let lib = default_starter_tool_library();
        let flat_6mm = &lib.tools[0];
        let doc = CadDocument::empty();

        // 1. 3-axis Box Setup (tested on both Heightfield and TriDexel)
        let mut setup_3ax = Setup::new("s_3ax", "3-Axis Scrub", WorkOffsetSlot::G54);
        setup_3ax.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        };
        setup_3ax.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(100.0),
            depth_y: ParamLength::from_mm(100.0),
            height_z: ParamLength::from_mm(20.0),
            xy_align: chip_cam::FixedBoxXyAlign::CenterXy,
            z_align: chip_cam::FixedBoxZAlign::CenterZ,
        };
        let resolved_3ax = setup_3ax.resolve(&doc, DisplayUnit::Millimeter).unwrap();

        let mut b3 = ToolpathBuilder::new("op_3ax");
        b3.rapid_to(DVec3::new(-25.0, 0.0, 10.0));
        b3.linear_to(DVec3::new(-25.0, 0.0, -5.0), 300.0, MotionRole::Plunge);
        b3.linear_to(DVec3::new(25.0, 0.0, -5.0), 600.0, MotionRole::Cut);
        b3.arc_xy_to(XyArcMove {
            end_wcs: DVec3::new(-25.0, 0.0, -5.0),
            center_xy: DVec2::ZERO,
            radius_mm: 25.0,
            ccw: true,
            turns: 0,
            feed_mm_min: 600.0,
            role: MotionRole::Cut,
        });
        b3.rapid_to(DVec3::new(-25.0, 0.0, 10.0));
        let tp3 = b3.finish();

        // 2. 4-axis Rotary Cylinder Setup (Cylindrical engine)
        let mut setup_4ax = Setup::new("s_4ax", "4-Axis Scrub", WorkOffsetSlot::G54);
        setup_4ax.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::VOLUMETRIC_CENTER,
        };
        setup_4ax.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(40.0),
            length: ParamLength::from_mm(60.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup_4ax.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
        setup_4ax.rotary.axis = RotaryAxisKind::XAxis;
        let resolved_4ax = setup_4ax.resolve(&doc, DisplayUnit::Millimeter).unwrap();

        let mut b4 = ToolpathBuilder::new("op_4ax");
        b4.rotary_rapid_to(DVec3::new(-20.0, 0.0, 26.0), 0.0);
        b4.rotary_linear_to(
            DVec3::new(-20.0, 0.0, 16.0),
            0.0,
            400.0,
            16.0,
            MotionRole::Plunge,
        );
        b4.rotary_linear_to(
            DVec3::new(20.0, 0.0, 16.0),
            180.0,
            600.0,
            16.0,
            MotionRole::Cut,
        );
        b4.rotary_rapid_to(DVec3::new(20.0, 0.0, 26.0), 180.0);
        let tp4 = b4.finish();

        let sim_hf = run_setup_simulation_with_engine(
            &SetupSimulationInput {
                doc: &doc,
                setup: &resolved_3ax,
                target_bodies: &[],
                operations: &[SimOperationInput {
                    op_id: "op_3ax",
                    op_name: "3-Axis Cut",
                    tool: flat_6mm,
                    fallback_spindle_rpm: 12_000.0,
                    intentional_edge_break_mm: 0.0,
                    toolpath: &tp3,
                }],
                resolution: SimResolution::Standard,
                kinematics: MachineKinematicsConfig::default(),
                verify_config: SimVerificationConfig::default(),
                cancel: None,
            },
            SimEngineKind::Heightmap,
        )
        .unwrap();

        let sim_td = run_setup_simulation_with_engine(
            &SetupSimulationInput {
                doc: &doc,
                setup: &resolved_3ax,
                target_bodies: &[],
                operations: &[SimOperationInput {
                    op_id: "op_3ax",
                    op_name: "3-Axis Cut",
                    tool: flat_6mm,
                    fallback_spindle_rpm: 12_000.0,
                    intentional_edge_break_mm: 0.0,
                    toolpath: &tp3,
                }],
                resolution: SimResolution::Standard,
                kinematics: MachineKinematicsConfig::default(),
                verify_config: SimVerificationConfig::default(),
                cancel: None,
            },
            SimEngineKind::TriDexel,
        )
        .unwrap();

        let sim_cyl = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved_4ax,
            target_bodies: &[],
            operations: &[SimOperationInput {
                op_id: "op_4ax",
                op_name: "4-Axis Cut",
                tool: flat_6mm,
                fallback_spindle_rpm: 12_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp4,
            }],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            cancel: None,
        })
        .unwrap();

        assert!(sim_hf.as_heightfield().is_some());
        assert!(sim_td.as_tridexel().is_some());
        assert!(sim_cyl.as_cylindrical().is_some());

        for (label, sim) in [
            ("Heightfield", &sim_hf),
            ("TriDexel", &sim_td),
            ("Cylindrical", &sim_cyl),
        ] {
            let mut working = sim.final_stock.clone();
            let mut cursor: Option<StockPlaybackCursor> = None;
            let total_t = sim.timeline.total_time_sec;
            let fracs = [0.0, 0.20, 0.45, 0.70, 0.90, 0.30, 0.60, 1.0];

            for frac in fracs {
                let t = total_t * frac;
                let snap = sim.timeline.sample_at_time(t).unwrap();
                let _ = working.sync_to_scrub_snapshot(&mut cursor, sim, &snap);
                let reference = sim.reconstruct_stock_at_time(t);

                let vol_work = working.compute_volume_mm3();
                let vol_ref = reference.compute_volume_mm3();
                let tol = (sim.initial_stock_volume_mm3 * 1e-4).max(1.0);
                assert!(
                    (vol_work - vol_ref).abs() <= tol,
                    "{label} incremental scrub volume diverged at frac={frac}: work={vol_work:.3}, ref={vol_ref:.3}"
                );
            }
        }
    }

    #[cfg(feature = "gpu-compute")]
    #[test]
    fn gpu_vs_cpu_heightfield_end_to_end_parity_all_rules_and_cutters() {
        let step = sample_machined_plate_step().unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("plate.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let lib = default_starter_tool_library();
        let flat_6 = &lib.tools[0];
        let vbit = lib
            .tools
            .iter()
            .find(|t| matches!(t.geometry, CutterGeometry::ChamferMill { .. }))
            .unwrap();
        let face = lib
            .tools
            .iter()
            .find(|t| matches!(t.geometry, CutterGeometry::FaceMill { .. }))
            .unwrap();

        let mut setup = Setup::new("s_hf_parity", "HF Parity", WorkOffsetSlot::G54);
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
        let maabb = resolved.model_aabb_wcs;
        let c = maabb.center();

        // Op 0: Clean sloped ramp + XY arc + deliberate rapid crash + target gouge
        let mut b0 = ToolpathBuilder::new("op_flat_fault");
        b0.rapid_to(DVec3::new(c.x - 12.0, c.y, maabb.max.z + 10.0));
        b0.linear_to(
            DVec3::new(c.x - 12.0, c.y, maabb.max.z - 1.5),
            300.0,
            MotionRole::Plunge,
        );
        b0.linear_to(
            DVec3::new(c.x, c.y + 4.0, maabb.max.z - 3.0),
            800.0,
            MotionRole::Cut,
        );
        b0.arc_xy_to(XyArcMove {
            end_wcs: DVec3::new(c.x + 10.0, c.y + 4.0, maabb.max.z - 3.0),
            center_xy: DVec2::new(c.x + 5.0, c.y + 4.0),
            radius_mm: 5.0,
            ccw: true,
            turns: 0,
            feed_mm_min: 700.0,
            role: MotionRole::Cut,
        });
        // Rapid crash + target gouge
        b0.rapid_to(DVec3::new(c.x, c.y - 6.0, maabb.max.z - 6.0));
        b0.rapid_to(DVec3::new(c.x, c.y - 6.0, maabb.max.z + 10.0));
        let tp0 = b0.finish();

        // Op 1: V-bit sloped climb
        let mut b1 = ToolpathBuilder::new("op_vbit");
        b1.rapid_to(DVec3::new(c.x - 8.0, c.y + 8.0, maabb.max.z + 10.0));
        b1.linear_to(
            DVec3::new(c.x - 8.0, c.y + 8.0, maabb.max.z - 3.5),
            400.0,
            MotionRole::Plunge,
        );
        b1.linear_to(
            DVec3::new(c.x + 8.0, c.y + 12.0, maabb.max.z - 0.5),
            600.0,
            MotionRole::Cut,
        );
        b1.rapid_to(DVec3::new(c.x + 8.0, c.y + 12.0, maabb.max.z + 10.0));
        let tp1 = b1.finish();

        // Op 2: Non-center-cutting FaceMill vertical plunge (Rule 5 check!)
        let mut b2 = ToolpathBuilder::new("op_face_plunge");
        b2.rapid_to(DVec3::new(c.x + 14.0, c.y - 10.0, maabb.max.z + 8.0));
        b2.linear_to(
            DVec3::new(c.x + 14.0, c.y - 10.0, maabb.max.z - 2.0),
            200.0,
            MotionRole::Plunge,
        );
        b2.rapid_to(DVec3::new(c.x + 14.0, c.y - 10.0, maabb.max.z + 8.0));
        let tp2 = b2.finish();

        let ops = [
            SimOperationInput {
                op_id: "op_flat_fault",
                op_name: "Flat Ramp & Fault",
                tool: flat_6,
                fallback_spindle_rpm: 12_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp0,
            },
            SimOperationInput {
                op_id: "op_vbit",
                op_name: "V-Carve Climb",
                tool: vbit,
                fallback_spindle_rpm: 14_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp1,
            },
            SimOperationInput {
                op_id: "op_face_plunge",
                op_name: "Face Plunge Rule 5",
                tool: face,
                fallback_spindle_rpm: 8_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp2,
            },
        ];

        let input = SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            operations: &ops,
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            cancel: None,
        };

        let sim_cpu = run_setup_simulation_with_choice(
            &input,
            SimEngineKind::Heightmap,
            SimComputeBackendChoice::Cpu,
        )
        .unwrap();
        let sim_gpu = run_setup_simulation_with_choice(
            &input,
            SimEngineKind::Heightmap,
            SimComputeBackendChoice::Gpu,
        )
        .unwrap();

        assert!(!sim_cpu.active_backend.is_gpu);
        assert!(sim_gpu.active_backend.is_gpu);

        let vol_err = (sim_gpu.remaining_stock_volume_mm3 - sim_cpu.remaining_stock_volume_mm3)
            .abs()
            / sim_cpu.remaining_stock_volume_mm3.max(1.0);
        assert!(
            vol_err < 0.0005,
            "Heightfield E2E volume parity failed: cpu={:.3}, gpu={:.3}, err={:.5}%",
            sim_cpu.remaining_stock_volume_mm3,
            sim_gpu.remaining_stock_volume_mm3,
            vol_err * 100.0
        );

        let hf_cpu = sim_cpu.as_heightfield().unwrap();
        let hf_gpu = sim_gpu.as_heightfield().unwrap();
        let mut max_delta = 0.0_f64;
        let mut worst = (0usize, 0usize, 0.0_f32, 0.0_f32, 0u32, 0u32);
        for (idx, (zc, zg)) in hf_cpu.z_stock.iter().zip(hf_gpu.z_stock.iter()).enumerate() {
            assert_eq!(zc.is_finite(), zg.is_finite());
            if zc.is_finite() {
                let d = (f64::from(*zg) - f64::from(*zc)).abs();
                if d > max_delta {
                    max_delta = d;
                    worst = (
                        idx % hf_cpu.nx,
                        idx / hf_cpu.nx,
                        *zc,
                        *zg,
                        hf_cpu.last_op_idx[idx],
                        hf_gpu.last_op_idx[idx],
                    );
                }
            }
        }
        assert!(
            max_delta <= 2e-3,
            "Heightfield E2E max cell delta {max_delta:.6} mm > 2e-3 mm at ({}, {}) wcs=({:.4},{:.4}) origin=({:.4},{:.4},{:.4}) d=({:.5},{:.5}) c=({:.4},{:.4},{:.4}) maxz={:.4}: cpu={} (op {}), gpu={} (op {})",
            worst.0,
            worst.1,
            hf_cpu.origin_wcs.x + (worst.0 as f64 + 0.5) * hf_cpu.dx_mm,
            hf_cpu.origin_wcs.y + (worst.1 as f64 + 0.5) * hf_cpu.dy_mm,
            hf_cpu.origin_wcs.x,
            hf_cpu.origin_wcs.y,
            hf_cpu.origin_wcs.z,
            hf_cpu.dx_mm,
            hf_cpu.dy_mm,
            c.x,
            c.y,
            c.z,
            maabb.max.z,
            worst.2,
            worst.4,
            worst.3,
            worst.5
        );

        // Identical Rule 1-5 diagnostic counts & severities
        assert_eq!(
            sim_cpu.diagnostics.len(),
            sim_gpu.diagnostics.len(),
            "Diagnostic count mismatch:\nCPU: {:#?}\nGPU: {:#?}",
            sim_cpu.diagnostics,
            sim_gpu.diagnostics
        );
        for (dc, dg) in sim_cpu.diagnostics.iter().zip(sim_gpu.diagnostics.iter()) {
            assert_eq!(dc.kind, dg.kind);
            assert_eq!(dc.severity, dg.severity);
            assert_eq!(dc.op_idx, dg.op_idx);
        }
        assert!(
            sim_gpu
                .diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::UnsafeVerticalPlunge),
            "Rule 5 must fire identically on GPU backend"
        );
    }

    #[cfg(feature = "gpu-compute")]
    #[test]
    fn gpu_vs_cpu_cylindrical_end_to_end_parity_rotary_shaft_wrapped_and_4axis_sim() {
        let lib = default_starter_tool_library();
        let flat_6 = &lib.tools[0];
        let vbit = lib
            .tools
            .iter()
            .find(|t| matches!(t.geometry, CutterGeometry::ChamferMill { .. }))
            .unwrap();

        // Case 1: Rotary shaft turning + wrapped V-engraving + rapid/gouge/chuck faults
        let stl_shaft = sample_rotary_shaft_stl_bytes();
        let doc_shaft = load_mesh_cad_document(
            "rotary_shaft.stl",
            &stl_shaft,
            &MeshImportOptions::default(),
        )
        .unwrap();
        let mut setup_shaft = Setup::new("s_shaft_gpu", "Rotary Shaft GPU", WorkOffsetSlot::G54);
        setup_shaft.wcs.origin = WcsOriginMode::ModelBoxPoint {
            anchor: BoxAnchor27::VOLUMETRIC_CENTER,
        };
        setup_shaft.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(44.0),
            length: ParamLength::from_mm(80.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup_shaft.rotary.mode = RotaryDriveMode::TwoLinearPlusRotary;
        setup_shaft.rotary.axis = RotaryAxisKind::XAxis;
        setup_shaft.rotary.chuck_length = ParamLength::from_mm(20.0);
        setup_shaft.rotary.chuck_diameter = ParamLength::from_mm(65.0);
        let res_shaft = setup_shaft
            .resolve(&doc_shaft, DisplayUnit::Millimeter)
            .unwrap();

        let mut bt = ToolpathBuilder::new("op_turn_wrap");
        bt.rotary_rapid_to(DVec3::new(-12.0, 0.0, 32.0), 0.0);
        bt.rotary_linear_to(
            DVec3::new(-12.0, 0.0, 19.0),
            0.0,
            400.0,
            19.0,
            MotionRole::Plunge,
        );
        bt.rotary_linear_to(
            DVec3::new(12.0, 0.0, 19.0),
            360.0,
            800.0,
            19.0,
            MotionRole::Cut,
        );
        // Rapid crash + target gouge + chuck collision
        bt.rotary_rapid_to(DVec3::new(0.0, 0.0, 11.0), 360.0);
        bt.rotary_rapid_to(DVec3::new(0.0, 0.0, 35.0), 360.0);
        bt.rotary_rapid_to(DVec3::new(-46.0, 0.0, 24.0), 360.0);
        let tp_turn = bt.finish();

        // Wrapped V-bit engraving groove around cylinder
        let mut bw = ToolpathBuilder::new("op_wrapped");
        bw.rotary_rapid_to(DVec3::new(-8.0, 0.0, 26.0), 45.0);
        bw.rotary_linear_to(
            DVec3::new(-8.0, 0.0, 17.5),
            45.0,
            350.0,
            17.5,
            MotionRole::Plunge,
        );
        bw.rotary_linear_to(
            DVec3::new(8.0, 0.0, 17.5),
            225.0,
            600.0,
            17.5,
            MotionRole::Cut,
        );
        bw.rotary_rapid_to(DVec3::new(8.0, 0.0, 26.0), 225.0);
        let tp_wrap = bw.finish();

        let ops_shaft = [
            SimOperationInput {
                op_id: "op_turn_wrap",
                op_name: "Rotary Turn & Faults",
                tool: flat_6,
                fallback_spindle_rpm: 10_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp_turn,
            },
            SimOperationInput {
                op_id: "op_wrapped",
                op_name: "Wrapped V-Engrave",
                tool: vbit,
                fallback_spindle_rpm: 14_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp_wrap,
            },
        ];
        let in_shaft = SetupSimulationInput {
            doc: &doc_shaft,
            setup: &res_shaft,
            target_bodies: &[],
            operations: &ops_shaft,
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            cancel: None,
        };
        let shaft_cpu = run_setup_simulation_with_choice(
            &in_shaft,
            SimEngineKind::Heightmap,
            SimComputeBackendChoice::Cpu,
        )
        .unwrap();
        let shaft_gpu = run_setup_simulation_with_choice(
            &in_shaft,
            SimEngineKind::Heightmap,
            SimComputeBackendChoice::Gpu,
        )
        .unwrap();
        assert!(shaft_gpu.active_backend.is_gpu);

        let shaft_vol_err =
            (shaft_gpu.remaining_stock_volume_mm3 - shaft_cpu.remaining_stock_volume_mm3).abs()
                / shaft_cpu.remaining_stock_volume_mm3.max(1.0);
        assert!(
            shaft_vol_err < 0.001,
            "Rotary shaft volume parity failed: cpu={:.3}, gpu={:.3}, err={:.5}%",
            shaft_cpu.remaining_stock_volume_mm3,
            shaft_gpu.remaining_stock_volume_mm3,
            shaft_vol_err * 100.0
        );
        let mut max_d_shaft = 0.0_f64;
        let mut worst_shaft = (0usize, 0usize, 0.0_f32, 0.0_f32, 0u32, 0u32);
        let sc = shaft_cpu.as_cylindrical().unwrap();
        let sg = shaft_gpu.as_cylindrical().unwrap();
        for (idx, (a, b)) in sc.r_stock.iter().zip(sg.r_stock.iter()).enumerate() {
            let d = (f64::from(*a) - f64::from(*b)).abs();
            if d > max_d_shaft {
                max_d_shaft = d;
                worst_shaft = (
                    idx / sc.ntheta,
                    idx % sc.ntheta,
                    *a,
                    *b,
                    sc.last_op_idx[idx],
                    sg.last_op_idx[idx],
                );
            }
        }
        assert!(
            max_d_shaft <= 5e-3,
            "Rotary shaft max cell delta {max_d_shaft:.6} mm > 5e-3 mm at (iu={}, it={}): cpu={} (op {}), gpu={} (op {})",
            worst_shaft.0,
            worst_shaft.1,
            worst_shaft.2,
            worst_shaft.4,
            worst_shaft.3,
            worst_shaft.5
        );
        assert_eq!(shaft_cpu.diagnostics.len(), shaft_gpu.diagnostics.len());
        for (dc, dg) in shaft_cpu
            .diagnostics
            .iter()
            .zip(shaft_gpu.diagnostics.iter())
        {
            assert_eq!(dc.kind, dg.kind);
            assert_eq!(dc.severity, dg.severity);
        }

        // Case 2: Simultaneous 4-axis off-centerline helical flute shaft + flank gouge & holder crash
        let stl_helical = chip_cad::sample_helical_flute_shaft_stl_bytes();
        let doc_helical = load_mesh_cad_document(
            "helical_flute_shaft.stl",
            &stl_helical,
            &MeshImportOptions::default(),
        )
        .unwrap();
        let mut setup_4ax = Setup::new("s_4ax_gpu", "Helical 4-Axis GPU", WorkOffsetSlot::G54);
        setup_4ax.wcs.origin = WcsOriginMode::ModelBoxPoint {
            anchor: BoxAnchor27::VOLUMETRIC_CENTER,
        };
        setup_4ax.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(42.0),
            length: ParamLength::from_mm(80.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup_4ax.rotary.mode = RotaryDriveMode::Simultaneous4Axis;
        setup_4ax.rotary.axis = RotaryAxisKind::XAxis;
        let res_4ax = setup_4ax
            .resolve(&doc_helical, DisplayUnit::Millimeter)
            .unwrap();

        let mut b4 = ToolpathBuilder::new("op_4ax_helical");
        b4.rotary_rapid_to(DVec3::new(-10.0, 7.5, 34.0), 0.0);
        b4.rotary_linear_to(
            DVec3::new(-10.0, 7.5, 10.0),
            0.0,
            400.0,
            12.0,
            MotionRole::Cut,
        );
        b4.rotary_linear_to(
            DVec3::new(12.0, 7.5, 10.0),
            60.0,
            400.0,
            12.0,
            MotionRole::Cut,
        );
        b4.rotary_rapid_to(DVec3::new(12.0, 7.5, 34.0), 60.0);
        b4.rotary_rapid_to(DVec3::new(-30.0, -7.0, 34.0), 60.0);
        b4.rotary_linear_to(
            DVec3::new(-30.0, -7.0, -24.0),
            60.0,
            300.0,
            20.0,
            MotionRole::Plunge,
        );
        let tp_4ax = b4.finish();

        let in_4ax = SetupSimulationInput {
            doc: &doc_helical,
            setup: &res_4ax,
            target_bodies: &[],
            operations: &[SimOperationInput {
                op_id: "op_4ax_helical",
                op_name: "4-Axis Helical Flute",
                tool: flat_6,
                fallback_spindle_rpm: 10_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp_4ax,
            }],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            cancel: None,
        };
        let sim4_cpu = run_setup_simulation_with_choice(
            &in_4ax,
            SimEngineKind::Heightmap,
            SimComputeBackendChoice::Cpu,
        )
        .unwrap();
        let sim4_gpu = run_setup_simulation_with_choice(
            &in_4ax,
            SimEngineKind::Heightmap,
            SimComputeBackendChoice::Gpu,
        )
        .unwrap();
        assert!(sim4_gpu.active_backend.is_gpu);

        let vol4_err = (sim4_gpu.remaining_stock_volume_mm3 - sim4_cpu.remaining_stock_volume_mm3)
            .abs()
            / sim4_cpu.remaining_stock_volume_mm3.max(1.0);
        assert!(
            vol4_err < 0.001,
            "4-axis helical volume parity failed: cpu={:.3}, gpu={:.3}, err={:.5}%",
            sim4_cpu.remaining_stock_volume_mm3,
            sim4_gpu.remaining_stock_volume_mm3,
            vol4_err * 100.0
        );
        let max_d_4ax = sim4_cpu
            .as_cylindrical()
            .unwrap()
            .r_stock
            .iter()
            .zip(sim4_gpu.as_cylindrical().unwrap().r_stock.iter())
            .map(|(a, b)| (f64::from(*a) - f64::from(*b)).abs())
            .fold(0.0_f64, f64::max);
        assert!(
            max_d_4ax <= 5e-3,
            "4-axis helical max cell delta {max_d_4ax:.6} mm > 5e-3 mm"
        );
        assert_eq!(
            sim4_cpu.diagnostics.len(),
            sim4_gpu.diagnostics.len(),
            "4-axis diag mismatch:\nCPU: {:#?}\nGPU: {:#?}",
            sim4_cpu.diagnostics,
            sim4_gpu.diagnostics
        );
        for (dc, dg) in sim4_cpu.diagnostics.iter().zip(sim4_gpu.diagnostics.iter()) {
            assert_eq!(dc.kind, dg.kind);
            assert_eq!(dc.severity, dg.severity);
        }

        // Case 3: TriDexel gracefully delegates to CPU under Auto until Milestone 53
        let empty_doc = CadDocument::empty();
        let mut setup_td = Setup::new("s_td_fb", "TriDexel Fallback", WorkOffsetSlot::G54);
        setup_td.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        };
        setup_td.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(100.0),
            depth_y: ParamLength::from_mm(100.0),
            height_z: ParamLength::from_mm(20.0),
            xy_align: chip_cam::FixedBoxXyAlign::CenterXy,
            z_align: chip_cam::FixedBoxZAlign::default(),
        };
        let res_td = setup_td
            .resolve(&empty_doc, DisplayUnit::Millimeter)
            .unwrap();
        let sim_td = run_setup_simulation_with_choice(
            &SetupSimulationInput {
                doc: &empty_doc,
                setup: &res_td,
                target_bodies: &[],
                operations: &[],
                resolution: SimResolution::Standard,
                kinematics: MachineKinematicsConfig::default(),
                verify_config: SimVerificationConfig::default(),
                cancel: None,
            },
            SimEngineKind::TriDexel,
            SimComputeBackendChoice::Auto,
        )
        .unwrap();
        assert!(!sim_td.active_backend.is_gpu);
    }
}
