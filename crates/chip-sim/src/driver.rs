//! Unified setup simulation orchestrator and single polymorphic
//! [`run_setup_simulation_with_backend`] driver covering `Heightfield`, `TriDexel`, and
//! 4th-axis `Cylindrical` engines.

#[cfg(feature = "gpu-compute")]
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Instant;

use chip_cad::{BodyId, CadDocument};
use chip_cam::{
    ResolvedFixtureSolid, ResolvedSetup, SegmentMotion3D, ToolAssemblyPart, ToolAssemblyProfile,
    check_segments_fixture_collisions_par,
};
use chip_stock::{
    CpuExactBackend, CylindricalStock, HeightfieldStock, OperationErrorSnapshot,
    RadialCutterProfile, SimComputeBackendChoice, SimEngineKind, SimError, SimResolution,
    SimStockBackend, SimulatedStock, StockVerifyTolerances, SweepChord, SweepToolSpec,
    TriDexelStock,
};
#[cfg(feature = "gpu-compute")]
use chip_stock::{GpuSimContext, WgpuComputeBackend};
use chip_tools::CutterGeometry;

use crate::{
    cad_adapter::{
        extract_heightfield_target_from_cad, extract_target_trimesh_from_cad,
        extract_tridexel_target_from_cad, rotary_stock_spec_from_cam, stock_shape_spec_from_cam,
    },
    progress::{SimProgressScope, SimProgressStage},
    timeline::{MachineKinematicsConfig, SimOperationInput, SimulationTimeline},
    verifier::{SimIssueKind, SimVerificationConfig},
    verify_common::{
        RuleViolationHit, ViolationBatchCoalescer, evaluate_vertical_plunge_violation,
        push_segment_sweep_chords,
    },
};

pub use crate::result::{SetupSimulationResult, SimTimingBreakdown, SimulatedStockPlaybackExt};

/// Borrowed input bundle passed to [`run_setup_simulation`] and [`run_setup_simulation_with_backend`].
#[derive(Clone, Copy)]
pub struct SetupSimulationInput<'a> {
    pub doc: &'a CadDocument,
    pub setup: &'a ResolvedSetup,
    pub target_bodies: &'a [BodyId],
    pub fixtures: &'a [ResolvedFixtureSolid],
    pub operations: &'a [SimOperationInput<'a>],
    pub resolution: SimResolution,
    pub engine: SimEngineKind,
    pub backend_choice: SimComputeBackendChoice,
    pub kinematics: MachineKinematicsConfig,
    pub verify_config: SimVerificationConfig,
    /// Optional carried-over volumetric stock transferred from a preceding Setup (`M62`).
    /// When `Some`, Setup simulation starts from this machined solid instead of a fresh raw blank.
    pub initial_stock: Option<&'a TriDexelStock>,
    /// Optional original raw blank volume (`mm³`) from the root Setup of a carryover chain,
    /// preserved for cumulative `removed_volume_mm3` and `StockErrorReport` efficiency metrology.
    pub initial_raw_volume_mm3: Option<f64>,
    /// Optional display name of the preceding Setup that produced `initial_stock`.
    pub carried_over_from_setup: Option<&'a str>,
    /// Optional explicit shared GPU compute context (`None` uses `GpuSimContext::shared_headless()`
    /// when `backend_choice != SimComputeBackendChoice::Cpu`).
    #[cfg(feature = "gpu-compute")]
    pub gpu_ctx: Option<&'a Arc<GpuSimContext>>,
    /// Optional live completion progress reporter (`0%`–`100%`).
    pub progress: Option<SimProgressScope<'a>>,
    pub cancel: Option<&'a AtomicBool>,
}

impl<'a> SetupSimulationInput<'a> {
    /// Creates a [`SetupSimulationInput`] with default `Heightmap` engine, exact `Cpu` backend,
    /// `Standard` resolution, and default kinematics & verification tolerances.
    pub fn new(
        doc: &'a CadDocument,
        setup: &'a ResolvedSetup,
        operations: &'a [SimOperationInput<'a>],
    ) -> Self {
        Self {
            doc,
            setup,
            target_bodies: &[],
            fixtures: &[],
            operations,
            resolution: SimResolution::Standard,
            engine: SimEngineKind::Heightmap,
            backend_choice: SimComputeBackendChoice::Cpu,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            #[cfg(feature = "gpu-compute")]
            gpu_ctx: None,
            progress: None,
            cancel: None,
        }
    }

    #[inline]
    pub const fn with_engine(mut self, engine: SimEngineKind) -> Self {
        self.engine = engine;
        self
    }

    #[inline]
    pub const fn with_backend_choice(mut self, backend_choice: SimComputeBackendChoice) -> Self {
        self.backend_choice = backend_choice;
        self
    }

    #[inline]
    pub const fn with_resolution(mut self, resolution: SimResolution) -> Self {
        self.resolution = resolution;
        self
    }

    #[inline]
    pub const fn with_kinematics(mut self, kinematics: MachineKinematicsConfig) -> Self {
        self.kinematics = kinematics;
        self
    }

    #[inline]
    pub const fn with_verify_config(mut self, verify_config: SimVerificationConfig) -> Self {
        self.verify_config = verify_config;
        self
    }

    #[inline]
    pub const fn with_target_bodies(mut self, target_bodies: &'a [BodyId]) -> Self {
        self.target_bodies = target_bodies;
        self
    }

    #[inline]
    pub const fn with_fixtures(mut self, fixtures: &'a [ResolvedFixtureSolid]) -> Self {
        self.fixtures = fixtures;
        self
    }

    #[inline]
    pub const fn with_progress(mut self, progress: Option<SimProgressScope<'a>>) -> Self {
        self.progress = progress;
        self
    }

    #[inline]
    pub const fn with_cancel(mut self, cancel: Option<&'a AtomicBool>) -> Self {
        self.cancel = cancel;
        self
    }

    #[cfg(feature = "gpu-compute")]
    #[inline]
    pub const fn with_gpu_ctx(mut self, gpu_ctx: Option<&'a Arc<GpuSimContext>>) -> Self {
        self.gpu_ctx = gpu_ctx;
        self
    }
}

/// Executes volumetric stock removal (`Heightmap`, `TriDexel`, or 4th-axis `Cylindrical`),
/// periodic checkpoint capture, and 6-rule crash/gouge/fixture verification across all active
/// Setup operations using the engine, backend choice, GPU context, and progress scope configured
/// on [`SetupSimulationInput`] (`06-simulation.md`).
pub fn run_setup_simulation(
    input: &SetupSimulationInput<'_>,
) -> Result<SetupSimulationResult, SimError> {
    let t_total = Instant::now();
    let t_setup = Instant::now();

    #[cfg(feature = "gpu-compute")]
    let active_ctx = match input.backend_choice {
        SimComputeBackendChoice::Cpu => None,
        SimComputeBackendChoice::Auto | SimComputeBackendChoice::Gpu => input
            .gpu_ctx
            .cloned()
            .or_else(GpuSimContext::shared_headless),
    };

    let (effective_engine, stock, target_td) = prepare_initial_stock(
        input,
        input.engine,
        #[cfg(feature = "gpu-compute")]
        active_ctx.as_deref(),
        input.progress,
    )?;

    #[cfg(feature = "gpu-compute")]
    let mut res = if let Some(ctx) = active_ctx {
        let backend = WgpuComputeBackend::new(ctx, stock, target_td);
        let target_setup_ms = t_setup.elapsed().as_secs_f64() * 1000.0;
        let mut out = run_setup_simulation_with_backend(input, effective_engine, backend)?;
        finalize_timing(&mut out.timing, target_setup_ms, t_total);
        out
    } else {
        let backend = CpuExactBackend::new(stock, target_td);
        let target_setup_ms = t_setup.elapsed().as_secs_f64() * 1000.0;
        let mut out = run_setup_simulation_with_backend(input, effective_engine, backend)?;
        finalize_timing(&mut out.timing, target_setup_ms, t_total);
        out
    };

    #[cfg(not(feature = "gpu-compute"))]
    let mut res = {
        let backend = CpuExactBackend::new(stock, target_td);
        let target_setup_ms = t_setup.elapsed().as_secs_f64() * 1000.0;
        let mut out = run_setup_simulation_with_backend(input, effective_engine, backend)?;
        finalize_timing(&mut out.timing, target_setup_ms, t_total);
        out
    };

    res.timing.total_ms = t_total.elapsed().as_secs_f64() * 1000.0;
    res.timing.segments_per_sec =
        compute_segments_per_sec(res.timing.total_segments, res.timing.total_ms);
    Ok(res)
}

#[inline]
fn compute_segments_per_sec(total_segments: usize, total_ms: f64) -> f64 {
    if total_ms > 1e-6 {
        (total_segments as f64) / (total_ms / 1000.0)
    } else {
        0.0
    }
}

#[inline]
fn finalize_timing(timing: &mut SimTimingBreakdown, target_setup_ms: f64, t_total: Instant) {
    timing.target_setup_ms = target_setup_ms;
    timing.total_ms = t_total.elapsed().as_secs_f64() * 1000.0;
    timing.segments_per_sec = compute_segments_per_sec(timing.total_segments, timing.total_ms);
}

fn prepare_initial_stock(
    input: &SetupSimulationInput<'_>,
    engine: SimEngineKind,
    #[cfg(feature = "gpu-compute")] gpu_ctx: Option<&GpuSimContext>,
    progress: Option<SimProgressScope<'_>>,
) -> Result<(SimEngineKind, SimulatedStock, Option<TriDexelStock>), SimError> {
    if let Some(p) = progress {
        p.report(0.0, SimProgressStage::PreparingStock);
    }
    check_cancelled(input.cancel)?;
    let stock_spec = stock_shape_spec_from_cam(&input.setup.stock);

    if let Some(rotary) = input.setup.rotary
        && engine != SimEngineKind::TriDexel
    {
        let rotary_spec = rotary_stock_spec_from_cam(rotary);
        let mut cyl =
            CylindricalStock::from_stock_spec(&stock_spec, rotary_spec, input.resolution)?;
        if let Some(p) = progress {
            p.report(0.03, SimProgressStage::PreparingStock);
        }
        check_cancelled(input.cancel)?;
        if let Some(target_mesh) =
            extract_target_trimesh_from_cad(input.doc, input.setup, input.target_bodies)
        {
            #[cfg(feature = "gpu-compute")]
            if let Some(ctx) = gpu_ctx {
                ctx.rasterize_cylindrical_target_mesh(&mut cyl, &target_mesh);
            } else {
                cyl.rasterize_target_mesh(&target_mesh);
            }
            #[cfg(not(feature = "gpu-compute"))]
            cyl.rasterize_target_mesh(&target_mesh);
        }
        if let Some(p) = progress {
            p.report(0.08, SimProgressStage::PreparingStock);
        }
        Ok((
            SimEngineKind::Heightmap,
            SimulatedStock::Cylindrical(cyl),
            None,
        ))
    } else if engine == SimEngineKind::TriDexel {
        let mut td = if let Some(carried) = input.initial_stock {
            carried.clone()
        } else {
            TriDexelStock::from_stock_spec(&stock_spec, input.resolution)?
        };
        td.rotary = input.setup.rotary.map(rotary_stock_spec_from_cam);
        if let Some(p) = progress {
            p.report(0.03, SimProgressStage::PreparingStock);
        }
        check_cancelled(input.cancel)?;
        let target_td = {
            #[cfg(feature = "gpu-compute")]
            if let Some(ctx) = gpu_ctx
                && input.doc.has_3d_geometry()
                && input.doc.faces.is_empty()
            {
                extract_target_trimesh_from_cad(input.doc, input.setup, input.target_bodies)
                    .map(|mesh| ctx.rasterize_tridexel_target_mesh(&td, &mesh))
            } else {
                extract_tridexel_target_from_cad(input.doc, input.setup, input.target_bodies, &td)
            }
            #[cfg(not(feature = "gpu-compute"))]
            extract_tridexel_target_from_cad(input.doc, input.setup, input.target_bodies, &td)
        };
        if let Some(p) = progress {
            p.report(0.08, SimProgressStage::PreparingStock);
        }
        Ok((
            SimEngineKind::TriDexel,
            SimulatedStock::TriDexel(Box::new(td)),
            target_td,
        ))
    } else {
        let mut hf = if let Some(carried) = input.initial_stock {
            HeightfieldStock::from_tridexel_projection(carried)
        } else {
            HeightfieldStock::from_stock_spec(&stock_spec, input.resolution)?
        };
        if let Some(p) = progress {
            p.report(0.03, SimProgressStage::PreparingStock);
        }
        check_cancelled(input.cancel)?;
        if input.doc.has_3d_geometry() {
            #[cfg(feature = "gpu-compute")]
            let gpu_rasterized = if let Some(ctx) = gpu_ctx
                && input.doc.faces.is_empty()
                && let Some(target_mesh) =
                    extract_target_trimesh_from_cad(input.doc, input.setup, input.target_bodies)
            {
                ctx.rasterize_heightfield_target_mesh(&mut hf, &target_mesh);
                true
            } else {
                false
            };
            #[cfg(not(feature = "gpu-compute"))]
            let gpu_rasterized = false;
            if !gpu_rasterized {
                let (horiz_up, tris) = extract_heightfield_target_from_cad(
                    input.doc,
                    input.setup,
                    input.target_bodies,
                );
                hf.rasterize_target(&horiz_up, &tris);
            }
        }
        if let Some(p) = progress {
            p.report(0.08, SimProgressStage::PreparingStock);
        }
        Ok((
            SimEngineKind::Heightmap,
            SimulatedStock::Heightfield(hf),
            None,
        ))
    }
}

#[allow(clippy::too_many_arguments)]
fn flush_gpu_sweep_batch<B: SimStockBackend>(
    backend: &mut B,
    span_chords: &mut Vec<SweepChord>,
    batch_chord_spans: &mut Vec<usize>,
    batch_span_count: &mut usize,
    fallback_span_idx: usize,
    timeline: &SimulationTimeline,
    operations: &[SimOperationInput<'_>],
    operation_profiles: &[SweepToolSpec],
    coalescer: &mut ViolationBatchCoalescer,
) {
    if span_chords.is_empty() {
        return;
    }
    let sweep_out = backend.sweep_batch(span_chords, operation_profiles, true);
    if !sweep_out.violations.is_empty() {
        let fallback_idx = batch_chord_spans
            .last()
            .copied()
            .unwrap_or(fallback_span_idx);
        coalescer.push_batch_violations(&sweep_out.violations, |src_chord_idx| {
            let hit_idx = batch_chord_spans
                .get(src_chord_idx as usize)
                .copied()
                .unwrap_or(fallback_idx);
            let s = &timeline.motion_spans[hit_idx];
            let op = &operations[s.op_idx];
            let prof = &operation_profiles[s.op_idx].profile;
            (
                s.global_seg_idx,
                s.op_idx,
                op.op_id,
                op.op_name,
                op.tool.tool_number,
                s.t_start_sec,
                prof,
            )
        });
    }
    span_chords.clear();
    batch_chord_spans.clear();
    *batch_span_count = 0;
}

/// Single unified polymorphic setup simulation loop driving any [`SimStockBackend`] with optional
/// live completion progress reporting from `input.progress` (`0%`–`100%`).
pub fn run_setup_simulation_with_backend<B: SimStockBackend>(
    input: &SetupSimulationInput<'_>,
    engine: SimEngineKind,
    mut backend: B,
) -> Result<SetupSimulationResult, SimError> {
    let progress = input.progress;
    let t_driver_start = Instant::now();
    if let Some(p) = progress {
        p.report(0.05, SimProgressStage::PreparingStock);
    }
    check_cancelled(input.cancel)?;
    let timeline = SimulationTimeline::from_operations(input.operations, input.kinematics);
    let starting_stock_volume_mm3 = backend.compute_volume_mm3();
    let initial_stock_volume_mm3 = input
        .initial_raw_volume_mm3
        .unwrap_or(starting_stock_volume_mm3);

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
    let mut prev_vol = starting_stock_volume_mm3;
    let is_gpu_backend = backend.backend_info().is_gpu;
    const GPU_SWEEP_BATCH_MAX_SPANS: usize = 128;
    const GPU_SWEEP_BATCH_MAX_CHORDS: usize = 256;
    let mut span_chords: Vec<SweepChord> = Vec::with_capacity(if is_gpu_backend {
        GPU_SWEEP_BATCH_MAX_CHORDS + 16
    } else {
        16
    });
    let mut batch_chord_spans: Vec<usize> = Vec::with_capacity(if is_gpu_backend {
        GPU_SWEEP_BATCH_MAX_CHORDS + 16
    } else {
        0
    });
    let mut batch_span_count: usize = 0;

    if let Some(p) = progress {
        p.report(0.10, SimProgressStage::Sweeping);
    }
    let mut last_sweep_permille: u32 = 100;
    let t_sweep = Instant::now();
    for (span_idx, span) in timeline.motion_spans.iter().enumerate() {
        check_cancelled(input.cancel)?;

        let op_in = &input.operations[span.op_idx];
        let tool_spec = &operation_profiles[span.op_idx];
        let mut op_verify = input.verify_config;
        op_verify.gouge_tol_mm += op_in.intentional_edge_break_mm.max(0.0);

        let is_vertical_plunge = !matches!(span.segment.motion, SegmentMotion3D::Rapid)
            && (span.segment.end_wcs.x - span.segment.start_wcs.x)
                .hypot(span.segment.end_wcs.y - span.segment.start_wcs.y)
                <= 1e-4
            && span.segment.end_wcs.z < span.segment.start_wcs.z - 1e-4;

        // Flush any buffered GPU sweep batch before evaluating Rule 5 vertical plunge depth so the
        // stock height query sees exact up-to-the-segment material removal.
        if is_gpu_backend && is_vertical_plunge {
            flush_gpu_sweep_batch(
                &mut backend,
                &mut span_chords,
                &mut batch_chord_spans,
                &mut batch_span_count,
                span_idx,
                &timeline,
                input.operations,
                &operation_profiles,
                &mut coalescer,
            );
        }

        // Rule 5: Vertical Plunge check (shared across 3-axis Heightfield & Tri-Dexel; 0.0 on Cylindrical)
        if is_vertical_plunge
            && let Some(hit) = evaluate_vertical_plunge_violation(
                span,
                op_in.op_id,
                op_in.op_name,
                op_in.tool.tool_number,
                &tool_spec.profile,
                &op_verify,
                |x, y, z_start, z_end, thread_check_r| {
                    backend.query_vertical_plunge_depth(x, y, z_start, z_end, thread_check_r)
                },
            )
        {
            coalescer.push(hit);
        }

        let completed = span_idx + 1;
        let is_op_end = timeline.operations[span.op_idx].end_global_seg == completed;
        let need_checkpoint =
            (completed.is_multiple_of(checkpoint_stride) || is_op_end) && completed < total_spans;

        if is_gpu_backend {
            let chord_start = span_chords.len();
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
            batch_chord_spans.resize(span_chords.len(), span_idx);
            if span_chords.len() > chord_start {
                batch_span_count += 1;
            }

            let must_flush = is_op_end
                || need_checkpoint
                || completed == total_spans
                || batch_span_count >= GPU_SWEEP_BATCH_MAX_SPANS
                || span_chords.len() >= GPU_SWEEP_BATCH_MAX_CHORDS;
            if must_flush {
                flush_gpu_sweep_batch(
                    &mut backend,
                    &mut span_chords,
                    &mut batch_chord_spans,
                    &mut batch_span_count,
                    span_idx,
                    &timeline,
                    input.operations,
                    &operation_profiles,
                    &mut coalescer,
                );
            }
        } else {
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
        }

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
        if need_checkpoint {
            checkpoints.push(backend.capture_checkpoint(completed));
        }
        if let Some(p) = progress {
            let local_permille =
                100 + ((completed as u64).saturating_mul(820) / (total_spans.max(1) as u64)) as u32;
            if local_permille > last_sweep_permille {
                last_sweep_permille = local_permille;
                p.report(
                    f64::from(local_permille) / 1000.0,
                    SimProgressStage::Sweeping,
                );
            }
        }
    }
    let sweep_ms = t_sweep.elapsed().as_secs_f64() * 1000.0;
    if let Some(p) = progress {
        p.report(0.92, SimProgressStage::Sweeping);
    }

    // Rule 6: Full-assembly 3D workholding fixture collision verification (`M61`).
    // Runs in parallel over the shared `timeline.motion_spans` chord stream with 0.0 ms overhead
    // when `input.fixtures.is_empty()`, guaranteeing 100% identical diagnostics across CPU & GPU
    // backends and across Heightfield, Tri-Dexel, and Cylindrical engines!
    if !input.fixtures.is_empty() && !timeline.motion_spans.is_empty() {
        check_cancelled(input.cancel)?;
        if let Some(p) = progress {
            p.report(0.92, SimProgressStage::CheckingFixtures);
        }
        let assembly_profiles: Vec<ToolAssemblyProfile> = input
            .operations
            .iter()
            .map(|op| ToolAssemblyProfile::from_tool(op.tool))
            .collect();
        let span_items: Vec<(usize, usize, chip_cam::ToolpathSegment3D)> = timeline
            .motion_spans
            .iter()
            .enumerate()
            .map(|(idx, s)| (idx, s.op_idx, s.segment))
            .collect();
        let raw_hits = check_segments_fixture_collisions_par(
            &span_items,
            &assembly_profiles,
            input.fixtures,
            false,
        );
        for (span_pos, hit) in raw_hits {
            let span = &timeline.motion_spans[span_pos];
            let op_in = &input.operations[span.op_idx];
            let is_rapid = matches!(span.segment.motion, SegmentMotion3D::Rapid);
            let action_verb = match (is_rapid, hit.contact_part) {
                (true, ToolAssemblyPart::Flute) => "G0 rapid move collided with",
                (true, ToolAssemblyPart::Shank) => "Non-cutting shank rapid collided with",
                (true, ToolAssemblyPart::Holder) => "Holder/collet rapid collided with",
                (false, ToolAssemblyPart::Flute) => "Cutter flutes cut into",
                (false, ToolAssemblyPart::Shank) => "Non-cutting shank collided with",
                (false, ToolAssemblyPart::Holder) => "Holder/collet collided with",
            };
            let detail = format!(
                "{action_verb} '{}' by {:.2} mm at WCS ({:.2}, {:.2}, {:.2}).",
                hit.fixture_name,
                hit.penetration_mm,
                hit.hit_point_wcs.x,
                hit.hit_point_wcs.y,
                hit.hit_point_wcs.z,
            );
            coalescer.push(RuleViolationHit {
                severity: SimIssueKind::FixtureCollision.default_severity(),
                kind: SimIssueKind::FixtureCollision,
                span_idx: span.global_seg_idx,
                op_idx: span.op_idx,
                op_id: op_in.op_id.to_string(),
                op_name: op_in.op_name.to_string(),
                tool_number: op_in.tool.tool_number,
                time_sec: span.t_start_sec,
                location_wcs: hit.hit_point_wcs,
                excess_mm: hit.penetration_mm,
                detail,
                fixture_id: Some(hit.fixture_id),
                fixture_name: Some(hit.fixture_name),
            });
        }
    }

    if let Some(p) = progress {
        p.report(0.96, SimProgressStage::ComputingMetrics);
    }
    let t_metrics = Instant::now();
    let remaining_stock_volume_mm3 = backend.compute_volume_mm3();
    let removed_volume_mm3 = (initial_stock_volume_mm3 - remaining_stock_volume_mm3).max(0.0);
    let error_report = backend.compute_error_report(initial_stock_volume_mm3, 0.05, per_op_errors);
    let metrics_ms = t_metrics.elapsed().as_secs_f64() * 1000.0;
    let active_backend = backend.backend_info();
    let (final_stock, target_tridexel_stock) = backend.into_parts();
    let diagnostics = coalescer.into_diagnostics();
    let total_ms = t_driver_start.elapsed().as_secs_f64() * 1000.0;
    let timing = SimTimingBreakdown {
        target_setup_ms: 0.0,
        sweep_ms,
        metrics_ms,
        total_ms,
        total_segments: total_spans,
        segments_per_sec: compute_segments_per_sec(total_spans, total_ms),
    };
    if let Some(p) = progress {
        p.report(1.0, SimProgressStage::Complete);
    }

    Ok(SetupSimulationResult {
        engine,
        resolution: input.resolution,
        active_backend,
        timing,
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
        carried_over_from_setup: input.carried_over_from_setup.map(ToString::to_string),
    })
}

#[inline]
fn check_cancelled(cancel: Option<&AtomicBool>) -> Result<(), SimError> {
    if cancel.is_some_and(|flag| flag.load(Ordering::Relaxed)) {
        return Err(SimError::Cancelled);
    }
    Ok(())
}
