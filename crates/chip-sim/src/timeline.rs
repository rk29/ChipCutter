use chip_cam::{
    ArcDir, ArcPlane, ChamferSourceMode, ChamferTraceMode, ClEvent, GeneratedToolpath, Operation,
    OperationKind, SegmentMotion3D, StrategyParams, ToolpathSegment3D,
};
use chip_math::DVec3;
use chip_tools::{Tool, ToolId};
use serde::{Deserialize, Serialize};

fn default_max_rotary_rate_deg_min() -> f64 {
    3_600.0
}

/// Machine velocity, acceleration, cornering, and ATC parameters for cycle-time estimation (`06-simulation.md`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct MachineKinematicsConfig {
    /// Machine `G0` rapid traverse feedrate (`mm/min`). Default: `5000.0`.
    pub rapid_rate_mm_min: f64,
    /// Axis acceleration limit (`mm/s²`). Default: `500.0`.
    pub accel_mm_s2: f64,
    /// Cornering junction deviation (`mm`, GRBL `$11` model). Default: `0.05`.
    pub junction_deviation_mm: f64,
    /// Tool change duration (`s`) added whenever `tool_number` changes after the first tool. Default: `15.0`.
    pub tool_change_time_sec: f64,
    /// Maximum 4th-axis rotary angular slew rate (`deg/min`). Default: `3600.0`.
    #[serde(default = "default_max_rotary_rate_deg_min")]
    pub max_rotary_rate_deg_min: f64,
}

impl Default for MachineKinematicsConfig {
    fn default() -> Self {
        Self {
            rapid_rate_mm_min: 5_000.0,
            accel_mm_s2: 500.0,
            junction_deviation_mm: 0.05,
            tool_change_time_sec: 15.0,
            max_rotary_rate_deg_min: default_max_rotary_rate_deg_min(),
        }
    }
}

/// Borrowed operation descriptor passed to [`SimulationTimeline::from_operations`] and `run_setup_simulation`.
#[derive(Debug, Clone, Copy)]
pub struct SimOperationInput<'a> {
    pub op_id: &'a str,
    pub op_name: &'a str,
    pub tool: &'a Tool,
    pub fallback_spindle_rpm: f64,
    /// Intentional axial edge-break or engraving depth (`mm`) on unmodeled sharp CAD edges (`0.0` for all B-Rep surfaces).
    pub intentional_edge_break_mm: f64,
    pub toolpath: &'a GeneratedToolpath,
}

impl<'a> SimOperationInput<'a> {
    /// Builds a [`SimOperationInput`] from a CAM [`Operation`], [`Tool`], and [`GeneratedToolpath`].
    pub fn from_operation(
        op: &'a Operation,
        tool: &'a Tool,
        toolpath: &'a GeneratedToolpath,
    ) -> Self {
        let cutting = op.effective_cutting_params(tool);
        let all_sketch_selections = !op.selections.is_empty()
            && op.selections.iter().all(|s| {
                matches!(
                    s,
                    chip_cam::GeometrySelectionItem::SketchRegion { .. }
                        | chip_cam::GeometrySelectionItem::SketchEntity { .. }
                        | chip_cam::GeometrySelectionItem::SketchLayerAll { .. }
                )
            });

        let base_edge_break_mm = match op.kind {
            OperationKind::ChamferTrace => match op.resolved_strategy() {
                StrategyParams::ChamferTrace(p) => match p.mode {
                    ChamferTraceMode::EdgeChamfer => match p.chamfer_source {
                        ChamferSourceMode::SharpEdge { width } => {
                            let half_ang = 0.5 * tool.geometry.tip_angle_rad();
                            if (0.05..1.55).contains(&half_ang) {
                                width.value_mm.max(0.0) / half_ang.tan()
                            } else {
                                width.value_mm.max(0.0)
                            }
                        }
                        ChamferSourceMode::ModeledFace => 0.0,
                    },
                    ChamferTraceMode::CurveTrace => p.trace_axial_offset.value_mm.max(0.0),
                },
                _ => 0.0,
            },
            OperationKind::VCarve if toolpath.stats.bounds_wcs.is_valid() => {
                (toolpath.stats.bounds_wcs.max.z - toolpath.stats.bounds_wcs.min.z).max(0.0) + 0.5
            }
            _ => 0.0,
        };

        let sketch_depth_allowance_mm =
            if all_sketch_selections && toolpath.stats.bounds_wcs.is_valid() {
                (toolpath.stats.bounds_wcs.max.z - toolpath.stats.bounds_wcs.min.z).max(0.0) + 0.5
            } else {
                0.0
            };
        let intentional_edge_break_mm = base_edge_break_mm.max(sketch_depth_allowance_mm);

        Self {
            op_id: &op.id,
            op_name: &op.name,
            tool,
            fallback_spindle_rpm: cutting.spindle_rpm,
            intentional_edge_break_mm,
            toolpath,
        }
    }
}

/// Per-operation summary inside a [`SimulationTimeline`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OperationTimelineSummary {
    pub op_idx: usize,
    pub op_id: String,
    pub op_name: String,
    pub tool_id: ToolId,
    pub tool_number: u32,
    pub tool_name: String,
    pub spindle_rpm: f64,
    pub first_global_seg: usize,
    pub end_global_seg: usize,
    pub start_time_sec: f64,
    pub end_time_sec: f64,
    pub cutting_time_sec: f64,
    pub rapid_time_sec: f64,
    pub dwell_time_sec: f64,
    pub tool_change_time_sec: f64,
    pub cutting_distance_mm: f64,
    pub rapid_distance_mm: f64,
}

/// Exact motion span indexed by `global_seg_idx` (`0 .. motion_spans.len()`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MotionSpan {
    pub global_seg_idx: usize,
    pub op_idx: usize,
    pub local_seg_idx: usize,
    pub segment: ToolpathSegment3D,
    pub t_start_sec: f64,
    pub t_end_sec: f64,
    pub dist_start_mm: f64,
    pub dist_end_mm: f64,
    pub commanded_feed_mm_min: f64,
    pub spindle_rpm: f64,
}

/// Live state interpolated at any scrub timestamp or path distance (`06-simulation.md` §1).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ScrubSnapshot {
    pub time_sec: f64,
    pub total_time_sec: f64,
    pub distance_mm: f64,
    pub total_distance_mm: f64,
    pub op_idx: usize,
    pub op_id: String,
    pub op_name: String,
    pub tool_id: ToolId,
    pub tool_number: u32,
    pub tool_name: String,
    pub global_seg_idx: usize,
    pub seg_progress_t: f64,
    pub completed_segments: usize,
    pub tip_pos_wcs: DVec3,
    #[serde(default)]
    pub rotary_deg: f64,
    pub motion_code: &'static str,
    pub commanded_feed_mm_min: f64,
    pub spindle_rpm: f64,
}

/// Unified Setup-wide timeline with trapezoidal corner-limited cycle time and `O(log N)` scrub queries.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SimulationTimeline {
    pub kinematics: MachineKinematicsConfig,
    pub operations: Vec<OperationTimelineSummary>,
    pub motion_spans: Vec<MotionSpan>,
    pub tool_change_times_sec: Vec<f64>,
    pub total_time_sec: f64,
    pub total_cutting_time_sec: f64,
    pub total_rapid_time_sec: f64,
    pub total_dwell_time_sec: f64,
    pub total_tool_change_time_sec: f64,
    pub total_distance_mm: f64,
}

impl SimulationTimeline {
    /// Builds a complete [`SimulationTimeline`] for an ordered list of Setup operations.
    pub fn from_operations(
        ops: &[SimOperationInput<'_>],
        kinematics: MachineKinematicsConfig,
    ) -> Self {
        let rapid_mm_s = (kinematics.rapid_rate_mm_min.max(60.0)) / 60.0;
        let accel = kinematics.accel_mm_s2.max(10.0);
        let junc_dev = kinematics.junction_deviation_mm.max(1e-4);

        let mut timeline = Self {
            kinematics,
            ..Self::default()
        };

        let mut cur_time_sec = 0.0_f64;
        let mut cur_dist_mm = 0.0_f64;
        let mut active_tool_num: Option<u32> = None;

        for (op_idx, op) in ops.iter().enumerate() {
            let op_start_time = cur_time_sec;
            let first_global_seg = timeline.motion_spans.len();

            let mut tc_sec = 0.0;
            if let Some(prev_t) = active_tool_num
                && prev_t != op.tool.tool_number
                && kinematics.tool_change_time_sec > 0.0
            {
                timeline.tool_change_times_sec.push(cur_time_sec);
                tc_sec = kinematics.tool_change_time_sec;
                cur_time_sec += tc_sec;
                timeline.total_tool_change_time_sec += tc_sec;
            }
            active_tool_num = Some(op.tool.tool_number);

            // Extract spindle RPM and total G4 dwell seconds from typed CL events
            let mut spindle_rpm = op.fallback_spindle_rpm;
            let mut op_dwell_sec = 0.0;
            for ev in &op.toolpath.events {
                match *ev {
                    ClEvent::SpindleStart { rpm, .. } if rpm > 0.0 => {
                        spindle_rpm = rpm;
                    }
                    ClEvent::Dwell { seconds } if seconds.is_finite() && seconds > 0.0 => {
                        op_dwell_sec += seconds;
                    }
                    _ => {}
                }
            }

            let segs = &op.toolpath.segments_wcs;
            let n = segs.len();
            let mut op_cut_time = 0.0;
            let mut op_rapid_time = 0.0;
            let mut op_cut_dist = 0.0;
            let mut op_rapid_dist = 0.0;

            if n > 0 {
                // Precompute target speeds (mm/s), lengths (mm), and entry/exit unit tangents
                let mut lengths = Vec::with_capacity(n);
                let mut target_v = Vec::with_capacity(n);
                let mut t_start_dirs = Vec::with_capacity(n);
                let mut t_end_dirs = Vec::with_capacity(n);

                for seg in segs {
                    let len = seg.length_mm();
                    lengths.push(len);
                    let v_mm_s = match seg.motion {
                        SegmentMotion3D::Rapid => rapid_mm_s,
                        SegmentMotion3D::Linear { feed_mm_min, .. }
                        | SegmentMotion3D::Arc { feed_mm_min, .. } => (feed_mm_min.max(1.0)) / 60.0,
                    };
                    target_v.push(v_mm_s);
                    let (d0, d1) = segment_unit_tangents(seg);
                    t_start_dirs.push(d0);
                    t_end_dirs.push(d1);
                }

                // Compute corner junction velocities `v_j[0..=n]` with forward/backward acceleration lookahead
                let mut v_j = vec![0.0_f64; n + 1];
                for i in 1..n {
                    let is_prev_rapid = matches!(segs[i - 1].motion, SegmentMotion3D::Rapid);
                    let is_curr_rapid = matches!(segs[i].motion, SegmentMotion3D::Rapid);
                    if is_prev_rapid != is_curr_rapid {
                        v_j[i] = 0.0;
                    } else {
                        let vmax = target_v[i - 1].min(target_v[i]);
                        let vj_corner = corner_junction_speed(
                            t_end_dirs[i - 1],
                            t_start_dirs[i],
                            accel,
                            junc_dev,
                        );
                        v_j[i] = vj_corner.min(vmax);
                    }
                }
                // Forward pass: v_j[i+1]^2 <= v_j[i]^2 + 2 * a * L[i]
                for i in 0..n {
                    let reachable = (v_j[i] * v_j[i] + 2.0 * accel * lengths[i]).sqrt();
                    v_j[i + 1] = v_j[i + 1].min(reachable);
                }
                // Backward pass: v_j[i]^2 <= v_j[i+1]^2 + 2 * a * L[i]
                for i in (0..n).rev() {
                    let reachable = (v_j[i + 1] * v_j[i + 1] + 2.0 * accel * lengths[i]).sqrt();
                    v_j[i] = v_j[i].min(reachable);
                }

                // Distribute any operation dwell evenly at the end of the operation
                for (local_idx, seg) in segs.iter().enumerate() {
                    let len = lengths[local_idx];
                    let dt_linear = trapezoidal_segment_duration(
                        len,
                        v_j[local_idx],
                        v_j[local_idx + 1],
                        target_v[local_idx],
                        accel,
                    );
                    let d_deg = (seg.end_rotary_deg - seg.start_rotary_deg).abs();
                    let dt_rotary = if d_deg > 1e-6 {
                        let rot_rate_deg_s = (kinematics.max_rotary_rate_deg_min.max(60.0)) / 60.0;
                        d_deg / rot_rate_deg_s
                    } else {
                        0.0
                    };
                    let dt = dt_linear.max(dt_rotary);
                    let commanded_feed_mm_min = match seg.motion {
                        SegmentMotion3D::Rapid => kinematics.rapid_rate_mm_min,
                        SegmentMotion3D::Linear { feed_mm_min, .. }
                        | SegmentMotion3D::Arc { feed_mm_min, .. } => feed_mm_min,
                    };

                    if matches!(seg.motion, SegmentMotion3D::Rapid) {
                        op_rapid_time += dt;
                        op_rapid_dist += len;
                    } else {
                        op_cut_time += dt;
                        op_cut_dist += len;
                    }

                    let global_seg_idx = timeline.motion_spans.len();
                    let t0 = cur_time_sec;
                    let t1 = t0 + dt;
                    let d0 = cur_dist_mm;
                    let d1 = d0 + len;

                    timeline.motion_spans.push(MotionSpan {
                        global_seg_idx,
                        op_idx,
                        local_seg_idx: local_idx,
                        segment: *seg,
                        t_start_sec: t0,
                        t_end_sec: t1,
                        dist_start_mm: d0,
                        dist_end_mm: d1,
                        commanded_feed_mm_min,
                        spindle_rpm,
                    });

                    cur_time_sec = t1;
                    cur_dist_mm = d1;
                }
            }

            if op_dwell_sec > 0.0 {
                cur_time_sec += op_dwell_sec;
                timeline.total_dwell_time_sec += op_dwell_sec;
            }

            timeline.total_cutting_time_sec += op_cut_time;
            timeline.total_rapid_time_sec += op_rapid_time;

            let end_global_seg = timeline.motion_spans.len();
            timeline.operations.push(OperationTimelineSummary {
                op_idx,
                op_id: op.op_id.to_string(),
                op_name: op.op_name.to_string(),
                tool_id: op.tool.id.clone(),
                tool_number: op.tool.tool_number,
                tool_name: op.tool.name.clone(),
                spindle_rpm,
                first_global_seg,
                end_global_seg,
                start_time_sec: op_start_time,
                end_time_sec: cur_time_sec,
                cutting_time_sec: op_cut_time,
                rapid_time_sec: op_rapid_time,
                dwell_time_sec: op_dwell_sec,
                tool_change_time_sec: tc_sec,
                cutting_distance_mm: op_cut_dist,
                rapid_distance_mm: op_rapid_dist,
            });
        }

        timeline.total_time_sec = cur_time_sec;
        timeline.total_distance_mm = cur_dist_mm;
        timeline
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.motion_spans.is_empty()
    }

    /// Interpolates exact tool position, active segment, and HUD metadata at `t_sec` in `O(log N)`.
    pub fn sample_at_time(&self, t_sec: f64) -> Option<ScrubSnapshot> {
        let first = self.motion_spans.first()?;
        let t_clamped = t_sec.clamp(0.0, self.total_time_sec);

        if t_clamped <= first.t_start_sec {
            return Some(self.snapshot_for_span(first, 0.0, t_clamped, false));
        }

        let idx = self
            .motion_spans
            .partition_point(|s| s.t_end_sec < t_clamped);
        if idx >= self.motion_spans.len() {
            let last = self.motion_spans.last()?;
            return Some(self.snapshot_for_span(last, 1.0, t_clamped, true));
        }

        let span = &self.motion_spans[idx];
        if t_clamped < span.t_start_sec {
            // Inside a tool-change or dwell pause between `idx - 1` and `idx`
            if idx > 0 {
                let prev = &self.motion_spans[idx - 1];
                let mut snap = self.snapshot_for_span(prev, 1.0, t_clamped, true);
                snap.motion_code = "M6/G4";
                snap.commanded_feed_mm_min = 0.0;
                return Some(snap);
            }
            return Some(self.snapshot_for_span(span, 0.0, t_clamped, false));
        }

        let dur = (span.t_end_sec - span.t_start_sec).max(1e-12);
        let u = ((t_clamped - span.t_start_sec) / dur).clamp(0.0, 1.0);
        Some(self.snapshot_for_span(span, u, t_clamped, u >= 1.0 - 1e-9))
    }

    /// Interpolates exact tool position, active segment, and HUD metadata at path distance `dist_mm` in `O(log N)`.
    pub fn sample_at_distance(&self, dist_mm: f64) -> Option<ScrubSnapshot> {
        let first = self.motion_spans.first()?;
        let d_clamped = dist_mm.clamp(0.0, self.total_distance_mm);

        if d_clamped <= 0.0 {
            return Some(self.snapshot_for_span(first, 0.0, first.t_start_sec, false));
        }

        let idx = self
            .motion_spans
            .partition_point(|s| s.dist_end_mm < d_clamped);
        if idx >= self.motion_spans.len() {
            let last = self.motion_spans.last()?;
            return Some(self.snapshot_for_span(last, 1.0, self.total_time_sec, true));
        }

        let span = &self.motion_spans[idx];
        let seg_len = (span.dist_end_mm - span.dist_start_mm).max(1e-12);
        let u = ((d_clamped - span.dist_start_mm) / seg_len).clamp(0.0, 1.0);
        let t_interp = span.t_start_sec + (span.t_end_sec - span.t_start_sec) * u;
        Some(self.snapshot_for_span(span, u, t_interp, u >= 1.0 - 1e-9))
    }

    /// Returns the timestamp at the end of segment `cur_seg + delta` (`←` / `→` single-segment step).
    pub fn step_segment_time(&self, cur_seg: usize, delta: isize) -> f64 {
        if self.motion_spans.is_empty() {
            return 0.0;
        }
        let max_idx = (self.motion_spans.len() - 1) as isize;
        let next = (cur_seg as isize + delta).clamp(0, max_idx) as usize;
        if delta < 0 && cur_seg == 0 {
            0.0
        } else {
            self.motion_spans[next].t_end_sec
        }
    }

    /// Jumps to the start timestamp of the next operation after `t_sec` (or `total_time_sec` at the end).
    pub fn next_operation_time(&self, t_sec: f64) -> f64 {
        for op in &self.operations {
            if op.start_time_sec > t_sec + 1e-4 {
                return op.start_time_sec;
            }
        }
        self.total_time_sec
    }

    /// Jumps to the start timestamp of the previous operation before `t_sec`.
    pub fn prev_operation_time(&self, t_sec: f64) -> f64 {
        for op in self.operations.iter().rev() {
            if op.start_time_sec < t_sec - 1e-4 {
                return op.start_time_sec;
            }
        }
        0.0
    }

    /// Jumps to the next tool change timestamp after `t_sec` (or `total_time_sec` if none).
    pub fn next_tool_change_time(&self, t_sec: f64) -> f64 {
        for &tc in &self.tool_change_times_sec {
            if tc > t_sec + 1e-4 {
                return tc;
            }
        }
        self.total_time_sec
    }

    fn snapshot_for_span(
        &self,
        span: &MotionSpan,
        u: f64,
        time_sec: f64,
        span_complete: bool,
    ) -> ScrubSnapshot {
        let op_meta = &self.operations[span.op_idx];
        let tip_pos_wcs = span.segment.sample_point(u);
        let rotary_deg = span.segment.sample_rotary_deg(u);
        let distance_mm =
            span.dist_start_mm + (span.dist_end_mm - span.dist_start_mm) * u.clamp(0.0, 1.0);
        let motion_code = match span.segment.motion {
            SegmentMotion3D::Rapid => "G0",
            SegmentMotion3D::Linear { .. } => "G1",
            SegmentMotion3D::Arc {
                dir: ArcDir::CW, ..
            } => "G2",
            SegmentMotion3D::Arc {
                dir: ArcDir::CCW, ..
            } => "G3",
        };
        let completed_segments = if span_complete {
            span.global_seg_idx + 1
        } else {
            span.global_seg_idx
        };

        ScrubSnapshot {
            time_sec,
            total_time_sec: self.total_time_sec,
            distance_mm,
            total_distance_mm: self.total_distance_mm,
            op_idx: span.op_idx,
            op_id: op_meta.op_id.clone(),
            op_name: op_meta.op_name.clone(),
            tool_id: op_meta.tool_id.clone(),
            tool_number: op_meta.tool_number,
            tool_name: op_meta.tool_name.clone(),
            global_seg_idx: span.global_seg_idx,
            seg_progress_t: u.clamp(0.0, 1.0),
            completed_segments,
            tip_pos_wcs,
            rotary_deg,
            motion_code,
            commanded_feed_mm_min: span.commanded_feed_mm_min,
            spindle_rpm: span.spindle_rpm,
        }
    }
}

/// Formats seconds as `HH:MM:SS` (or `MM:SS.d` when requested).
pub fn format_cycle_time_hms(seconds: f64) -> String {
    let total = seconds.max(0.0).round() as u64;
    let hrs = total / 3600;
    let mins = (total % 3600) / 60;
    let secs = total % 60;
    format!("{hrs:02}:{mins:02}:{secs:02}")
}

fn segment_unit_tangents(seg: &ToolpathSegment3D) -> (DVec3, DVec3) {
    match seg.motion {
        SegmentMotion3D::Rapid | SegmentMotion3D::Linear { .. } => {
            let d = (seg.end_wcs - seg.start_wcs).normalize_or_zero();
            (d, d)
        }
        SegmentMotion3D::Arc {
            plane,
            center_wcs,
            radius_mm,
            signed_sweep_rad,
            ..
        } => {
            if radius_mm <= 1e-9 || signed_sweep_rad.abs() <= 1e-9 {
                let d = (seg.end_wcs - seg.start_wcs).normalize_or_zero();
                return (d, d);
            }
            let sign = signed_sweep_rad.signum();
            match plane {
                ArcPlane::XY => {
                    let a0 = (seg.start_wcs.y - center_wcs.y).atan2(seg.start_wcs.x - center_wcs.x);
                    let a1 = a0 + signed_sweep_rad;
                    let dz_dtheta = (seg.end_wcs.z - seg.start_wcs.z) / signed_sweep_rad.abs();
                    let t0 = DVec3::new(
                        -sign * radius_mm * a0.sin(),
                        sign * radius_mm * a0.cos(),
                        dz_dtheta,
                    )
                    .normalize_or_zero();
                    let t1 = DVec3::new(
                        -sign * radius_mm * a1.sin(),
                        sign * radius_mm * a1.cos(),
                        dz_dtheta,
                    )
                    .normalize_or_zero();
                    (t0, t1)
                }
                ArcPlane::XZ => {
                    let a0 = (seg.start_wcs.z - center_wcs.z).atan2(seg.start_wcs.x - center_wcs.x);
                    let a1 = a0 + signed_sweep_rad;
                    let dy_dtheta = (seg.end_wcs.y - seg.start_wcs.y) / signed_sweep_rad.abs();
                    let t0 = DVec3::new(
                        -sign * radius_mm * a0.sin(),
                        dy_dtheta,
                        sign * radius_mm * a0.cos(),
                    )
                    .normalize_or_zero();
                    let t1 = DVec3::new(
                        -sign * radius_mm * a1.sin(),
                        dy_dtheta,
                        sign * radius_mm * a1.cos(),
                    )
                    .normalize_or_zero();
                    (t0, t1)
                }
                ArcPlane::YZ => {
                    let a0 = (seg.start_wcs.z - center_wcs.z).atan2(seg.start_wcs.y - center_wcs.y);
                    let a1 = a0 + signed_sweep_rad;
                    let dx_dtheta = (seg.end_wcs.x - seg.start_wcs.x) / signed_sweep_rad.abs();
                    let t0 = DVec3::new(
                        dx_dtheta,
                        -sign * radius_mm * a0.sin(),
                        sign * radius_mm * a0.cos(),
                    )
                    .normalize_or_zero();
                    let t1 = DVec3::new(
                        dx_dtheta,
                        -sign * radius_mm * a1.sin(),
                        sign * radius_mm * a1.cos(),
                    )
                    .normalize_or_zero();
                    (t0, t1)
                }
            }
        }
    }
}

fn corner_junction_speed(t_in: DVec3, t_out: DVec3, accel: f64, junc_dev: f64) -> f64 {
    if t_in.length_squared() < 0.5 || t_out.length_squared() < 0.5 {
        return 0.0;
    }
    let cos_theta = t_in.dot(t_out).clamp(-1.0, 1.0);
    if cos_theta > 0.999_9 {
        return f64::INFINITY;
    }
    if cos_theta < -0.999_9 {
        return 0.0;
    }
    // Note: deflection angle between forward directions `t_in` and `t_out` has `cos(alpha) = cos_theta`.
    // In GRBL's junction deviation formula, `cos_junc = -t_in · t_out = -cos_theta`,
    // `sin(theta_junc / 2) = sqrt(0.5 * (1 - cos_junc)) = sqrt(0.5 * (1 + cos_theta))`?
    // Wait: when `t_in == t_out` (`cos_theta = 1`, straight line), junction speed should be INFINITY;
    // when `t_in == -t_out` (`cos_theta = -1`, 180° reversal), junction speed should be 0!
    // Let `sin_half_interior = sqrt(0.5 * (1.0 + cos_theta))` -> at `cos_theta = 1`, `sin_half = 1` -> `denom = 0 -> INF`;
    // at `cos_theta = -1`, `sin_half = 0` -> `v = 0`; at 90° (`cos_theta = 0`), `sin_half = sqrt(0.5) = 0.7071` -> `sin/(1-sin) = 2.414`!
    let sin_half = (0.5 * (1.0 + cos_theta)).clamp(0.0, 1.0).sqrt();
    let denom = (1.0 - sin_half).max(1e-9);
    (accel * junc_dev * sin_half / denom).sqrt()
}

fn trapezoidal_segment_duration(
    len_mm: f64,
    v_start: f64,
    v_end: f64,
    v_cruise: f64,
    accel: f64,
) -> f64 {
    if len_mm <= 1e-9 {
        return 0.0;
    }
    let vc = v_cruise.max(1e-3);
    let v0 = v_start.clamp(0.0, vc);
    let v1 = v_end.clamp(0.0, vc);

    let d_accel = (vc * vc - v0 * v0).max(0.0) / (2.0 * accel);
    let d_decel = (vc * vc - v1 * v1).max(0.0) / (2.0 * accel);

    if d_accel + d_decel <= len_mm {
        let t_acc = (vc - v0) / accel;
        let t_dec = (vc - v1) / accel;
        let t_cruise = (len_mm - d_accel - d_decel) / vc;
        t_acc + t_cruise + t_dec
    } else {
        // Triangular profile: peak speed `v_peak^2 = a * len + 0.5 * (v0^2 + v1^2)`
        let v_peak = (accel * len_mm + 0.5 * (v0 * v0 + v1 * v1))
            .max(v0.max(v1).powi(2))
            .sqrt();
        let t_acc = (v_peak - v0).max(0.0) / accel;
        let t_dec = (v_peak - v1).max(0.0) / accel;
        t_acc + t_dec
    }
}
