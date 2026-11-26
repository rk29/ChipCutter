use chip_math::{Aabb3, ArcPolyline2D, ArcSegment2D, DVec2, DVec3};
use serde::{Deserialize, Serialize};

use crate::cl_data::{ArcDir, ArcPlane, ClEvent, DrillCycleKind, MotionRole};

/// Default assumed rapid traverse rate (`mm/min`) for cycle-time estimation prior to machine-specific M4 simulation.
pub const DEFAULT_RAPID_RATE_MM_MIN: f64 = 4_000.0;

/// Exact 3D motion kinematics for a single span inside a [`GeneratedToolpath`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum SegmentMotion3D {
    Rapid,
    Linear {
        feed_mm_min: f64,
        role: MotionRole,
    },
    Arc {
        plane: ArcPlane,
        dir: ArcDir,
        center_wcs: DVec3,
        radius_mm: f64,
        /// Total signed sweep angle in radians (`> 0` CCW, `< 0` CW in the plane's local `(u, v)` basis),
        /// including any multi-turn helical revolutions.
        signed_sweep_rad: f64,
        feed_mm_min: f64,
        role: MotionRole,
    },
}

/// Connected 3D geometric motion span in canonical `f64` Setup WCS millimeters (with optional
/// synchronized 4th-axis rotary angles in degrees).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ToolpathSegment3D {
    pub start_wcs: DVec3,
    pub end_wcs: DVec3,
    pub motion: SegmentMotion3D,
    #[serde(default)]
    pub start_rotary_deg: f64,
    #[serde(default)]
    pub end_rotary_deg: f64,
}

impl ToolpathSegment3D {
    /// Evaluates the interpolated 4th-axis rotary angle (`deg`) at normalized parameter `t ∈ [0, 1]`.
    #[inline]
    pub fn sample_rotary_deg(&self, t: f64) -> f64 {
        let u = t.clamp(0.0, 1.0);
        self.start_rotary_deg + (self.end_rotary_deg - self.start_rotary_deg) * u
    }

    /// Evaluates exact 3D WCS tool tip position at normalized parameter `t ∈ [0, 1]`.
    pub fn sample_point(&self, t: f64) -> DVec3 {
        let u = t.clamp(0.0, 1.0);
        match self.motion {
            SegmentMotion3D::Rapid | SegmentMotion3D::Linear { .. } => {
                self.start_wcs.lerp(self.end_wcs, u)
            }
            SegmentMotion3D::Arc {
                plane,
                center_wcs,
                radius_mm,
                signed_sweep_rad,
                ..
            } => {
                if radius_mm <= 1e-12 {
                    return self.start_wcs.lerp(self.end_wcs, u);
                }
                match plane {
                    ArcPlane::XY => {
                        let a0 = (self.start_wcs.y - center_wcs.y)
                            .atan2(self.start_wcs.x - center_wcs.x);
                        let theta = a0 + signed_sweep_rad * u;
                        let z = self.start_wcs.z + (self.end_wcs.z - self.start_wcs.z) * u;
                        DVec3::new(
                            center_wcs.x + radius_mm * theta.cos(),
                            center_wcs.y + radius_mm * theta.sin(),
                            z,
                        )
                    }
                    ArcPlane::XZ => {
                        let a0 = (self.start_wcs.z - center_wcs.z)
                            .atan2(self.start_wcs.x - center_wcs.x);
                        let theta = a0 + signed_sweep_rad * u;
                        let y = self.start_wcs.y + (self.end_wcs.y - self.start_wcs.y) * u;
                        DVec3::new(
                            center_wcs.x + radius_mm * theta.cos(),
                            y,
                            center_wcs.z + radius_mm * theta.sin(),
                        )
                    }
                    ArcPlane::YZ => {
                        let a0 = (self.start_wcs.z - center_wcs.z)
                            .atan2(self.start_wcs.y - center_wcs.y);
                        let theta = a0 + signed_sweep_rad * u;
                        let x = self.start_wcs.x + (self.end_wcs.x - self.start_wcs.x) * u;
                        DVec3::new(
                            x,
                            center_wcs.y + radius_mm * theta.cos(),
                            center_wcs.z + radius_mm * theta.sin(),
                        )
                    }
                }
            }
        }
    }

    /// Exact 3D path length in millimeters (including helical Z rise/drop).
    pub fn length_mm(&self) -> f64 {
        match self.motion {
            SegmentMotion3D::Rapid | SegmentMotion3D::Linear { .. } => {
                self.start_wcs.distance(self.end_wcs)
            }
            SegmentMotion3D::Arc {
                plane,
                radius_mm,
                signed_sweep_rad,
                ..
            } => {
                let planar_len = radius_mm * signed_sweep_rad.abs();
                let axial_delta = match plane {
                    ArcPlane::XY => self.end_wcs.z - self.start_wcs.z,
                    ArcPlane::XZ => self.end_wcs.y - self.start_wcs.y,
                    ArcPlane::YZ => self.end_wcs.x - self.start_wcs.x,
                };
                planar_len.hypot(axial_delta)
            }
        }
    }
}

/// Visual 3D marker for a contour holding tab bridge (`08-ui-viewport.md`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TabMarker3D {
    pub center_wcs: DVec3,
    pub tangent_xy: DVec2,
    pub width_mm: f64,
    pub height_mm: f64,
    pub is_triangular: bool,
    pub contour_index: usize,
    pub s_norm: f64,
}

/// Summary statistics for a [`GeneratedToolpath`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ToolpathStats {
    pub cutting_distance_mm: f64,
    pub rapid_distance_mm: f64,
    pub est_cutting_time_sec: f64,
    pub est_rapid_time_sec: f64,
    pub total_est_time_sec: f64,
    pub pass_count: usize,
    pub linear_move_count: usize,
    pub arc_move_count: usize,
    pub drill_cycle_count: usize,
    pub bounds_wcs: Aabb3,
}

impl Default for ToolpathStats {
    fn default() -> Self {
        Self {
            cutting_distance_mm: 0.0,
            rapid_distance_mm: 0.0,
            est_cutting_time_sec: 0.0,
            est_rapid_time_sec: 0.0,
            total_est_time_sec: 0.0,
            pass_count: 0,
            linear_move_count: 0,
            arc_move_count: 0,
            drill_cycle_count: 0,
            bounds_wcs: Aabb3::EMPTY,
        }
    }
}

/// Optional Stage 1 Flat-Endmill area clearance sub-toolpath attached to a two-stage
/// [`crate::params::VCarveParams`] operation (`05-operations.md` §6B).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ClearingSubToolpath {
    pub tool_id: chip_tools::ToolId,
    pub toolpath: GeneratedToolpath,
}

fn is_zero_usize(v: &usize) -> bool {
    *v == 0
}

fn is_zero_f64(v: &f64) -> bool {
    v.abs() <= 1e-12
}

/// Cached output of generating a single 2.5D CAM [`crate::operation::Operation`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GeneratedToolpath {
    pub op_id: String,
    pub events: Vec<ClEvent>,
    pub segments_wcs: Vec<ToolpathSegment3D>,
    pub stats: ToolpathStats,
    pub tab_markers_wcs: Vec<TabMarker3D>,
    pub warnings: Vec<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub clearing_stage: Option<Box<ClearingSubToolpath>>,
    /// Number of non-cutting rapid/link moves automatically lifted over 3D workholding fixtures.
    #[serde(default, skip_serializing_if = "is_zero_usize")]
    pub rapid_lifts_inserted: usize,
    /// Peak Setup WCS Z height (`mm`) reached by automatically lifted safe-rapid bridges (`0.0` if none).
    #[serde(default, skip_serializing_if = "is_zero_f64")]
    pub max_lifted_z_wcs: f64,
}

impl GeneratedToolpath {
    /// Recomputes [`ToolpathStats`] and updates `OperationStart.est_time_sec` from `self.events`
    /// and `self.segments_wcs` after post-processing passes such as fixture safe-rapid routing.
    pub fn recompute_stats(&mut self) {
        let mut cutting_distance_mm = 0.0;
        let mut rapid_distance_mm = 0.0;
        let mut est_cutting_time_sec = 0.0;
        let mut est_rapid_time_sec = 0.0;
        let mut bounds_wcs = Aabb3::EMPTY;

        for ev in &self.events {
            if let ClEvent::Dwell { seconds } = ev
                && seconds.is_finite()
                && *seconds > 0.0
            {
                est_cutting_time_sec += *seconds;
            }
        }

        for seg in &self.segments_wcs {
            bounds_wcs.extend_point(seg.start_wcs);
            bounds_wcs.extend_point(seg.end_wcs);
            if matches!(seg.motion, SegmentMotion3D::Arc { .. }) {
                for i in 1..=8 {
                    bounds_wcs.extend_point(seg.sample_point(f64::from(i) / 8.0));
                }
            }
            let len = seg.length_mm();
            match seg.motion {
                SegmentMotion3D::Rapid => {
                    rapid_distance_mm += len;
                    est_rapid_time_sec += (len / DEFAULT_RAPID_RATE_MM_MIN) * 60.0;
                }
                SegmentMotion3D::Linear { feed_mm_min, .. }
                | SegmentMotion3D::Arc { feed_mm_min, .. } => {
                    cutting_distance_mm += len;
                    if feed_mm_min > 1e-6 {
                        est_cutting_time_sec += (len / feed_mm_min) * 60.0;
                    }
                }
            }
        }

        let mut linear_move_count = 0usize;
        let mut arc_move_count = 0usize;
        let mut drill_cycle_count = 0usize;
        for ev in &self.events {
            match ev {
                ClEvent::LinearMove { .. } | ClEvent::RotaryLinearMove { .. } => {
                    linear_move_count += 1;
                }
                ClEvent::ArcMove { .. } => arc_move_count += 1,
                ClEvent::DrillCycle { .. } => drill_cycle_count += 1,
                _ => {}
            }
        }

        let total_est_time_sec = est_cutting_time_sec + est_rapid_time_sec;
        for ev in &mut self.events {
            if let ClEvent::OperationStart { est_time_sec, .. } = ev {
                *est_time_sec = total_est_time_sec;
            }
        }

        self.stats = ToolpathStats {
            cutting_distance_mm,
            rapid_distance_mm,
            est_cutting_time_sec,
            est_rapid_time_sec,
            total_est_time_sec,
            pass_count: self.stats.pass_count,
            linear_move_count,
            arc_move_count,
            drill_cycle_count,
            bounds_wcs,
        };
    }
}

/// Planar or helical `G17` XY circular arc specification passed to [`ToolpathBuilder::arc_xy_to`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct XyArcMove {
    pub end_wcs: DVec3,
    pub center_xy: DVec2,
    pub radius_mm: f64,
    pub ccw: bool,
    pub turns: u32,
    pub feed_mm_min: f64,
    pub role: MotionRole,
}

/// Vertical `G18` (`XZ`) or `G19` (`YZ`) circular arc specification passed to [`ToolpathBuilder::arc_vertical_plane_to`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VerticalArcMove {
    pub plane: ArcPlane,
    pub end_wcs: DVec3,
    pub center_wcs: DVec3,
    pub radius_mm: f64,
    pub signed_sweep_rad: f64,
    pub feed_mm_min: f64,
    pub role: MotionRole,
}

/// Parameters describing a canned or expanded drilling cycle (`G81`/`G82`/`G83`/`G73`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DrillCycleSpec {
    pub kind: DrillCycleKind,
    pub bottom_wcs: DVec3,
    pub r_plane_z_wcs: f64,
    pub clear_z_wcs: f64,
    pub peck_q_mm: f64,
    pub decreasing_factor: Option<f64>,
    pub min_peck_mm: f64,
    pub chip_break_retract_mm: f64,
    pub dwell_sec: f64,
    pub feed_mm_min: f64,
}

/// Stateful builder that emits synchronized [`ClEvent`]s and exact [`ToolpathSegment3D`] spans
/// while guarding against `NaN`/`Inf`, zero-length segments, and zero-radius arcs.
pub struct ToolpathBuilder {
    op_id: String,
    current_wcs: Option<DVec3>,
    current_rotary_deg: f64,
    pub events: Vec<ClEvent>,
    pub segments_wcs: Vec<ToolpathSegment3D>,
    pub tab_markers_wcs: Vec<TabMarker3D>,
    pub warnings: Vec<String>,
    pub clearing_stage: Option<Box<ClearingSubToolpath>>,
    pub pass_count: usize,
    dwell_total_sec: f64,
}

impl ToolpathBuilder {
    pub fn new(op_id: impl Into<String>) -> Self {
        Self {
            op_id: op_id.into(),
            current_wcs: None,
            current_rotary_deg: 0.0,
            events: Vec::new(),
            segments_wcs: Vec::new(),
            tab_markers_wcs: Vec::new(),
            warnings: Vec::new(),
            clearing_stage: None,
            pass_count: 0,
            dwell_total_sec: 0.0,
        }
    }

    #[inline]
    pub fn current_pos(&self) -> Option<DVec3> {
        self.current_wcs
    }

    #[inline]
    pub fn current_rotary_deg(&self) -> f64 {
        self.current_rotary_deg
    }

    pub fn push_event(&mut self, event: ClEvent) {
        if let ClEvent::Dwell { seconds } = event
            && seconds.is_finite()
            && seconds > 0.0
        {
            self.dwell_total_sec += seconds;
        }
        self.events.push(event);
    }

    pub fn rapid_to(&mut self, target_wcs: DVec3) {
        if !is_finite_vec3(target_wcs) {
            return;
        }
        if let Some(prev) = self.current_wcs {
            if prev.distance(target_wcs) <= 1e-7 {
                return;
            }
            self.segments_wcs.push(ToolpathSegment3D {
                start_wcs: prev,
                end_wcs: target_wcs,
                motion: SegmentMotion3D::Rapid,
                start_rotary_deg: self.current_rotary_deg,
                end_rotary_deg: self.current_rotary_deg,
            });
        }
        self.current_wcs = Some(target_wcs);
        self.events.push(ClEvent::RapidMove {
            end_wcs: target_wcs,
        });
    }

    pub fn linear_to(&mut self, target_wcs: DVec3, feed_mm_min: f64, role: MotionRole) {
        if !is_finite_vec3(target_wcs) || !feed_mm_min.is_finite() || feed_mm_min <= 1e-6 {
            return;
        }
        if let Some(prev) = self.current_wcs {
            if prev.distance(target_wcs) <= 1e-7 {
                return;
            }
            self.segments_wcs.push(ToolpathSegment3D {
                start_wcs: prev,
                end_wcs: target_wcs,
                motion: SegmentMotion3D::Linear { feed_mm_min, role },
                start_rotary_deg: self.current_rotary_deg,
                end_rotary_deg: self.current_rotary_deg,
            });
        }
        self.current_wcs = Some(target_wcs);
        self.events.push(ClEvent::LinearMove {
            end_wcs: target_wcs,
            feed_mm_min,
            role,
        });
    }

    /// Emits a pure 4th-axis angular positioning move (`ClEvent::RotaryIndex`) at the current WCS XYZ.
    pub fn rotary_index_to(&mut self, angle_deg: f64, rapid: bool, feed_deg_min: Option<f64>) {
        if !angle_deg.is_finite() {
            return;
        }
        let prev_rot = self.current_rotary_deg;
        if let Some(cur) = self.current_wcs
            && (angle_deg - prev_rot).abs() > 1e-6
        {
            let motion = if rapid {
                SegmentMotion3D::Rapid
            } else {
                SegmentMotion3D::Linear {
                    feed_mm_min: feed_deg_min.unwrap_or(1_800.0).max(1.0),
                    role: MotionRole::LinkTransition,
                }
            };
            self.segments_wcs.push(ToolpathSegment3D {
                start_wcs: cur,
                end_wcs: cur,
                motion,
                start_rotary_deg: prev_rot,
                end_rotary_deg: angle_deg,
            });
        }
        self.current_rotary_deg = angle_deg;
        self.events.push(ClEvent::RotaryIndex {
            angle_deg,
            rapid,
            feed_deg_min,
        });
    }

    /// Emits a coordinated 4-axis rapid traverse (`ClEvent::RotaryRapidMove`).
    pub fn rotary_rapid_to(&mut self, target_wcs: DVec3, rotary_deg: f64) {
        if !is_finite_vec3(target_wcs) || !rotary_deg.is_finite() {
            return;
        }
        let prev_rot = self.current_rotary_deg;
        if let Some(prev) = self.current_wcs {
            if prev.distance(target_wcs) <= 1e-7 && (rotary_deg - prev_rot).abs() <= 1e-6 {
                return;
            }
            self.segments_wcs.push(ToolpathSegment3D {
                start_wcs: prev,
                end_wcs: target_wcs,
                motion: SegmentMotion3D::Rapid,
                start_rotary_deg: prev_rot,
                end_rotary_deg: rotary_deg,
            });
        }
        self.current_wcs = Some(target_wcs);
        self.current_rotary_deg = rotary_deg;
        self.events.push(ClEvent::RotaryRapidMove {
            end_wcs: target_wcs,
            rotary_deg,
        });
    }

    /// Emits a coordinated 4-axis linear + rotary cutting/linking move (`ClEvent::RotaryLinearMove`).
    pub fn rotary_linear_to(
        &mut self,
        target_wcs: DVec3,
        rotary_deg: f64,
        feed_mm_min: f64,
        effective_radius_mm: f64,
        role: MotionRole,
    ) {
        if !is_finite_vec3(target_wcs)
            || !rotary_deg.is_finite()
            || !feed_mm_min.is_finite()
            || feed_mm_min <= 1e-6
        {
            return;
        }
        let prev_rot = self.current_rotary_deg;
        if let Some(prev) = self.current_wcs {
            if prev.distance(target_wcs) <= 1e-7 && (rotary_deg - prev_rot).abs() <= 1e-6 {
                return;
            }
            self.segments_wcs.push(ToolpathSegment3D {
                start_wcs: prev,
                end_wcs: target_wcs,
                motion: SegmentMotion3D::Linear { feed_mm_min, role },
                start_rotary_deg: prev_rot,
                end_rotary_deg: rotary_deg,
            });
        }
        self.current_wcs = Some(target_wcs);
        self.current_rotary_deg = rotary_deg;
        self.events.push(ClEvent::RotaryLinearMove {
            end_wcs: target_wcs,
            rotary_deg,
            feed_mm_min,
            effective_radius_mm: effective_radius_mm.max(0.0),
            role,
        });
    }

    /// Emits a planar or helical XY arc (`G17` `G2`/`G3`) from `current_pos` to `arc.end_wcs` around `arc.center_xy`.
    pub fn arc_xy_to(&mut self, arc: XyArcMove) {
        let Some(prev) = self.current_wcs else {
            return;
        };
        if !is_finite_vec3(arc.end_wcs)
            || !arc.center_xy.is_finite()
            || !arc.radius_mm.is_finite()
            || arc.radius_mm <= 1e-5
            || !arc.feed_mm_min.is_finite()
            || arc.feed_mm_min <= 1e-6
        {
            return;
        }

        let start_xy = DVec2::new(prev.x, prev.y);
        let end_xy = DVec2::new(arc.end_wcs.x, arc.end_wcs.y);
        let a0 = (start_xy.y - arc.center_xy.y).atan2(start_xy.x - arc.center_xy.x);
        let a1 = (end_xy.y - arc.center_xy.y).atan2(end_xy.x - arc.center_xy.x);

        let base_mag = if arc.ccw {
            let mut s = (a1 - a0).rem_euclid(std::f64::consts::TAU);
            if s <= 1e-9 {
                if start_xy.distance(end_xy) <= 1e-5 {
                    s = std::f64::consts::TAU;
                } else {
                    return;
                }
            }
            s
        } else {
            let mut s = (a0 - a1).rem_euclid(std::f64::consts::TAU);
            if s <= 1e-9 {
                if start_xy.distance(end_xy) <= 1e-5 {
                    s = std::f64::consts::TAU;
                } else {
                    return;
                }
            }
            s
        };

        // If a full 360° circle has `turns >= 1`, `turns` counts EXTRA 360° revolutions beyond the base sweep
        let total_mag = base_mag + f64::from(arc.turns) * std::f64::consts::TAU;
        if total_mag * arc.radius_mm <= 1e-6 && (arc.end_wcs.z - prev.z).abs() <= 1e-6 {
            return;
        }

        let signed_sweep_rad = if arc.ccw { total_mag } else { -total_mag };
        let dir = if arc.ccw { ArcDir::CCW } else { ArcDir::CW };
        let center_offset_ijk = DVec3::new(arc.center_xy.x - prev.x, arc.center_xy.y - prev.y, 0.0);

        self.segments_wcs.push(ToolpathSegment3D {
            start_wcs: prev,
            end_wcs: arc.end_wcs,
            motion: SegmentMotion3D::Arc {
                plane: ArcPlane::XY,
                dir,
                center_wcs: DVec3::new(arc.center_xy.x, arc.center_xy.y, prev.z),
                radius_mm: arc.radius_mm,
                signed_sweep_rad,
                feed_mm_min: arc.feed_mm_min,
                role: arc.role,
            },
            start_rotary_deg: self.current_rotary_deg,
            end_rotary_deg: self.current_rotary_deg,
        });
        self.current_wcs = Some(arc.end_wcs);
        self.events.push(ClEvent::ArcMove {
            plane: ArcPlane::XY,
            dir,
            end_wcs: arc.end_wcs,
            center_offset_ijk,
            radius_mm: arc.radius_mm,
            turns: arc.turns,
            feed_mm_min: arc.feed_mm_min,
            role: arc.role,
        });
    }

    /// Emits a vertical `XZ` (`G18`) or `YZ` (`G19`) circular arc (used for vertical lead-in/lead-out when aligned).
    pub fn arc_vertical_plane_to(&mut self, arc: VerticalArcMove) {
        let Some(prev) = self.current_wcs else {
            return;
        };
        if !is_finite_vec3(arc.end_wcs)
            || !is_finite_vec3(arc.center_wcs)
            || arc.radius_mm <= 1e-5
            || arc.signed_sweep_rad.abs() * arc.radius_mm <= 1e-6
            || arc.feed_mm_min <= 1e-6
        {
            return;
        }
        let dir = if arc.signed_sweep_rad >= 0.0 {
            ArcDir::CCW
        } else {
            ArcDir::CW
        };
        let center_offset_ijk = arc.center_wcs - prev;
        self.segments_wcs.push(ToolpathSegment3D {
            start_wcs: prev,
            end_wcs: arc.end_wcs,
            motion: SegmentMotion3D::Arc {
                plane: arc.plane,
                dir,
                center_wcs: arc.center_wcs,
                radius_mm: arc.radius_mm,
                signed_sweep_rad: arc.signed_sweep_rad,
                feed_mm_min: arc.feed_mm_min,
                role: arc.role,
            },
            start_rotary_deg: self.current_rotary_deg,
            end_rotary_deg: self.current_rotary_deg,
        });
        self.current_wcs = Some(arc.end_wcs);
        self.events.push(ClEvent::ArcMove {
            plane: arc.plane,
            dir,
            end_wcs: arc.end_wcs,
            center_offset_ijk,
            radius_mm: arc.radius_mm,
            turns: 0,
            feed_mm_min: arc.feed_mm_min,
            role: arc.role,
        });
    }

    /// Emits all segments of an [`ArcPolyline2D`] at constant `z_wcs` (or linearly interpolating from `z_start` to `z_end`
    /// proportional to arc-length for spiral/linear ramps!), preserving every analytic `ArcSegment2D::Arc` as `ClEvent::ArcMove`!
    pub fn emit_polyline_2d_z_interp(
        &mut self,
        poly: &ArcPolyline2D,
        z_start: f64,
        z_end: f64,
        feed_mm_min: f64,
        role: MotionRole,
    ) {
        let total_len = poly.perimeter();
        if total_len <= 1e-7 {
            return;
        }
        let mut accum = 0.0;
        for seg in &poly.segments {
            let len = seg.length();
            if len <= 1e-7 {
                continue;
            }
            let s0 = seg.start();
            let z0 = z_start + (z_end - z_start) * (accum / total_len);
            if self
                .current_wcs
                .is_none_or(|cur| cur.distance(DVec3::new(s0.x, s0.y, z0)) > 1e-4)
            {
                self.linear_to(DVec3::new(s0.x, s0.y, z0), feed_mm_min, role);
            }
            accum += len;
            let z1 = z_start + (z_end - z_start) * (accum / total_len).clamp(0.0, 1.0);
            match *seg {
                ArcSegment2D::Line { end, .. } => {
                    self.linear_to(DVec3::new(end.x, end.y, z1), feed_mm_min, role);
                }
                ArcSegment2D::Arc {
                    end,
                    center,
                    radius,
                    ccw,
                    ..
                } => {
                    self.arc_xy_to(XyArcMove {
                        end_wcs: DVec3::new(end.x, end.y, z1),
                        center_xy: center,
                        radius_mm: radius,
                        ccw,
                        turns: 0,
                        feed_mm_min,
                        role,
                    });
                }
            }
        }
    }

    /// Emits a typed [`ClEvent::DrillCycle`] AND records its expanded physical peck/retract motions in `segments_wcs`.
    pub fn emit_drill_cycle(&mut self, spec: DrillCycleSpec) {
        let hole_xy = DVec2::new(spec.bottom_wcs.x, spec.bottom_wcs.y);
        let start_rapid = DVec3::new(
            hole_xy.x,
            hole_xy.y,
            spec.clear_z_wcs.max(spec.r_plane_z_wcs),
        );
        if self.current_wcs.is_none_or(|c| {
            DVec2::new(c.x, c.y).distance(hole_xy) > 1e-5 || c.z < spec.r_plane_z_wcs - 1e-5
        }) {
            self.rapid_to(start_rapid);
        }

        // Expand physical motions into `segments_wcs` for viewport visualization, time estimation, and crash checks
        let mut expanded =
            expand_drill_cycle_segments(self.current_wcs.unwrap_or(start_rapid), spec);
        for seg in &mut expanded {
            seg.start_rotary_deg = self.current_rotary_deg;
            seg.end_rotary_deg = self.current_rotary_deg;
        }
        if let Some(last) = expanded.last() {
            self.current_wcs = Some(last.end_wcs);
        }
        self.segments_wcs.extend(expanded);
        if spec.dwell_sec.is_finite() && spec.dwell_sec > 0.0 {
            self.dwell_total_sec += spec.dwell_sec;
        }

        self.events.push(ClEvent::DrillCycle {
            kind: spec.kind,
            bottom_wcs: spec.bottom_wcs,
            r_plane_z_wcs: spec.r_plane_z_wcs,
            clear_z_wcs: spec.clear_z_wcs,
            peck_q_mm: spec.peck_q_mm,
            decreasing_factor: spec.decreasing_factor,
            min_peck_mm: spec.min_peck_mm,
            chip_break_retract_mm: spec.chip_break_retract_mm,
            dwell_sec: spec.dwell_sec,
            feed_mm_min: spec.feed_mm_min,
        });
    }

    /// Finalizes `GeneratedToolpath` and patches `OperationStart.est_time_sec` with computed cycle time.
    pub fn finish(mut self) -> GeneratedToolpath {
        let mut cutting_distance_mm = 0.0;
        let mut rapid_distance_mm = 0.0;
        let mut est_cutting_time_sec = self.dwell_total_sec;
        let mut est_rapid_time_sec = 0.0;
        let mut bounds_wcs = Aabb3::EMPTY;

        for seg in &self.segments_wcs {
            bounds_wcs.extend_point(seg.start_wcs);
            bounds_wcs.extend_point(seg.end_wcs);
            if matches!(seg.motion, SegmentMotion3D::Arc { .. }) {
                for i in 1..=8 {
                    bounds_wcs.extend_point(seg.sample_point(f64::from(i) / 8.0));
                }
            }
            let len = seg.length_mm();
            match seg.motion {
                SegmentMotion3D::Rapid => {
                    rapid_distance_mm += len;
                    est_rapid_time_sec += (len / DEFAULT_RAPID_RATE_MM_MIN) * 60.0;
                }
                SegmentMotion3D::Linear { feed_mm_min, .. }
                | SegmentMotion3D::Arc { feed_mm_min, .. } => {
                    cutting_distance_mm += len;
                    if feed_mm_min > 1e-6 {
                        est_cutting_time_sec += (len / feed_mm_min) * 60.0;
                    }
                }
            }
        }

        let mut linear_move_count = 0usize;
        let mut arc_move_count = 0usize;
        let mut drill_cycle_count = 0usize;
        for ev in &self.events {
            match ev {
                ClEvent::LinearMove { .. } | ClEvent::RotaryLinearMove { .. } => {
                    linear_move_count += 1;
                }
                ClEvent::ArcMove { .. } => arc_move_count += 1,
                ClEvent::DrillCycle { .. } => drill_cycle_count += 1,
                _ => {}
            }
        }

        let total_est_time_sec = est_cutting_time_sec + est_rapid_time_sec;
        for ev in &mut self.events {
            if let ClEvent::OperationStart { est_time_sec, .. } = ev {
                *est_time_sec = total_est_time_sec;
            }
        }

        GeneratedToolpath {
            op_id: self.op_id,
            events: self.events,
            segments_wcs: self.segments_wcs,
            stats: ToolpathStats {
                cutting_distance_mm,
                rapid_distance_mm,
                est_cutting_time_sec,
                est_rapid_time_sec,
                total_est_time_sec,
                pass_count: self.pass_count,
                linear_move_count,
                arc_move_count,
                drill_cycle_count,
                bounds_wcs,
            },
            tab_markers_wcs: self.tab_markers_wcs,
            warnings: self.warnings,
            clearing_stage: self.clearing_stage,
            rapid_lifts_inserted: 0,
            max_lifted_z_wcs: 0.0,
        }
    }
}

/// Expands a [`DrillCycleSpec`] into explicit `Rapid` and `Linear` plunge/retract spans.
pub fn expand_drill_cycle_segments(
    start_wcs: DVec3,
    spec: DrillCycleSpec,
) -> Vec<ToolpathSegment3D> {
    let DrillCycleSpec {
        kind,
        bottom_wcs,
        r_plane_z_wcs,
        clear_z_wcs,
        peck_q_mm,
        decreasing_factor,
        min_peck_mm,
        chip_break_retract_mm,
        dwell_sec: _,
        feed_mm_min,
    } = spec;
    let mut out = Vec::new();
    let mut cur = start_wcs;
    let push_rapid = |out: &mut Vec<ToolpathSegment3D>, cur: &mut DVec3, dst: DVec3| {
        if cur.distance(dst) > 1e-7 {
            out.push(ToolpathSegment3D {
                start_wcs: *cur,
                end_wcs: dst,
                motion: SegmentMotion3D::Rapid,
                start_rotary_deg: 0.0,
                end_rotary_deg: 0.0,
            });
            *cur = dst;
        }
    };
    let push_plunge = |out: &mut Vec<ToolpathSegment3D>, cur: &mut DVec3, dst: DVec3| {
        if cur.distance(dst) > 1e-7 {
            out.push(ToolpathSegment3D {
                start_wcs: *cur,
                end_wcs: dst,
                motion: SegmentMotion3D::Linear {
                    feed_mm_min,
                    role: MotionRole::Plunge,
                },
                start_rotary_deg: 0.0,
                end_rotary_deg: 0.0,
            });
            *cur = dst;
        }
    };

    let xy = DVec2::new(bottom_wcs.x, bottom_wcs.y);
    let r_pt = DVec3::new(xy.x, xy.y, r_plane_z_wcs);
    let clear_pt = DVec3::new(xy.x, xy.y, clear_z_wcs.max(r_plane_z_wcs));
    push_rapid(&mut out, &mut cur, r_pt);

    match kind {
        DrillCycleKind::Spot | DrillCycleKind::Simple => {
            push_plunge(&mut out, &mut cur, bottom_wcs);
            push_rapid(&mut out, &mut cur, clear_pt);
        }
        DrillCycleKind::ChipBreak | DrillCycleKind::DeepPeck => {
            let mut current_peck = peck_q_mm.max(0.1);
            let min_peck = min_peck_mm.clamp(0.05, current_peck);
            let factor = decreasing_factor.unwrap_or(1.0).clamp(0.25, 1.0);
            let chip_ret = chip_break_retract_mm.max(0.1);
            let re_entry_clearance = 0.5_f64;

            let mut drilled_z = r_plane_z_wcs;
            let mut guard = 0usize;
            while drilled_z > bottom_wcs.z + 1e-6 && guard < 2_000 {
                guard += 1;
                let next_z = (drilled_z - current_peck).max(bottom_wcs.z);
                push_plunge(&mut out, &mut cur, DVec3::new(xy.x, xy.y, next_z));
                drilled_z = next_z;

                if drilled_z <= bottom_wcs.z + 1e-6 {
                    break;
                }

                match kind {
                    DrillCycleKind::ChipBreak => {
                        let lift_z = (drilled_z + chip_ret).min(r_plane_z_wcs);
                        push_rapid(&mut out, &mut cur, DVec3::new(xy.x, xy.y, lift_z));
                    }
                    DrillCycleKind::DeepPeck => {
                        push_rapid(&mut out, &mut cur, r_pt);
                        let safe_resume_z = (drilled_z + re_entry_clearance).min(r_plane_z_wcs);
                        push_rapid(&mut out, &mut cur, DVec3::new(xy.x, xy.y, safe_resume_z));
                    }
                    _ => {}
                }
                current_peck = (current_peck * factor).max(min_peck);
            }
            push_rapid(&mut out, &mut cur, clear_pt);
        }
    }

    out
}

#[inline]
fn is_finite_vec3(v: DVec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}
