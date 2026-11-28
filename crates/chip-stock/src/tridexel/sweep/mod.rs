mod cardinal;
mod oriented;
#[cfg(feature = "gpu-compute")]
mod replay;

use chip_math::DVec3;

use crate::types::{
    Axis3, BatchSweepOutput, RadialCutterProfile, RotaryStockSpec, SpanRuleViolations,
    StockVerifyTolerances, SweepChord, SweepMotion, SweepToolSpec,
};

use super::grid::TriDexelStock;

#[cfg(feature = "gpu-compute")]
pub(crate) use oriented::oriented_pose_stock_aabb;
#[cfg(feature = "gpu-compute")]
pub(crate) use replay::replay_single_ray_primitive;

/// Canonical axis-aligned spindle orientation in workpiece-fixed coordinates (`0°`, `90°`, `180°`,
/// `270°` indexed 3+1 or standard 3-axis `+Z`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CardinalSpindle {
    PosZ = 0,
    NegZ = 1,
    PosY = 2,
    NegY = 3,
    PosX = 4,
    NegX = 5,
}

impl CardinalSpindle {
    #[inline]
    pub(crate) fn from_unit_vec(s: DVec3) -> Option<Self> {
        const TOL: f64 = 1e-5;
        if (s.z - 1.0).abs() <= TOL && s.x.abs() <= TOL && s.y.abs() <= TOL {
            Some(Self::PosZ)
        } else if (s.z + 1.0).abs() <= TOL && s.x.abs() <= TOL && s.y.abs() <= TOL {
            Some(Self::NegZ)
        } else if (s.y - 1.0).abs() <= TOL && s.x.abs() <= TOL && s.z.abs() <= TOL {
            Some(Self::PosY)
        } else if (s.y + 1.0).abs() <= TOL && s.x.abs() <= TOL && s.z.abs() <= TOL {
            Some(Self::NegY)
        } else if (s.x - 1.0).abs() <= TOL && s.y.abs() <= TOL && s.z.abs() <= TOL {
            Some(Self::PosX)
        } else if (s.x + 1.0).abs() <= TOL && s.y.abs() <= TOL && s.z.abs() <= TOL {
            Some(Self::NegX)
        } else {
            None
        }
    }

    #[cfg(feature = "gpu-compute")]
    #[inline]
    pub(crate) const fn code(self) -> u32 {
        self as u32
    }

    #[inline]
    pub(crate) const fn axis(self) -> Axis3 {
        match self {
            Self::PosZ | Self::NegZ => Axis3::Z,
            Self::PosY | Self::NegY => Axis3::Y,
            Self::PosX | Self::NegX => Axis3::X,
        }
    }

    #[inline]
    pub(crate) const fn sign(self) -> f64 {
        match self {
            Self::PosZ | Self::PosY | Self::PosX => 1.0,
            Self::NegZ | Self::NegY | Self::NegX => -1.0,
        }
    }
}

/// Single arbitrarily oriented rotary cutter pose in workpiece-fixed coordinates (`tip`, `spindle`)
/// alongside the original WCS tip position (`tip_wcs`) for diagnostic reporting.
#[derive(Debug, Clone, Copy)]
pub(crate) struct OrientedPose3D {
    pub(crate) tip: DVec3,
    pub(crate) spindle: DVec3,
    pub(crate) tip_wcs: DVec3,
}

/// Workpiece-fixed volumetric sweep primitive emitted by [`decompose_chord_for_tridexel`].
#[derive(Debug, Clone, Copy)]
pub(crate) enum SweptPrimitive {
    CardinalLinear {
        chord: LinearChord3D,
        spindle: CardinalSpindle,
        rot_deg: f64,
    },
    OrientedPose(OrientedPose3D),
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct LinearChord3D {
    pub(crate) p0: DVec3,
    pub(crate) p1: DVec3,
}

impl LinearChord3D {
    #[inline]
    pub(super) fn min_z(self) -> f64 {
        self.p0.z.min(self.p1.z)
    }

    #[inline]
    pub(super) fn max_z(self) -> f64 {
        self.p0.z.max(self.p1.z)
    }
}

pub(super) struct SweepContext<'a> {
    pub(super) prof: &'a RadialCutterProfile,
    pub(super) is_flat_endmill: bool,
    pub(super) is_rapid: bool,
    pub(super) tol: StockVerifyTolerances,
    pub(super) op_idx: u32,
    pub(super) target: Option<&'a TriDexelStock>,
    pub(super) rotary: Option<RotaryStockSpec>,
    pub(super) rot_deg: f64,
}

impl SweepContext<'_> {
    #[inline]
    pub(super) fn to_diag_wcs(&self, pt_part: DVec3) -> DVec3 {
        if let Some(rot) = self.rotary
            && self.rot_deg.abs() > 1e-6
        {
            rot.rotate_point_wcs(pt_part, self.rot_deg)
        } else {
            pt_part
        }
    }
}

#[inline]
fn check_rotary_chuck_pose(
    rot: RotaryStockSpec,
    tip_wcs: DVec3,
    tip_part: DVec3,
    r_cut: f64,
    src_chord_idx: u32,
    violations: &mut SpanRuleViolations,
) {
    let (u_tip, _, r_tip) = rot.wcs_to_cylindrical(tip_part);
    let chuck_clearance_r = rot.chuck_radius_mm + r_cut;
    let u_overlaps_chuck = (u_tip + r_cut) >= rot.chuck_u_min && (u_tip - r_cut) <= rot.chuck_u_max;
    if u_overlaps_chuck && r_tip < chuck_clearance_r - 0.05 {
        let excess = chuck_clearance_r - r_tip;
        violations.record_holder_at(excess, tip_wcs, src_chord_idx);
    }
}

/// Decomposes a [`SweepChord`] into workpiece-fixed [`SweptPrimitive`] items shared identically
/// by both CPU `TriDexelStock` and GPU `GpuTriDexelState`, also checking rotary chuck keep-out
/// when `verify == true` and `rotary` is active.
#[allow(clippy::too_many_arguments)]
pub(crate) fn decompose_chord_for_tridexel(
    ch: &SweepChord,
    src_chord_idx: u32,
    rotary: Option<RotaryStockSpec>,
    min_pitch_mm: f64,
    prof: &RadialCutterProfile,
    verify: bool,
    skip_start_pose: bool,
    arc_buf: &mut Vec<LinearChord3D>,
    out_prims: &mut Vec<SweptPrimitive>,
    out_chuck_viol: &mut SpanRuleViolations,
) {
    let Some(rot) = rotary else {
        motion_to_linear_chords(ch.motion, arc_buf);
        for &lin in arc_buf.iter() {
            out_prims.push(SweptPrimitive::CardinalLinear {
                chord: lin,
                spindle: CardinalSpindle::PosZ,
                rot_deg: 0.0,
            });
        }
        return;
    };

    let has_chuck = verify && rot.chuck_u_max > rot.chuck_u_min + 1e-4;
    let r_cut = prof.cutting_radius_mm.max(0.05);
    let d_deg = ch.motion.rotary_delta_deg();

    if d_deg <= 1e-5 {
        let rot_deg = ch.motion.sample_rotary_deg(0.0);
        let spindle_part = rot.rotate_vector_wcs(DVec3::Z, -rot_deg);
        if let Some(cardinal) = CardinalSpindle::from_unit_vec(spindle_part) {
            motion_to_linear_chords(ch.motion, arc_buf);
            for &lin in arc_buf.iter() {
                let p0_part = if rot_deg.abs() > 1e-6 {
                    rot.rotate_point_wcs(lin.p0, -rot_deg)
                } else {
                    lin.p0
                };
                let p1_part = if rot_deg.abs() > 1e-6 {
                    rot.rotate_point_wcs(lin.p1, -rot_deg)
                } else {
                    lin.p1
                };
                if has_chuck {
                    check_rotary_chuck_pose(
                        rot,
                        lin.p0,
                        p0_part,
                        r_cut,
                        src_chord_idx,
                        out_chuck_viol,
                    );
                    let mid_wcs = lin.p0.lerp(lin.p1, 0.5);
                    let mid_part = p0_part.lerp(p1_part, 0.5);
                    check_rotary_chuck_pose(
                        rot,
                        mid_wcs,
                        mid_part,
                        r_cut,
                        src_chord_idx,
                        out_chuck_viol,
                    );
                    check_rotary_chuck_pose(
                        rot,
                        lin.p1,
                        p1_part,
                        r_cut,
                        src_chord_idx,
                        out_chuck_viol,
                    );
                }
                out_prims.push(SweptPrimitive::CardinalLinear {
                    chord: LinearChord3D {
                        p0: p0_part,
                        p1: p1_part,
                    },
                    spindle: cardinal,
                    rot_deg,
                });
            }
            return;
        }
    }

    // Non-cardinal constant angle (e.g. 30°/45° indexed) or simultaneous 4-axis rotary motion:
    // subdivide into workpiece-fixed oriented cutter poses scaled cleanly to voxel pitch and
    // effective rotary contact radius so scallops stay << voxel pitch without pose explosion.
    let step_lin = (0.5 * min_pitch_mm)
        .clamp(0.08, 0.30)
        .min((0.45 * r_cut).max(0.08));
    let p0_wcs = ch.motion.sample_point(0.0);
    let p1_wcs = ch.motion.sample_point(1.0);
    let (_, _, r0) = rot.wcs_to_cylindrical(p0_wcs);
    let (_, _, r1) = rot.wcs_to_cylindrical(p1_wcs);
    let r_ref = (r0.max(r1).min(rot.stock_outer_radius_mm) + r_cut)
        .clamp(r_cut.max(1.0), rot.stock_outer_radius_mm + r_cut);
    let step_deg = (step_lin / r_ref).to_degrees().clamp(0.35, 1.5);
    let lin_len = ch.motion.length_mm();
    let n_lin = (lin_len / step_lin).ceil() as usize;
    let n_rot = (d_deg / step_deg).ceil() as usize;
    let n_steps = n_lin.max(n_rot).clamp(1, 2_000);
    let k_start = usize::from(skip_start_pose);

    for k in k_start..=n_steps {
        let t = k as f64 / n_steps as f64;
        let tip_wcs = ch.motion.sample_point(t);
        let rot_deg = ch.motion.sample_rotary_deg(t);
        let tip_part = rot.rotate_point_wcs(tip_wcs, -rot_deg);
        let spindle_part = rot.rotate_vector_wcs(DVec3::Z, -rot_deg);
        if has_chuck {
            check_rotary_chuck_pose(rot, tip_wcs, tip_part, r_cut, src_chord_idx, out_chuck_viol);
        }
        out_prims.push(SweptPrimitive::OrientedPose(OrientedPose3D {
            tip: tip_part,
            spindle: spindle_part,
            tip_wcs,
        }));
    }
}

pub(crate) fn motion_to_linear_chords(motion: SweepMotion, out: &mut Vec<LinearChord3D>) {
    out.clear();
    match motion {
        SweepMotion::Linear { p0, p1, .. } => {
            out.push(LinearChord3D { p0, p1 });
        }
        SweepMotion::PlanarArcXY {
            start,
            end,
            radius_mm,
            signed_sweep_rad,
            ..
        } => {
            let r = radius_mm.max(1e-6);
            let max_sag = 0.005;
            let cos_half = (1.0 - (max_sag / r).min(0.5)).clamp(-1.0, 1.0);
            let max_dtheta = (2.0 * cos_half.acos()).clamp(0.02, 0.05);
            let steps = ((signed_sweep_rad.abs() / max_dtheta).ceil() as usize).clamp(4, 2_048);
            let mut prev = start;
            for k in 1..=steps {
                let u = k as f64 / steps as f64;
                let next = if k == steps {
                    end
                } else {
                    motion.sample_point(u)
                };
                out.push(LinearChord3D { p0: prev, p1: next });
                prev = next;
            }
        }
    }
}

impl TriDexelStock {
    /// Sweeps a batch of [`SweepChord`] primitives across all 3 orthogonal dexel planes,
    /// supporting both 3-axis and 4th-axis rotary motions (`self.rotary`) and optionally
    /// evaluating Rules 1–4 against `target` (`verify = true`).
    pub fn sweep_chords(
        &mut self,
        chords: &[SweepChord],
        tools: &[SweepToolSpec],
        target: Option<&TriDexelStock>,
        verify: bool,
    ) -> BatchSweepOutput {
        let mut out = BatchSweepOutput::default();
        let mut arc_buf: Vec<LinearChord3D> = Vec::with_capacity(8);
        let mut prim_buf: Vec<SweptPrimitive> = Vec::with_capacity(16);
        let min_pitch = self.pitch_mm.x.min(self.pitch_mm.y).min(self.pitch_mm.z);
        let rotary = self.rotary;
        let mut prev_oriented_end: Option<(DVec3, f64, u32, usize, bool)> = None;

        for (raw_idx, ch) in chords.iter().enumerate() {
            let Some(tool) = tools.get(ch.profile_idx) else {
                continue;
            };
            let start_pt = ch.motion.sample_point(0.0);
            let start_rot = ch.motion.sample_rotary_deg(0.0);
            let skip_start = prev_oriented_end.is_some_and(|(pp, pr, pop, pprof, prap)| {
                pop == ch.op_idx
                    && pprof == ch.profile_idx
                    && prap == ch.is_rapid
                    && pp.distance_squared(start_pt) <= 1e-10
                    && (pr - start_rot).abs() <= 1e-6
            });
            prim_buf.clear();
            decompose_chord_for_tridexel(
                ch,
                raw_idx as u32,
                rotary,
                min_pitch,
                &tool.profile,
                verify,
                skip_start,
                &mut arc_buf,
                &mut prim_buf,
                &mut out.violations,
            );
            prev_oriented_end = if prim_buf
                .last()
                .is_some_and(|p| matches!(p, SweptPrimitive::OrientedPose(_)))
            {
                Some((
                    ch.motion.sample_point(1.0),
                    ch.motion.sample_rotary_deg(1.0),
                    ch.op_idx,
                    ch.profile_idx,
                    ch.is_rapid,
                ))
            } else {
                None
            };
            for &prim in &prim_buf {
                match prim {
                    SweptPrimitive::CardinalLinear {
                        chord,
                        spindle,
                        rot_deg,
                    } => {
                        let ctx = SweepContext {
                            prof: &tool.profile,
                            is_flat_endmill: tool.is_flat_endmill,
                            is_rapid: ch.is_rapid,
                            tol: ch.verify_tol,
                            op_idx: ch.op_idx,
                            target,
                            rotary,
                            rot_deg,
                        };
                        let (mutated, viol) =
                            self.apply_cardinal_linear_chord(chord, spindle, &ctx, verify);
                        out.mutated |= mutated;
                        out.violations.merge(viol);
                    }
                    SweptPrimitive::OrientedPose(pose) => {
                        let ctx = SweepContext {
                            prof: &tool.profile,
                            is_flat_endmill: tool.is_flat_endmill,
                            is_rapid: ch.is_rapid,
                            tol: ch.verify_tol,
                            op_idx: ch.op_idx,
                            target,
                            rotary,
                            rot_deg: 0.0,
                        };
                        let (mutated, viol) = self.apply_oriented_pose(pose, &ctx, verify);
                        out.mutated |= mutated;
                        out.violations.merge(viol);
                    }
                }
            }
        }

        out
    }
}
