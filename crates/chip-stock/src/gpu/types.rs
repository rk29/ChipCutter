//! Tightly-packed, 16-byte-aligned GPU compute data contracts (`#[repr(C)] bytemuck::Pod`)
//! shared between Rust host code and WGSL compute shaders (`heightfield_sweep.wgsl`,
//! `cylindrical_sweep.wgsl`).

use chip_math::DVec3;
use chip_tools::CutterEnvelopeShape;

use crate::types::{SpanRuleViolations, SweepChord, SweepMotion, SweepToolSpec};

/// Workgroup / tile edge size in cells for both 2.5D Heightfield and 4th-Axis Cylindrical sweeps.
pub const GPU_TILE_SIZE: usize = 16;

/// `GpuToolProfile::shape_kind` discriminants matching WGSL constants.
pub const SHAPE_FLAT: u32 = 0;
pub const SHAPE_BULLNOSE: u32 = 1;
pub const SHAPE_BALL: u32 = 2;
pub const SHAPE_CONICAL: u32 = 3;
pub const SHAPE_FACE: u32 = 4;
pub const SHAPE_THREADMILL: u32 = 5;
pub const SHAPE_LOLLIPOP: u32 = 6;
pub const SHAPE_SLOTMILL: u32 = 7;
pub const SHAPE_DOVETAIL: u32 = 8;

/// `GpuToolProfile::flags` bitmask constants.
pub const PROF_FLAG_FLAT_ENDMILL: u32 = 1 << 0;
pub const PROF_FLAG_THREAD_MILL: u32 = 1 << 1;
pub const PROF_FLAG_CONICAL: u32 = 1 << 2;
pub const PROF_FLAG_RELIEVED_NECK: u32 = 1 << 3;

/// `GpuSweepChord::flags` bitmask constants.
pub const CHORD_FLAG_RAPID: u32 = 1 << 0;
pub const CHORD_FLAG_VERIFY: u32 = 1 << 1;
pub const CHORD_FLAG_PLANAR_ARC_XY: u32 = 1 << 2;
pub const CHORD_FLAG_CHECK_HOLDER: u32 = 1 << 3;
/// Tri-Dexel rotary flag: `p0_rot.xyz` is a workpiece-fixed stock-local cutter tip and
/// `arc_params.xyz` is the workpiece-fixed unit spindle vector for an arbitrary oriented pose.
pub const CHORD_FLAG_ORIENTED_POSE: u32 = 1 << 4;
/// Tri-Dexel cardinal spindle axis code stored in bits `8..11` of `flags`:
/// `0 = +Z` (default 3-axis / `0°`), `1 = -Z`, `2 = +Y`, `3 = -Y`, `4 = +X`, `5 = -X`.
pub const CHORD_CARDINAL_SHIFT: u32 = 8;
pub const CHORD_CARDINAL_MASK: u32 = 0x7;

/// `GpuRotaryPose::flags` bitmask constants.
pub const POSE_FLAG_RAPID: u32 = 1 << 0;
pub const POSE_FLAG_VERIFY: u32 = 1 << 1;
pub const POSE_FLAG_CHECK_HOLDER: u32 = 1 << 2;

/// Compact 64-byte (`4 × 16B`) GPU encoding of [`crate::types::SweepToolSpec`] /
/// [`chip_tools::AnalyticCutterEnvelope`] covering all 7 cutter geometries.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GpuToolProfile {
    pub shape_kind: u32,
    pub flags: u32,
    pub teeth_count: u32,
    pub _pad0: u32,

    pub cutting_radius_mm: f32,
    pub shank_radius_mm: f32,
    pub holder_radius_mm: f32,
    pub flute_length_mm: f32,

    pub effective_reach_mm: f32,
    pub stickout_length_mm: f32,
    pub neck_radius_mm: f32,
    pub neck_length_mm: f32,

    /// Shape-specific parameter 0:
    /// - `BullNose` / `Face`: `flat_radius_mm`
    /// - `Conical`: `tip_radius_mm`
    /// - `ThreadMill`: `pitch_mm`
    /// - `Lollipop`: `z_neck_join_mm`
    /// - `SlotMill`: `cutting_thickness_mm`
    /// - `Dovetail`: `cutting_height_mm`
    pub param0: f32,
    /// Shape-specific parameter 1:
    /// - `BullNose` / `Face`: `corner_radius_mm`
    /// - `Conical`: `tan_half_angle`
    /// - `ThreadMill`: `z_start`
    /// - `Lollipop`: `flat_radius_mm`
    /// - `SlotMill`: `corner_radius_mm`
    /// - `Dovetail`: `tan_side_angle`
    pub param1: f32,
    /// Shape-specific parameter 2:
    /// - `Face`: `bottom_radius_mm`
    /// - `Conical`: `cot_half_angle` (`1.0 / tan_half_angle`)
    /// - `ThreadMill`: `tooth_step`
    /// - `Lollipop`: `sphere_radius_mm`
    /// - `SlotMill`: `flat_radius_mm`
    /// - `Dovetail`: `corner_radius_mm`
    pub param2: f32,
    /// Shape-specific parameter 3:
    /// - `Face`: `outer_radius_mm`
    /// - `ThreadMill`: `tan_half_thread_angle`
    /// - `Dovetail`: `top_radius_mm`
    pub param3: f32,
}

impl GpuToolProfile {
    /// Packs a [`SweepToolSpec`] into its GPU-resident representation.
    pub fn from_tool_spec(spec: &SweepToolSpec) -> Self {
        let prof = &spec.profile;
        let env = prof.envelope();

        let mut flags = 0u32;
        if spec.is_flat_endmill {
            flags |= PROF_FLAG_FLAT_ENDMILL;
        }
        if prof.is_thread_mill {
            flags |= PROF_FLAG_THREAD_MILL;
        }
        if prof.has_relieved_neck {
            flags |= PROF_FLAG_RELIEVED_NECK;
        }

        let (shape_kind, teeth_count, param0, param1, param2, param3) = match env.shape() {
            CutterEnvelopeShape::Flat { .. } => (SHAPE_FLAT, 0, 0.0, 0.0, 0.0, 0.0),
            CutterEnvelopeShape::BullNose {
                flat_radius_mm,
                corner_radius_mm,
            } => (
                SHAPE_BULLNOSE,
                0,
                flat_radius_mm as f32,
                corner_radius_mm as f32,
                0.0,
                0.0,
            ),
            CutterEnvelopeShape::Ball { .. } => (SHAPE_BALL, 0, 0.0, 0.0, 0.0, 0.0),
            CutterEnvelopeShape::Conical {
                tip_radius_mm,
                tan_half_angle,
                ..
            } => {
                flags |= PROF_FLAG_CONICAL;
                let cot_half = 1.0 / tan_half_angle.max(1e-6);
                (
                    SHAPE_CONICAL,
                    0,
                    tip_radius_mm as f32,
                    tan_half_angle as f32,
                    cot_half as f32,
                    0.0,
                )
            }
            CutterEnvelopeShape::Face {
                flat_radius_mm,
                bottom_radius_mm,
                outer_radius_mm,
                corner_radius_mm,
            } => (
                SHAPE_FACE,
                0,
                flat_radius_mm as f32,
                corner_radius_mm as f32,
                bottom_radius_mm as f32,
                outer_radius_mm as f32,
            ),
            CutterEnvelopeShape::ThreadMill { pitch_mm, .. } => {
                let (z_start, tooth_step, n_teeth, tan_half) = prof.thread_mill_tooth_band_params();
                (
                    SHAPE_THREADMILL,
                    n_teeth,
                    pitch_mm as f32,
                    z_start as f32,
                    tooth_step as f32,
                    tan_half as f32,
                )
            }
            CutterEnvelopeShape::Lollipop {
                flat_radius_mm,
                sphere_radius_mm,
                z_neck_join_mm,
                ..
            } => (
                SHAPE_LOLLIPOP,
                0,
                z_neck_join_mm as f32,
                flat_radius_mm as f32,
                sphere_radius_mm as f32,
                0.0,
            ),
            CutterEnvelopeShape::SlotMill {
                flat_radius_mm,
                cutting_thickness_mm,
                corner_radius_mm,
                ..
            } => (
                SHAPE_SLOTMILL,
                0,
                cutting_thickness_mm as f32,
                corner_radius_mm as f32,
                flat_radius_mm as f32,
                0.0,
            ),
            CutterEnvelopeShape::Dovetail {
                top_radius_mm,
                cutting_height_mm,
                tan_side_angle,
                corner_radius_mm,
                ..
            } => (
                SHAPE_DOVETAIL,
                0,
                cutting_height_mm as f32,
                tan_side_angle as f32,
                corner_radius_mm as f32,
                top_radius_mm as f32,
            ),
        };

        Self {
            shape_kind,
            flags,
            teeth_count,
            _pad0: 0,
            cutting_radius_mm: prof.cutting_radius_mm as f32,
            shank_radius_mm: prof.shank_radius_mm as f32,
            holder_radius_mm: prof.holder_radius_mm as f32,
            flute_length_mm: prof.flute_length_mm as f32,
            effective_reach_mm: prof.effective_reach_mm() as f32,
            stickout_length_mm: prof.stickout_length_mm as f32,
            neck_radius_mm: prof.neck_radius_mm as f32,
            neck_length_mm: prof.neck_length_mm as f32,
            param0,
            param1,
            param2,
            param3,
        }
    }
}

/// Compact 80-byte (`5 × 16B`) GPU encoding of a stock-local [`SweepChord`] supporting both
/// 3D linear motions (horizontal, vertical, sloped, rotary) and exact closed-form `PlanarArcXY`.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GpuSweepChord {
    /// `[p0_local.x, p0_local.y, p0_local.z, rot_deg_0]`
    pub p0_rot: [f32; 4],
    /// `[p1_local.x, p1_local.y, p1_local.z, rot_deg_1]`
    pub p1_rot: [f32; 4],
    /// `[arc_center_local.x, arc_center_local.y, arc_radius_mm, a0_rad]`
    pub arc_params: [f32; 4],
    /// `[signed_sweep_rad, rapid_crash_tol_mm, gouge_tol_mm, radial_wall_skin_mm]`
    pub sweep_and_tols: [f32; 4],
    /// `[op_idx, profile_idx, flags, chord_index]`
    pub meta: [u32; 4],
}

impl GpuSweepChord {
    /// Packs `chord` into stock-local coordinates relative to `origin_wcs`.
    pub fn from_heightfield_chord(
        chord: &SweepChord,
        origin_wcs: DVec3,
        verify: bool,
        check_holder: bool,
        chord_index: u32,
    ) -> Self {
        let mut flags = 0u32;
        if chord.is_rapid {
            flags |= CHORD_FLAG_RAPID;
        }
        if verify {
            flags |= CHORD_FLAG_VERIFY;
        }
        if check_holder {
            flags |= CHORD_FLAG_CHECK_HOLDER;
        }

        let (p0_rot, p1_rot, arc_params, signed_sweep) = match chord.motion {
            SweepMotion::Linear {
                p0,
                p1,
                rotary_deg_0,
                rotary_deg_1,
            } => (
                [
                    (p0.x - origin_wcs.x) as f32,
                    (p0.y - origin_wcs.y) as f32,
                    (p0.z - origin_wcs.z) as f32,
                    rotary_deg_0 as f32,
                ],
                [
                    (p1.x - origin_wcs.x) as f32,
                    (p1.y - origin_wcs.y) as f32,
                    (p1.z - origin_wcs.z) as f32,
                    rotary_deg_1 as f32,
                ],
                [0.0; 4],
                0.0_f32,
            ),
            SweepMotion::PlanarArcXY {
                start,
                end,
                center_xy,
                radius_mm,
                a0_rad,
                signed_sweep_rad,
                rotary_deg_0,
                rotary_deg_1,
            } => {
                flags |= CHORD_FLAG_PLANAR_ARC_XY;
                (
                    [
                        (start.x - origin_wcs.x) as f32,
                        (start.y - origin_wcs.y) as f32,
                        (start.z - origin_wcs.z) as f32,
                        rotary_deg_0 as f32,
                    ],
                    [
                        (end.x - origin_wcs.x) as f32,
                        (end.y - origin_wcs.y) as f32,
                        (end.z - origin_wcs.z) as f32,
                        rotary_deg_1 as f32,
                    ],
                    [
                        (center_xy.x - origin_wcs.x) as f32,
                        (center_xy.y - origin_wcs.y) as f32,
                        radius_mm as f32,
                        a0_rad as f32,
                    ],
                    signed_sweep_rad as f32,
                )
            }
        };

        Self {
            p0_rot,
            p1_rot,
            arc_params,
            sweep_and_tols: [
                signed_sweep,
                chord.verify_tol.rapid_crash_tol_mm as f32,
                chord.verify_tol.gouge_tol_mm as f32,
                chord.verify_tol.radial_wall_skin_mm as f32,
            ],
            meta: [chord.op_idx, chord.profile_idx as u32, flags, chord_index],
        }
    }
}

/// Compact 48-byte (`3 × 16B`) GPU encoding of a sampled 4th-axis rotary cutter pose in
/// workpiece-fixed coordinates relative to `rotary.center_wcs`.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GpuRotaryPose {
    /// `[tip_part_rel.x, tip_part_rel.y, tip_part_rel.z, rapid_crash_tol_mm]`
    pub tip_and_rapid_tol: [f32; 4],
    /// `[spindle_part.x, spindle_part.y, spindle_part.z, gouge_tol_mm]`
    pub spindle_and_gouge_tol: [f32; 4],
    /// `[op_idx, profile_idx, flags, pose_index]`
    pub meta: [u32; 4],
}

/// Single 32-byte (`2 × 16B`) atomic violation channel read back from GPU.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GpuViolationSlot {
    pub flag: u32,
    /// IEEE-754 bitcast of positive `f32` peak penetration depth (`mm`).
    /// Positive IEEE-754 `f32` bit patterns are strictly monotonic under unsigned `u32` `atomicMax`.
    pub max_depth_bits: u32,
    pub chord_idx: u32,
    pub cell_idx: u32,
    /// Stock-local `f32` Z bitcast (Heightfield) or `0` (Cylindrical).
    pub loc_z_bits: u32,
    /// `u32::MAX - first_chord_idx` accumulated via `atomicMax` (zero-init safe) so batched GPU
    /// sweeps preserve exact chronological onset segment attribution alongside peak depth/location.
    pub inv_first_chord: u32,
    pub _pad1: u32,
    pub _pad2: u32,
}

impl GpuViolationSlot {
    /// Decodes `(depth_mm, chord_idx, cell_idx, loc_z_local)` if this violation channel fired.
    #[inline]
    pub fn decode(&self) -> Option<(f64, u32, u32, f32)> {
        if self.flag == 0 {
            return None;
        }
        let depth = f64::from(f32::from_bits(self.max_depth_bits));
        let loc_z = f32::from_bits(self.loc_z_bits);
        Some((depth, self.chord_idx, self.cell_idx, loc_z))
    }

    /// Decodes `(depth_mm, peak_chord_idx, first_chord_idx, cell_idx, loc_z_local)` if this
    /// violation channel fired.
    #[inline]
    pub fn decode_with_onset(&self) -> Option<(f64, u32, u32, u32, f32)> {
        let (depth, peak_ch, cell, loc_z) = self.decode()?;
        let first_ch = if self.inv_first_chord > 0 {
            u32::MAX - self.inv_first_chord
        } else {
            peak_ch
        };
        Some((depth, peak_ch, first_ch, cell, loc_z))
    }
}

/// 176-byte (`11 × 16B`) per-batch atomic verification accumulator matching the 44-word
/// `array<atomic<u32>, 44>` header in `heightfield_sweep.wgsl`, `cylindrical_sweep.wgsl`,
/// and `tridexel_sweep.wgsl`.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GpuViolationAccum {
    pub rapid: GpuViolationSlot,
    pub gouge: GpuViolationSlot,
    pub flute: GpuViolationSlot,
    pub neck: GpuViolationSlot,
    pub holder: GpuViolationSlot,
    pub mutated_flag: u32,
    /// Non-zero when any Tri-Dexel ray overflowed `K = 4` inline spans (triggers CPU safety valve).
    pub overflow_flag: u32,
    pub _pad1: u32,
    pub _pad2: u32,
}

impl GpuViolationAccum {
    /// Decodes all 5 violation slots into [`SpanRuleViolations`] using a caller-provided
    /// `(cell_idx, loc_z_local) -> DVec3` WCS coordinate resolver.
    pub fn into_span_violations(
        &self,
        mut resolve_loc_wcs: impl FnMut(u32, f32) -> DVec3,
    ) -> (bool, SpanRuleViolations) {
        self.into_span_violations_with_chord(|_ch, cell, z_loc| resolve_loc_wcs(cell, z_loc))
    }

    /// Decodes all 5 violation slots into [`SpanRuleViolations`] using a caller-provided
    /// `(peak_chord_idx, first_chord_idx, cell_idx, loc_z_local) -> (DVec3, onset_src_chord_idx)` resolver.
    pub fn into_span_violations_mapped(
        &self,
        mut resolve: impl FnMut(u32, u32, u32, f32) -> (DVec3, u32),
    ) -> (bool, SpanRuleViolations) {
        let mut out = SpanRuleViolations::default();
        if let Some((depth, peak_ch, first_ch, cell, z_loc)) = self.rapid.decode_with_onset() {
            let (loc, src) = resolve(peak_ch, first_ch, cell, z_loc);
            out.record_rapid_at(depth, loc, src);
        }
        if let Some((depth, peak_ch, first_ch, cell, z_loc)) = self.gouge.decode_with_onset() {
            let (loc, src) = resolve(peak_ch, first_ch, cell, z_loc);
            out.record_gouge_at(depth, loc, src);
        }
        if let Some((depth, peak_ch, first_ch, cell, z_loc)) = self.flute.decode_with_onset() {
            let (loc, src) = resolve(peak_ch, first_ch, cell, z_loc);
            out.record_flute_at(depth, loc, src);
        }
        if let Some((depth, peak_ch, first_ch, cell, z_loc)) = self.neck.decode_with_onset() {
            let (loc, src) = resolve(peak_ch, first_ch, cell, z_loc);
            out.record_neck_at(depth, loc, src);
        }
        if let Some((depth, peak_ch, first_ch, cell, z_loc)) = self.holder.decode_with_onset() {
            let (loc, src) = resolve(peak_ch, first_ch, cell, z_loc);
            out.record_holder_at(depth, loc, src);
        }
        (self.mutated_flag != 0, out)
    }

    /// Decodes all 5 violation slots into [`SpanRuleViolations`] using a caller-provided
    /// `(chord_idx, cell_idx, loc_z_local) -> DVec3` WCS coordinate resolver.
    pub fn into_span_violations_with_chord(
        &self,
        mut resolve_loc_wcs: impl FnMut(u32, u32, f32) -> DVec3,
    ) -> (bool, SpanRuleViolations) {
        self.into_span_violations_mapped(|peak_ch, first_ch, cell, z_loc| {
            (resolve_loc_wcs(peak_ch, cell, z_loc), first_ch)
        })
    }
}

/// 32-byte (`2 × 16B`) uniform buffer bound by `heightfield_sweep.wgsl`.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct HeightfieldUniforms {
    pub nx: u32,
    pub ny: u32,
    pub tiles_x: u32,
    pub num_tiles: u32,
    pub dx_mm: f32,
    pub dy_mm: f32,
    pub _pad0: f32,
    pub _pad1: f32,
}

/// 80-byte (`5 × 16B`) uniform buffer bound by `cylindrical_sweep.wgsl`.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct CylindricalUniforms {
    /// `[e1.x, e1.y, e1.z, u_min]`
    pub e1_and_u_min: [f32; 4],
    /// `[e2.x, e2.y, e2.z, du]`
    pub e2_and_du: [f32; 4],
    /// `[u_vec.x, u_vec.y, u_vec.z, dtheta]`
    pub u_vec_and_dtheta: [f32; 4],
    /// `[nu, ntheta, tiles_theta, num_tiles]`
    pub dims: [u32; 4],
    /// `[u_center, direction_sign, 0.0, 0.0]`
    pub params: [f32; 4],
}

/// 48-byte (`3 × 16B`) uniform buffer bound by `tridexel_sweep.wgsl` (axis-generic across X/Y/Z).
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct TriDexelPlaneUniforms {
    pub nu: u32,
    pub nv: u32,
    pub tiles_u: u32,
    pub num_tiles: u32,
    pub axis_id: u32,
    pub has_target: u32,
    pub plane_ray_offset: u32,
    pub _pad0: u32,
    pub du_mm: f32,
    pub dv_mm: f32,
    pub dz_step_mm: f32,
    pub _pad1: f32,
}

/// 112-byte (`7 × 16B`) unified uniform buffer bound by `bvh_distance.wgsl`.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct BvhDispatchUniforms {
    pub dims: [u32; 4],
    pub v0: [f32; 4],
    pub v1: [f32; 4],
    pub v2: [f32; 4],
    pub v3: [f32; 4],
    pub v4: [f32; 4],
    pub counts: [u32; 4],
}

/// Number of logarithmic error histogram bins in `stock_metrics.wgsl` (`0.0005 mm .. 100.0 mm`).
pub const GPU_METRICS_HIST_BINS: usize = 256;
/// Maximum parallel reduction workgroups dispatched by `stock_metrics.wgsl`.
pub const GPU_METRICS_MAX_WORKGROUPS: usize = 512;

/// 128-byte (`8 × 16B`) unified uniform buffer bound by `stock_metrics.wgsl`.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct MetricsUniforms {
    pub dims: [u32; 4],
    pub td_offsets: [u32; 4],
    pub origin_and_tol: [f32; 4],
    pub pitch_and_mean: [f32; 4],
    pub cyl_e1_pol: [f32; 4],
    pub cyl_e2: [f32; 4],
    pub cyl_u_vec: [f32; 4],
    pub dispatch: [u32; 4],
}

/// 96-byte (`6 × 16B`) per-workgroup partial reduction output from `stock_metrics.wgsl`.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GpuMetricsPartial {
    pub remaining_vol: f32,
    pub target_vol: f32,
    pub excess_vol: f32,
    pub gouge_vol: f32,
    pub sum_sq: f32,
    pub sum_abs: f32,
    pub sum_signed: f32,
    pub excess_sum_sq: f32,
    pub gouge_sum_sq: f32,
    pub max_excess: f32,
    pub max_gouge: f32,
    pub _pad0: f32,
    pub max_excess_loc: [f32; 3],
    pub sample_count: u32,
    pub max_gouge_loc: [f32; 3],
    pub in_tol_count: u32,
    pub excess_count: u32,
    pub gouge_count: u32,
    pub _pad1: u32,
    pub _pad2: u32,
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::mem::{align_of, offset_of, size_of};

    use super::*;

    #[test]
    fn gpu_struct_layouts_are_16_byte_aligned_and_match_wgsl_offsets() {
        assert_eq!(size_of::<GpuToolProfile>(), 64);
        assert_eq!(size_of::<GpuToolProfile>() % 16, 0);
        assert_eq!(align_of::<GpuToolProfile>(), 4);
        assert_eq!(offset_of!(GpuToolProfile, shape_kind), 0);
        assert_eq!(offset_of!(GpuToolProfile, cutting_radius_mm), 16);
        assert_eq!(offset_of!(GpuToolProfile, effective_reach_mm), 32);
        assert_eq!(offset_of!(GpuToolProfile, param0), 48);

        assert_eq!(size_of::<GpuSweepChord>(), 80);
        assert_eq!(size_of::<GpuSweepChord>() % 16, 0);
        assert_eq!(offset_of!(GpuSweepChord, p0_rot), 0);
        assert_eq!(offset_of!(GpuSweepChord, p1_rot), 16);
        assert_eq!(offset_of!(GpuSweepChord, arc_params), 32);
        assert_eq!(offset_of!(GpuSweepChord, sweep_and_tols), 48);
        assert_eq!(offset_of!(GpuSweepChord, meta), 64);

        assert_eq!(size_of::<GpuRotaryPose>(), 48);
        assert_eq!(size_of::<GpuRotaryPose>() % 16, 0);
        assert_eq!(offset_of!(GpuRotaryPose, tip_and_rapid_tol), 0);
        assert_eq!(offset_of!(GpuRotaryPose, spindle_and_gouge_tol), 16);
        assert_eq!(offset_of!(GpuRotaryPose, meta), 32);

        assert_eq!(size_of::<GpuViolationSlot>(), 32);
        assert_eq!(size_of::<GpuViolationAccum>(), 176);
        assert_eq!(size_of::<GpuViolationAccum>() % 16, 0);
        assert_eq!(offset_of!(GpuViolationAccum, rapid), 0);
        assert_eq!(offset_of!(GpuViolationAccum, gouge), 32);
        assert_eq!(offset_of!(GpuViolationAccum, flute), 64);
        assert_eq!(offset_of!(GpuViolationAccum, neck), 96);
        assert_eq!(offset_of!(GpuViolationAccum, holder), 128);
        assert_eq!(offset_of!(GpuViolationAccum, mutated_flag), 160);

        assert_eq!(size_of::<HeightfieldUniforms>(), 32);
        assert_eq!(size_of::<CylindricalUniforms>(), 80);
        assert_eq!(size_of::<TriDexelPlaneUniforms>(), 48);
        assert_eq!(size_of::<BvhDispatchUniforms>(), 112);
        assert_eq!(size_of::<MetricsUniforms>(), 128);
        assert_eq!(size_of::<GpuMetricsPartial>(), 96);
        assert_eq!(offset_of!(GpuMetricsPartial, max_excess_loc), 48);
        assert_eq!(offset_of!(GpuMetricsPartial, sample_count), 60);
        assert_eq!(offset_of!(GpuMetricsPartial, max_gouge_loc), 64);
        assert_eq!(offset_of!(GpuMetricsPartial, in_tol_count), 76);
        assert_eq!(offset_of!(GpuMetricsPartial, excess_count), 80);
        assert_eq!(offset_of!(GpuViolationAccum, overflow_flag), 164);

        // Verify Pod byte round-trip for GpuViolationAccum
        let mut acc = GpuViolationAccum::default();
        acc.rapid.flag = 1;
        acc.rapid.max_depth_bits = 1.25_f32.to_bits();
        acc.rapid.chord_idx = 7;
        acc.rapid.cell_idx = 42;
        acc.rapid.loc_z_bits = (-3.5_f32).to_bits();
        acc.mutated_flag = 1;
        let bytes = bytemuck::bytes_of(&acc);
        assert_eq!(bytes.len(), 176);
        let roundtrip: &GpuViolationAccum = bytemuck::from_bytes(bytes);
        assert_eq!(*roundtrip, acc);
        let (mutated, viol) = roundtrip
            .into_span_violations(|cell, z| DVec3::new(f64::from(cell), 0.0, f64::from(z)));
        assert!(mutated);
        let (d, loc) = viol.rapid.unwrap();
        assert!((d - 1.25).abs() < 1e-6);
        assert!((loc.x - 42.0).abs() < 1e-6);
        assert!((loc.z - (-3.5)).abs() < 1e-6);
    }
}
