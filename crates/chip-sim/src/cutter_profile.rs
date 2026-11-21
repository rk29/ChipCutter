use chip_tools::{AnalyticCutterEnvelope, CutterEnvelopeShape, CutterGeometry, Tool};
use serde::{Deserialize, Serialize};

/// Material zone of a 3D revolved cutter assembly mesh.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ToolMeshZone {
    CuttingFlutes,
    Shank,
    ColletHolder,
}

/// UI-agnostic 3D lit vertex (`pos_mm`, `normal`, `color_rgba`, 40 bytes) ready for zero-copy
/// `bytemuck::cast_slice` GPU upload.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct SimColoredVertex {
    pub pos_mm: [f32; 3],
    pub normal: [f32; 3],
    pub color_rgba: [f32; 4],
}

/// Revolved 3D triangle mesh for a [`Tool`] (`tip = (0, 0, 0)`, axis = `+Z` in Setup WCS).
#[derive(Debug, Clone, Default)]
pub struct SimToolMesh {
    pub vertices: Vec<SimColoredVertex>,
    pub indices: Vec<u32>,
}

/// Exact `f64` radial profile `P(r)` and shank/holder envelope of a [`Tool`] (`06-simulation.md`),
/// delegating all analytic cutter profile math to [`AnalyticCutterEnvelope`].
#[derive(Debug, Clone, Copy)]
pub struct RadialCutterProfile {
    pub cutting_radius_mm: f64,
    pub shank_radius_mm: f64,
    pub holder_radius_mm: f64,
    pub flute_length_mm: f64,
    pub stickout_length_mm: f64,
    pub overall_length_mm: f64,
    pub is_center_cutting: bool,
    pub max_safe_plunge_depth_mm: f64,
    /// Relieved neck radius (`mm`) for [`CutterGeometry::ThreadMill`] (`< cutting_radius_mm`).
    /// Equals `cutting_radius_mm` for standard monotonic cutters.
    pub neck_radius_mm: f64,
    /// Usable relieved-neck reach (`mm`) above the tip before the full-diameter shank begins.
    /// Equals `flute_length_mm` for standard monotonic cutters.
    pub neck_length_mm: f64,
    /// Axial tooth pitch (`mm`) for [`CutterGeometry::ThreadMill`] (`0.0` for other cutters).
    pub thread_pitch_mm: f64,
    /// Active V-tooth ring count (`1` = single-point, `>1` = multi-form; `0` for other cutters).
    pub teeth_count: u32,
    /// Included thread flank angle in degrees (typically `60°`; `0.0` for other cutters).
    pub thread_angle_deg: f64,
    /// `true` when constructed from [`CutterGeometry::ThreadMill`] (non-monotonic undercut profile).
    pub is_thread_mill: bool,
    envelope: AnalyticCutterEnvelope,
}

impl RadialCutterProfile {
    /// Builds the exact radial height function `P(r)` and holder clearance envelope for `tool`.
    pub fn from_tool(tool: &Tool) -> Self {
        let envelope = AnalyticCutterEnvelope::from_geometry(&tool.geometry);
        let flute_length_mm = envelope.flute_length_mm();
        let overall_length_mm = envelope.overall_length_mm();
        let stickout_length_mm = tool.stickout_length_mm.max(flute_length_mm);
        let shank_radius_mm = envelope.shank_radius_mm();
        let cutting_radius_mm = envelope.cutting_radius_mm();
        let neck_radius_mm = envelope.neck_radius_mm();
        let neck_length_mm = envelope.effective_reach_mm();
        let is_thread_mill = envelope.is_thread_mill();

        let (thread_pitch_mm, teeth_count, thread_angle_deg) = match envelope.shape() {
            CutterEnvelopeShape::ThreadMill {
                pitch_mm,
                teeth_count,
                thread_angle_deg,
                ..
            } => (pitch_mm, teeth_count, thread_angle_deg),
            _ => (0.0, 0, 0.0),
        };

        let (is_center_cutting, max_safe_plunge_depth_mm) = match tool.geometry {
            CutterGeometry::FlatEndmill { diameter_mm, .. }
            | CutterGeometry::BullNoseEndmill { diameter_mm, .. }
            | CutterGeometry::BallEndmill { diameter_mm, .. } => {
                (true, (1.5 * diameter_mm).min(flute_length_mm))
            }
            CutterGeometry::ChamferMill { .. } | CutterGeometry::TwistDrill { .. } => {
                (true, flute_length_mm)
            }
            CutterGeometry::FaceMill { .. } | CutterGeometry::ThreadMill { .. } => (false, 0.0),
        };

        let holder_radius_mm = (shank_radius_mm * 2.2).max(12.0).max(shank_radius_mm + 4.0);

        Self {
            cutting_radius_mm,
            shank_radius_mm,
            holder_radius_mm,
            flute_length_mm,
            stickout_length_mm,
            overall_length_mm,
            is_center_cutting,
            max_safe_plunge_depth_mm,
            neck_radius_mm,
            neck_length_mm,
            thread_pitch_mm,
            teeth_count,
            thread_angle_deg,
            is_thread_mill,
            envelope,
        }
    }

    /// Underlying shared [`AnalyticCutterEnvelope`] from `chip-tools`.
    #[inline]
    pub const fn envelope(&self) -> &AnalyticCutterEnvelope {
        &self.envelope
    }

    /// Maximum usable reach above the cutter tip before the non-cutting shank begins:
    /// `neck_length_mm` for relieved-neck thread mills, `flute_length_mm` for all other cutters.
    #[inline]
    pub fn effective_reach_mm(&self) -> f64 {
        self.envelope.effective_reach_mm()
    }

    /// Returns `(z_start, tooth_step, n_teeth, tan_half_angle)` describing the periodic V-tooth
    /// cutting band of a [`CutterGeometry::ThreadMill`] within `z ∈ [0, flute_length_mm]`.
    #[inline]
    pub(crate) fn thread_mill_tooth_band_params(&self) -> (f64, f64, u32, f64) {
        self.envelope.thread_mill_tooth_band_params()
    }

    /// Evaluates the V-tooth radial envelope `R_tooth(z_local)` inside `[0, flute_length_mm]`
    /// for a [`CutterGeometry::ThreadMill`], clamped to `[neck_radius_mm, cutting_radius_mm]`.
    #[inline]
    pub fn thread_mill_tooth_radius_at_z(&self, z_local: f64) -> f64 {
        self.envelope.thread_mill_tooth_radius_at_z(z_local)
    }

    /// Evaluates the height-dependent cutter radius `R(z_local)` at axial height `z_local` above
    /// the tool tip (`0 <= z_local <= stickout_length_mm`), supporting non-monotonic undercut
    /// profiles such as relieved-neck thread mills:
    /// - `[0, flute_length_mm]`: cutting-tooth / flute envelope (`R_tooth(z)`)
    /// - `(flute_length_mm, neck_length_mm]`: relieved neck (`neck_radius_mm < cutting_radius_mm`)
    /// - `(neck_length_mm, stickout_length_mm]`: cylindrical shank (`shank_radius_mm`)
    ///
    /// Returns `None` outside `[0, stickout_length_mm]` or for non-finite inputs.
    #[inline]
    pub fn cutter_radius_at_z(&self, z_local: f64) -> Option<f64> {
        let max_z = self.stickout_length_mm.max(self.effective_reach_mm());
        self.envelope.cutter_radius_at_z_within(z_local, max_z)
    }

    /// Pushes exact local axial intervals `(dz_lo, dz_hi)` where a [`CutterGeometry::ThreadMill`]
    /// occupies radial distance `r_mm` from the spindle axis (non-monotonic undercut support):
    /// - `r_mm > cutting_radius_mm`: pushes nothing
    /// - `r_mm <= neck_radius_mm`: pushes `(0.0, neck_length_mm)` (core + relieved neck)
    /// - `neck_radius_mm < r_mm <= cutting_radius_mm`: pushes one V-tooth interval per tooth!
    #[inline]
    pub(crate) fn thread_mill_axial_intervals_at_radius(
        &self,
        r_mm: f64,
        out: &mut Vec<(f64, f64)>,
    ) {
        self.envelope
            .push_thread_mill_axial_intervals_at_radius(r_mm, out);
    }

    /// Returns `cot(θ) = 1 / tan(θ)` when the cutter has a conical flute envelope (`Chamfer` / `V-Bit` / `Drill`),
    /// enabling closed-form analytical 3D V-carve stationary-point sweeping in `sweeper.rs`.
    #[inline]
    pub fn conical_slope_cot_theta(&self) -> Option<f64> {
        self.envelope.conical_slope_cot()
    }

    /// Evaluates axial offset `dz >= 0` from the cutter tip `(0, 0, 0)` to the lower cutting
    /// envelope at radial distance `r_mm` from the spindle axis (`None` if `r_mm > cutting_radius_mm`).
    #[inline]
    pub fn tip_to_envelope_dz(&self, r_mm: f64) -> Option<f64> {
        self.envelope.profile_dz_at_radius(r_mm)
    }

    /// Evaluates the exact cutting envelope radius `r(dz) >= 0` at axial height `dz_mm >= 0` above
    /// the cutter tip `(0, 0, 0)` (exact inverse of [`Self::tip_to_envelope_dz`]).
    ///
    /// Returns `None` if `dz_mm < 0.0` (below the cutter tip). For [`CutterGeometry::ThreadMill`],
    /// returns the non-monotonic V-tooth radius in `[0, flute_length_mm]` and `neck_radius_mm`
    /// along the relieved neck `(flute_length_mm, neck_length_mm]`.
    #[inline]
    pub fn radius_at_height_dz(&self, dz_mm: f64) -> Option<f64> {
        self.envelope.radius_at_dz(dz_mm)
    }

    /// Maximum envelope radius among cutting flutes, shank, and collet/holder.
    #[inline]
    pub fn max_assembly_radius_mm(&self) -> f64 {
        self.cutting_radius_mm
            .max(self.shank_radius_mm)
            .max(self.holder_radius_mm)
    }
}

/// Builds a 3D surface-of-revolution mesh for `tool` (`tip = (0, 0, 0)`, spindle up = `+Z`).
///
/// Distinct colors distinguish:
/// 1. **Cutting Flutes / V-Tooth Band** (`0 .. flute_length`)
/// 2. **Relieved Neck & Non-Cutting Shank** (`flute_length .. stickout_length`)
/// 3. **Collet Nut & Holder Cone** (`> stickout_length`)
pub fn build_cutter_assembly_mesh(
    tool: &Tool,
    radial_slices: u32,
    crash_highlight: bool,
) -> SimToolMesh {
    let prof = RadialCutterProfile::from_tool(tool);
    let slices = radial_slices.clamp(12, 96) as usize;

    let (flute_color, shank_color, holder_color) = if crash_highlight {
        (
            [0.98, 0.20, 0.22, 1.0],
            [0.92, 0.34, 0.24, 1.0],
            [0.78, 0.16, 0.18, 1.0],
        )
    } else {
        (
            [0.94, 0.75, 0.24, 1.0], // TiAlN / ZrN gold cutting flutes
            [0.68, 0.73, 0.79, 1.0], // Ground carbide shank / relieved neck
            [0.24, 0.27, 0.32, 1.0], // Anodized collet nut / holder
        )
    };

    let mut flute_pts: Vec<(f64, f64)> = Vec::new();
    let (shank_pts, stickout_z) = if prof.is_thread_mill {
        let push_station = |pts: &mut Vec<(f64, f64)>, r: f64, z: f64| {
            if pts
                .last()
                .is_none_or(|&(pr, pz)| (r - pr).hypot(z - pz) > 1e-5)
            {
                pts.push((r, z));
            }
        };

        let r_cut = prof.cutting_radius_mm;
        let r_neck = prof.neck_radius_mm;
        let flute_top_z = prof.flute_length_mm;
        let (z_start, tooth_step, n_teeth, tan_half) = prof.thread_mill_tooth_band_params();
        let flank_dz = ((r_cut - r_neck).max(0.0) * tan_half).min(0.5 * tooth_step);

        // Flat cutter tip face out to the lower root radius
        push_station(&mut flute_pts, 0.0, 0.0);
        let r_base = prof.thread_mill_tooth_radius_at_z(0.0);
        push_station(&mut flute_pts, r_base, 0.0);
        if z_start > 1e-5 {
            push_station(&mut flute_pts, r_neck, z_start);
        }

        // Crisp periodic 60° (or `thread_angle_deg`) V-tooth rings across `[0, flute_length_mm]`
        for k in 0..n_teeth {
            let z_lo = z_start + (k as f64) * tooth_step;
            let z_crest = z_lo + 0.5 * tooth_step;
            let z_hi = z_lo + tooth_step;
            let r_lo = (r_cut - (z_crest - z_lo) / tan_half).clamp(r_neck, r_cut);
            let r_hi = (r_cut - (z_hi - z_crest) / tan_half).clamp(r_neck, r_cut);
            let z_flank_lo = z_crest - flank_dz;
            let z_flank_hi = z_crest + flank_dz;

            push_station(&mut flute_pts, r_lo, z_lo);
            if z_flank_lo > z_lo + 1e-5 {
                push_station(&mut flute_pts, r_neck, z_flank_lo);
            }
            push_station(&mut flute_pts, r_cut, z_crest);
            if z_flank_hi < z_hi - 1e-5 {
                push_station(&mut flute_pts, r_neck, z_flank_hi);
            }
            push_station(&mut flute_pts, r_hi, z_hi);
        }
        // Step cleanly inward to the relieved neck top shoulder at `z = flute_length_mm`
        push_station(&mut flute_pts, r_neck, flute_top_z);

        // Relieved cylindrical neck (`flute_length .. neck_length`) + smooth transition taper + shank
        let neck_top_z = prof.neck_length_mm.max(flute_top_z + 0.2);
        let stickout_z = prof.stickout_length_mm.max(neck_top_z + 0.5);
        let taper_len = ((prof.shank_radius_mm - r_neck).abs() * 1.2).clamp(0.4, 2.5);
        let taper_end_z = (neck_top_z + taper_len).min(stickout_z);
        let shank = vec![
            (r_neck, flute_top_z),
            (r_neck, neck_top_z),
            (prof.shank_radius_mm, taper_end_z),
            (prof.shank_radius_mm, stickout_z),
        ];
        (shank, stickout_z)
    } else {
        // Build profile stations `(r_mm, z_mm)` for each zone and revolve cleanly with sharp zone normals
        flute_pts.push((0.0, 0.0));
        let tip_steps = 8usize;
        for i in 1..=tip_steps {
            let r = prof.cutting_radius_mm * (i as f64 / tip_steps as f64);
            let dz = prof
                .tip_to_envelope_dz(r)
                .unwrap_or(0.0)
                .min(prof.flute_length_mm);
            if flute_pts
                .last()
                .is_none_or(|&(pr, pz)| (r - pr).hypot(dz - pz) > 1e-4)
            {
                flute_pts.push((r, dz));
            }
        }
        let tip_top_z = flute_pts.last().map_or(0.0, |p| p.1);
        let flute_top_z = prof.flute_length_mm.max(tip_top_z + 0.2);
        flute_pts.push((prof.cutting_radius_mm, flute_top_z));

        let stickout_z = prof.stickout_length_mm.max(flute_top_z + 0.5);
        let shank = vec![
            (prof.cutting_radius_mm, flute_top_z),
            (prof.shank_radius_mm, (flute_top_z + 0.8).min(stickout_z)),
            (prof.shank_radius_mm, stickout_z),
        ];
        (shank, stickout_z)
    };

    let nut_r = prof.holder_radius_mm;
    let body_r = nut_r * 1.25;
    let holder_pts = vec![
        (prof.shank_radius_mm, stickout_z),
        (nut_r, stickout_z + 6.0),
        (nut_r, stickout_z + 18.0),
        (body_r, stickout_z + 28.0),
        (body_r, stickout_z + 42.0),
        (0.0, stickout_z + 42.0),
    ];

    let mut mesh = SimToolMesh::default();
    append_revolved_strip(&mut mesh, &flute_pts, slices, flute_color);
    append_revolved_strip(&mut mesh, &shank_pts, slices, shank_color);
    append_revolved_strip(&mut mesh, &holder_pts, slices, holder_color);
    mesh
}

fn append_revolved_strip(
    mesh: &mut SimToolMesh,
    pts: &[(f64, f64)],
    slices: usize,
    color_rgba: [f32; 4],
) {
    for seg in pts.windows(2) {
        let (r0, z0) = seg[0];
        let (r1, z1) = seg[1];
        let dr = r1 - r0;
        let dz = z1 - z0;
        let seg_len = dr.hypot(dz);
        if seg_len <= 1e-6 {
            continue;
        }
        // Outward normal in `(r, z)` meridional plane for curve walking bottom -> top: `(dz, -dr) / len`
        let nr = (dz / seg_len) as f32;
        let nz = (-dr / seg_len) as f32;

        let base_idx = mesh.vertices.len() as u32;
        for s in 0..=slices {
            let theta = std::f64::consts::TAU * (s as f64 / slices as f64);
            let (sin_t, cos_t) = theta.sin_cos();
            let c = cos_t as f32;
            let si = sin_t as f32;
            let n = [nr * c, nr * si, nz];

            mesh.vertices.push(SimColoredVertex {
                pos_mm: [(r0 * cos_t) as f32, (r0 * sin_t) as f32, z0 as f32],
                normal: n,
                color_rgba,
            });
            mesh.vertices.push(SimColoredVertex {
                pos_mm: [(r1 * cos_t) as f32, (r1 * sin_t) as f32, z1 as f32],
                normal: n,
                color_rgba,
            });
        }

        for s in 0..(slices as u32) {
            let i0 = base_idx + 2 * s;
            let i1 = i0 + 1;
            let i2 = i0 + 2;
            let i3 = i0 + 3;
            if r0 > 1e-5 {
                mesh.indices.extend_from_slice(&[i0, i2, i1]);
            }
            if r1 > 1e-5 {
                mesh.indices.extend_from_slice(&[i1, i2, i3]);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sim_colored_vertex_has_exact_40_byte_pod_gpu_layout() {
        assert_eq!(std::mem::size_of::<SimColoredVertex>(), 40);
        assert_eq!(std::mem::align_of::<SimColoredVertex>(), 4);
        assert_eq!(std::mem::offset_of!(SimColoredVertex, pos_mm), 0);
        assert_eq!(std::mem::offset_of!(SimColoredVertex, normal), 12);
        assert_eq!(std::mem::offset_of!(SimColoredVertex, color_rgba), 24);

        let verts = [
            SimColoredVertex {
                pos_mm: [1.0, 2.0, 3.0],
                normal: [0.0, 0.0, 1.0],
                color_rgba: [0.5, 0.6, 0.7, 1.0],
            },
            bytemuck::Zeroable::zeroed(),
        ];
        let bytes: &[u8] = bytemuck::cast_slice(&verts);
        assert_eq!(bytes.len(), 80);
        let roundtrip: &[SimColoredVertex] = bytemuck::cast_slice(bytes);
        assert_eq!(roundtrip, &verts);
    }
}
