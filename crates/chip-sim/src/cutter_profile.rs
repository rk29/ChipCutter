use chip_tools::{CutterGeometry, Tool};
use serde::{Deserialize, Serialize};

/// Material zone of a 3D revolved cutter assembly mesh.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ToolMeshZone {
    CuttingFlutes,
    Shank,
    ColletHolder,
}

/// UI-agnostic 3D lit vertex (`pos_mm`, `normal`, `color_rgba`) ready for direct GPU upload.
#[derive(Debug, Clone, Copy, PartialEq)]
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

#[derive(Debug, Clone, Copy)]
enum TipEnvelopeKind {
    Flat,
    BullNose {
        flat_r_mm: f64,
        corner_r_mm: f64,
    },
    Ball {
        ball_r_mm: f64,
    },
    Chamfer {
        tip_r_mm: f64,
        tan_half_angle: f64,
    },
    Drill {
        tan_half_angle: f64,
    },
    Face {
        flat_r_mm: f64,
        bottom_r_mm: f64,
        corner_r_mm: f64,
    },
}

/// Exact `f64` radial profile `P(r)` and shank/holder envelope of a [`Tool`] (`06-simulation.md`).
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
    kind: TipEnvelopeKind,
}

impl RadialCutterProfile {
    /// Builds the exact radial height function `P(r)` and holder clearance envelope for `tool`.
    pub fn from_tool(tool: &Tool) -> Self {
        let flute_length_mm = tool.geometry.flute_length_mm().max(0.1);
        let overall_length_mm = tool.geometry.overall_length_mm().max(flute_length_mm);
        let stickout_length_mm = tool.stickout_length_mm.max(flute_length_mm);
        let shank_radius_mm = (0.5 * tool.geometry.shank_diameter_mm()).max(0.25);

        let (cutting_radius_mm, is_center_cutting, max_safe_plunge_depth_mm, kind) =
            match tool.geometry {
                CutterGeometry::FlatEndmill { diameter_mm, .. } => {
                    let r = (0.5 * diameter_mm).max(0.05);
                    (
                        r,
                        true,
                        (1.5 * diameter_mm).min(flute_length_mm),
                        TipEnvelopeKind::Flat,
                    )
                }
                CutterGeometry::BullNoseEndmill {
                    diameter_mm,
                    corner_radius_mm,
                    ..
                } => {
                    let r = (0.5 * diameter_mm).max(0.05);
                    let rc = corner_radius_mm.clamp(0.0, r);
                    let flat_r = (r - rc).max(0.0);
                    (
                        r,
                        true,
                        (1.5 * diameter_mm).min(flute_length_mm),
                        TipEnvelopeKind::BullNose {
                            flat_r_mm: flat_r,
                            corner_r_mm: rc,
                        },
                    )
                }
                CutterGeometry::BallEndmill { diameter_mm, .. } => {
                    let r = (0.5 * diameter_mm).max(0.05);
                    (
                        r,
                        true,
                        (1.5 * diameter_mm).min(flute_length_mm),
                        TipEnvelopeKind::Ball { ball_r_mm: r },
                    )
                }
                CutterGeometry::ChamferMill {
                    diameter_mm,
                    tip_diameter_mm,
                    included_angle_deg,
                    ..
                } => {
                    let r = (0.5 * diameter_mm).max(0.05);
                    let tip_r = (0.5 * tip_diameter_mm).clamp(0.0, r);
                    let half_rad = (0.5 * included_angle_deg).clamp(1.0, 89.5).to_radians();
                    (
                        r,
                        true,
                        flute_length_mm,
                        TipEnvelopeKind::Chamfer {
                            tip_r_mm: tip_r,
                            tan_half_angle: half_rad.tan().max(1e-6),
                        },
                    )
                }
                CutterGeometry::TwistDrill {
                    diameter_mm,
                    point_angle_deg,
                    ..
                } => {
                    let r = (0.5 * diameter_mm).max(0.05);
                    let half_rad = (0.5 * point_angle_deg).clamp(5.0, 89.5).to_radians();
                    (
                        r,
                        true,
                        flute_length_mm,
                        TipEnvelopeKind::Drill {
                            tan_half_angle: half_rad.tan().max(1e-6),
                        },
                    )
                }
                CutterGeometry::FaceMill {
                    cutting_diameter_mm,
                    outside_diameter_mm,
                    corner_radius_mm,
                    ..
                } => {
                    let bottom_r = (0.5 * cutting_diameter_mm).max(0.1);
                    let outer_r = (0.5 * outside_diameter_mm).max(bottom_r);
                    let rc = corner_radius_mm.clamp(0.0, bottom_r);
                    let flat_r = (bottom_r - rc).max(0.0);
                    (
                        outer_r,
                        false,
                        0.0,
                        TipEnvelopeKind::Face {
                            flat_r_mm: flat_r,
                            bottom_r_mm: bottom_r,
                            corner_r_mm: rc,
                        },
                    )
                }
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
            kind,
        }
    }

    /// Returns `cot(θ) = 1 / tan(θ)` when the cutter has a conical flute envelope (`Chamfer` / `V-Bit` / `Drill`),
    /// enabling closed-form analytical 3D V-carve stationary-point sweeping in `sweeper.rs`.
    #[inline]
    pub fn conical_slope_cot_theta(&self) -> Option<f64> {
        match self.kind {
            TipEnvelopeKind::Chamfer { tan_half_angle, .. }
            | TipEnvelopeKind::Drill { tan_half_angle } => Some(1.0 / tan_half_angle.max(1e-6)),
            _ => None,
        }
    }

    /// Evaluates axial offset `dz >= 0` from the cutter tip `(0, 0, 0)` to the lower cutting
    /// envelope at radial distance `r_mm` from the spindle axis (`None` if `r_mm > cutting_radius_mm`).
    #[inline]
    pub fn tip_to_envelope_dz(&self, r_mm: f64) -> Option<f64> {
        if r_mm < 0.0 || r_mm > self.cutting_radius_mm + 1e-9 {
            return None;
        }
        let r = r_mm.min(self.cutting_radius_mm);
        let dz = match self.kind {
            TipEnvelopeKind::Flat => 0.0,
            TipEnvelopeKind::BullNose {
                flat_r_mm,
                corner_r_mm,
            } => {
                if r <= flat_r_mm || corner_r_mm <= 1e-9 {
                    0.0
                } else {
                    let dr = (r - flat_r_mm).min(corner_r_mm);
                    corner_r_mm - (corner_r_mm * corner_r_mm - dr * dr).max(0.0).sqrt()
                }
            }
            TipEnvelopeKind::Ball { ball_r_mm } => {
                let rc = r.min(ball_r_mm);
                ball_r_mm - (ball_r_mm * ball_r_mm - rc * rc).max(0.0).sqrt()
            }
            TipEnvelopeKind::Chamfer {
                tip_r_mm,
                tan_half_angle,
            } => {
                if r <= tip_r_mm {
                    0.0
                } else {
                    (r - tip_r_mm) / tan_half_angle
                }
            }
            TipEnvelopeKind::Drill { tan_half_angle } => r / tan_half_angle,
            TipEnvelopeKind::Face {
                flat_r_mm,
                bottom_r_mm,
                corner_r_mm,
            } => {
                if r <= flat_r_mm {
                    0.0
                } else if r <= bottom_r_mm {
                    if corner_r_mm <= 1e-9 {
                        0.0
                    } else {
                        let dr = (r - flat_r_mm).min(corner_r_mm);
                        corner_r_mm - (corner_r_mm * corner_r_mm - dr * dr).max(0.0).sqrt()
                    }
                } else {
                    corner_r_mm + (r - bottom_r_mm)
                }
            }
        };
        Some(dz)
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
/// 1. **Cutting Flutes** (`0 .. flute_length`)
/// 2. **Non-Cutting Shank** (`flute_length .. stickout_length`)
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
            [0.68, 0.73, 0.79, 1.0], // Ground carbide shank
            [0.24, 0.27, 0.32, 1.0], // Anodized collet nut / holder
        )
    };

    // Build profile stations `(r_mm, z_mm)` for each zone and revolve cleanly with sharp zone normals
    let mut flute_pts: Vec<(f64, f64)> = Vec::new();
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
    let shank_pts = vec![
        (prof.cutting_radius_mm, flute_top_z),
        (prof.shank_radius_mm, (flute_top_z + 0.8).min(stickout_z)),
        (prof.shank_radius_mm, stickout_z),
    ];

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
