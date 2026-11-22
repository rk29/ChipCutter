use chip_math::{Aabb2, Aabb3, ArcPolyline2D, DVec2, DVec3};
use chip_tools::{AnalyticCutterEnvelope, CutterEnvelopeShape, CutterGeometry, Tool};
use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Sentinel operation tag marking unmachined raw stock across Heightfield, Tri-Dexel, and
/// Cylindrical stock buffers (WGSL-compatible `u32`).
pub const UNCUT_OP_IDX: u32 = u32::MAX;

/// Principal Cartesian coordinate axis descriptor used to parameterize axis-symmetric
/// Tri-Dexel ray planes, cylinder stock specs, and rotary axes without X/Y/Z code duplication.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Axis3 {
    X = 0,
    Y = 1,
    #[default]
    Z = 2,
}

impl Axis3 {
    pub const ALL: [Self; 3] = [Self::X, Self::Y, Self::Z];

    #[inline]
    pub const fn index(self) -> usize {
        self as usize
    }

    /// First grid axis (`u`) of the orthogonal dexel plane whose rays march along `self`.
    /// - `X` -> `Y`
    /// - `Y` -> `X`
    /// - `Z` -> `X`
    #[inline]
    pub const fn u_axis(self) -> Self {
        match self {
            Self::X => Self::Y,
            Self::Y | Self::Z => Self::X,
        }
    }

    /// Second grid axis (`v`) of the orthogonal dexel plane whose rays march along `self`.
    /// - `X` -> `Z`
    /// - `Y` -> `Z`
    /// - `Z` -> `Y`
    #[inline]
    pub const fn v_axis(self) -> Self {
        match self {
            Self::X | Self::Y => Self::Z,
            Self::Z => Self::Y,
        }
    }

    /// Ray marching axis (`w`, equal to `self`).
    #[inline]
    pub const fn w_axis(self) -> Self {
        self
    }

    /// The two coordinate axes orthogonal to `self` in canonical `(u_axis, v_axis)` order.
    #[inline]
    pub const fn ortho_axes(self) -> [Self; 2] {
        [self.u_axis(), self.v_axis()]
    }

    #[inline]
    pub const fn get(self, p: DVec3) -> f64 {
        match self {
            Self::X => p.x,
            Self::Y => p.y,
            Self::Z => p.z,
        }
    }

    #[inline]
    pub const fn get_usize(self, dims: [usize; 3]) -> usize {
        dims[self.index()]
    }

    /// Reconstructs a 3D point `(x, y, z)` from plane-local `(u, v, w)` coordinates.
    #[inline]
    pub const fn from_uvw(self, u: f64, v: f64, w: f64) -> DVec3 {
        match self {
            Self::X => DVec3::new(w, u, v),
            Self::Y => DVec3::new(u, w, v),
            Self::Z => DVec3::new(u, v, w),
        }
    }

    /// Reconstructs a 3D lattice index `[ix, iy, iz]` from plane-local `(iu, iv, iw)` indices.
    #[inline]
    pub const fn from_uvw_usize(self, u: usize, v: usize, w: usize) -> [usize; 3] {
        match self {
            Self::X => [w, u, v],
            Self::Y => [u, w, v],
            Self::Z => [u, v, w],
        }
    }

    /// Returns `p` shifted by `delta` along `self`.
    #[inline]
    pub fn add_scaled(self, mut p: DVec3, delta: f64) -> DVec3 {
        match self {
            Self::X => p.x += delta,
            Self::Y => p.y += delta,
            Self::Z => p.z += delta,
        }
        p
    }

    #[inline]
    pub const fn unit_vec(self) -> DVec3 {
        match self {
            Self::X => DVec3::X,
            Self::Y => DVec3::Y,
            Self::Z => DVec3::Z,
        }
    }

    #[inline]
    pub const fn unit_vec_f32(self) -> [f32; 3] {
        match self {
            Self::X => [1.0, 0.0, 0.0],
            Self::Y => [0.0, 1.0, 0.0],
            Self::Z => [0.0, 0.0, 1.0],
        }
    }

    /// Returns `true` for vertical wall axes (`X`, `Y`), `false` for top/bottom (`Z`).
    #[inline]
    pub const fn is_side_axis(self) -> bool {
        !matches!(self, Self::Z)
    }

    /// Corner `(du, dv)` winding offsets for a positive-`+w` outward quad on the dual lattice
    /// (compensating for `(X × Z) = -Y` handedness on the `Y` plane).
    #[inline]
    pub const fn quad_corner_offsets(self) -> [(usize, usize); 4] {
        match self {
            Self::X | Self::Z => [(0, 0), (1, 0), (1, 1), (0, 1)],
            Self::Y => [(0, 0), (0, 1), (1, 1), (1, 0)],
        }
    }
}

/// Pure geometric raw stock shape specification in Setup WCS millimeters (zero `chip-cam` dependency).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum StockShapeSpec {
    Box {
        aabb_wcs: Aabb3,
    },
    Cylinder {
        axis: Axis3,
        center_wcs: DVec3,
        radius_mm: f64,
        min_axial_mm: f64,
        max_axial_mm: f64,
        aabb_wcs: Aabb3,
    },
}

impl StockShapeSpec {
    #[inline]
    pub const fn aabb_wcs(&self) -> Aabb3 {
        match *self {
            Self::Box { aabb_wcs } | Self::Cylinder { aabb_wcs, .. } => aabb_wcs,
        }
    }
}

/// Pure geometric 4th-axis rotary coordinate frame & chuck keep-out specification for
/// [`crate::cylindrical::CylindricalStock`] (zero `chip-cam` dependency).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RotaryStockSpec {
    pub axis: Axis3,
    /// `+1.0` for standard ISO right-hand CCW, `-1.0` for inverted CW.
    pub direction_sign: f64,
    pub axis_letter: char,
    /// Point lying on the rotary centerline in Setup WCS (`mm`).
    pub center_wcs: DVec3,
    /// Axial span `[stock_u_min, stock_u_max]` along `axis.unit_vec()`.
    pub stock_u_min: f64,
    pub stock_u_max: f64,
    /// Maximum outer radius (`mm`) of raw stock relative to `center_wcs`.
    pub stock_outer_radius_mm: f64,
    pub chuck_u_min: f64,
    pub chuck_u_max: f64,
    pub chuck_radius_mm: f64,
}

impl RotaryStockSpec {
    /// Right-handed orthonormal basis `(e1, e2, u_axis)` such that `e1 × e2 = u_axis`.
    /// For `X` (`A`) and `Y` (`B`), `e1 = +Z` so `θ = 0°` points straight up toward the spindle.
    #[inline]
    pub const fn basis(&self) -> (DVec3, DVec3, DVec3) {
        match self.axis {
            Axis3::X => (
                DVec3::new(0.0, 0.0, 1.0),
                DVec3::new(0.0, -1.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
            ),
            Axis3::Y => (
                DVec3::new(0.0, 0.0, 1.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            ),
            Axis3::Z => (
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
                DVec3::new(0.0, 0.0, 1.0),
            ),
        }
    }

    #[inline]
    pub const fn axis_unit(&self) -> DVec3 {
        self.basis().2
    }

    /// Rotates a WCS point around this rotary centerline by `angle_deg` honoring `direction_sign`.
    #[inline]
    pub fn rotate_point_wcs(&self, pt_wcs: DVec3, angle_deg: f64) -> DVec3 {
        let (e1, e2, u_vec) = self.basis();
        let rel = pt_wcs - self.center_wcs;
        let u = rel.dot(u_vec);
        let c1 = rel.dot(e1);
        let c2 = rel.dot(e2);
        let a = (angle_deg * self.direction_sign).to_radians();
        let (sin_a, cos_a) = a.sin_cos();
        let r1 = c1 * cos_a - c2 * sin_a;
        let r2 = c1 * sin_a + c2 * cos_a;
        self.center_wcs + u_vec * u + e1 * r1 + e2 * r2
    }

    /// Rotates a free direction vector around this rotary axis by `angle_deg` honoring `direction_sign`.
    #[inline]
    pub fn rotate_vector_wcs(&self, vec: DVec3, angle_deg: f64) -> DVec3 {
        let (e1, e2, u_vec) = self.basis();
        let u = vec.dot(u_vec);
        let c1 = vec.dot(e1);
        let c2 = vec.dot(e2);
        let a = (angle_deg * self.direction_sign).to_radians();
        let (sin_a, cos_a) = a.sin_cos();
        let r1 = c1 * cos_a - c2 * sin_a;
        let r2 = c1 * sin_a + c2 * cos_a;
        (u_vec * u + e1 * r1 + e2 * r2).normalize_or_zero()
    }

    /// Transforms a WCS point into cylindrical coordinates `(u_axial_mm, theta_rad, radius_mm)`.
    #[inline]
    pub fn wcs_to_cylindrical(&self, pt_wcs: DVec3) -> (f64, f64, f64) {
        let (e1, e2, u_vec) = self.basis();
        let u_axial = pt_wcs.dot(u_vec);
        let rel = pt_wcs - self.center_wcs;
        let c1 = rel.dot(e1);
        let c2 = rel.dot(e2) * self.direction_sign;
        let r = c1.hypot(c2);
        let theta_rad = c2.atan2(c1);
        (u_axial, theta_rad, r)
    }

    /// Converts cylindrical coordinates `(u_axial_mm, theta_rad, radius_mm)` back into Setup WCS.
    #[inline]
    pub fn cylindrical_to_wcs(&self, u_axial: f64, theta_rad: f64, radius_mm: f64) -> DVec3 {
        let (e1, e2, u_vec) = self.basis();
        let (sin_t, cos_t) = theta_rad.sin_cos();
        let c1 = radius_mm * cos_t;
        let c2 = radius_mm * sin_t * self.direction_sign;
        let u_center = self.center_wcs.dot(u_vec);
        self.center_wcs + u_vec * (u_axial - u_center) + e1 * c1 + e2 * c2
    }

    /// Axial midpoint of the stock along the rotary axis in Setup WCS.
    #[inline]
    pub fn stock_center_wcs(&self) -> DVec3 {
        let u_mid = 0.5 * (self.stock_u_min + self.stock_u_max);
        let u_vec = self.axis_unit();
        self.center_wcs + u_vec * (u_mid - self.center_wcs.dot(u_vec))
    }
}

/// Errors raised during stock grid construction or simulation.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum SimError {
    #[error("simulation cancelled")]
    Cancelled,
    #[error("resolved stock bounding box is invalid or degenerate")]
    InvalidStockBounds,
}

/// Selectable compute backend preference for volumetric stock simulation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SimComputeBackendChoice {
    /// Automatically prefer `WgpuComputeBackend` when compiled with `gpu-compute` and an adapter
    /// is available, transparently falling back to `CpuExactBackend` otherwise.
    #[default]
    Auto,
    /// Prefer `WgpuComputeBackend` (falling back cleanly to `CpuExactBackend` if unavailable).
    Gpu,
    /// Force exact CPU + Rayon volumetric simulation (`CpuExactBackend`).
    Cpu,
}

impl SimComputeBackendChoice {
    pub const ALL: [Self; 3] = [Self::Auto, Self::Gpu, Self::Cpu];

    pub const fn label(self) -> &'static str {
        match self {
            Self::Auto => "Auto (GPU / CPU)",
            Self::Gpu => "GPU Compute (wgpu)",
            Self::Cpu => "CPU Exact (Rayon)",
        }
    }
}

/// Metadata identifying which concrete simulation backend executed a simulation run.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ActiveSimBackendInfo {
    pub is_gpu: bool,
    pub adapter_name: String,
    pub backend_api: String,
}

impl ActiveSimBackendInfo {
    pub fn cpu() -> Self {
        Self {
            is_gpu: false,
            adapter_name: "CPU Exact (Rayon)".to_string(),
            backend_api: "CPU".to_string(),
        }
    }
}

impl Default for ActiveSimBackendInfo {
    fn default() -> Self {
        Self::cpu()
    }
}

/// Selectable volumetric stock simulation engine (`Phase 11`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SimEngineKind {
    /// Fast 2.5D Z-Dexel heightmap (or polar cylindrical grid on 4th-axis setups).
    #[default]
    Heightmap,
    /// Full 3D 3-axis orthogonal Tri-Dexel volumetric solid (accurate vertical walls & undercuts).
    TriDexel,
}

impl SimEngineKind {
    pub const ALL: [Self; 2] = [Self::Heightmap, Self::TriDexel];

    pub const fn label(self) -> &'static str {
        match self {
            Self::Heightmap => "Heightmap (Fast)",
            Self::TriDexel => "Tri-Dexel (Volumetric 3D)",
        }
    }

    pub const fn short_slug(self) -> &'static str {
        match self {
            Self::Heightmap => "heightmap",
            Self::TriDexel => "tri-dexel",
        }
    }

    pub const fn description(self) -> &'static str {
        match self {
            Self::Heightmap => {
                "Fast 2.5D Z-dexel heightfield (or polar cylindrical grid on 4th-axis setups). Best for rapid interactive scrubbing and quick crash checks; vertical walls snap to XY columns and overhangs/undercuts are not modeled."
            }
            Self::TriDexel => {
                "Full 3D volumetric stock using 3 orthogonal ray grids (X, Y, Z) with exact sub-voxel endpoints. Accurately models vertical walls, steep reliefs, undercuts, and 3D error-volume/RMS metrology (slower than Heightmap)."
            }
        }
    }
}

/// Configurable volumetric grid resolution (`06-simulation.md` §3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SimResolution {
    /// `~512 × 512` heightfield / `160³` Tri-Dexel (instant interactive updates).
    #[default]
    Standard,
    /// `~1024 × 1024` heightfield / `288³` Tri-Dexel (high-precision verification).
    Fine,
    /// `~2048 × 2048` heightfield / `448³` Tri-Dexel (ultra-fine inspection).
    Ultra,
}

impl SimResolution {
    pub const ALL: [Self; 3] = [Self::Standard, Self::Fine, Self::Ultra];

    pub const fn max_grid_dim(self) -> usize {
        match self {
            Self::Standard => 512,
            Self::Fine => 1024,
            Self::Ultra => 2048,
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            Self::Standard => "Standard (512)",
            Self::Fine => "Fine (1024)",
            Self::Ultra => "Ultra (2048)",
        }
    }

    /// Engine-aware display label reflecting the actual max-axis grid dimension used by `engine`.
    pub const fn label_for_engine(self, engine: SimEngineKind) -> &'static str {
        match (engine, self) {
            (SimEngineKind::Heightmap, Self::Standard) => "Standard (512)",
            (SimEngineKind::Heightmap, Self::Fine) => "Fine (1024)",
            (SimEngineKind::Heightmap, Self::Ultra) => "Ultra (2048)",
            (SimEngineKind::TriDexel, Self::Standard) => "Standard (160)",
            (SimEngineKind::TriDexel, Self::Fine) => "Fine (288)",
            (SimEngineKind::TriDexel, Self::Ultra) => "Ultra (448)",
        }
    }

    /// Human-readable tooltip explaining what this detail level means for `engine`.
    pub const fn description_for_engine(self, engine: SimEngineKind) -> &'static str {
        match (engine, self) {
            (SimEngineKind::Heightmap, Self::Standard) => {
                "Up to 512 cells along the longest XY stock axis. Fast interactive scrubbing and quick crash checks."
            }
            (SimEngineKind::Heightmap, Self::Fine) => {
                "Up to 1024 cells along the longest XY stock axis (4x cells). Crisper small features and finer surface metrology."
            }
            (SimEngineKind::Heightmap, Self::Ultra) => {
                "Up to 2048 cells along the longest XY stock axis (16x cells). Highest 2.5D detail for tiny engravings and fine finishes (uses more RAM/CPU)."
            }
            (SimEngineKind::TriDexel, Self::Standard) => {
                "Up to 160 isotropic cells along the longest 3D axis across X, Y, and Z ray planes. Fast 3D volumetric verification and metrology."
            }
            (SimEngineKind::TriDexel, Self::Fine) => {
                "Up to 288 isotropic cells along the longest 3D axis across X, Y, and Z ray planes (~3.2x rays). High-precision 3D walls, scallops, and error-volume benchmarking."
            }
            (SimEngineKind::TriDexel, Self::Ultra) => {
                "Up to 448 isotropic cells along the longest 3D axis across X, Y, and Z ray planes (~7.8x rays). Ultra-fine 3D inspection and metrology (slower, higher memory use)."
            }
        }
    }
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

/// Triangulated 3D machined stock mesh (`pos_mm`, `normal`, `color_rgba` in Setup WCS).
#[derive(Debug, Clone, Default)]
pub struct SimStockMesh {
    pub vertices: Vec<SimColoredVertex>,
    pub indices: Vec<u32>,
}

/// Tracks the exact timeline position `(seg_idx, seg_u)` of an incrementally synchronized
/// [`crate::SimulatedStock`]: all segments `0..seg_idx` are fully swept, plus the prefix
/// `[0.0 .. seg_u]` of segment `seg_idx`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StockPlaybackCursor {
    pub seg_idx: usize,
    pub seg_u: f64,
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
    pub fn thread_mill_tooth_band_params(&self) -> (f64, f64, u32, f64) {
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
    /// profiles such as relieved-neck thread mills.
    #[inline]
    pub fn cutter_radius_at_z(&self, z_local: f64) -> Option<f64> {
        let max_z = self.stickout_length_mm.max(self.effective_reach_mm());
        self.envelope.cutter_radius_at_z_within(z_local, max_z)
    }

    /// Pushes exact local axial intervals `(dz_lo, dz_hi)` where a [`CutterGeometry::ThreadMill`]
    /// occupies radial distance `r_mm` from the spindle axis.
    #[inline]
    pub fn thread_mill_axial_intervals_at_radius(&self, r_mm: f64, out: &mut Vec<(f64, f64)>) {
        self.envelope
            .push_thread_mill_axial_intervals_at_radius(r_mm, out);
    }

    /// Returns `cot(θ) = 1 / tan(θ)` when the cutter has a conical flute envelope (`Chamfer` / `V-Bit` / `Drill`).
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

/// Per-operation cutter descriptor passed to [`crate::SimStockBackend::sweep_batch`].
#[derive(Debug, Clone, Copy)]
pub struct SweepToolSpec {
    pub profile: RadialCutterProfile,
    pub is_flat_endmill: bool,
}

/// Verification tolerances governing Rules 1–4 inside a volumetric stock sweep.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct StockVerifyTolerances {
    pub gouge_tol_mm: f64,
    pub rapid_crash_tol_mm: f64,
    pub radial_wall_skin_mm: f64,
}

impl Default for StockVerifyTolerances {
    fn default() -> Self {
        Self {
            gouge_tol_mm: 0.02,
            rapid_crash_tol_mm: 0.02,
            radial_wall_skin_mm: 0.018,
        }
    }
}

/// Pure geometric sweep primitive consumed by all three stock engines (zero `chip-cam` dependency).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SweepMotion {
    /// Linear 3D chord (`p0 -> p1`) with optional simultaneous 4th-axis rotary interpolation.
    Linear {
        p0: DVec3,
        p1: DVec3,
        rotary_deg_0: f64,
        rotary_deg_1: f64,
    },
    /// Planar constant-Z circular arc in XY (exact closed-form distance in Heightfield;
    /// arc-sampled in Tri-Dexel and Cylindrical).
    PlanarArcXY {
        start: DVec3,
        end: DVec3,
        center_xy: DVec2,
        radius_mm: f64,
        a0_rad: f64,
        signed_sweep_rad: f64,
        rotary_deg_0: f64,
        rotary_deg_1: f64,
    },
}

impl SweepMotion {
    #[inline]
    pub fn length_mm(&self) -> f64 {
        match *self {
            Self::Linear { p0, p1, .. } => (p1 - p0).length(),
            Self::PlanarArcXY {
                radius_mm,
                signed_sweep_rad,
                ..
            } => radius_mm.abs() * signed_sweep_rad.abs(),
        }
    }

    #[inline]
    pub fn rotary_delta_deg(&self) -> f64 {
        match *self {
            Self::Linear {
                rotary_deg_0,
                rotary_deg_1,
                ..
            }
            | Self::PlanarArcXY {
                rotary_deg_0,
                rotary_deg_1,
                ..
            } => (rotary_deg_1 - rotary_deg_0).abs(),
        }
    }

    #[inline]
    pub fn sample_point(&self, t: f64) -> DVec3 {
        let u = t.clamp(0.0, 1.0);
        match *self {
            Self::Linear { p0, p1, .. } => p0.lerp(p1, u),
            Self::PlanarArcXY {
                start,
                end,
                center_xy,
                radius_mm,
                a0_rad,
                signed_sweep_rad,
                ..
            } => {
                if u <= 0.0 {
                    start
                } else if u >= 1.0 {
                    end
                } else {
                    let ang = a0_rad + signed_sweep_rad * u;
                    let (sin_a, cos_a) = ang.sin_cos();
                    DVec3::new(
                        center_xy.x + radius_mm * cos_a,
                        center_xy.y + radius_mm * sin_a,
                        start.z + (end.z - start.z) * u,
                    )
                }
            }
        }
    }

    #[inline]
    pub fn sample_rotary_deg(&self, t: f64) -> f64 {
        let u = t.clamp(0.0, 1.0);
        match *self {
            Self::Linear {
                rotary_deg_0,
                rotary_deg_1,
                ..
            }
            | Self::PlanarArcXY {
                rotary_deg_0,
                rotary_deg_1,
                ..
            } => rotary_deg_0 + (rotary_deg_1 - rotary_deg_0) * u,
        }
    }
}

/// One motion chord inside a batch submitted to [`crate::SimStockBackend::sweep_batch`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SweepChord {
    pub motion: SweepMotion,
    pub op_idx: u32,
    pub is_rapid: bool,
    pub profile_idx: usize,
    pub verify_tol: StockVerifyTolerances,
}

/// Per-span / per-row worst-case Rule 1–4 violation accumulator shared by all 3 stock engines.
#[derive(Debug, Default, Clone, Copy)]
pub struct SpanRuleViolations {
    pub rapid: Option<(f64, DVec3)>,
    pub gouge: Option<(f64, DVec3)>,
    pub flute: Option<(f64, DVec3)>,
    pub neck: Option<(f64, DVec3)>,
    pub holder: Option<(f64, DVec3)>,
}

impl SpanRuleViolations {
    #[inline]
    pub fn update_slot(slot: &mut Option<(f64, DVec3)>, depth_mm: f64, loc_wcs: DVec3) {
        if slot.is_none_or(|(cur, _)| depth_mm > cur) {
            *slot = Some((depth_mm, loc_wcs));
        }
    }

    #[inline]
    pub fn record_rapid(&mut self, depth_mm: f64, loc_wcs: DVec3) {
        Self::update_slot(&mut self.rapid, depth_mm, loc_wcs);
    }

    #[inline]
    pub fn record_gouge(&mut self, depth_mm: f64, loc_wcs: DVec3) {
        Self::update_slot(&mut self.gouge, depth_mm, loc_wcs);
    }

    #[inline]
    pub fn record_flute(&mut self, depth_mm: f64, loc_wcs: DVec3) {
        Self::update_slot(&mut self.flute, depth_mm, loc_wcs);
    }

    #[inline]
    pub fn record_neck(&mut self, depth_mm: f64, loc_wcs: DVec3) {
        Self::update_slot(&mut self.neck, depth_mm, loc_wcs);
    }

    #[inline]
    pub fn record_holder(&mut self, depth_mm: f64, loc_wcs: DVec3) {
        Self::update_slot(&mut self.holder, depth_mm, loc_wcs);
    }

    #[inline]
    pub fn merge(&mut self, other: Self) {
        if let Some((d, p)) = other.rapid {
            self.record_rapid(d, p);
        }
        if let Some((d, p)) = other.gouge {
            self.record_gouge(d, p);
        }
        if let Some((d, p)) = other.flute {
            self.record_flute(d, p);
        }
        if let Some((d, p)) = other.neck {
            self.record_neck(d, p);
        }
        if let Some((d, p)) = other.holder {
            self.record_holder(d, p);
        }
    }
}

/// Output returned by [`crate::SimStockBackend::sweep_batch`].
#[derive(Debug, Default, Clone, Copy)]
pub struct BatchSweepOutput {
    pub mutated: bool,
    pub violations: SpanRuleViolations,
}

/// Exact analytic horizontal planar B-Rep face (`±Z`) used by Heightfield and Tri-Dexel target
/// rasterizers so circular holes, counterbores, and bosses suffer zero polygonal chord sag.
#[derive(Debug, Clone)]
pub struct AnalyticHorizFace {
    pub outer: ArcPolyline2D,
    pub inners: Vec<ArcPolyline2D>,
    pub aabb: Aabb2,
    pub z_wcs: f64,
    /// `+1` for downward-facing (`n_wcs.z < 0`, entering solid along `+Z`),
    /// `-1` for upward-facing (`n_wcs.z > 0`, exiting solid along `+Z`).
    pub dir_sign: i8,
}

/// Target surface triangle in Setup WCS with optional exact analytic plane `(origin_wcs, normal_wcs)`
/// for sloped planar/chamfer B-Rep faces.
#[derive(Debug, Clone, Copy)]
pub struct TargetTriangle {
    pub v: [DVec3; 3],
    pub analytic_plane: Option<(DVec3, DVec3)>,
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

    #[test]
    fn axis3_coordinate_permutations_are_self_consistent() {
        let p = DVec3::new(11.0, 22.0, 33.0);
        for axis in Axis3::ALL {
            let u = axis.u_axis().get(p);
            let v = axis.v_axis().get(p);
            let w = axis.w_axis().get(p);
            assert_eq!(axis.from_uvw(u, v, w), p);
            let dims = [10usize, 20, 30];
            assert_eq!(
                axis.from_uvw_usize(
                    axis.u_axis().get_usize(dims),
                    axis.v_axis().get_usize(dims),
                    axis.w_axis().get_usize(dims),
                ),
                dims
            );
        }
    }
}
