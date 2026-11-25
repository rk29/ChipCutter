//! Exact `f64` cutting-mechanics, flute-aware stepped cantilever deflection/stress,
//! spindle power-curve derating, and concave internal-bore engagement physics (`chip-tools`).

use serde::{Deserialize, Serialize};

use crate::{
    feeds_speeds::{chipload_from_feed, surface_speed_from_rpm},
    machine::MachineSpindleSpec,
    materials::WorkpieceMaterialProfile,
    model::{CutterGeometry, Tool},
};

/// Solid micro-grain tungsten carbide elastic modulus (`600 GPa = 600,000 N/mm^2`).
pub const CARBIDE_YOUNGS_MODULUS_MPA: f64 = 600_000.0;

/// High-speed steel (HSS) elastic modulus (`210 GPa = 210,000 N/mm^2`).
pub const HSS_YOUNGS_MODULUS_MPA: f64 = 210_000.0;

/// Typical sub-micron carbide transverse rupture strength (`2,800 MPa`).
pub const CARBIDE_TRS_MPA: f64 = 2_800.0;

/// Typical HSS bending yield / rupture strength (`1,600 MPa`).
pub const HSS_TRS_MPA: f64 = 1_600.0;

/// Default mechanical/electrical spindle drive efficiency when omitted on [`MachineSpindleSpec`].
pub const DEFAULT_SPINDLE_EFFICIENCY: f64 = 0.85;

/// Cyclic milling fatigue knockdown applied to static TRS on continuous cuts.
const CONTINUOUS_FATIGUE_KNOCKDOWN: f64 = 0.85;

/// Combined fatigue + flute-impact knockdown applied to static TRS on interrupted cuts (`z_c < 1`).
const INTERRUPTED_IMPACT_KNOCKDOWN: f64 = 0.65;

/// Cutter substrate material determining elastic modulus and bending rupture strength.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ToolSubstrate {
    #[default]
    Carbide,
    Hss,
}

impl ToolSubstrate {
    #[inline]
    pub const fn youngs_modulus_mpa(self) -> f64 {
        match self {
            Self::Carbide => CARBIDE_YOUNGS_MODULUS_MPA,
            Self::Hss => HSS_YOUNGS_MODULUS_MPA,
        }
    }

    #[inline]
    pub const fn transverse_rupture_strength_mpa(self) -> f64 {
        match self {
            Self::Carbide => CARBIDE_TRS_MPA,
            Self::Hss => HSS_TRS_MPA,
        }
    }
}

/// Stepped cantilever deflection and extreme-fiber bending stress result.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DeflectionResult {
    /// Lateral tip deflection in millimeters (`mm`).
    pub tip_deflection_mm: f64,
    /// Lateral tip deflection in micrometers (`µm`).
    pub tip_deflection_um: f64,
    /// Tip deflection expressed as a percentage of cutter diameter (`100 * delta / D`).
    pub deflection_pct_diameter: f64,
    /// Maximum extreme-fiber bending stress across fluted/neck/shank sections (`MPa`).
    pub max_bending_stress_mpa: f64,
    /// Effective allowable bending stress after fatigue/interrupted-cut knockdown (`MPa`).
    pub allowable_stress_mpa: f64,
    /// Bending fracture safety factor (`allowable_stress_mpa / max_bending_stress_mpa`).
    pub bending_safety_factor: f64,
    /// Flute-count / neck-aware effective bending core diameter (`mm`).
    pub effective_core_diameter_mm: f64,
    /// Cutter stickout aspect ratio (`stickout_length_mm / cutting_diameter_mm`).
    pub stickout_ld_ratio: f64,
}

/// Physical inputs to [`compute_engagement_physics`].
#[derive(Debug, Clone, Copy)]
pub struct CutPhysicsInput<'a> {
    pub tool: &'a Tool,
    pub material: &'a WorkpieceMaterialProfile,
    pub spindle: Option<&'a MachineSpindleSpec>,
    pub substrate: ToolSubstrate,
    pub spindle_rpm: f64,
    /// Actual tool-center linear feedrate emitted on the machine (`mm/min`).
    pub feed_mm_min: f64,
    /// Axial depth of cut per pass (`ap`, `mm`; helical pitch per revolution for helical bores).
    pub axial_depth_mm: f64,
    /// Radial engagement (`ae`, `mm`).
    pub radial_engagement_mm: f64,
    /// Optional concave internal bore diameter (`mm`) when cutting inside a circular hole/arc.
    pub bore_diameter_mm: Option<f64>,
    /// `true` when cutting a helical bore ramp (uses exact swept disk/annulus MRR per revolution).
    pub is_helical_bore: bool,
}

/// Full cutting-mechanics telemetry computed by [`compute_engagement_physics`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CutPhysicsMetrics {
    /// Peripheral cutting surface speed `Vc` (`m/min`).
    pub surface_speed_m_min: f64,
    /// Tool-center feedrate (`mm/min`).
    pub programmed_center_feed_mm_min: f64,
    /// Effective cutting-edge wall feedrate (`F_center * R_hole / r_orbit`, `mm/min`).
    pub effective_wall_feed_mm_min: f64,
    /// Inner-arc periphery feed amplification factor (`R_hole / r_orbit`; `1.0` for straight cuts).
    pub inner_arc_feed_multiplier: f64,
    /// Centerline chipload `f_z` (`mm/tooth`).
    pub nominal_chipload_mm: f64,
    /// Effective periphery wall chipload (`mm/tooth`).
    pub wall_chipload_mm: f64,
    /// Cutter radial engagement arc `theta` (`deg`, in `0..=180`).
    pub engagement_arc_deg: f64,
    /// Radial chip thinning ratio `h_max / f_z_wall` in `(0, 1]`.
    pub chip_thinning_ratio: f64,
    /// Effective maximum uncut chip thickness `h_max` (`mm`).
    pub effective_chip_thickness_mm: f64,
    /// Average number of flutes simultaneously engaged (`z * theta / (2 * pi)`).
    pub avg_teeth_in_cut: f64,
    /// `true` when fewer than one flute is engaged on average (`avg_teeth_in_cut < 1.0`).
    pub interrupted_cut: bool,
    /// Material Removal Rate (`cm^3/min`).
    pub mrr_cm3_min: f64,
    /// Estimated cutting power at the spindle (`W`).
    pub cutting_power_watts: f64,
    /// Available spindle power at `spindle_rpm` after torque-curve derating (`W`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub available_spindle_power_watts: Option<f64>,
    /// Cutting power as percentage of available spindle power (`0..inf`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spindle_load_pct: Option<f64>,
    /// Revolution-averaged tangential cutting force `F_t` (`N`).
    pub tangential_force_n: f64,
    /// Revolution-averaged radial cutting force `F_r` (`N`).
    pub radial_force_n: f64,
    /// Peak instantaneous per-tooth radial cutting force (`N`) governing brittle carbide fracture.
    pub peak_radial_force_n: f64,
    /// Stepped cantilever deflection (mean radial load) and carbide bending stress (peak impact load).
    pub deflection: DeflectionResult,
}

#[inline]
const fn endmill_core_fraction(flutes: u32) -> f64 {
    match flutes {
        0 | 1 => 0.54,
        2 => 0.62,
        3 => 0.72,
        _ => 0.78,
    }
}

/// Computes the flute-count-aware effective bending core diameter (`mm`) for any [`CutterGeometry`].
///
/// - Endmills use flute-count core fractions (`1FL ≈ 0.54*D`, `2FL ≈ 0.62*D`, `3FL ≈ 0.72*D`,
///   `4+FL ≈ 0.78*D`).
/// - Relieved-neck cutters (`ThreadMill`, `LollipopEndmill`, `SlotMill`, `DovetailCutter`)
///   return `neck_diameter_mm`, which governs cantilever compliance and bending stress.
pub fn effective_core_diameter_mm(geom: &CutterGeometry) -> f64 {
    let raw = match *geom {
        CutterGeometry::FlatEndmill {
            diameter_mm,
            flutes,
            ..
        }
        | CutterGeometry::BullNoseEndmill {
            diameter_mm,
            flutes,
            ..
        } => diameter_mm * endmill_core_fraction(flutes),
        CutterGeometry::BallEndmill {
            diameter_mm,
            flutes,
            ..
        } => diameter_mm * (endmill_core_fraction(flutes) * 0.96),
        CutterGeometry::ChamferMill {
            diameter_mm,
            tip_diameter_mm,
            flutes,
            ..
        } => {
            let mid_d = (0.5 * (diameter_mm + tip_diameter_mm.max(0.0))).max(0.35 * diameter_mm);
            mid_d * endmill_core_fraction(flutes).max(0.65)
        }
        CutterGeometry::TwistDrill { diameter_mm, .. } => diameter_mm * 0.60,
        CutterGeometry::FaceMill {
            cutting_diameter_mm,
            shank_diameter_mm,
            ..
        } => shank_diameter_mm
            .min(cutting_diameter_mm)
            .max(0.40 * cutting_diameter_mm),
        CutterGeometry::ThreadMill {
            neck_diameter_mm, ..
        }
        | CutterGeometry::LollipopEndmill {
            neck_diameter_mm, ..
        }
        | CutterGeometry::SlotMill {
            neck_diameter_mm, ..
        }
        | CutterGeometry::DovetailCutter {
            neck_diameter_mm, ..
        } => neck_diameter_mm,
    };

    if raw.is_finite() && raw > 0.0 {
        raw.max(1e-3)
    } else {
        0.0
    }
}

/// Segment of a stepped cantilever cutter in tip-distance coordinates `[z_start, z_end]` (`mm`).
#[derive(Debug, Clone, Copy)]
struct BeamSegment {
    z_start_mm: f64,
    z_end_mm: f64,
    diameter_mm: f64,
}

fn build_cantilever_segments(tool: &Tool) -> Vec<BeamSegment> {
    let stickout = tool.stickout_length_mm.max(0.0);
    if !stickout.is_finite() || stickout <= 1e-6 {
        return Vec::new();
    }

    let d_core = effective_core_diameter_mm(&tool.geometry);
    let d_shank = tool.geometry.shank_diameter_mm().max(d_core).max(1e-3);
    let flute_len = tool.geometry.flute_length_mm().clamp(0.0, stickout);

    let mut segs = Vec::with_capacity(3);
    match tool.geometry {
        CutterGeometry::ThreadMill {
            diameter_mm,
            neck_diameter_mm,
            neck_length_mm,
            flute_length_mm,
            ..
        }
        | CutterGeometry::LollipopEndmill {
            diameter_mm,
            neck_diameter_mm,
            neck_length_mm,
            flute_length_mm,
            ..
        } => {
            let head_len = flute_length_mm.clamp(0.0, stickout);
            let neck_end = neck_length_mm.clamp(head_len, stickout);
            let head_d = (0.75 * diameter_mm).max(neck_diameter_mm).max(1e-3);
            let neck_d = neck_diameter_mm.max(1e-3);
            if head_len > 1e-6 {
                segs.push(BeamSegment {
                    z_start_mm: 0.0,
                    z_end_mm: head_len,
                    diameter_mm: head_d,
                });
            }
            if neck_end > head_len + 1e-6 {
                segs.push(BeamSegment {
                    z_start_mm: head_len,
                    z_end_mm: neck_end,
                    diameter_mm: neck_d,
                });
            }
            if stickout > neck_end + 1e-6 {
                segs.push(BeamSegment {
                    z_start_mm: neck_end,
                    z_end_mm: stickout,
                    diameter_mm: d_shank,
                });
            }
        }
        CutterGeometry::SlotMill {
            diameter_mm,
            cutting_thickness_mm,
            neck_diameter_mm,
            neck_length_mm,
            ..
        } => {
            let head_len = cutting_thickness_mm.clamp(0.0, stickout);
            let neck_end = neck_length_mm.clamp(head_len, stickout);
            let head_d = (0.80 * diameter_mm).max(neck_diameter_mm).max(1e-3);
            let neck_d = neck_diameter_mm.max(1e-3);
            if head_len > 1e-6 {
                segs.push(BeamSegment {
                    z_start_mm: 0.0,
                    z_end_mm: head_len,
                    diameter_mm: head_d,
                });
            }
            if neck_end > head_len + 1e-6 {
                segs.push(BeamSegment {
                    z_start_mm: head_len,
                    z_end_mm: neck_end,
                    diameter_mm: neck_d,
                });
            }
            if stickout > neck_end + 1e-6 {
                segs.push(BeamSegment {
                    z_start_mm: neck_end,
                    z_end_mm: stickout,
                    diameter_mm: d_shank,
                });
            }
        }
        CutterGeometry::DovetailCutter {
            diameter_mm,
            cutting_height_mm,
            neck_diameter_mm,
            neck_length_mm,
            ..
        } => {
            let head_len = cutting_height_mm.clamp(0.0, stickout);
            let neck_end = neck_length_mm.clamp(head_len, stickout);
            let head_d = (0.72 * diameter_mm).max(neck_diameter_mm).max(1e-3);
            let neck_d = neck_diameter_mm.max(1e-3);
            if head_len > 1e-6 {
                segs.push(BeamSegment {
                    z_start_mm: 0.0,
                    z_end_mm: head_len,
                    diameter_mm: head_d,
                });
            }
            if neck_end > head_len + 1e-6 {
                segs.push(BeamSegment {
                    z_start_mm: head_len,
                    z_end_mm: neck_end,
                    diameter_mm: neck_d,
                });
            }
            if stickout > neck_end + 1e-6 {
                segs.push(BeamSegment {
                    z_start_mm: neck_end,
                    z_end_mm: stickout,
                    diameter_mm: d_shank,
                });
            }
        }
        _ => {
            if flute_len > 1e-6 {
                segs.push(BeamSegment {
                    z_start_mm: 0.0,
                    z_end_mm: flute_len,
                    diameter_mm: d_core.max(1e-3),
                });
            }
            if stickout > flute_len + 1e-6 {
                segs.push(BeamSegment {
                    z_start_mm: flute_len,
                    z_end_mm: stickout,
                    diameter_mm: d_shank,
                });
            }
        }
    }

    if segs.is_empty() {
        segs.push(BeamSegment {
            z_start_mm: 0.0,
            z_end_mm: stickout,
            diameter_mm: d_core.max(1e-3),
        });
    }
    segs
}

/// Evaluates stepped-cantilever tip deflection (`mm` / `µm`) and maximum extreme-fiber bending
/// stress (`MPa`) for a carbide cutter under lateral force `radial_force_n` applied at
/// `cut_z_centroid_mm` above the tool tip.
pub fn cantilever_deflection_and_stress(
    tool: &Tool,
    radial_force_n: f64,
    cut_z_centroid_mm: f64,
    interrupted_cut: bool,
) -> DeflectionResult {
    cantilever_deflection_and_stress_with_substrate(
        tool,
        ToolSubstrate::Carbide,
        radial_force_n,
        cut_z_centroid_mm,
        interrupted_cut,
    )
}

/// Same as [`cantilever_deflection_and_stress`], with selectable [`ToolSubstrate`]
/// (`Carbide` `E = 600 GPa` vs `Hss` `E = 210 GPa`).
pub fn cantilever_deflection_and_stress_with_substrate(
    tool: &Tool,
    substrate: ToolSubstrate,
    radial_force_n: f64,
    cut_z_centroid_mm: f64,
    interrupted_cut: bool,
) -> DeflectionResult {
    let tool_d = tool.geometry.cutting_diameter_mm().max(1e-6);
    let stickout = if tool.stickout_length_mm.is_finite() {
        tool.stickout_length_mm.max(0.0)
    } else {
        0.0
    };
    let d_core = effective_core_diameter_mm(&tool.geometry);
    let ld_ratio = stickout / tool_d;
    let knockdown = if interrupted_cut {
        INTERRUPTED_IMPACT_KNOCKDOWN
    } else {
        CONTINUOUS_FATIGUE_KNOCKDOWN
    };
    let allowable_stress_mpa = substrate.transverse_rupture_strength_mpa() * knockdown;

    if !radial_force_n.is_finite() || radial_force_n <= 0.0 || stickout <= 1e-6 || d_core <= 0.0 {
        return DeflectionResult {
            tip_deflection_mm: 0.0,
            tip_deflection_um: 0.0,
            deflection_pct_diameter: 0.0,
            max_bending_stress_mpa: 0.0,
            allowable_stress_mpa,
            bending_safety_factor: f64::INFINITY,
            effective_core_diameter_mm: d_core,
            stickout_ld_ratio: ld_ratio,
        };
    }

    let z_load = if cut_z_centroid_mm.is_finite() {
        cut_z_centroid_mm.clamp(0.0, (stickout - 1e-4).max(0.0))
    } else {
        0.0
    };

    let e_mpa = substrate.youngs_modulus_mpa();
    let segments = build_cantilever_segments(tool);

    let mut delta_load_mm = 0.0;
    let mut slope_load_rad = 0.0;
    let mut max_stress_mpa = 0.0_f64;

    for seg in segments {
        if seg.z_end_mm <= z_load + 1e-9 || seg.diameter_mm <= 1e-6 {
            continue;
        }
        let z0 = seg.z_start_mm.max(z_load);
        let z1 = seg.z_end_mm;
        if z1 <= z0 + 1e-9 {
            continue;
        }
        let xa = z0 - z_load;
        let xb = z1 - z_load;
        let d = seg.diameter_mm;
        let i_mm4 = std::f64::consts::PI * d.powi(4) / 64.0;
        if i_mm4 <= 1e-18 {
            continue;
        }

        // Exact Castigliano integration of M(x) = F * x over [xa, xb]
        delta_load_mm += radial_force_n * (xb.powi(3) - xa.powi(3)) / (3.0 * e_mpa * i_mm4);
        slope_load_rad += radial_force_n * (xb.powi(2) - xa.powi(2)) / (2.0 * e_mpa * i_mm4);

        // Extreme-fiber bending stress at section root xb: sigma = 32 * F * xb / (pi * d^3)
        let sigma_mpa = (32.0 * radial_force_n * xb) / (std::f64::consts::PI * d.powi(3));
        if sigma_mpa.is_finite() {
            max_stress_mpa = max_stress_mpa.max(sigma_mpa);
        }
    }

    let tip_deflection_mm = (delta_load_mm + slope_load_rad * z_load).max(0.0);
    let safety_factor = if max_stress_mpa > 1e-9 {
        allowable_stress_mpa / max_stress_mpa
    } else {
        f64::INFINITY
    };

    DeflectionResult {
        tip_deflection_mm,
        tip_deflection_um: tip_deflection_mm * 1_000.0,
        deflection_pct_diameter: (tip_deflection_mm / tool_d) * 100.0,
        max_bending_stress_mpa: max_stress_mpa,
        allowable_stress_mpa,
        bending_safety_factor: safety_factor,
        effective_core_diameter_mm: d_core,
        stickout_ld_ratio: ld_ratio,
    }
}

/// Evaluates available mechanical spindle cutting power (`W`) at `rpm`, applying constant-torque
/// linear derating below `rated_power_rpm` and mechanical drive efficiency.
///
/// Returns `None` if `spindle.power_watts` is unspecified or inputs are invalid.
pub fn available_spindle_power_watts(spindle: &MachineSpindleSpec, rpm: f64) -> Option<f64> {
    let rated_w = spindle.power_watts?;
    if !rated_w.is_finite() || rated_w <= 0.0 || !rpm.is_finite() || rpm <= 0.0 {
        return None;
    }

    let eff = spindle
        .spindle_efficiency
        .filter(|e| e.is_finite() && *e > 0.0)
        .unwrap_or(DEFAULT_SPINDLE_EFFICIENCY)
        .clamp(0.10, 1.0);

    let max_rpm = if spindle.max_rpm.is_finite() && spindle.max_rpm > 0.0 {
        spindle.max_rpm
    } else {
        rpm
    };
    let min_rpm = if spindle.min_rpm.is_finite() && spindle.min_rpm > 0.0 {
        spindle.min_rpm.min(max_rpm)
    } else {
        1.0
    };

    // Default rated base speed when omitted:
    // - Desktop BLDC spindles (<= 500 W): constant-torque up to ~80% of max RPM
    // - High-speed VFD spindles (> 500 W): constant-torque up to ~65% of max RPM
    let rated_rpm = spindle
        .rated_power_rpm
        .filter(|r| r.is_finite() && *r > 0.0)
        .unwrap_or_else(|| {
            let knee_frac = if rated_w <= 500.0 { 0.80 } else { 0.65 };
            (max_rpm * knee_frac).clamp(min_rpm.max(1.0), max_rpm)
        })
        .max(1.0);

    let torque_curve_scale = (rpm / rated_rpm).clamp(0.05, 1.0);
    Some(rated_w * eff * torque_curve_scale)
}

/// Computes the exact radial engagement arc `theta` (`radians` in `[0, pi]`) for both straight
/// cuts (`bore_diameter_mm = None`) and concave internal circular bore cuts
/// (`bore_diameter_mm = Some(hole_d)`) via the law of cosines.
pub fn engagement_arc_rad(
    tool_diameter_mm: f64,
    radial_engagement_mm: f64,
    bore_diameter_mm: Option<f64>,
) -> f64 {
    if !tool_diameter_mm.is_finite()
        || !radial_engagement_mm.is_finite()
        || tool_diameter_mm <= 1e-6
        || radial_engagement_mm <= 1e-9
    {
        return 0.0;
    }

    let r_tool = 0.5 * tool_diameter_mm;
    let ae = radial_engagement_mm.clamp(0.0, tool_diameter_mm);

    if let Some(hole_d) = bore_diameter_mm
        && hole_d.is_finite()
        && hole_d > tool_diameter_mm + 1e-6
    {
        let r_hole = 0.5 * hole_d;
        let r_orbit = r_hole - r_tool;
        let r_prev = (r_hole - ae).max(0.0);
        let denom = 2.0 * r_orbit * r_tool;
        if denom > 1e-12 {
            // Triangle (hole center O, tool center C, entry intersection P2):
            // R_prev^2 = r_orbit^2 + r_tool^2 + 2 * r_orbit * r_tool * cos(theta)
            let cos_theta =
                ((r_prev * r_prev - r_orbit * r_orbit - r_tool * r_tool) / denom).clamp(-1.0, 1.0);
            return cos_theta.acos();
        }
    }

    // Straight-wall engagement: cos(theta) = 1 - 2 * ae / D
    let cos_theta = (1.0 - 2.0 * ae / tool_diameter_mm).clamp(-1.0, 1.0);
    cos_theta.acos()
}

/// Computes full cutting-mechanics telemetry for a milling/boring engagement:
/// engagement arc (straight or concave internal bore), radial chip thinning `h_max`, inner-arc
/// wall feed amplification, MRR, spindle power/load `%`, cutting forces, stepped cantilever
/// deflection, and carbide bending-stress safety factor.
pub fn compute_engagement_physics(input: &CutPhysicsInput<'_>) -> CutPhysicsMetrics {
    let tool_d = input.tool.geometry.cutting_diameter_mm().max(1e-6);
    let tool_r = 0.5 * tool_d;
    let flutes = input.tool.geometry.flutes().max(1);

    let rpm = if input.spindle_rpm.is_finite() {
        input.spindle_rpm.max(0.0)
    } else {
        0.0
    };
    let feed_center = if input.feed_mm_min.is_finite() {
        input.feed_mm_min.max(0.0)
    } else {
        0.0
    };
    let ap = if input.axial_depth_mm.is_finite() {
        input.axial_depth_mm.max(0.0)
    } else {
        0.0
    };
    let ae = if input.radial_engagement_mm.is_finite() {
        input.radial_engagement_mm.clamp(0.0, tool_d)
    } else {
        0.0
    };

    let surface_speed_m_min = surface_speed_from_rpm(rpm, tool_d).unwrap_or(0.0);

    // Concave internal bore wall feed amplification: F_wall = F_center * (R_hole / r_orbit)
    let valid_bore_r = input
        .bore_diameter_mm
        .filter(|&d| d.is_finite() && d > tool_d + 1e-4)
        .map(|d| 0.5 * d);
    let inner_arc_feed_multiplier = match valid_bore_r {
        Some(r_hole) => {
            let r_orbit = (r_hole - tool_r).max(1e-4);
            (r_hole / r_orbit).max(1.0)
        }
        None => 1.0,
    };

    let effective_wall_feed_mm_min = feed_center * inner_arc_feed_multiplier;
    let nominal_chipload_mm = chipload_from_feed(rpm, flutes, feed_center).unwrap_or(0.0);
    let wall_chipload_mm =
        chipload_from_feed(rpm, flutes, effective_wall_feed_mm_min).unwrap_or(0.0);

    let theta_rad = engagement_arc_rad(tool_d, ae, input.bore_diameter_mm);
    let engagement_arc_deg = theta_rad.to_degrees();
    let chip_thinning_ratio = if theta_rad <= 1e-9 {
        0.0
    } else if theta_rad >= std::f64::consts::FRAC_PI_2 {
        1.0
    } else {
        theta_rad.sin().clamp(0.0, 1.0)
    };
    let effective_chip_thickness_mm = wall_chipload_mm * chip_thinning_ratio;

    let avg_teeth_in_cut = (f64::from(flutes) * theta_rad) / std::f64::consts::TAU;
    let interrupted_cut = avg_teeth_in_cut < 1.0 - 1e-6;

    // Material removal rate (mm^3/min)
    let mrr_mm3_min = if input.is_helical_bore
        && let Some(r_hole) = valid_bore_r
    {
        let r_orbit = (r_hole - tool_r).max(1e-4);
        let r_prev = (r_hole - ae).max(0.0);
        // Volume per revolution: pi * (R_hole^2 - R_prev^2) * ap; time per rev: 2*pi*r_orbit / F_center
        ((r_hole * r_hole - r_prev * r_prev) * ap * feed_center) / (2.0 * r_orbit)
    } else {
        ap * ae * effective_wall_feed_mm_min
    };
    let mrr_cm3_min = mrr_mm3_min / 1_000.0;

    let u_s = input.material.specific_cutting_energy_w_s_mm3.max(0.0);
    let k_c = input.material.specific_cutting_force_n_mm2.max(0.0);
    let k_r = input.material.radial_force_ratio.clamp(0.10, 1.0);

    let cutting_power_watts = u_s * (mrr_mm3_min / 60.0);
    let available_spindle_power_watts = input
        .spindle
        .and_then(|sp| available_spindle_power_watts(sp, rpm));
    let spindle_load_pct = available_spindle_power_watts
        .filter(|&avail| avail > 1e-6)
        .map(|avail| (cutting_power_watts / avail) * 100.0);

    // Revolution-averaged tangential & radial cutting forces
    let v_c_m_s = surface_speed_m_min / 60.0;
    let tangential_force_n = if v_c_m_s > 1e-9 {
        cutting_power_watts / v_c_m_s
    } else {
        0.0
    };
    let radial_force_n = k_r * tangential_force_n;

    // Peak instantaneous per-tooth radial force governing brittle carbide bending snap
    let simultaneous_teeth = avg_teeth_in_cut.max(1.0);
    let peak_tangential_force_n = if effective_chip_thickness_mm > 0.0 && ap > 0.0 {
        k_c * ap * effective_chip_thickness_mm * simultaneous_teeth
    } else {
        0.0
    };
    let peak_radial_force_n = (k_r * peak_tangential_force_n).max(radial_force_n);

    let cut_z_centroid_mm = 0.5 * ap;
    let steady_deflection = cantilever_deflection_and_stress_with_substrate(
        input.tool,
        input.substrate,
        radial_force_n,
        cut_z_centroid_mm,
        interrupted_cut,
    );
    let peak_stress = cantilever_deflection_and_stress_with_substrate(
        input.tool,
        input.substrate,
        peak_radial_force_n,
        cut_z_centroid_mm,
        interrupted_cut,
    );

    let deflection = DeflectionResult {
        tip_deflection_mm: steady_deflection.tip_deflection_mm,
        tip_deflection_um: steady_deflection.tip_deflection_um,
        deflection_pct_diameter: steady_deflection.deflection_pct_diameter,
        max_bending_stress_mpa: peak_stress.max_bending_stress_mpa,
        allowable_stress_mpa: peak_stress.allowable_stress_mpa,
        bending_safety_factor: peak_stress.bending_safety_factor,
        effective_core_diameter_mm: steady_deflection.effective_core_diameter_mm,
        stickout_ld_ratio: steady_deflection.stickout_ld_ratio,
    };

    CutPhysicsMetrics {
        surface_speed_m_min,
        programmed_center_feed_mm_min: feed_center,
        effective_wall_feed_mm_min,
        inner_arc_feed_multiplier,
        nominal_chipload_mm,
        wall_chipload_mm,
        engagement_arc_deg,
        chip_thinning_ratio,
        effective_chip_thickness_mm,
        avg_teeth_in_cut,
        interrupted_cut,
        mrr_cm3_min,
        cutting_power_watts,
        available_spindle_power_watts,
        spindle_load_pct,
        tangential_force_n,
        radial_force_n,
        peak_radial_force_n,
        deflection,
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::{DisplayUnit, approx_eq};

    use crate::{
        ALUMINUM_6061_T6_ID, ALUMINUM_6063_T5_ID, ProjectMaterialSelection,
        builtin_carvera_air_machine, builtin_carvera_z1_machine, builtin_milo_1_5_machine,
        model::{CoolantMode, SpindleDirection, ToolId},
        resolve_workpiece_material,
    };

    use super::*;

    fn flat_endmill(d: f64, loc: f64, stickout: f64, flutes: u32) -> Tool {
        Tool {
            id: ToolId(format!("em_{d}_{flutes}fl")),
            tool_number: 1,
            length_offset_number: 1,
            diameter_offset_number: 1,
            name: format!("Ø{d}mm {flutes}FL Flat"),
            vendor: None,
            product_id: None,
            native_unit: DisplayUnit::Millimeter,
            spindle_direction: SpindleDirection::Cw,
            default_coolant: CoolantMode::AirBlast,
            stickout_length_mm: stickout,
            geometry: CutterGeometry::FlatEndmill {
                diameter_mm: d,
                flute_length_mm: loc,
                overall_length_mm: stickout + 20.0,
                shank_diameter_mm: d,
                flutes,
            },
            presets: Vec::new(),
        }
    }

    #[test]
    fn effective_core_diameter_handles_all_10_cutter_geometry_variants() {
        let cases: Vec<(CutterGeometry, f64)> = vec![
            (
                CutterGeometry::FlatEndmill {
                    diameter_mm: 10.0,
                    flute_length_mm: 20.0,
                    overall_length_mm: 50.0,
                    shank_diameter_mm: 10.0,
                    flutes: 1,
                },
                5.4,
            ),
            (
                CutterGeometry::FlatEndmill {
                    diameter_mm: 10.0,
                    flute_length_mm: 20.0,
                    overall_length_mm: 50.0,
                    shank_diameter_mm: 10.0,
                    flutes: 2,
                },
                6.2,
            ),
            (
                CutterGeometry::FlatEndmill {
                    diameter_mm: 10.0,
                    flute_length_mm: 20.0,
                    overall_length_mm: 50.0,
                    shank_diameter_mm: 10.0,
                    flutes: 3,
                },
                7.2,
            ),
            (
                CutterGeometry::BullNoseEndmill {
                    diameter_mm: 10.0,
                    corner_radius_mm: 1.0,
                    flute_length_mm: 20.0,
                    overall_length_mm: 50.0,
                    shank_diameter_mm: 10.0,
                    flutes: 4,
                },
                7.8,
            ),
            (
                CutterGeometry::BallEndmill {
                    diameter_mm: 10.0,
                    flute_length_mm: 20.0,
                    overall_length_mm: 50.0,
                    shank_diameter_mm: 10.0,
                    flutes: 2,
                },
                10.0 * 0.62 * 0.96,
            ),
            (
                CutterGeometry::ChamferMill {
                    diameter_mm: 6.0,
                    tip_diameter_mm: 0.2,
                    included_angle_deg: 90.0,
                    flute_length_mm: 10.0,
                    overall_length_mm: 50.0,
                    shank_diameter_mm: 6.0,
                    flutes: 2,
                },
                3.1 * 0.65,
            ),
            (
                CutterGeometry::TwistDrill {
                    diameter_mm: 6.0,
                    point_angle_deg: 118.0,
                    flute_length_mm: 30.0,
                    overall_length_mm: 60.0,
                    shank_diameter_mm: 6.0,
                    flutes: 2,
                },
                3.6,
            ),
            (
                CutterGeometry::FaceMill {
                    cutting_diameter_mm: 38.0,
                    outside_diameter_mm: 40.0,
                    corner_radius_mm: 0.4,
                    flute_length_mm: 10.0,
                    overall_length_mm: 45.0,
                    shank_diameter_mm: 16.0,
                    flutes: 3,
                },
                16.0,
            ),
            (
                CutterGeometry::ThreadMill {
                    diameter_mm: 6.0,
                    neck_diameter_mm: 3.4,
                    neck_length_mm: 18.0,
                    pitch_mm: 1.25,
                    teeth_count: 1,
                    thread_angle_deg: 60.0,
                    flute_length_mm: 1.5,
                    overall_length_mm: 50.0,
                    shank_diameter_mm: 6.0,
                    flutes: 3,
                },
                3.4,
            ),
            (
                CutterGeometry::LollipopEndmill {
                    diameter_mm: 6.0,
                    neck_diameter_mm: 3.2,
                    neck_length_mm: 20.0,
                    flute_length_mm: 5.5,
                    overall_length_mm: 55.0,
                    shank_diameter_mm: 6.0,
                    flutes: 4,
                },
                3.2,
            ),
            (
                CutterGeometry::SlotMill {
                    diameter_mm: 12.0,
                    cutting_thickness_mm: 3.0,
                    corner_radius_mm: 0.2,
                    neck_diameter_mm: 5.0,
                    neck_length_mm: 18.0,
                    overall_length_mm: 55.0,
                    shank_diameter_mm: 6.0,
                    flutes: 4,
                },
                5.0,
            ),
            (
                CutterGeometry::DovetailCutter {
                    diameter_mm: 12.0,
                    side_angle_deg: 60.0,
                    cutting_height_mm: 5.0,
                    corner_radius_mm: 0.2,
                    neck_diameter_mm: 5.5,
                    neck_length_mm: 16.0,
                    overall_length_mm: 55.0,
                    shank_diameter_mm: 6.0,
                    flutes: 4,
                },
                5.5,
            ),
        ];

        for (geom, expected) in cases {
            let actual = effective_core_diameter_mm(&geom);
            assert!(
                approx_eq(actual, expected, 1e-6),
                "core diameter mismatch for {:?}: got {actual}, expected {expected}",
                geom.kind_label()
            );
        }
    }

    #[test]
    fn cantilever_deflection_obeys_cubic_stickout_and_fourth_power_core_laws() {
        // Uniform full-flute beam (flute_length == stickout): exact delta = 64 * F * L^3 / (3 * pi * E * d_core^4)
        let t_short = flat_endmill(6.0, 15.0, 15.0, 2);
        let t_double_l = flat_endmill(6.0, 30.0, 30.0, 2);
        let r_short = cantilever_deflection_and_stress(&t_short, 20.0, 0.0, false);
        let r_double = cantilever_deflection_and_stress(&t_double_l, 20.0, 0.0, false);

        // Doubling L increases tip deflection by exactly 8x and bending stress by 2x
        assert!(approx_eq(
            r_double.tip_deflection_mm / r_short.tip_deflection_mm,
            8.0,
            1e-9
        ));
        assert!(approx_eq(
            r_double.max_bending_stress_mpa / r_short.max_bending_stress_mpa,
            2.0,
            1e-9
        ));

        // 1FL (0.54*D) vs 4FL (0.78*D) on identical stickout: deflection ratio = (0.78/0.54)^4
        let t_1fl = flat_endmill(3.175, 25.0, 25.0, 1);
        let t_4fl = flat_endmill(3.175, 25.0, 25.0, 4);
        let r_1fl = cantilever_deflection_and_stress(&t_1fl, 10.0, 0.0, true);
        let r_4fl = cantilever_deflection_and_stress(&t_4fl, 10.0, 0.0, true);
        assert!(approx_eq(
            r_1fl.tip_deflection_mm / r_4fl.tip_deflection_mm,
            (0.78_f64 / 0.54_f64).powi(4),
            1e-9
        ));

        // HSS (210 GPa) deflects 600/210 times more than Carbide (600 GPa)
        let r_hss = cantilever_deflection_and_stress_with_substrate(
            &t_short,
            ToolSubstrate::Hss,
            20.0,
            0.0,
            false,
        );
        assert!(approx_eq(
            r_hss.tip_deflection_mm / r_short.tip_deflection_mm,
            CARBIDE_YOUNGS_MODULUS_MPA / HSS_YOUNGS_MODULUS_MPA,
            1e-9
        ));
    }

    #[test]
    fn spindle_power_curve_derates_linearly_below_rated_rpm() {
        let mut sp = MachineSpindleSpec {
            min_rpm: 1_000.0,
            max_rpm: 24_000.0,
            power_watts: Some(1_500.0),
            rated_power_rpm: Some(12_000.0),
            spindle_efficiency: Some(0.80),
            ..MachineSpindleSpec::default()
        };
        let p_rated = available_spindle_power_watts(&sp, 12_000.0).unwrap();
        let p_high = available_spindle_power_watts(&sp, 24_000.0).unwrap();
        let p_half = available_spindle_power_watts(&sp, 6_000.0).unwrap();

        assert!(approx_eq(p_rated, 1_200.0, 1e-6));
        assert!(approx_eq(p_high, 1_200.0, 1e-6));
        assert!(approx_eq(p_half, 600.0, 1e-6));

        // Omitted rated_power_rpm still produces realistic constant-torque derating
        sp.rated_power_rpm = None;
        let p_default_hi = available_spindle_power_watts(&sp, 20_000.0).unwrap();
        let p_default_lo = available_spindle_power_watts(&sp, 5_000.0).unwrap();
        assert!(p_default_lo < 0.5 * p_default_hi);
    }

    #[test]
    fn bartrim_facing_and_spindle_stall_physics_match_real_world_cases() {
        let carvera_air = builtin_carvera_air_machine();
        let carvera_z1 = builtin_carvera_z1_machine();
        let milo = builtin_milo_1_5_machine();
        let al6063 = resolve_workpiece_material(Some(&ProjectMaterialSelection::builtin(
            ALUMINUM_6063_T5_ID,
        )));
        let al6061 = resolve_workpiece_material(Some(&ProjectMaterialSelection::builtin(
            ALUMINUM_6061_T6_ID,
        )));

        // BarTrim T2: Ø3.175 x 25 mm LOC 1FL O-flute, 28 mm stickout (L/D = 8.82x)
        let t2 = carvera_air
            .tools
            .iter()
            .find(|t| t.tool_number == 2)
            .unwrap();
        let bad_face = compute_engagement_physics(&CutPhysicsInput {
            tool: t2,
            material: &al6063,
            spindle: Some(&carvera_air.spindle),
            substrate: ToolSubstrate::Carbide,
            spindle_rpm: 11_028.0,
            feed_mm_min: 309.0,
            axial_depth_mm: 4.04,
            radial_engagement_mm: 0.50,
            bore_diameter_mm: None,
            is_helical_bore: false,
        });
        assert!(
            bad_face.deflection.bending_safety_factor < 1.5,
            "4.04 x 0.50 must show critical snap hazard (SF < 1.5), got {:.2}",
            bad_face.deflection.bending_safety_factor
        );

        let safe_face = compute_engagement_physics(&CutPhysicsInput {
            tool: t2,
            material: &al6063,
            spindle: Some(&carvera_air.spindle),
            substrate: ToolSubstrate::Carbide,
            spindle_rpm: 11_028.0,
            feed_mm_min: 309.0,
            axial_depth_mm: 2.02,
            radial_engagement_mm: 0.30,
            bore_diameter_mm: None,
            is_helical_bore: false,
        });
        assert!(
            safe_face.deflection.bending_safety_factor >= 2.5
                && safe_face.deflection.tip_deflection_mm <= 0.025,
            "2.02 x 0.30 must be safe: SF={:.2}, delta={:.1}um",
            safe_face.deflection.bending_safety_factor,
            safe_face.deflection.tip_deflection_um
        );

        // Concave Ø5.0 mm bore with Ø3.175 mm cutter amplifies wall feed by R_hole / r_orbit = 2.74x
        let bore_phys = compute_engagement_physics(&CutPhysicsInput {
            tool: t2,
            material: &al6063,
            spindle: Some(&carvera_air.spindle),
            substrate: ToolSubstrate::Carbide,
            spindle_rpm: 11_028.0,
            feed_mm_min: 309.0,
            axial_depth_mm: 0.25,
            radial_engagement_mm: 0.15,
            bore_diameter_mm: Some(5.0),
            is_helical_bore: false,
        });
        let expected_mult = 2.5 / (2.5 - 1.5875);
        assert!(approx_eq(
            bore_phys.inner_arc_feed_multiplier,
            expected_mult,
            1e-6
        ));
        // Concave engagement arc must exceed straight-wall arc for the same ae
        let straight_arc = engagement_arc_rad(3.175, 0.15, None).to_degrees();
        assert!(bore_phys.engagement_arc_deg > straight_arc * 1.4);

        // Heavy Ø6 mm slotting stalls Carvera Z1 (150 W) but passes comfortably on Milo (1.5 kW)
        let em6 = flat_endmill(6.0, 16.0, 22.0, 3);
        let z1_slot = compute_engagement_physics(&CutPhysicsInput {
            tool: &em6,
            material: &al6061,
            spindle: Some(&carvera_z1.spindle),
            substrate: ToolSubstrate::Carbide,
            spindle_rpm: 12_000.0,
            feed_mm_min: 800.0,
            axial_depth_mm: 3.0,
            radial_engagement_mm: 6.0,
            bore_diameter_mm: None,
            is_helical_bore: false,
        });
        let milo_slot = compute_engagement_physics(&CutPhysicsInput {
            tool: &em6,
            material: &al6061,
            spindle: Some(&milo.spindle),
            substrate: ToolSubstrate::Carbide,
            spindle_rpm: 12_000.0,
            feed_mm_min: 800.0,
            axial_depth_mm: 3.0,
            radial_engagement_mm: 6.0,
            bore_diameter_mm: None,
            is_helical_bore: false,
        });
        assert!(z1_slot.spindle_load_pct.unwrap() > 100.0);
        assert!(milo_slot.spindle_load_pct.unwrap() < 30.0);
    }
}
