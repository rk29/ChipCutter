use std::fmt;

use chip_math::{DisplayUnit, EvalContext};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::feeds_speeds::CuttingPreset;

/// Stable identifier of a [`Tool`] inside a library or `.chip.toml` project.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ToolId(pub String);

impl fmt::Display for ToolId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

/// Spindle rotation direction (`M3` vs `M4`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum SpindleDirection {
    #[default]
    Cw,
    Ccw,
}

impl SpindleDirection {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Cw => "CW (M3)",
            Self::Ccw => "CCW (M4)",
        }
    }
}

/// Coolant command mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum CoolantMode {
    #[default]
    Disabled,
    Flood,
    Mist,
    AirBlast,
}

impl CoolantMode {
    pub const ALL: [Self; 4] = [Self::Disabled, Self::Flood, Self::Mist, Self::AirBlast];

    pub const fn label(self) -> &'static str {
        match self {
            Self::Disabled => "Disabled (M9)",
            Self::Flood => "Flood (M8)",
            Self::Mist => "Mist (M7)",
            Self::AirBlast => "Air Blast",
        }
    }
}

/// High-level 2.5D operation classes used to validate tool compatibility.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ToolOperationClass {
    Facing,
    Contour2D,
    Pocket2D,
    Drill,
    HelicalBore,
    Chamfer,
    Trace,
    VCarve,
    VCarveFlatClearance,
    Waterline3D,
    Adaptive3D,
    Parallel3D,
    Scallop3D,
    Pencil3D,
    Rotary2Linear,
    Rotary4AxisSim,
    ThreadMill,
    Deburr3D,
    SlotUndercut,
    Dovetail,
    Undercut3D,
}

/// Physical cutter geometry stored in canonical `f64` millimeters.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum CutterGeometry {
    FlatEndmill {
        diameter_mm: f64,
        flute_length_mm: f64,
        overall_length_mm: f64,
        shank_diameter_mm: f64,
        flutes: u32,
    },
    BullNoseEndmill {
        diameter_mm: f64,
        corner_radius_mm: f64,
        flute_length_mm: f64,
        overall_length_mm: f64,
        shank_diameter_mm: f64,
        flutes: u32,
    },
    BallEndmill {
        diameter_mm: f64,
        flute_length_mm: f64,
        overall_length_mm: f64,
        shank_diameter_mm: f64,
        flutes: u32,
    },
    ChamferMill {
        diameter_mm: f64,
        tip_diameter_mm: f64,
        included_angle_deg: f64,
        flute_length_mm: f64,
        overall_length_mm: f64,
        shank_diameter_mm: f64,
        flutes: u32,
    },
    TwistDrill {
        diameter_mm: f64,
        point_angle_deg: f64,
        flute_length_mm: f64,
        overall_length_mm: f64,
        shank_diameter_mm: f64,
        flutes: u32,
    },
    FaceMill {
        cutting_diameter_mm: f64,
        outside_diameter_mm: f64,
        corner_radius_mm: f64,
        flute_length_mm: f64,
        overall_length_mm: f64,
        shank_diameter_mm: f64,
        flutes: u32,
    },
    #[serde(rename = "thread_mill")]
    ThreadMill {
        diameter_mm: f64,
        neck_diameter_mm: f64,
        neck_length_mm: f64,
        pitch_mm: f64,
        teeth_count: u32,
        thread_angle_deg: f64,
        flute_length_mm: f64,
        overall_length_mm: f64,
        shank_diameter_mm: f64,
        flutes: u32,
    },
    #[serde(rename = "lollipop_endmill")]
    LollipopEndmill {
        diameter_mm: f64,
        neck_diameter_mm: f64,
        neck_length_mm: f64,
        flute_length_mm: f64,
        overall_length_mm: f64,
        shank_diameter_mm: f64,
        flutes: u32,
    },
    #[serde(rename = "slot_mill")]
    SlotMill {
        diameter_mm: f64,
        cutting_thickness_mm: f64,
        corner_radius_mm: f64,
        neck_diameter_mm: f64,
        neck_length_mm: f64,
        overall_length_mm: f64,
        shank_diameter_mm: f64,
        flutes: u32,
    },
    #[serde(rename = "dovetail_cutter")]
    DovetailCutter {
        diameter_mm: f64,
        side_angle_deg: f64,
        cutting_height_mm: f64,
        corner_radius_mm: f64,
        neck_diameter_mm: f64,
        neck_length_mm: f64,
        overall_length_mm: f64,
        shank_diameter_mm: f64,
        flutes: u32,
    },
}

impl CutterGeometry {
    /// Analytic axial height of a spherical lollipop head up to the relieved neck join:
    /// `R + sqrt(R^2 - r_neck^2)`.
    #[inline]
    pub fn lollipop_sphere_height_mm(diameter_mm: f64, neck_diameter_mm: f64) -> f64 {
        let r = (0.5 * diameter_mm).max(0.0);
        let r_neck = (0.5 * neck_diameter_mm).clamp(0.0, r);
        r + (r * r - r_neck * r_neck).max(0.0).sqrt()
    }

    /// Analytic minor top diameter (`D_top`) of an inverted conical dovetail cutter at
    /// `dz = cutting_height_mm`, accounting for any bottom corner fillet `corner_radius_mm`.
    #[inline]
    pub fn dovetail_top_diameter_mm(
        diameter_mm: f64,
        side_angle_deg: f64,
        cutting_height_mm: f64,
        corner_radius_mm: f64,
    ) -> f64 {
        let r_bot = (0.5 * diameter_mm).max(0.0);
        let h = cutting_height_mm.max(0.0);
        let alpha = side_angle_deg.clamp(1.0, 89.0).to_radians();
        let tan_a = alpha.tan().max(1e-6);
        let rc = corner_radius_mm.max(0.0);
        let r_top = if rc <= 1e-9 {
            r_bot - h / tan_a
        } else {
            let (sin_a, cos_a) = alpha.sin_cos();
            let r_c = (r_bot - rc).max(0.0);
            let z_tan = rc * (1.0 + cos_a);
            if h <= z_tan {
                let dz = h - rc;
                r_c + (rc * rc - dz * dz).max(0.0).sqrt()
            } else {
                let r_tan = r_c + rc * sin_a;
                r_tan - (h - z_tan) / tan_a
            }
        };
        2.0 * r_top
    }

    pub const fn kind_label(&self) -> &'static str {
        match self {
            Self::FlatEndmill { .. } => "Flat Endmill",
            Self::BullNoseEndmill { .. } => "Bull-Nose Endmill",
            Self::BallEndmill { .. } => "Ball Endmill",
            Self::ChamferMill { .. } => "Chamfer / Spot Mill",
            Self::TwistDrill { .. } => "Twist Drill",
            Self::FaceMill { .. } => "Face / Fly Cutter",
            Self::ThreadMill { .. } => "Thread Mill",
            Self::LollipopEndmill { .. } => "Lollipop Endmill",
            Self::SlotMill { .. } => "Slot / T-Slot Mill",
            Self::DovetailCutter { .. } => "Dovetail Cutter",
        }
    }

    pub fn cutting_diameter_mm(&self) -> f64 {
        match *self {
            Self::FlatEndmill { diameter_mm, .. }
            | Self::BullNoseEndmill { diameter_mm, .. }
            | Self::BallEndmill { diameter_mm, .. }
            | Self::ChamferMill { diameter_mm, .. }
            | Self::TwistDrill { diameter_mm, .. }
            | Self::ThreadMill { diameter_mm, .. }
            | Self::LollipopEndmill { diameter_mm, .. }
            | Self::SlotMill { diameter_mm, .. }
            | Self::DovetailCutter { diameter_mm, .. } => diameter_mm,
            Self::FaceMill {
                cutting_diameter_mm,
                ..
            } => cutting_diameter_mm,
        }
    }

    /// Nominal bottom corner fillet radius (`mm`) of this cutter (`0.5 * diameter` for
    /// `BallEndmill` / `LollipopEndmill`, explicit `corner_radius_mm` for bull-nose / face /
    /// slot / dovetail cutters, `0.0` otherwise).
    pub fn corner_radius_mm(&self) -> f64 {
        match *self {
            Self::BullNoseEndmill {
                corner_radius_mm, ..
            }
            | Self::FaceMill {
                corner_radius_mm, ..
            }
            | Self::SlotMill {
                corner_radius_mm, ..
            }
            | Self::DovetailCutter {
                corner_radius_mm, ..
            } => corner_radius_mm.max(0.0),
            Self::BallEndmill { diameter_mm, .. } | Self::LollipopEndmill { diameter_mm, .. } => {
                (0.5 * diameter_mm).max(0.0)
            }
            _ => 0.0,
        }
    }

    pub fn flute_length_mm(&self) -> f64 {
        match *self {
            Self::FlatEndmill {
                flute_length_mm, ..
            }
            | Self::BullNoseEndmill {
                flute_length_mm, ..
            }
            | Self::BallEndmill {
                flute_length_mm, ..
            }
            | Self::ChamferMill {
                flute_length_mm, ..
            }
            | Self::TwistDrill {
                flute_length_mm, ..
            }
            | Self::FaceMill {
                flute_length_mm, ..
            }
            | Self::ThreadMill {
                flute_length_mm, ..
            }
            | Self::LollipopEndmill {
                flute_length_mm, ..
            } => flute_length_mm,
            Self::SlotMill {
                cutting_thickness_mm,
                ..
            } => cutting_thickness_mm,
            Self::DovetailCutter {
                cutting_height_mm, ..
            } => cutting_height_mm,
        }
    }

    pub fn overall_length_mm(&self) -> f64 {
        match *self {
            Self::FlatEndmill {
                overall_length_mm, ..
            }
            | Self::BullNoseEndmill {
                overall_length_mm, ..
            }
            | Self::BallEndmill {
                overall_length_mm, ..
            }
            | Self::ChamferMill {
                overall_length_mm, ..
            }
            | Self::TwistDrill {
                overall_length_mm, ..
            }
            | Self::FaceMill {
                overall_length_mm, ..
            }
            | Self::ThreadMill {
                overall_length_mm, ..
            }
            | Self::LollipopEndmill {
                overall_length_mm, ..
            }
            | Self::SlotMill {
                overall_length_mm, ..
            }
            | Self::DovetailCutter {
                overall_length_mm, ..
            } => overall_length_mm,
        }
    }

    pub fn shank_diameter_mm(&self) -> f64 {
        match *self {
            Self::FlatEndmill {
                shank_diameter_mm, ..
            }
            | Self::BullNoseEndmill {
                shank_diameter_mm, ..
            }
            | Self::BallEndmill {
                shank_diameter_mm, ..
            }
            | Self::ChamferMill {
                shank_diameter_mm, ..
            }
            | Self::TwistDrill {
                shank_diameter_mm, ..
            }
            | Self::FaceMill {
                shank_diameter_mm, ..
            }
            | Self::ThreadMill {
                shank_diameter_mm, ..
            }
            | Self::LollipopEndmill {
                shank_diameter_mm, ..
            }
            | Self::SlotMill {
                shank_diameter_mm, ..
            }
            | Self::DovetailCutter {
                shank_diameter_mm, ..
            } => shank_diameter_mm,
        }
    }

    pub fn flutes(&self) -> u32 {
        match *self {
            Self::FlatEndmill { flutes, .. }
            | Self::BullNoseEndmill { flutes, .. }
            | Self::BallEndmill { flutes, .. }
            | Self::ChamferMill { flutes, .. }
            | Self::TwistDrill { flutes, .. }
            | Self::FaceMill { flutes, .. }
            | Self::ThreadMill { flutes, .. }
            | Self::LollipopEndmill { flutes, .. }
            | Self::SlotMill { flutes, .. }
            | Self::DovetailCutter { flutes, .. } => flutes,
        }
    }

    /// Included tip angle in radians (`included_angle` for chamfer mills, `point_angle` for drills,
    /// `π` for flat-bottomed cutters).
    pub fn tip_angle_rad(&self) -> f64 {
        match *self {
            Self::ChamferMill {
                included_angle_deg, ..
            } => included_angle_deg.to_radians(),
            Self::TwistDrill {
                point_angle_deg, ..
            } => point_angle_deg.to_radians(),
            _ => std::f64::consts::PI,
        }
    }

    /// Checks whether this cutter type is valid for `op_class` per `04-tool-library.md`.
    pub fn supports_operation(&self, op_class: ToolOperationClass) -> bool {
        match self {
            Self::FlatEndmill { .. } => matches!(
                op_class,
                ToolOperationClass::Facing
                    | ToolOperationClass::Contour2D
                    | ToolOperationClass::Pocket2D
                    | ToolOperationClass::HelicalBore
                    | ToolOperationClass::Trace
                    | ToolOperationClass::VCarveFlatClearance
                    | ToolOperationClass::Waterline3D
                    | ToolOperationClass::Adaptive3D
                    | ToolOperationClass::Parallel3D
                    | ToolOperationClass::Scallop3D
                    | ToolOperationClass::Pencil3D
                    | ToolOperationClass::Rotary2Linear
                    | ToolOperationClass::Rotary4AxisSim
                    | ToolOperationClass::SlotUndercut
            ),
            Self::BullNoseEndmill { .. } => matches!(
                op_class,
                ToolOperationClass::Facing
                    | ToolOperationClass::Contour2D
                    | ToolOperationClass::Pocket2D
                    | ToolOperationClass::HelicalBore
                    | ToolOperationClass::Trace
                    | ToolOperationClass::Waterline3D
                    | ToolOperationClass::Adaptive3D
                    | ToolOperationClass::Parallel3D
                    | ToolOperationClass::Scallop3D
                    | ToolOperationClass::Pencil3D
                    | ToolOperationClass::Rotary2Linear
                    | ToolOperationClass::Rotary4AxisSim
            ),
            Self::BallEndmill { .. } => matches!(
                op_class,
                ToolOperationClass::Contour2D
                    | ToolOperationClass::Pocket2D
                    | ToolOperationClass::Trace
                    | ToolOperationClass::Waterline3D
                    | ToolOperationClass::Adaptive3D
                    | ToolOperationClass::Parallel3D
                    | ToolOperationClass::Scallop3D
                    | ToolOperationClass::Pencil3D
                    | ToolOperationClass::Rotary2Linear
                    | ToolOperationClass::Rotary4AxisSim
                    | ToolOperationClass::Deburr3D
            ),
            Self::ChamferMill {
                included_angle_deg, ..
            } => match op_class {
                ToolOperationClass::VCarve => (10.0..=160.0).contains(included_angle_deg),
                ToolOperationClass::Chamfer
                | ToolOperationClass::Trace
                | ToolOperationClass::Drill
                | ToolOperationClass::Parallel3D
                | ToolOperationClass::Pencil3D
                | ToolOperationClass::Rotary2Linear
                | ToolOperationClass::Rotary4AxisSim
                | ToolOperationClass::Deburr3D => true,
                _ => false,
            },
            Self::TwistDrill { .. } => matches!(op_class, ToolOperationClass::Drill),
            Self::FaceMill { .. } => matches!(
                op_class,
                ToolOperationClass::Facing | ToolOperationClass::Contour2D
            ),
            Self::ThreadMill { .. } => matches!(op_class, ToolOperationClass::ThreadMill),
            Self::LollipopEndmill { .. } => matches!(
                op_class,
                ToolOperationClass::Deburr3D | ToolOperationClass::Undercut3D
            ),
            Self::SlotMill { .. } => matches!(op_class, ToolOperationClass::SlotUndercut),
            Self::DovetailCutter { .. } => matches!(op_class, ToolOperationClass::Dovetail),
        }
    }
}

/// Validation failure on a [`Tool`] definition.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum ToolValidationError {
    #[error("tool '{name}' has an empty ID or name")]
    EmptyIdentity { name: String },
    #[error("tool '{name}' cutting diameter ({diameter_mm} mm) must be finite and > 0")]
    InvalidDiameter { name: String, diameter_mm: f64 },
    #[error("tool '{name}' flute count must be >= 1")]
    ZeroFlutes { name: String },
    #[error(
        "tool '{name}' flute length ({flute_mm} mm) must be > 0 and <= overall length ({overall_mm} mm)"
    )]
    InvalidLengths {
        name: String,
        flute_mm: f64,
        overall_mm: f64,
    },
    #[error(
        "tool '{name}' stickout length ({stickout_mm} mm) must be >= flute length ({flute_mm} mm)"
    )]
    StickoutShorterThanFlutes {
        name: String,
        stickout_mm: f64,
        flute_mm: f64,
    },
    #[error(
        "bull-nose tool '{name}' corner radius ({radius_mm} mm) must satisfy 0 < R < D/2 ({half_d_mm} mm)"
    )]
    InvalidBullNoseRadius {
        name: String,
        radius_mm: f64,
        half_d_mm: f64,
    },
    #[error("chamfer/drill tool '{name}' angle ({angle_deg}°) must satisfy 0° < angle < 180°")]
    InvalidTipAngle { name: String, angle_deg: f64 },
    #[error(
        "chamfer tool '{name}' tip diameter ({tip_mm} mm) must satisfy 0 <= tip < diameter ({diameter_mm} mm)"
    )]
    InvalidChamferTip {
        name: String,
        tip_mm: f64,
        diameter_mm: f64,
    },
    #[error(
        "face mill '{name}' outside diameter ({outside_mm} mm) must be >= cutting diameter ({cutting_mm} mm)"
    )]
    InvalidFaceMillOutsideDiameter {
        name: String,
        cutting_mm: f64,
        outside_mm: f64,
    },
    #[error(
        "thread mill '{name}' neck diameter ({neck_diameter_mm} mm) must satisfy 0 < neck < cutting diameter ({diameter_mm} mm)"
    )]
    InvalidThreadMillNeck {
        name: String,
        neck_diameter_mm: f64,
        diameter_mm: f64,
    },
    #[error("thread mill '{name}' pitch ({pitch_mm} mm) must be finite and > 0")]
    InvalidThreadMillPitch { name: String, pitch_mm: f64 },
    #[error("thread mill '{name}' teeth count must be >= 1")]
    InvalidThreadMillTeethCount { name: String },
    #[error("thread mill '{name}' thread angle ({angle_deg}°) must be within 29°..=90°")]
    InvalidThreadMillAngle { name: String, angle_deg: f64 },
    #[error(
        "thread mill '{name}' neck length ({neck_length_mm} mm) must satisfy flute length ({flute_mm} mm) <= neck length < overall length ({overall_mm} mm)"
    )]
    InvalidThreadMillReach {
        name: String,
        flute_mm: f64,
        neck_length_mm: f64,
        overall_mm: f64,
    },
    #[error(
        "undercut tool '{name}' neck diameter ({neck_diameter_mm} mm) must satisfy 0 < neck < cutting diameter ({diameter_mm} mm)"
    )]
    InvalidRelievedNeck {
        name: String,
        neck_diameter_mm: f64,
        diameter_mm: f64,
    },
    #[error(
        "undercut tool '{name}' neck length ({neck_length_mm} mm) must satisfy cutting height ({cutting_height_mm} mm) <= neck length < overall length ({overall_mm} mm)"
    )]
    InvalidRelievedReach {
        name: String,
        cutting_height_mm: f64,
        neck_length_mm: f64,
        overall_mm: f64,
    },
    #[error(
        "lollipop endmill '{name}' flute height ({flute_mm} mm) must satisfy 0 < flute <= sphere diameter ({diameter_mm} mm)"
    )]
    InvalidLollipopHeight {
        name: String,
        flute_mm: f64,
        diameter_mm: f64,
    },
    #[error(
        "slot/dovetail tool '{name}' corner radius ({radius_mm} mm) must satisfy 0 <= R <= {max_radius_mm:.3} mm"
    )]
    InvalidUndercutCornerRadius {
        name: String,
        radius_mm: f64,
        max_radius_mm: f64,
    },
    #[error("dovetail cutter '{name}' side angle ({angle_deg}°) must satisfy 15° < angle < 85°")]
    InvalidDovetailAngle { name: String, angle_deg: f64 },
    #[error(
        "dovetail cutter '{name}' neck diameter ({neck_diameter_mm} mm) must satisfy 0 < neck <= derived top diameter ({top_diameter_mm:.3} mm)"
    )]
    InvalidDovetailTopNeck {
        name: String,
        neck_diameter_mm: f64,
        top_diameter_mm: f64,
    },
}

/// Complete CNC cutting tool definition with machine metadata and per-material [`CuttingPreset`]s.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Tool {
    pub id: ToolId,
    pub tool_number: u32,
    pub length_offset_number: u32,
    pub diameter_offset_number: u32,
    pub name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vendor: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub product_id: Option<String>,
    /// Original authoring unit (`mm` vs `in`) so imperial cutters display cleanly as `0.2500 in`.
    #[serde(default)]
    pub native_unit: DisplayUnit,
    #[serde(default)]
    pub spindle_direction: SpindleDirection,
    #[serde(default)]
    pub default_coolant: CoolantMode,
    pub stickout_length_mm: f64,
    pub geometry: CutterGeometry,
    #[serde(default)]
    pub presets: Vec<CuttingPreset>,
}

impl Tool {
    /// Validates physical cutter geometry and machine metadata invariants.
    pub fn validate(&self) -> Result<(), ToolValidationError> {
        if self.id.0.trim().is_empty() || self.name.trim().is_empty() {
            return Err(ToolValidationError::EmptyIdentity {
                name: self.name.clone(),
            });
        }
        let d = self.geometry.cutting_diameter_mm();
        if !d.is_finite() || d <= 0.0 {
            return Err(ToolValidationError::InvalidDiameter {
                name: self.name.clone(),
                diameter_mm: d,
            });
        }
        if self.geometry.flutes() == 0 {
            return Err(ToolValidationError::ZeroFlutes {
                name: self.name.clone(),
            });
        }
        let flute_mm = self.geometry.flute_length_mm();
        let overall_mm = self.geometry.overall_length_mm();
        if !flute_mm.is_finite()
            || !overall_mm.is_finite()
            || flute_mm <= 0.0
            || flute_mm > overall_mm + 1e-6
        {
            return Err(ToolValidationError::InvalidLengths {
                name: self.name.clone(),
                flute_mm,
                overall_mm,
            });
        }
        if !self.stickout_length_mm.is_finite() || self.stickout_length_mm + 1e-6 < flute_mm {
            return Err(ToolValidationError::StickoutShorterThanFlutes {
                name: self.name.clone(),
                stickout_mm: self.stickout_length_mm,
                flute_mm,
            });
        }

        match self.geometry {
            CutterGeometry::BullNoseEndmill {
                diameter_mm,
                corner_radius_mm,
                ..
            } => {
                let half_d = 0.5 * diameter_mm;
                if !corner_radius_mm.is_finite()
                    || corner_radius_mm <= 0.0
                    || corner_radius_mm >= half_d - 1e-6
                {
                    return Err(ToolValidationError::InvalidBullNoseRadius {
                        name: self.name.clone(),
                        radius_mm: corner_radius_mm,
                        half_d_mm: half_d,
                    });
                }
            }
            CutterGeometry::ChamferMill {
                diameter_mm,
                tip_diameter_mm,
                included_angle_deg,
                ..
            } => {
                if !included_angle_deg.is_finite()
                    || included_angle_deg <= 0.0
                    || included_angle_deg >= 180.0
                {
                    return Err(ToolValidationError::InvalidTipAngle {
                        name: self.name.clone(),
                        angle_deg: included_angle_deg,
                    });
                }
                if !tip_diameter_mm.is_finite()
                    || tip_diameter_mm < 0.0
                    || tip_diameter_mm >= diameter_mm - 1e-6
                {
                    return Err(ToolValidationError::InvalidChamferTip {
                        name: self.name.clone(),
                        tip_mm: tip_diameter_mm,
                        diameter_mm,
                    });
                }
            }
            CutterGeometry::TwistDrill {
                point_angle_deg, ..
            } => {
                if !point_angle_deg.is_finite()
                    || point_angle_deg <= 0.0
                    || point_angle_deg >= 180.0
                {
                    return Err(ToolValidationError::InvalidTipAngle {
                        name: self.name.clone(),
                        angle_deg: point_angle_deg,
                    });
                }
            }
            CutterGeometry::FaceMill {
                cutting_diameter_mm,
                outside_diameter_mm,
                ..
            } => {
                if !outside_diameter_mm.is_finite()
                    || outside_diameter_mm + 1e-6 < cutting_diameter_mm
                {
                    return Err(ToolValidationError::InvalidFaceMillOutsideDiameter {
                        name: self.name.clone(),
                        cutting_mm: cutting_diameter_mm,
                        outside_mm: outside_diameter_mm,
                    });
                }
            }
            CutterGeometry::ThreadMill {
                diameter_mm,
                neck_diameter_mm,
                neck_length_mm,
                pitch_mm,
                teeth_count,
                thread_angle_deg,
                flute_length_mm,
                overall_length_mm,
                ..
            } => {
                if !neck_diameter_mm.is_finite()
                    || neck_diameter_mm <= 0.0
                    || neck_diameter_mm >= diameter_mm - 1e-6
                {
                    return Err(ToolValidationError::InvalidThreadMillNeck {
                        name: self.name.clone(),
                        neck_diameter_mm,
                        diameter_mm,
                    });
                }
                if !pitch_mm.is_finite() || pitch_mm <= 0.0 {
                    return Err(ToolValidationError::InvalidThreadMillPitch {
                        name: self.name.clone(),
                        pitch_mm,
                    });
                }
                if teeth_count == 0 {
                    return Err(ToolValidationError::InvalidThreadMillTeethCount {
                        name: self.name.clone(),
                    });
                }
                if !thread_angle_deg.is_finite() || !(29.0..=90.0).contains(&thread_angle_deg) {
                    return Err(ToolValidationError::InvalidThreadMillAngle {
                        name: self.name.clone(),
                        angle_deg: thread_angle_deg,
                    });
                }
                if !neck_length_mm.is_finite()
                    || neck_length_mm + 1e-6 < flute_length_mm
                    || overall_length_mm <= neck_length_mm + 1e-6
                {
                    return Err(ToolValidationError::InvalidThreadMillReach {
                        name: self.name.clone(),
                        flute_mm: flute_length_mm,
                        neck_length_mm,
                        overall_mm: overall_length_mm,
                    });
                }
            }
            CutterGeometry::LollipopEndmill {
                diameter_mm,
                neck_diameter_mm,
                neck_length_mm,
                flute_length_mm,
                overall_length_mm,
                ..
            } => {
                if !neck_diameter_mm.is_finite()
                    || neck_diameter_mm <= 0.0
                    || neck_diameter_mm >= diameter_mm - 1e-6
                {
                    return Err(ToolValidationError::InvalidRelievedNeck {
                        name: self.name.clone(),
                        neck_diameter_mm,
                        diameter_mm,
                    });
                }
                if flute_length_mm > diameter_mm + 1e-6 {
                    return Err(ToolValidationError::InvalidLollipopHeight {
                        name: self.name.clone(),
                        flute_mm: flute_length_mm,
                        diameter_mm,
                    });
                }
                if !neck_length_mm.is_finite()
                    || neck_length_mm + 1e-6 < flute_length_mm
                    || overall_length_mm <= neck_length_mm + 1e-6
                {
                    return Err(ToolValidationError::InvalidRelievedReach {
                        name: self.name.clone(),
                        cutting_height_mm: flute_length_mm,
                        neck_length_mm,
                        overall_mm: overall_length_mm,
                    });
                }
            }
            CutterGeometry::SlotMill {
                diameter_mm,
                cutting_thickness_mm,
                corner_radius_mm,
                neck_diameter_mm,
                neck_length_mm,
                overall_length_mm,
                ..
            } => {
                if !neck_diameter_mm.is_finite()
                    || neck_diameter_mm <= 0.0
                    || neck_diameter_mm >= diameter_mm - 1e-6
                {
                    return Err(ToolValidationError::InvalidRelievedNeck {
                        name: self.name.clone(),
                        neck_diameter_mm,
                        diameter_mm,
                    });
                }
                let max_rc = (0.5 * cutting_thickness_mm)
                    .min(0.5 * diameter_mm - 1e-4)
                    .max(0.0);
                if !corner_radius_mm.is_finite()
                    || corner_radius_mm < 0.0
                    || corner_radius_mm > max_rc + 1e-6
                {
                    return Err(ToolValidationError::InvalidUndercutCornerRadius {
                        name: self.name.clone(),
                        radius_mm: corner_radius_mm,
                        max_radius_mm: max_rc,
                    });
                }
                if !neck_length_mm.is_finite()
                    || neck_length_mm + 1e-6 < cutting_thickness_mm
                    || overall_length_mm <= neck_length_mm + 1e-6
                {
                    return Err(ToolValidationError::InvalidRelievedReach {
                        name: self.name.clone(),
                        cutting_height_mm: cutting_thickness_mm,
                        neck_length_mm,
                        overall_mm: overall_length_mm,
                    });
                }
            }
            CutterGeometry::DovetailCutter {
                diameter_mm,
                side_angle_deg,
                cutting_height_mm,
                corner_radius_mm,
                neck_diameter_mm,
                neck_length_mm,
                overall_length_mm,
                ..
            } => {
                if !side_angle_deg.is_finite() || side_angle_deg <= 15.0 || side_angle_deg >= 85.0 {
                    return Err(ToolValidationError::InvalidDovetailAngle {
                        name: self.name.clone(),
                        angle_deg: side_angle_deg,
                    });
                }
                let max_rc = (0.5 * cutting_height_mm)
                    .min(0.5 * diameter_mm - 1e-4)
                    .max(0.0);
                if !corner_radius_mm.is_finite()
                    || corner_radius_mm < 0.0
                    || corner_radius_mm > max_rc + 1e-6
                {
                    return Err(ToolValidationError::InvalidUndercutCornerRadius {
                        name: self.name.clone(),
                        radius_mm: corner_radius_mm,
                        max_radius_mm: max_rc,
                    });
                }
                if !neck_diameter_mm.is_finite()
                    || neck_diameter_mm <= 0.0
                    || neck_diameter_mm >= diameter_mm - 1e-6
                {
                    return Err(ToolValidationError::InvalidRelievedNeck {
                        name: self.name.clone(),
                        neck_diameter_mm,
                        diameter_mm,
                    });
                }
                let top_d = CutterGeometry::dovetail_top_diameter_mm(
                    diameter_mm,
                    side_angle_deg,
                    cutting_height_mm,
                    corner_radius_mm,
                );
                if !top_d.is_finite() || top_d <= 1e-4 || neck_diameter_mm > top_d + 1e-6 {
                    return Err(ToolValidationError::InvalidDovetailTopNeck {
                        name: self.name.clone(),
                        neck_diameter_mm,
                        top_diameter_mm: top_d.max(0.0),
                    });
                }
                if !neck_length_mm.is_finite()
                    || neck_length_mm + 1e-6 < cutting_height_mm
                    || overall_length_mm <= neck_length_mm + 1e-6
                {
                    return Err(ToolValidationError::InvalidRelievedReach {
                        name: self.name.clone(),
                        cutting_height_mm,
                        neck_length_mm,
                        overall_mm: overall_length_mm,
                    });
                }
            }
            CutterGeometry::FlatEndmill { .. } | CutterGeometry::BallEndmill { .. } => {}
        }

        Ok(())
    }

    /// Injects tool-scoped variables (`tool_d`, `tool_r`, `flute_len`, `flutes`, `tip_angle`)
    /// into an [`EvalContext`] per `01-architecture.md`.
    pub fn populate_eval_context(&self, ctx: &mut EvalContext) {
        let d = self.geometry.cutting_diameter_mm();
        ctx.set_var("tool_d", d);
        ctx.set_var("tool_r", 0.5 * d);
        ctx.set_var("flute_len", self.geometry.flute_length_mm());
        ctx.set_scalar_var("flutes", self.geometry.flutes() as f64);
        ctx.set_scalar_var("tip_angle", self.geometry.tip_angle_rad());
    }

    /// Formats a concise display label such as `"T1 — Ø6.000 mm Flat Endmill (6mm 3FL Carbide)"`.
    pub fn summary_label(&self) -> String {
        let d = self.geometry.cutting_diameter_mm();
        format!(
            "T{} — Ø{} {} ({})",
            self.tool_number,
            self.native_unit.format_length(d),
            self.geometry.kind_label(),
            self.name
        )
    }
}

/// Returns `true` when `candidate` is an exact machine-swap match for `reference`:
/// identical [`ToolId`], same [`CutterGeometry`] variant, and cutting diameter within `1e-3 mm`.
pub fn is_exact_tool_match(candidate: &Tool, reference: &Tool) -> bool {
    candidate.id == reference.id
        && std::mem::discriminant(&candidate.geometry)
            == std::mem::discriminant(&reference.geometry)
        && (candidate.geometry.cutting_diameter_mm() - reference.geometry.cutting_diameter_mm())
            .abs()
            < 1e-3
}

/// Ranks `candidates` as replacements for `reference` in deterministic preference order:
/// 1. Same [`CutterGeometry`] variant first,
/// 2. Closest cutting diameter (`mm`),
/// 3. Closest flute count,
/// 4. Lowest `tool_number`,
/// 5. Lexicographic `id`.
pub fn rank_replacement_tools(reference: &Tool, candidates: &[Tool]) -> Vec<Tool> {
    let ref_disc = std::mem::discriminant(&reference.geometry);
    let ref_dia = reference.geometry.cutting_diameter_mm();
    let ref_flutes = reference.geometry.flutes();

    let mut ranked = candidates.to_vec();
    ranked.sort_by(|a, b| {
        let a_same = std::mem::discriminant(&a.geometry) == ref_disc;
        let b_same = std::mem::discriminant(&b.geometry) == ref_disc;
        b_same
            .cmp(&a_same)
            .then_with(|| {
                let da = (a.geometry.cutting_diameter_mm() - ref_dia).abs();
                let db = (b.geometry.cutting_diameter_mm() - ref_dia).abs();
                da.total_cmp(&db)
            })
            .then_with(|| {
                let fa = a.geometry.flutes().abs_diff(ref_flutes);
                let fb = b.geometry.flutes().abs_diff(ref_flutes);
                fa.cmp(&fb)
            })
            .then_with(|| a.tool_number.cmp(&b.tool_number))
            .then_with(|| a.id.cmp(&b.id))
    });
    ranked
}
