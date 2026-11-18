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
}

impl CutterGeometry {
    pub const fn kind_label(&self) -> &'static str {
        match self {
            Self::FlatEndmill { .. } => "Flat Endmill",
            Self::BullNoseEndmill { .. } => "Bull-Nose Endmill",
            Self::BallEndmill { .. } => "Ball Endmill",
            Self::ChamferMill { .. } => "Chamfer / Spot Mill",
            Self::TwistDrill { .. } => "Twist Drill",
            Self::FaceMill { .. } => "Face / Fly Cutter",
        }
    }

    pub fn cutting_diameter_mm(&self) -> f64 {
        match *self {
            Self::FlatEndmill { diameter_mm, .. }
            | Self::BullNoseEndmill { diameter_mm, .. }
            | Self::BallEndmill { diameter_mm, .. }
            | Self::ChamferMill { diameter_mm, .. }
            | Self::TwistDrill { diameter_mm, .. } => diameter_mm,
            Self::FaceMill {
                cutting_diameter_mm,
                ..
            } => cutting_diameter_mm,
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
            } => flute_length_mm,
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
            | Self::FaceMill { flutes, .. } => flutes,
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
            ),
            Self::BullNoseEndmill { .. } => matches!(
                op_class,
                ToolOperationClass::Facing
                    | ToolOperationClass::Contour2D
                    | ToolOperationClass::Pocket2D
                    | ToolOperationClass::HelicalBore
                    | ToolOperationClass::Trace
            ),
            Self::BallEndmill { .. } => matches!(
                op_class,
                ToolOperationClass::Contour2D
                    | ToolOperationClass::Pocket2D
                    | ToolOperationClass::Trace
            ),
            Self::ChamferMill {
                included_angle_deg, ..
            } => match op_class {
                ToolOperationClass::VCarve => (10.0..=160.0).contains(included_angle_deg),
                ToolOperationClass::Chamfer
                | ToolOperationClass::Trace
                | ToolOperationClass::Drill => true,
                _ => false,
            },
            Self::TwistDrill { .. } => matches!(op_class, ToolOperationClass::Drill),
            Self::FaceMill { .. } => matches!(
                op_class,
                ToolOperationClass::Facing | ToolOperationClass::Contour2D
            ),
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
