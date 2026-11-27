use serde::{Deserialize, Serialize};

/// Exact international inch definition in millimeters.
pub const MM_PER_INCH: f64 = 25.4;

/// Exact international foot definition in meters (`1 ft == 0.3048 m`), used for `SFM <-> m/min`.
pub const METERS_PER_FOOT: f64 = 0.3048;

/// Curve endpoint chaining and point coincidence tolerance (`0.1 µm`).
pub const COINCIDENCE_TOL_MM: f64 = 1e-4;

/// Tolerance used when recognizing analytic Planes, Cylinders, Cones, and Circles from B-Rep entities.
pub const ANALYTIC_CLASSIFY_TOL_MM: f64 = 1e-5;

/// Default arc-fitting / discretization fallback tolerance (`0.01 mm`).
pub const DEFAULT_ARC_FIT_TOL_MM: f64 = 0.01;

/// Default 3D viewport tessellation chord tolerance (`0.02 mm`).
pub const DEFAULT_TESS_CHORD_TOL_MM: f64 = 0.02;

/// Tolerance-aware floating-point equality check.
#[inline]
pub fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

/// User-facing length display unit. Internal storage is always canonical `f64` millimeters.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "lowercase")]
pub enum DisplayUnit {
    #[default]
    #[serde(alias = "mm", alias = "millimeter", alias = "millimeters")]
    Millimeter,
    #[serde(alias = "in", alias = "inch", alias = "inches")]
    Inch,
}

impl DisplayUnit {
    /// Short suffix label (`"mm"` or `"in"`).
    pub const fn suffix(self) -> &'static str {
        match self {
            Self::Millimeter => "mm",
            Self::Inch => "in",
        }
    }

    /// Converts a canonical millimeter value into this display unit.
    #[inline]
    pub fn from_mm(self, mm: f64) -> f64 {
        match self {
            Self::Millimeter => mm,
            Self::Inch => mm / MM_PER_INCH,
        }
    }

    /// Converts a value expressed in this display unit into canonical millimeters.
    #[inline]
    pub fn to_mm(self, val: f64) -> f64 {
        match self {
            Self::Millimeter => val,
            Self::Inch => val * MM_PER_INCH,
        }
    }

    /// Converts an area in `mm²` into `mm²` or `in²`.
    #[inline]
    pub fn area_from_mm2(self, mm2: f64) -> f64 {
        match self {
            Self::Millimeter => mm2,
            Self::Inch => mm2 / (MM_PER_INCH * MM_PER_INCH),
        }
    }

    /// Converts an area expressed in `mm²` or `in²` into canonical `mm²`.
    #[inline]
    pub fn area_to_mm2(self, val: f64) -> f64 {
        match self {
            Self::Millimeter => val,
            Self::Inch => val * (MM_PER_INCH * MM_PER_INCH),
        }
    }

    /// Feedrate unit suffix (`"mm/min"` or `"ipm"`).
    pub const fn feed_suffix(self) -> &'static str {
        match self {
            Self::Millimeter => "mm/min",
            Self::Inch => "ipm",
        }
    }

    /// Converts canonical `mm/min` feedrate into display units (`mm/min` or `in/min`).
    #[inline]
    pub fn feed_from_mm_min(self, mm_min: f64) -> f64 {
        self.from_mm(mm_min)
    }

    /// Converts display-unit feedrate (`mm/min` or `ipm`) into canonical `mm/min`.
    #[inline]
    pub fn feed_to_mm_min(self, val: f64) -> f64 {
        self.to_mm(val)
    }

    /// Surface speed unit suffix (`"m/min"` or `"SFM"`).
    pub const fn surface_speed_suffix(self) -> &'static str {
        match self {
            Self::Millimeter => "m/min",
            Self::Inch => "SFM",
        }
    }

    /// Converts canonical `m/min` surface speed into display units (`m/min` or `SFM`).
    #[inline]
    pub fn surface_speed_from_m_min(self, m_min: f64) -> f64 {
        match self {
            Self::Millimeter => m_min,
            Self::Inch => m_min / METERS_PER_FOOT,
        }
    }

    /// Converts display-unit surface speed (`m/min` or `SFM`) into canonical `m/min`.
    #[inline]
    pub fn surface_speed_to_m_min(self, val: f64) -> f64 {
        match self {
            Self::Millimeter => val,
            Self::Inch => val * METERS_PER_FOOT,
        }
    }

    /// Chipload unit suffix (`"mm/tooth"` or `"in/tooth"`).
    pub const fn chipload_suffix(self) -> &'static str {
        match self {
            Self::Millimeter => "mm/tooth",
            Self::Inch => "in/tooth",
        }
    }

    /// Converts canonical `mm/tooth` chipload into display units (`mm/tooth` or `in/tooth`).
    #[inline]
    pub fn chipload_from_mm(self, mm_tooth: f64) -> f64 {
        self.from_mm(mm_tooth)
    }

    /// Converts display-unit chipload (`mm/tooth` or `in/tooth`) into canonical `mm/tooth`.
    #[inline]
    pub fn chipload_to_mm(self, val: f64) -> f64 {
        self.to_mm(val)
    }

    /// Formats a millimeter length with appropriate decimal precision and unit suffix.
    pub fn format_length(self, mm: f64) -> String {
        match self {
            Self::Millimeter => format!("{:.3} mm", mm),
            Self::Inch => format!("{:.4} in", mm / MM_PER_INCH),
        }
    }

    /// Formats a bare numeric value in this display unit without trailing zeros noise.
    pub fn format_number(self, mm: f64) -> String {
        let raw = match self {
            Self::Millimeter => format!("{:.4}", mm),
            Self::Inch => format!("{:.5}", mm / MM_PER_INCH),
        };
        let trimmed = raw.trim_end_matches('0').trim_end_matches('.');
        if trimmed.is_empty() || trimmed == "-0" {
            "0".to_string()
        } else {
            trimmed.to_string()
        }
    }
}

/// Formats a non-negative duration in seconds as `HH:MM:SS`.
pub fn format_cycle_time_hms(seconds: f64) -> String {
    let total = if seconds.is_finite() {
        seconds.max(0.0).round() as u64
    } else {
        0
    };
    let hrs = total / 3600;
    let mins = (total % 3600) / 60;
    let secs = total % 60;
    format!("{hrs:02}:{mins:02}:{secs:02}")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unit_conversions_are_exact() {
        assert!(approx_eq(DisplayUnit::Inch.to_mm(0.25), 6.35, 1e-12));
        assert!(approx_eq(DisplayUnit::Inch.from_mm(25.4), 1.0, 1e-12));
        assert!(approx_eq(
            DisplayUnit::Inch.area_from_mm2(645.16),
            1.0,
            1e-12
        ));
        assert!(approx_eq(DisplayUnit::Inch.area_to_mm2(1.0), 645.16, 1e-12));
        assert_eq!(DisplayUnit::Millimeter.format_length(12.5), "12.500 mm");
        assert_eq!(DisplayUnit::Inch.format_length(6.35), "0.2500 in");
    }
}
