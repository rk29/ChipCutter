use serde::{Deserialize, Serialize};

/// Exact international inch definition in millimeters.
pub const MM_PER_INCH: f64 = 25.4;

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unit_conversions_are_exact() {
        assert!(approx_eq(DisplayUnit::Inch.to_mm(0.25), 6.35, 1e-12));
        assert!(approx_eq(DisplayUnit::Inch.from_mm(25.4), 1.0, 1e-12));
        assert_eq!(DisplayUnit::Millimeter.format_length(12.5), "12.500 mm");
        assert_eq!(DisplayUnit::Inch.format_length(6.35), "0.2500 in");
    }
}
