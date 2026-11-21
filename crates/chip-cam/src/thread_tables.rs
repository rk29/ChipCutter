use serde::{Deserialize, Serialize};

/// Fundamental triangle height ratio `H / P = sqrt(3) / 2` for standard 60° V-threads
/// (ISO 68-1 metric and ASME B1.1 Unified inch threads).
pub const ISO_60_H_OVER_P: f64 = 0.866_025_403_784_438_6;

/// Millimeters per inch (`25.4 mm/in`).
const MM_PER_IN: f64 = 25.4;

/// Computes the exact ISO 68-1 / ASME B1.1 60° pitch diameter `D2 = D - (3/4) * H`.
#[inline]
pub const fn iso60_pitch_diameter_mm(major_diameter_mm: f64, pitch_mm: f64) -> f64 {
    major_diameter_mm - 0.75 * ISO_60_H_OVER_P * pitch_mm
}

/// Computes the exact ISO 68-1 / ASME B1.1 60° internal minor diameter `D1 = D - (5/4) * H`.
#[inline]
pub const fn iso60_minor_diameter_internal_mm(major_diameter_mm: f64, pitch_mm: f64) -> f64 {
    major_diameter_mm - 1.25 * ISO_60_H_OVER_P * pitch_mm
}

/// Computes the exact ISO 68-1 / ASME B1.1 60° external minor diameter `d3 = D - (17/12) * H`.
#[inline]
pub const fn iso60_minor_diameter_external_mm(major_diameter_mm: f64, pitch_mm: f64) -> f64 {
    major_diameter_mm - (17.0 / 12.0) * ISO_60_H_OVER_P * pitch_mm
}

/// Standard thread family classification (`45-milestone-45-plan.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ThreadStandardKind {
    #[default]
    IsoMetricCoarse,
    IsoMetricFine,
    UnifiedUnc,
    UnifiedUnf,
    Custom,
}

impl ThreadStandardKind {
    pub const ALL: [Self; 5] = [
        Self::IsoMetricCoarse,
        Self::IsoMetricFine,
        Self::UnifiedUnc,
        Self::UnifiedUnf,
        Self::Custom,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::IsoMetricCoarse => "ISO Metric Coarse",
            Self::IsoMetricFine => "ISO Metric Fine",
            Self::UnifiedUnc => "Unified Coarse (UNC)",
            Self::UnifiedUnf => "Unified Fine (UNF)",
            Self::Custom => "Custom",
        }
    }
}

/// Canonical standard thread specification entry in `f64` millimeters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThreadSpecEntry {
    pub designation: &'static str,
    pub standard: ThreadStandardKind,
    pub major_diameter_mm: f64,
    pub pitch_mm: f64,
    pub minor_diameter_internal_mm: f64,
    pub pitch_diameter_mm: f64,
    pub recommended_tap_drill_mm: f64,
}

const fn metric_entry(
    designation: &'static str,
    standard: ThreadStandardKind,
    major_mm: f64,
    pitch_mm: f64,
    tap_drill_mm: f64,
) -> ThreadSpecEntry {
    ThreadSpecEntry {
        designation,
        standard,
        major_diameter_mm: major_mm,
        pitch_mm,
        minor_diameter_internal_mm: iso60_minor_diameter_internal_mm(major_mm, pitch_mm),
        pitch_diameter_mm: iso60_pitch_diameter_mm(major_mm, pitch_mm),
        recommended_tap_drill_mm: tap_drill_mm,
    }
}

const fn unified_entry(
    designation: &'static str,
    standard: ThreadStandardKind,
    major_in: f64,
    tpi: f64,
    tap_drill_in: f64,
) -> ThreadSpecEntry {
    let major_mm = major_in * MM_PER_IN;
    let pitch_mm = MM_PER_IN / tpi;
    ThreadSpecEntry {
        designation,
        standard,
        major_diameter_mm: major_mm,
        pitch_mm,
        minor_diameter_internal_mm: iso60_minor_diameter_internal_mm(major_mm, pitch_mm),
        pitch_diameter_mm: iso60_pitch_diameter_mm(major_mm, pitch_mm),
        recommended_tap_drill_mm: tap_drill_in * MM_PER_IN,
    }
}

/// Complete built-in ISO Metric Coarse, ISO Metric Fine, Unified UNC, and Unified UNF thread table.
static STANDARD_THREAD_SPECS: &[ThreadSpecEntry] = &[
    // --- ISO Metric Coarse (M2 through M24) ---
    metric_entry(
        "M2 x 0.4",
        ThreadStandardKind::IsoMetricCoarse,
        2.0,
        0.4,
        1.6,
    ),
    metric_entry(
        "M2.5 x 0.45",
        ThreadStandardKind::IsoMetricCoarse,
        2.5,
        0.45,
        2.05,
    ),
    metric_entry(
        "M3 x 0.5",
        ThreadStandardKind::IsoMetricCoarse,
        3.0,
        0.5,
        2.5,
    ),
    metric_entry(
        "M4 x 0.7",
        ThreadStandardKind::IsoMetricCoarse,
        4.0,
        0.7,
        3.3,
    ),
    metric_entry(
        "M5 x 0.8",
        ThreadStandardKind::IsoMetricCoarse,
        5.0,
        0.8,
        4.2,
    ),
    metric_entry(
        "M6 x 1.0",
        ThreadStandardKind::IsoMetricCoarse,
        6.0,
        1.0,
        5.0,
    ),
    metric_entry(
        "M8 x 1.25",
        ThreadStandardKind::IsoMetricCoarse,
        8.0,
        1.25,
        6.8,
    ),
    metric_entry(
        "M10 x 1.5",
        ThreadStandardKind::IsoMetricCoarse,
        10.0,
        1.5,
        8.5,
    ),
    metric_entry(
        "M12 x 1.75",
        ThreadStandardKind::IsoMetricCoarse,
        12.0,
        1.75,
        10.2,
    ),
    metric_entry(
        "M14 x 2.0",
        ThreadStandardKind::IsoMetricCoarse,
        14.0,
        2.0,
        12.0,
    ),
    metric_entry(
        "M16 x 2.0",
        ThreadStandardKind::IsoMetricCoarse,
        16.0,
        2.0,
        14.0,
    ),
    metric_entry(
        "M20 x 2.5",
        ThreadStandardKind::IsoMetricCoarse,
        20.0,
        2.5,
        17.5,
    ),
    metric_entry(
        "M24 x 3.0",
        ThreadStandardKind::IsoMetricCoarse,
        24.0,
        3.0,
        21.0,
    ),
    // --- ISO Metric Fine (M6x0.75 through M16x1.5) ---
    metric_entry(
        "M6 x 0.75",
        ThreadStandardKind::IsoMetricFine,
        6.0,
        0.75,
        5.25,
    ),
    metric_entry("M8 x 1.0", ThreadStandardKind::IsoMetricFine, 8.0, 1.0, 7.0),
    metric_entry(
        "M10 x 1.0",
        ThreadStandardKind::IsoMetricFine,
        10.0,
        1.0,
        9.0,
    ),
    metric_entry(
        "M10 x 1.25",
        ThreadStandardKind::IsoMetricFine,
        10.0,
        1.25,
        8.8,
    ),
    metric_entry(
        "M12 x 1.25",
        ThreadStandardKind::IsoMetricFine,
        12.0,
        1.25,
        10.8,
    ),
    metric_entry(
        "M12 x 1.5",
        ThreadStandardKind::IsoMetricFine,
        12.0,
        1.5,
        10.5,
    ),
    metric_entry(
        "M16 x 1.5",
        ThreadStandardKind::IsoMetricFine,
        16.0,
        1.5,
        14.5,
    ),
    // --- Unified Coarse UNC (#4-40 through 3/4-10) ---
    unified_entry(
        "#4-40 UNC",
        ThreadStandardKind::UnifiedUnc,
        0.112,
        40.0,
        0.0890,
    ),
    unified_entry(
        "#6-32 UNC",
        ThreadStandardKind::UnifiedUnc,
        0.138,
        32.0,
        0.1065,
    ),
    unified_entry(
        "#8-32 UNC",
        ThreadStandardKind::UnifiedUnc,
        0.164,
        32.0,
        0.1360,
    ),
    unified_entry(
        "#10-24 UNC",
        ThreadStandardKind::UnifiedUnc,
        0.190,
        24.0,
        0.1495,
    ),
    unified_entry(
        "1/4-20 UNC",
        ThreadStandardKind::UnifiedUnc,
        0.250,
        20.0,
        0.2010,
    ),
    unified_entry(
        "5/16-18 UNC",
        ThreadStandardKind::UnifiedUnc,
        0.3125,
        18.0,
        0.2570,
    ),
    unified_entry(
        "3/8-16 UNC",
        ThreadStandardKind::UnifiedUnc,
        0.375,
        16.0,
        0.3125,
    ),
    unified_entry(
        "7/16-14 UNC",
        ThreadStandardKind::UnifiedUnc,
        0.4375,
        14.0,
        0.3680,
    ),
    unified_entry(
        "1/2-13 UNC",
        ThreadStandardKind::UnifiedUnc,
        0.500,
        13.0,
        0.421875,
    ),
    unified_entry(
        "5/8-11 UNC",
        ThreadStandardKind::UnifiedUnc,
        0.625,
        11.0,
        0.53125,
    ),
    unified_entry(
        "3/4-10 UNC",
        ThreadStandardKind::UnifiedUnc,
        0.750,
        10.0,
        0.65625,
    ),
    // --- Unified Fine UNF (#6-40 through 1/2-20) ---
    unified_entry(
        "#6-40 UNF",
        ThreadStandardKind::UnifiedUnf,
        0.138,
        40.0,
        0.1130,
    ),
    unified_entry(
        "#8-36 UNF",
        ThreadStandardKind::UnifiedUnf,
        0.164,
        36.0,
        0.1360,
    ),
    unified_entry(
        "#10-32 UNF",
        ThreadStandardKind::UnifiedUnf,
        0.190,
        32.0,
        0.1590,
    ),
    unified_entry(
        "1/4-28 UNF",
        ThreadStandardKind::UnifiedUnf,
        0.250,
        28.0,
        0.2130,
    ),
    unified_entry(
        "5/16-24 UNF",
        ThreadStandardKind::UnifiedUnf,
        0.3125,
        24.0,
        0.2720,
    ),
    unified_entry(
        "3/8-24 UNF",
        ThreadStandardKind::UnifiedUnf,
        0.375,
        24.0,
        0.3320,
    ),
    unified_entry(
        "1/2-20 UNF",
        ThreadStandardKind::UnifiedUnf,
        0.500,
        20.0,
        0.453125,
    ),
];

/// Returns the complete deterministic built-in standard thread table.
pub fn all_thread_specs() -> &'static [ThreadSpecEntry] {
    STANDARD_THREAD_SPECS
}

/// Returns all standard thread specifications belonging to `standard`.
pub fn thread_specs_for_standard(standard: ThreadStandardKind) -> Vec<&'static ThreadSpecEntry> {
    STANDARD_THREAD_SPECS
        .iter()
        .filter(|entry| entry.standard == standard)
        .collect()
}

fn normalize_designation(raw: &str) -> String {
    raw.trim()
        .replace('×', "x")
        .to_ascii_lowercase()
        .split_whitespace()
        .collect::<Vec<_>>()
        .join("")
}

/// Looks up a standard thread specification by designation (case-insensitive, tolerant of
/// `x` vs `×` and spacing, e.g., `"M8 x 1.25"`, `"M8×1.25"`, `"1/4-20 UNC"`).
pub fn find_thread_spec(designation: &str) -> Option<&'static ThreadSpecEntry> {
    let needle = normalize_designation(designation);
    if needle.is_empty() {
        return None;
    }
    STANDARD_THREAD_SPECS
        .iter()
        .find(|entry| normalize_designation(entry.designation) == needle)
}

/// Suggests standard thread sizes whose internal minor / tap-drill diameter matches a picked
/// cylindrical B-Rep hole diameter (`minor_dia_mm`), ordered closest-first.
pub fn suggest_thread_for_hole_diameter(minor_dia_mm: f64) -> Vec<&'static ThreadSpecEntry> {
    if !minor_dia_mm.is_finite() || minor_dia_mm <= 0.0 {
        return Vec::new();
    }
    let tol = (0.04 * minor_dia_mm).clamp(0.18, 0.45);
    let mut matches: Vec<(f64, usize, &'static ThreadSpecEntry)> = STANDARD_THREAD_SPECS
        .iter()
        .enumerate()
        .filter_map(|(idx, entry)| {
            let err_tap = (minor_dia_mm - entry.recommended_tap_drill_mm).abs();
            let err_minor = (minor_dia_mm - entry.minor_diameter_internal_mm).abs();
            let best_err = err_tap.min(err_minor);
            if best_err <= tol && minor_dia_mm < entry.major_diameter_mm {
                Some((best_err, idx, entry))
            } else {
                None
            }
        })
        .collect();

    matches.sort_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.cmp(&b.1)));
    matches.into_iter().map(|(_, _, entry)| entry).collect()
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::approx_eq;

    use super::*;

    #[test]
    fn thread_tables_cover_all_standards_and_satisfy_iso60_formulas() {
        let all = all_thread_specs();
        assert_eq!(
            thread_specs_for_standard(ThreadStandardKind::IsoMetricCoarse).len(),
            13
        );
        assert_eq!(
            thread_specs_for_standard(ThreadStandardKind::IsoMetricFine).len(),
            7
        );
        assert_eq!(
            thread_specs_for_standard(ThreadStandardKind::UnifiedUnc).len(),
            11
        );
        assert_eq!(
            thread_specs_for_standard(ThreadStandardKind::UnifiedUnf).len(),
            7
        );
        assert_eq!(all.len(), 38);

        for entry in all {
            assert!(entry.major_diameter_mm > 0.0);
            assert!(entry.pitch_mm > 0.0);
            assert!(entry.minor_diameter_internal_mm > 0.0);
            assert!(entry.minor_diameter_internal_mm < entry.pitch_diameter_mm);
            assert!(entry.pitch_diameter_mm < entry.major_diameter_mm);
            assert!(entry.recommended_tap_drill_mm > 0.0);
            assert!(entry.recommended_tap_drill_mm < entry.major_diameter_mm);

            let h = 0.5 * 3.0_f64.sqrt() * entry.pitch_mm;
            assert!(approx_eq(
                entry.pitch_diameter_mm,
                entry.major_diameter_mm - 0.75 * h,
                1e-9
            ));
            assert!(approx_eq(
                entry.minor_diameter_internal_mm,
                entry.major_diameter_mm - 1.25 * h,
                1e-9
            ));
        }

        // Spot-check canonical shop sizes
        let m6 = find_thread_spec("M6 x 1.0").unwrap();
        assert!(approx_eq(m6.major_diameter_mm, 6.0, 1e-9));
        assert!(approx_eq(m6.pitch_mm, 1.0, 1e-9));
        assert!(approx_eq(m6.minor_diameter_internal_mm, 4.917_468, 1e-4));
        assert!(approx_eq(m6.pitch_diameter_mm, 5.350_481, 1e-4));
        assert!(approx_eq(m6.recommended_tap_drill_mm, 5.0, 1e-9));

        let m8 = find_thread_spec("m8×1.25").unwrap();
        assert_eq!(m8.designation, "M8 x 1.25");
        assert!(approx_eq(m8.recommended_tap_drill_mm, 6.8, 1e-9));

        let qtr20 = find_thread_spec("1/4-20 UNC").unwrap();
        assert!(approx_eq(qtr20.major_diameter_mm, 6.35, 1e-9));
        assert!(approx_eq(qtr20.pitch_mm, 1.27, 1e-9));
        assert!(approx_eq(qtr20.recommended_tap_drill_mm, 5.1054, 1e-4));

        let unf10 = find_thread_spec("#10-32 UNF").unwrap();
        assert!(approx_eq(unf10.major_diameter_mm, 4.826, 1e-6));
        assert!(approx_eq(unf10.pitch_mm, 25.4 / 32.0, 1e-9));
    }

    #[test]
    fn suggests_matching_threads_for_drilled_pilot_holes() {
        let m6_sugg = suggest_thread_for_hole_diameter(5.0);
        assert!(!m6_sugg.is_empty());
        assert_eq!(m6_sugg[0].designation, "M6 x 1.0");

        let m8_sugg = suggest_thread_for_hole_diameter(6.8);
        assert!(!m8_sugg.is_empty());
        assert_eq!(m8_sugg[0].designation, "M8 x 1.25");

        let qtr20_sugg = suggest_thread_for_hole_diameter(5.11);
        assert!(!qtr20_sugg.is_empty());
        assert_eq!(qtr20_sugg[0].designation, "1/4-20 UNC");

        let unf10_sugg = suggest_thread_for_hole_diameter(4.04);
        assert!(!unf10_sugg.is_empty());
        assert_eq!(unf10_sugg[0].designation, "#10-32 UNF");

        assert!(suggest_thread_for_hole_diameter(0.0).is_empty());
        assert!(suggest_thread_for_hole_diameter(f64::NAN).is_empty());
    }
}
