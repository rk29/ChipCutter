//! Rotary angular unwinding, inverse-time (`G93`) feedrate, and angular (`G94`) feedrate
//! helpers for 4th-axis post-processing (`07-post-processor.md`).

use chip_cl::RotaryUnwindMode;
use chip_math::DVec3;

/// Resolves the commanded target rotary angle (`deg`) from `current_deg` to `target_deg`
/// according to `unwind_mode`.
pub fn resolve_rotary_target_deg(
    current_deg: f64,
    target_deg: f64,
    unwind_mode: RotaryUnwindMode,
) -> f64 {
    if !current_deg.is_finite() || !target_deg.is_finite() {
        return 0.0;
    }
    match unwind_mode {
        RotaryUnwindMode::ContinuousMultiTurn => target_deg,
        RotaryUnwindMode::WrapShortestPath => {
            let mut delta = (target_deg - current_deg).rem_euclid(360.0);
            if delta > 180.0 {
                delta -= 360.0;
            }
            current_deg + delta
        }
        RotaryUnwindMode::Modulo0To360 => {
            let m = target_deg.rem_euclid(360.0);
            if (m - 360.0).abs() < 1e-9 { 0.0 } else { m }
        }
    }
}

/// Computes the combined workpiece-surface travel distance (`mm`) for a simultaneous linear +
/// rotary motion segment from `(start_wcs, start_deg)` to `(end_wcs, end_deg)` at `effective_radius_mm`.
#[inline]
pub fn compute_workpiece_arc_distance_mm(
    start_wcs: DVec3,
    end_wcs: DVec3,
    start_deg: f64,
    end_deg: f64,
    effective_radius_mm: f64,
) -> f64 {
    let linear_dist = start_wcs.distance(end_wcs);
    let d_rad = (end_deg - start_deg).abs().to_radians();
    let rotary_arc_dist = effective_radius_mm.max(0.0) * d_rad;
    linear_dist.hypot(rotary_arc_dist)
}

/// Computes the exact NIST RS274NGC / LinuxCNC / grblHAL **`G93` Inverse-Time Feedrate** (`1/min`)
/// for a combined linear + rotary motion segment so the cutter moves along the workpiece surface
/// at `target_surface_feed_mm_min`.
///
/// Returns `None` if motion duration is zero (degenerate zero-length segment).
pub fn compute_inverse_time_feed_g93(
    start_wcs: DVec3,
    end_wcs: DVec3,
    start_deg: f64,
    end_deg: f64,
    effective_radius_mm: f64,
    target_surface_feed_mm_min: f64,
    max_rotary_feed_deg_min: f64,
) -> Option<f64> {
    if !target_surface_feed_mm_min.is_finite() || target_surface_feed_mm_min <= 1e-6 {
        return None;
    }
    let d_deg = (end_deg - start_deg).abs();
    let combined_dist = compute_workpiece_arc_distance_mm(
        start_wcs,
        end_wcs,
        start_deg,
        end_deg,
        effective_radius_mm,
    );

    let dt_surface_min = combined_dist / target_surface_feed_mm_min;
    let dt_rotary_min = if d_deg > 1e-9 && max_rotary_feed_deg_min > 1e-6 {
        d_deg / max_rotary_feed_deg_min
    } else {
        0.0
    };
    let dt_min = dt_surface_min.max(dt_rotary_min);
    if !dt_min.is_finite() || dt_min <= 1e-9 {
        return None;
    }
    Some((1.0 / dt_min).clamp(0.01, 99_999.0))
}

/// Computes the **`G94` pure angular feedrate** (`deg/min`) matching `surface_feed_mm_min` at
/// `effective_radius_mm`, clamped to `max_deg_min`.
pub fn compute_rotary_angular_feed_deg_min(
    surface_feed_mm_min: f64,
    effective_radius_mm: f64,
    max_deg_min: f64,
) -> f64 {
    let max_f = max_deg_min.max(1.0);
    if !surface_feed_mm_min.is_finite() || surface_feed_mm_min <= 1e-6 {
        return max_f;
    }
    let r = effective_radius_mm.max(0.25);
    let deg_min = (surface_feed_mm_min / r).to_degrees();
    deg_min.clamp(1.0, max_f)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::approx_eq;

    use super::*;

    #[test]
    fn g93_inverse_time_feed_and_unwinding_modes_are_exact() {
        // Pure rotary 90° move at R = 20 mm -> arc length = 20 * pi/2 = 31.4159 mm
        // At 628.3185 mm/min -> dt = 0.05 min -> F_g93 = 20.0 (1/min)
        let f_g93 = compute_inverse_time_feed_g93(
            DVec3::new(0.0, 0.0, 20.0),
            DVec3::new(0.0, 0.0, 20.0),
            0.0,
            90.0,
            20.0,
            20.0 * std::f64::consts::PI * 10.0,
            36_000.0,
        )
        .unwrap();
        assert!(approx_eq(f_g93, 20.0, 1e-6), "expected 20.0, got {f_g93}");

        // Combined axial (30 mm) + rotary arc (40 mm) -> hypotenuse 50 mm @ 500 mm/min -> dt=0.1 -> F=10.0
        let arc_deg = (40.0_f64 / 20.0).to_degrees();
        let f_comb = compute_inverse_time_feed_g93(
            DVec3::new(0.0, 0.0, 20.0),
            DVec3::new(30.0, 0.0, 20.0),
            0.0,
            arc_deg,
            20.0,
            500.0,
            36_000.0,
        )
        .unwrap();
        assert!(approx_eq(f_comb, 10.0, 1e-6), "expected 10.0, got {f_comb}");

        // Pure angular G94 feedrate: surface_feed = 20 * pi mm/min @ R = 20 mm -> 180 deg/min
        let f_deg = compute_rotary_angular_feed_deg_min(20.0 * std::f64::consts::PI, 20.0, 3_600.0);
        assert!(
            approx_eq(f_deg, 180.0, 1e-6),
            "expected 180.0 deg/min, got {f_deg}"
        );

        // Unwinding policies
        assert!(approx_eq(
            resolve_rotary_target_deg(350.0, 10.0, RotaryUnwindMode::WrapShortestPath),
            370.0,
            1e-9
        ));
        assert!(approx_eq(
            resolve_rotary_target_deg(10.0, 350.0, RotaryUnwindMode::WrapShortestPath),
            -10.0,
            1e-9
        ));
        assert!(approx_eq(
            resolve_rotary_target_deg(720.0, 1080.0, RotaryUnwindMode::ContinuousMultiTurn),
            1080.0,
            1e-9
        ));
        assert!(approx_eq(
            resolve_rotary_target_deg(350.0, -90.0, RotaryUnwindMode::Modulo0To360),
            270.0,
            1e-9
        ));
    }
}
