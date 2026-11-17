use serde::{Deserialize, Serialize};

use crate::model::CoolantMode;

/// Radial chip thinning diagnostic when `stepover_mm < 0.5 * diameter_mm`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChipThinningHint {
    /// Ratio `h_ex / f_z` in `(0, 1)`.
    pub thickness_ratio: f64,
    /// Actual maximum chip thickness (`mm`) experienced at the current radial stepover without compensation.
    pub actual_hex_mm: f64,
    /// Compensated cutting feedrate (`mm/min`) required to restore the target `chipload_mm_tooth`
    /// (capped at `2.5x` nominal feedrate for spindle/rigidity safety).
    pub compensated_feed_mm_min: f64,
}

/// Computes `RPM = (Vc_m_min * 1000) / (π * D_mm)`.
#[inline]
pub fn rpm_from_surface_speed(vc_m_min: f64, diameter_mm: f64) -> Option<f64> {
    if !vc_m_min.is_finite() || !diameter_mm.is_finite() || vc_m_min <= 0.0 || diameter_mm <= 1e-9 {
        return None;
    }
    Some((vc_m_min * 1000.0) / (std::f64::consts::PI * diameter_mm))
}

/// Computes `Vc_m_min = (π * D_mm * RPM) / 1000`.
#[inline]
pub fn surface_speed_from_rpm(rpm: f64, diameter_mm: f64) -> Option<f64> {
    if !rpm.is_finite() || !diameter_mm.is_finite() || rpm <= 0.0 || diameter_mm <= 1e-9 {
        return None;
    }
    Some((std::f64::consts::PI * diameter_mm * rpm) / 1000.0)
}

/// Computes `Vf = RPM * flutes * fz`.
#[inline]
pub fn feed_from_chipload(rpm: f64, flutes: u32, chipload_mm_tooth: f64) -> f64 {
    if flutes == 0 || rpm <= 0.0 || chipload_mm_tooth <= 0.0 {
        0.0
    } else {
        rpm * (flutes as f64) * chipload_mm_tooth
    }
}

/// Computes `fz = Vf / (RPM * flutes)`.
#[inline]
pub fn chipload_from_feed(rpm: f64, flutes: u32, feed_mm_min: f64) -> Option<f64> {
    let denom = rpm * (flutes as f64);
    if !denom.is_finite() || denom <= 1e-9 || !feed_mm_min.is_finite() || feed_mm_min < 0.0 {
        None
    } else {
        Some(feed_mm_min / denom)
    }
}

/// Computes radial chip thinning when `0 < stepover_mm < 0.5 * diameter_mm`.
pub fn compute_radial_chip_thinning(
    diameter_mm: f64,
    stepover_mm: f64,
    chipload_mm_tooth: f64,
    cutting_feed_mm_min: f64,
) -> Option<ChipThinningHint> {
    if diameter_mm <= 1e-6
        || stepover_mm <= 1e-6
        || stepover_mm >= 0.5 * diameter_mm - 1e-6
        || chipload_mm_tooth <= 0.0
    {
        return None;
    }
    let ae_over_d = (stepover_mm / diameter_mm).clamp(1e-6, 0.5);
    let ratio = (2.0 * (ae_over_d * (1.0 - ae_over_d)).sqrt()).clamp(0.05, 1.0);
    let actual_hex_mm = chipload_mm_tooth * ratio;
    let boost = (1.0 / ratio).min(2.5);
    Some(ChipThinningHint {
        thickness_ratio: ratio,
        actual_hex_mm,
        compensated_feed_mm_min: cutting_feed_mm_min * boost,
    })
}

/// Per-material cutting parameters coupled via `RPM ↔ Surface Speed` and `Chipload ↔ Feedrate`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CuttingPreset {
    pub id: String,
    pub name: String,
    pub material: String,
    pub spindle_rpm: f64,
    pub surface_speed_m_min: f64,
    pub cutting_feed_mm_min: f64,
    pub chipload_mm_tooth: f64,
    pub lead_feed_mm_min: f64,
    pub ramp_feed_mm_min: f64,
    pub plunge_feed_mm_min: f64,
    pub default_stepdown_mm: f64,
    pub default_stepover_mm: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub coolant_override: Option<CoolantMode>,
}

impl CuttingPreset {
    /// Creates a consistent preset from surface speed (`m/min`) and chipload (`mm/tooth`),
    /// deriving `lead = 0.8 * feed`, `ramp = 0.5 * feed`, and `plunge = 0.33 * feed` per `04-tool-library.md`.
    #[allow(clippy::too_many_arguments)]
    pub fn from_surface_speed(
        id: impl Into<String>,
        name: impl Into<String>,
        material: impl Into<String>,
        diameter_mm: f64,
        flutes: u32,
        surface_speed_m_min: f64,
        chipload_mm_tooth: f64,
        default_stepdown_mm: f64,
        default_stepover_mm: f64,
        coolant_override: Option<CoolantMode>,
    ) -> Self {
        let rpm = rpm_from_surface_speed(surface_speed_m_min, diameter_mm).unwrap_or(10_000.0);
        let feed = feed_from_chipload(rpm, flutes, chipload_mm_tooth);
        Self {
            id: id.into(),
            name: name.into(),
            material: material.into(),
            spindle_rpm: rpm,
            surface_speed_m_min,
            cutting_feed_mm_min: feed,
            chipload_mm_tooth,
            lead_feed_mm_min: 0.8 * feed,
            ramp_feed_mm_min: 0.5 * feed,
            plunge_feed_mm_min: (feed / 3.0).max(10.0),
            default_stepdown_mm,
            default_stepover_mm,
            coolant_override,
        }
    }

    /// Updates **Surface Speed (`m/min`)**, recalculating **Spindle RPM** and either preserving
    /// `chipload_mm_tooth` (`lock_chipload = true`, scaling linear feeds proportionally) or
    /// preserving `cutting_feed_mm_min` (`lock_chipload = false`, updating `chipload_mm_tooth`).
    pub fn set_surface_speed_m_min(
        &mut self,
        vc_m_min: f64,
        diameter_mm: f64,
        flutes: u32,
        lock_chipload: bool,
    ) {
        let Some(new_rpm) = rpm_from_surface_speed(vc_m_min, diameter_mm) else {
            return;
        };
        self.surface_speed_m_min = vc_m_min;
        self.apply_new_rpm(new_rpm, flutes, lock_chipload);
    }

    /// Updates **Spindle RPM**, recalculating **Surface Speed (`m/min`)** and coupling feedrate/chipload.
    pub fn set_spindle_rpm(
        &mut self,
        rpm: f64,
        diameter_mm: f64,
        flutes: u32,
        lock_chipload: bool,
    ) {
        if !rpm.is_finite() || rpm <= 0.0 {
            return;
        }
        if let Some(vc) = surface_speed_from_rpm(rpm, diameter_mm) {
            self.surface_speed_m_min = vc;
        }
        self.apply_new_rpm(rpm, flutes, lock_chipload);
    }

    fn apply_new_rpm(&mut self, new_rpm: f64, flutes: u32, lock_chipload: bool) {
        let old_feed = self.cutting_feed_mm_min;
        self.spindle_rpm = new_rpm;
        if lock_chipload {
            let new_feed = feed_from_chipload(new_rpm, flutes, self.chipload_mm_tooth);
            self.scale_secondary_feeds(old_feed, new_feed);
            self.cutting_feed_mm_min = new_feed;
        } else if let Some(new_fz) = chipload_from_feed(new_rpm, flutes, self.cutting_feed_mm_min) {
            self.chipload_mm_tooth = new_fz;
        }
    }

    /// Updates **Chipload (`mm/tooth`)**, recalculating **Cutting Feedrate (`mm/min`)** and scaling secondary feeds.
    pub fn set_chipload_mm_tooth(&mut self, fz_mm: f64, flutes: u32) {
        if !fz_mm.is_finite() || fz_mm <= 0.0 {
            return;
        }
        let old_feed = self.cutting_feed_mm_min;
        let new_feed = feed_from_chipload(self.spindle_rpm, flutes, fz_mm);
        self.chipload_mm_tooth = fz_mm;
        self.scale_secondary_feeds(old_feed, new_feed);
        self.cutting_feed_mm_min = new_feed;
    }

    /// Updates **Cutting Feedrate (`mm/min`)**, recalculating **Chipload (`mm/tooth`)** and scaling secondary feeds.
    pub fn set_cutting_feed_mm_min(&mut self, vf_mm_min: f64, flutes: u32) {
        if !vf_mm_min.is_finite() || vf_mm_min <= 0.0 {
            return;
        }
        let old_feed = self.cutting_feed_mm_min;
        if let Some(fz) = chipload_from_feed(self.spindle_rpm, flutes, vf_mm_min) {
            self.chipload_mm_tooth = fz;
        }
        self.scale_secondary_feeds(old_feed, vf_mm_min);
        self.cutting_feed_mm_min = vf_mm_min;
    }

    fn scale_secondary_feeds(&mut self, old_feed: f64, new_feed: f64) {
        if old_feed > 1e-6 && new_feed > 0.0 {
            let ratio = new_feed / old_feed;
            self.lead_feed_mm_min *= ratio;
            self.ramp_feed_mm_min *= ratio;
            self.plunge_feed_mm_min *= ratio;
        } else if new_feed > 0.0 {
            self.lead_feed_mm_min = 0.8 * new_feed;
            self.ramp_feed_mm_min = 0.5 * new_feed;
            self.plunge_feed_mm_min = new_feed / 3.0;
        }
    }
}
