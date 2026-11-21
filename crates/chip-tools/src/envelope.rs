//! Shared canonical `f64` millimeter analytic cutter envelope (`AnalyticCutterEnvelope`)
//! used by both `chip-cam` (3D Drop-Cutter / Push-Cutter contact) and `chip-sim`
//! (Heightmap, Tri-Dexel, and Cylindrical stock removal & 5-rule verification).

use crate::model::CutterGeometry;

/// Canonical shape classification of an [`AnalyticCutterEnvelope`] in `f64` millimeters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CutterEnvelopeShape {
    Flat {
        radius_mm: f64,
    },
    BullNose {
        flat_radius_mm: f64,
        corner_radius_mm: f64,
    },
    Ball {
        radius_mm: f64,
    },
    /// Conical tip (`ChamferMill` or `TwistDrill`).
    Conical {
        tip_radius_mm: f64,
        upper_radius_mm: f64,
        half_angle_rad: f64,
        tan_half_angle: f64,
    },
    /// Face / fly mill with optional corner radius and 45° outer lead bevel.
    Face {
        flat_radius_mm: f64,
        bottom_radius_mm: f64,
        outer_radius_mm: f64,
        corner_radius_mm: f64,
    },
    /// Non-monotonic relieved-neck V-tooth thread mill.
    ThreadMill {
        cutting_radius_mm: f64,
        neck_radius_mm: f64,
        neck_length_mm: f64,
        pitch_mm: f64,
        teeth_count: u32,
        thread_angle_deg: f64,
    },
}

/// Exact canonical `f64` millimeter analytic envelope covering all 7 [`CutterGeometry`] variants,
/// including exact horizontal Minkowski disk dilation for radial/axial stock-to-leave and
/// non-monotonic relieved-neck V-tooth bands for [`CutterGeometry::ThreadMill`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AnalyticCutterEnvelope {
    shape: CutterEnvelopeShape,
    cutting_radius_mm: f64,
    flute_length_mm: f64,
    overall_length_mm: f64,
    shank_radius_mm: f64,
    stock_radial_mm: f64,
    stock_axial_mm: f64,
}

impl AnalyticCutterEnvelope {
    /// Builds the exact analytic `f64` mm envelope for any of the 7 [`CutterGeometry`] variants.
    pub fn from_geometry(geom: &CutterGeometry) -> Self {
        let flute_length_mm = geom.flute_length_mm().max(0.1);
        let overall_length_mm = geom.overall_length_mm().max(flute_length_mm);
        let shank_radius_mm = (0.5 * geom.shank_diameter_mm()).max(0.25);

        let (cutting_radius_mm, shape) = match *geom {
            CutterGeometry::FlatEndmill { diameter_mm, .. } => {
                let r = (0.5 * diameter_mm).max(1e-4);
                (r, CutterEnvelopeShape::Flat { radius_mm: r })
            }
            CutterGeometry::BullNoseEndmill {
                diameter_mm,
                corner_radius_mm,
                ..
            } => {
                let r = (0.5 * diameter_mm).max(1e-4);
                let rc = corner_radius_mm.clamp(0.0, r);
                let flat_r = (r - rc).max(0.0);
                let shape = if rc <= 1e-6 {
                    CutterEnvelopeShape::Flat { radius_mm: r }
                } else if flat_r <= 1e-9 {
                    CutterEnvelopeShape::Ball { radius_mm: rc }
                } else {
                    CutterEnvelopeShape::BullNose {
                        flat_radius_mm: flat_r,
                        corner_radius_mm: rc,
                    }
                };
                (r, shape)
            }
            CutterGeometry::BallEndmill { diameter_mm, .. } => {
                let r = (0.5 * diameter_mm).max(1e-4);
                (r, CutterEnvelopeShape::Ball { radius_mm: r })
            }
            CutterGeometry::ChamferMill {
                diameter_mm,
                tip_diameter_mm,
                included_angle_deg,
                ..
            } => {
                let r_upper = (0.5 * diameter_mm).max(1e-4);
                let r_tip = (0.5 * tip_diameter_mm).clamp(0.0, r_upper);
                let half_angle_rad = (0.5 * included_angle_deg).clamp(1.0, 89.5).to_radians();
                let tan_half_angle = half_angle_rad.tan().max(1e-6);
                (
                    r_upper,
                    CutterEnvelopeShape::Conical {
                        tip_radius_mm: r_tip,
                        upper_radius_mm: r_upper,
                        half_angle_rad,
                        tan_half_angle,
                    },
                )
            }
            CutterGeometry::TwistDrill {
                diameter_mm,
                point_angle_deg,
                ..
            } => {
                let r_upper = (0.5 * diameter_mm).max(1e-4);
                let half_angle_rad = (0.5 * point_angle_deg).clamp(5.0, 89.5).to_radians();
                let tan_half_angle = half_angle_rad.tan().max(1e-6);
                (
                    r_upper,
                    CutterEnvelopeShape::Conical {
                        tip_radius_mm: 0.0,
                        upper_radius_mm: r_upper,
                        half_angle_rad,
                        tan_half_angle,
                    },
                )
            }
            CutterGeometry::FaceMill {
                cutting_diameter_mm,
                outside_diameter_mm,
                corner_radius_mm,
                ..
            } => {
                let bottom_r = (0.5 * cutting_diameter_mm).max(1e-4);
                let outer_r = (0.5 * outside_diameter_mm).max(bottom_r);
                let rc = corner_radius_mm.clamp(0.0, bottom_r);
                let flat_r = (bottom_r - rc).max(0.0);
                (
                    outer_r,
                    CutterEnvelopeShape::Face {
                        flat_radius_mm: flat_r,
                        bottom_radius_mm: bottom_r,
                        outer_radius_mm: outer_r,
                        corner_radius_mm: rc,
                    },
                )
            }
            CutterGeometry::ThreadMill {
                diameter_mm,
                neck_diameter_mm,
                neck_length_mm,
                pitch_mm,
                teeth_count,
                thread_angle_deg,
                ..
            } => {
                let r = (0.5 * diameter_mm).max(1e-4);
                let neck_r = (0.5 * neck_diameter_mm).clamp(0.02_f64.min(r), r);
                let neck_len = neck_length_mm.max(flute_length_mm);
                (
                    r,
                    CutterEnvelopeShape::ThreadMill {
                        cutting_radius_mm: r,
                        neck_radius_mm: neck_r,
                        neck_length_mm: neck_len,
                        pitch_mm: pitch_mm.max(0.05),
                        teeth_count: teeth_count.max(1),
                        thread_angle_deg: thread_angle_deg.clamp(29.0, 90.0),
                    },
                )
            }
        };

        Self {
            shape,
            cutting_radius_mm,
            flute_length_mm,
            overall_length_mm,
            shank_radius_mm,
            stock_radial_mm: 0.0,
            stock_axial_mm: 0.0,
        }
    }

    /// Returns a dilated copy of `self` folding radial stock-to-leave (`radial_mm >= 0`) via exact
    /// horizontal Minkowski disk dilation and recording axial stock-to-leave (`axial_mm >= 0`):
    /// - `Flat(R) ⊕ Disk(sr)` = `Flat(R + sr)`
    /// - `Ball(R) ⊕ Disk(sr)` = `BullNose(flat = sr, corner = R)` (preserves exact `dz(0) = 0` on flats)
    /// - `BullNose(r0, rc) ⊕ Disk(sr)` = `BullNose(r0 + sr, rc)`
    /// - `Conical(r0, r1, α) ⊕ Disk(sr)` = `Conical(r0 + sr, r1 + sr, α)`
    /// - `Face(r0, rb, ro, rc) ⊕ Disk(sr)` = `Face(r0 + sr, rb + sr, ro + sr, rc)`
    /// - `ThreadMill(R, Rn) ⊕ Disk(sr)` = `ThreadMill(R + sr, Rn + sr)`
    pub fn with_stock_allowance(&self, radial_mm: f64, axial_mm: f64) -> Self {
        let sr = radial_mm.max(0.0);
        let sz = axial_mm.max(0.0);
        if sr <= 0.0 && sz <= 0.0 {
            return *self;
        }

        let (cutting_radius_mm, shape) = match self.shape {
            CutterEnvelopeShape::Flat { radius_mm } => {
                let r = radius_mm + sr;
                (r, CutterEnvelopeShape::Flat { radius_mm: r })
            }
            CutterEnvelopeShape::Ball { radius_mm } => {
                let r_max = radius_mm + sr;
                let shape = if sr > 1e-9 {
                    CutterEnvelopeShape::BullNose {
                        flat_radius_mm: sr,
                        corner_radius_mm: radius_mm,
                    }
                } else {
                    CutterEnvelopeShape::Ball { radius_mm }
                };
                (r_max, shape)
            }
            CutterEnvelopeShape::BullNose {
                flat_radius_mm,
                corner_radius_mm,
            } => {
                let new_flat = flat_radius_mm + sr;
                (
                    new_flat + corner_radius_mm,
                    CutterEnvelopeShape::BullNose {
                        flat_radius_mm: new_flat,
                        corner_radius_mm,
                    },
                )
            }
            CutterEnvelopeShape::Conical {
                tip_radius_mm,
                upper_radius_mm,
                half_angle_rad,
                tan_half_angle,
            } => {
                let new_upper = upper_radius_mm + sr;
                (
                    new_upper,
                    CutterEnvelopeShape::Conical {
                        tip_radius_mm: tip_radius_mm + sr,
                        upper_radius_mm: new_upper,
                        half_angle_rad,
                        tan_half_angle,
                    },
                )
            }
            CutterEnvelopeShape::Face {
                flat_radius_mm,
                bottom_radius_mm,
                outer_radius_mm,
                corner_radius_mm,
            } => {
                let new_outer = outer_radius_mm + sr;
                (
                    new_outer,
                    CutterEnvelopeShape::Face {
                        flat_radius_mm: flat_radius_mm + sr,
                        bottom_radius_mm: bottom_radius_mm + sr,
                        outer_radius_mm: new_outer,
                        corner_radius_mm,
                    },
                )
            }
            CutterEnvelopeShape::ThreadMill {
                cutting_radius_mm,
                neck_radius_mm,
                neck_length_mm,
                pitch_mm,
                teeth_count,
                thread_angle_deg,
            } => {
                let new_cut = cutting_radius_mm + sr;
                (
                    new_cut,
                    CutterEnvelopeShape::ThreadMill {
                        cutting_radius_mm: new_cut,
                        neck_radius_mm: neck_radius_mm + sr,
                        neck_length_mm,
                        pitch_mm,
                        teeth_count,
                        thread_angle_deg,
                    },
                )
            }
        };

        Self {
            shape,
            cutting_radius_mm,
            flute_length_mm: self.flute_length_mm,
            overall_length_mm: self.overall_length_mm,
            shank_radius_mm: self.shank_radius_mm + sr,
            stock_radial_mm: self.stock_radial_mm + sr,
            stock_axial_mm: self.stock_axial_mm + sz,
        }
    }

    #[inline]
    pub const fn shape(&self) -> CutterEnvelopeShape {
        self.shape
    }

    /// Maximum horizontal cutting envelope radius (`mm`), including any radial stock allowance.
    #[inline]
    pub const fn cutting_radius_mm(&self) -> f64 {
        self.cutting_radius_mm
    }

    #[inline]
    pub const fn flute_length_mm(&self) -> f64 {
        self.flute_length_mm
    }

    #[inline]
    pub const fn overall_length_mm(&self) -> f64 {
        self.overall_length_mm
    }

    #[inline]
    pub const fn shank_radius_mm(&self) -> f64 {
        self.shank_radius_mm
    }

    #[inline]
    pub const fn stock_radial_mm(&self) -> f64 {
        self.stock_radial_mm
    }

    #[inline]
    pub const fn stock_axial_mm(&self) -> f64 {
        self.stock_axial_mm
    }

    #[inline]
    pub const fn is_thread_mill(&self) -> bool {
        matches!(self.shape, CutterEnvelopeShape::ThreadMill { .. })
    }

    /// Relieved neck radius (`mm`) for [`CutterGeometry::ThreadMill`]; equals `cutting_radius_mm`
    /// for monotonic cutters.
    #[inline]
    pub const fn neck_radius_mm(&self) -> f64 {
        match self.shape {
            CutterEnvelopeShape::ThreadMill { neck_radius_mm, .. } => neck_radius_mm,
            _ => self.cutting_radius_mm,
        }
    }

    /// Usable relieved-neck reach (`mm`) for [`CutterGeometry::ThreadMill`]; equals `flute_length_mm`
    /// for monotonic cutters.
    #[inline]
    pub const fn effective_reach_mm(&self) -> f64 {
        match self.shape {
            CutterEnvelopeShape::ThreadMill { neck_length_mm, .. } => neck_length_mm,
            _ => self.flute_length_mm,
        }
    }

    /// Height above the cutter tip (`mm`) over which the cutter radius expands from tip to `cutting_radius_mm`.
    #[inline]
    pub fn tip_profile_height_mm(&self) -> f64 {
        match self.shape {
            CutterEnvelopeShape::Flat { .. } => 0.0,
            CutterEnvelopeShape::Ball { radius_mm } => radius_mm,
            CutterEnvelopeShape::BullNose {
                corner_radius_mm, ..
            } => corner_radius_mm,
            CutterEnvelopeShape::Conical {
                tip_radius_mm,
                upper_radius_mm,
                tan_half_angle,
                ..
            } => ((upper_radius_mm - tip_radius_mm).max(0.0) / tan_half_angle.max(1e-9))
                .min(self.flute_length_mm),
            CutterEnvelopeShape::Face {
                bottom_radius_mm,
                outer_radius_mm,
                corner_radius_mm,
                ..
            } => (corner_radius_mm + (outer_radius_mm - bottom_radius_mm).max(0.0))
                .min(self.flute_length_mm),
            CutterEnvelopeShape::ThreadMill { .. } => 0.0,
        }
    }

    /// Returns `cot(α) = 1 / tan(α)` when the cutter has a conical flute envelope
    /// (`ChamferMill` / `TwistDrill`), or `None` otherwise.
    #[inline]
    pub fn conical_slope_cot(&self) -> Option<f64> {
        match self.shape {
            CutterEnvelopeShape::Conical { tan_half_angle, .. } => {
                Some(1.0 / tan_half_angle.max(1e-6))
            }
            _ => None,
        }
    }

    /// Returns `(z_start, tooth_step, n_teeth, tan_half_angle)` describing the periodic V-tooth
    /// cutting band of a [`CutterGeometry::ThreadMill`] within `z ∈ [0, flute_length_mm]`.
    #[inline]
    pub fn thread_mill_tooth_band_params(&self) -> (f64, f64, u32, f64) {
        match self.shape {
            CutterEnvelopeShape::ThreadMill {
                pitch_mm,
                teeth_count,
                thread_angle_deg,
                ..
            } => {
                let n = teeth_count.max(1);
                let lf = self.flute_length_mm.max(1e-4);
                let pitch = pitch_mm.max(0.05);
                let tooth_step = if n == 1 {
                    lf
                } else if (n as f64) * pitch <= lf + 1e-6 {
                    pitch
                } else {
                    lf / (n as f64)
                };
                let band_h = (n as f64) * tooth_step;
                let z_start = 0.5 * (lf - band_h).max(0.0);
                let half_rad = (0.5 * thread_angle_deg.clamp(29.0, 90.0)).to_radians();
                let tan_half = half_rad.tan().max(1e-6);
                (z_start, tooth_step, n, tan_half)
            }
            _ => (0.0, self.flute_length_mm.max(1e-4), 1, 1.0),
        }
    }

    /// Evaluates the V-tooth radial envelope `R_tooth(z_local)` inside `[0, flute_length_mm]`
    /// for a [`CutterGeometry::ThreadMill`], clamped to `[neck_radius_mm, cutting_radius_mm]`.
    /// For non-thread-mill cutters, returns `cutting_radius_mm`.
    #[inline]
    pub fn thread_mill_tooth_radius_at_z(&self, z_local: f64) -> f64 {
        let CutterEnvelopeShape::ThreadMill {
            cutting_radius_mm,
            neck_radius_mm,
            ..
        } = self.shape
        else {
            return self.cutting_radius_mm;
        };
        let (z_start, tooth_step, n, tan_half) = self.thread_mill_tooth_band_params();
        let band_h = (n as f64) * tooth_step;
        if z_local < z_start - 1e-9 || z_local > z_start + band_h + 1e-9 {
            return neck_radius_mm;
        }
        let local = (z_local - z_start).clamp(0.0, band_h);
        let idx = ((local / tooth_step).floor() as u32).min(n - 1);
        let z_crest = z_start + (idx as f64 + 0.5) * tooth_step;
        let dz_from_crest = (z_local - z_crest).abs();
        let r_flank = cutting_radius_mm - dz_from_crest / tan_half;
        r_flank.clamp(neck_radius_mm, cutting_radius_mm)
    }

    /// Evaluates axial offset `dz >= 0` from the cutter tip `(0, 0, 0)` to the lower cutting
    /// envelope at radial distance `r_mm` (`0 <= r_mm <= cutting_radius_mm`).
    /// Returns `None` if `r_mm` is non-finite, negative, or exceeds `cutting_radius_mm + 1e-9`.
    #[inline]
    pub fn profile_dz_at_radius(&self, r_mm: f64) -> Option<f64> {
        if !r_mm.is_finite() || r_mm < 0.0 || r_mm > self.cutting_radius_mm + 1e-9 {
            return None;
        }
        let r = r_mm.min(self.cutting_radius_mm);
        let dz = match self.shape {
            CutterEnvelopeShape::Flat { .. } => 0.0,
            CutterEnvelopeShape::BullNose {
                flat_radius_mm,
                corner_radius_mm,
            } => {
                if r <= flat_radius_mm || corner_radius_mm <= 1e-9 {
                    0.0
                } else {
                    let dr = (r - flat_radius_mm).min(corner_radius_mm);
                    corner_radius_mm
                        - (corner_radius_mm * corner_radius_mm - dr * dr)
                            .max(0.0)
                            .sqrt()
                }
            }
            CutterEnvelopeShape::Ball { radius_mm } => {
                let rc = r.min(radius_mm);
                radius_mm - (radius_mm * radius_mm - rc * rc).max(0.0).sqrt()
            }
            CutterEnvelopeShape::Conical {
                tip_radius_mm,
                tan_half_angle,
                ..
            } => {
                if r <= tip_radius_mm {
                    0.0
                } else {
                    (r - tip_radius_mm) / tan_half_angle
                }
            }
            CutterEnvelopeShape::Face {
                flat_radius_mm,
                bottom_radius_mm,
                corner_radius_mm,
                ..
            } => {
                if r <= flat_radius_mm {
                    0.0
                } else if r <= bottom_radius_mm {
                    if corner_radius_mm <= 1e-9 {
                        0.0
                    } else {
                        let dr = (r - flat_radius_mm).min(corner_radius_mm);
                        corner_radius_mm
                            - (corner_radius_mm * corner_radius_mm - dr * dr)
                                .max(0.0)
                                .sqrt()
                    }
                } else {
                    corner_radius_mm + (r - bottom_radius_mm)
                }
            }
            CutterEnvelopeShape::ThreadMill {
                cutting_radius_mm,
                neck_radius_mm,
                ..
            } => {
                if r <= neck_radius_mm {
                    0.0
                } else {
                    let (z_start, tooth_step, _n, tan_half) = self.thread_mill_tooth_band_params();
                    let z_first_crest = z_start + 0.5 * tooth_step;
                    (z_first_crest - (cutting_radius_mm - r).max(0.0) * tan_half).max(0.0)
                }
            }
        };
        Some(dz)
    }

    /// Evaluates the exact cutting envelope radius `r(dz) >= 0` at axial height `dz_mm >= 0` above
    /// the cutter tip `(0, 0, 0)` (exact inverse of [`Self::profile_dz_at_radius`]).
    ///
    /// Returns `None` if `dz_mm` is non-finite or `< -1e-9`. For [`CutterGeometry::ThreadMill`],
    /// returns the non-monotonic V-tooth radius in `[0, flute_length_mm]`, `neck_radius_mm`
    /// along `(flute_length_mm, neck_length_mm]`, and `shank_radius_mm` above `neck_length_mm`.
    #[inline]
    pub fn radius_at_dz(&self, dz_mm: f64) -> Option<f64> {
        if !dz_mm.is_finite() || dz_mm < -1e-9 {
            return None;
        }
        let dz = dz_mm.max(0.0);
        let r = match self.shape {
            CutterEnvelopeShape::Flat { radius_mm } => radius_mm,
            CutterEnvelopeShape::BullNose {
                flat_radius_mm,
                corner_radius_mm,
            } => {
                if corner_radius_mm <= 1e-9 || dz >= corner_radius_mm {
                    flat_radius_mm + corner_radius_mm
                } else {
                    (flat_radius_mm + (2.0 * corner_radius_mm * dz - dz * dz).max(0.0).sqrt())
                        .min(self.cutting_radius_mm)
                }
            }
            CutterEnvelopeShape::Ball { radius_mm } => {
                if dz >= radius_mm {
                    radius_mm
                } else {
                    (2.0 * radius_mm * dz - dz * dz)
                        .max(0.0)
                        .sqrt()
                        .min(self.cutting_radius_mm)
                }
            }
            CutterEnvelopeShape::Conical {
                tip_radius_mm,
                upper_radius_mm,
                tan_half_angle,
                ..
            } => (tip_radius_mm + dz * tan_half_angle).min(upper_radius_mm),
            CutterEnvelopeShape::Face {
                flat_radius_mm,
                bottom_radius_mm,
                outer_radius_mm,
                corner_radius_mm,
            } => {
                if dz < corner_radius_mm && corner_radius_mm > 1e-9 {
                    (flat_radius_mm + (2.0 * corner_radius_mm * dz - dz * dz).max(0.0).sqrt())
                        .min(bottom_radius_mm)
                } else {
                    (bottom_radius_mm + (dz - corner_radius_mm)).min(outer_radius_mm)
                }
            }
            CutterEnvelopeShape::ThreadMill {
                neck_radius_mm,
                neck_length_mm,
                ..
            } => {
                if dz <= self.flute_length_mm + 1e-9 {
                    self.thread_mill_tooth_radius_at_z(dz)
                } else if dz <= neck_length_mm + 1e-9 {
                    neck_radius_mm
                } else {
                    self.shank_radius_mm
                }
            }
        };
        Some(r.max(0.0))
    }

    /// Evaluates the height-dependent cutter radius `R(z_local)` at axial height `z_local` above
    /// the cutter tip within `[0, overall_length_mm]`, supporting non-monotonic undercut profiles
    /// such as relieved-neck thread mills.
    #[inline]
    pub fn cutter_radius_at_z(&self, z_local: f64) -> Option<f64> {
        let max_z = self.overall_length_mm.max(self.effective_reach_mm());
        self.cutter_radius_at_z_within(z_local, max_z)
    }

    /// Evaluates the height-dependent cutter radius `R(z_local)` clipped to `[0, max_z_mm]`
    /// (e.g., `stickout_length_mm` when verifying a mounted tool assembly):
    /// - `[0, flute_length_mm]`: cutting-tooth / flute envelope
    /// - `(flute_length_mm, neck_length_mm]`: relieved neck for `ThreadMill`
    /// - `(effective_reach_mm, max_z_mm]`: cylindrical shank (`shank_radius_mm`)
    #[inline]
    pub fn cutter_radius_at_z_within(&self, z_local: f64, max_z_mm: f64) -> Option<f64> {
        let max_z = max_z_mm.max(self.effective_reach_mm());
        if !z_local.is_finite() || z_local < -1e-9 || z_local > max_z + 1e-9 {
            return None;
        }
        let z = z_local.clamp(0.0, max_z);
        if let CutterEnvelopeShape::ThreadMill {
            neck_radius_mm,
            neck_length_mm,
            ..
        } = self.shape
        {
            if z <= self.flute_length_mm + 1e-9 {
                Some(self.thread_mill_tooth_radius_at_z(z))
            } else if z <= neck_length_mm + 1e-9 {
                Some(neck_radius_mm)
            } else {
                Some(self.shank_radius_mm)
            }
        } else if z <= self.flute_length_mm + 1e-9 {
            self.radius_at_dz(z)
        } else {
            Some(self.shank_radius_mm)
        }
    }

    /// Appends exact local axial intervals `(dz_lo, dz_hi)` where a [`CutterGeometry::ThreadMill`]
    /// occupies radial distance `r_mm` from the spindle axis (non-monotonic undercut support):
    /// - `r_mm > cutting_radius_mm`: pushes nothing
    /// - `r_mm <= neck_radius_mm`: pushes `(0.0, neck_length_mm)` (core + relieved neck)
    /// - `neck_radius_mm < r_mm <= cutting_radius_mm`: pushes one V-tooth interval per tooth.
    #[inline]
    pub fn push_thread_mill_axial_intervals_at_radius(&self, r_mm: f64, out: &mut Vec<(f64, f64)>) {
        let CutterEnvelopeShape::ThreadMill {
            cutting_radius_mm,
            neck_radius_mm,
            neck_length_mm,
            ..
        } = self.shape
        else {
            return;
        };
        if !r_mm.is_finite() || r_mm < 0.0 || r_mm > cutting_radius_mm + 1e-9 {
            return;
        }
        if r_mm <= neck_radius_mm + 1e-9 {
            out.push((0.0, neck_length_mm));
            return;
        }
        let (z_start, tooth_step, n, tan_half) = self.thread_mill_tooth_band_params();
        let half_w = (cutting_radius_mm - r_mm).max(0.0) * tan_half;
        for k in 0..n {
            let z_k_lo = z_start + (k as f64) * tooth_step;
            let z_k_hi = z_k_lo + tooth_step;
            let z_crest = z_k_lo + 0.5 * tooth_step;
            let lo = (z_crest - half_w).max(z_k_lo);
            let hi = (z_crest + half_w).min(z_k_hi);
            if hi > lo + 1e-9 {
                out.push((lo, hi));
            }
        }
    }

    /// Returns all exact local axial intervals `(dz_lo, dz_hi)` where a [`CutterGeometry::ThreadMill`]
    /// occupies radial distance `r_mm` from the spindle axis.
    pub fn thread_mill_axial_intervals_at_radius(&self, r_mm: f64) -> Vec<(f64, f64)> {
        let mut out = Vec::new();
        self.push_thread_mill_axial_intervals_at_radius(r_mm, &mut out);
        out
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::approx_eq;

    use super::*;

    #[test]
    fn analytic_cutter_envelope_is_exact_across_all_seven_geometries() {
        // 1. FlatEndmill
        let flat_g = CutterGeometry::FlatEndmill {
            diameter_mm: 6.0,
            flute_length_mm: 18.0,
            overall_length_mm: 50.0,
            shank_diameter_mm: 6.0,
            flutes: 3,
        };
        let flat = AnalyticCutterEnvelope::from_geometry(&flat_g);
        assert!(approx_eq(flat.cutting_radius_mm(), 3.0, 1e-9));
        assert!(approx_eq(flat.effective_reach_mm(), 18.0, 1e-9));
        assert!(flat.conical_slope_cot().is_none());
        assert!(approx_eq(
            flat.profile_dz_at_radius(0.0).unwrap(),
            0.0,
            1e-9
        ));
        assert!(approx_eq(
            flat.profile_dz_at_radius(3.0).unwrap(),
            0.0,
            1e-9
        ));
        assert!(flat.profile_dz_at_radius(3.01).is_none());
        assert!(approx_eq(flat.radius_at_dz(0.0).unwrap(), 3.0, 1e-9));
        assert!(approx_eq(flat.radius_at_dz(10.0).unwrap(), 3.0, 1e-9));
        assert!(flat.radius_at_dz(-0.1).is_none());

        // 2. BullNoseEndmill (D=10, R=2 -> flat=3, corner=2)
        let bull_g = CutterGeometry::BullNoseEndmill {
            diameter_mm: 10.0,
            corner_radius_mm: 2.0,
            flute_length_mm: 20.0,
            overall_length_mm: 60.0,
            shank_diameter_mm: 10.0,
            flutes: 4,
        };
        let bull = AnalyticCutterEnvelope::from_geometry(&bull_g);
        assert!(approx_eq(
            bull.profile_dz_at_radius(0.0).unwrap(),
            0.0,
            1e-9
        ));
        assert!(approx_eq(
            bull.profile_dz_at_radius(3.0).unwrap(),
            0.0,
            1e-9
        ));
        assert!(approx_eq(
            bull.profile_dz_at_radius(5.0).unwrap(),
            2.0,
            1e-9
        ));
        let dz_mid = bull.profile_dz_at_radius(4.0).unwrap();
        assert!(approx_eq(dz_mid, 2.0 - 3.0_f64.sqrt(), 1e-9));
        assert!(approx_eq(bull.radius_at_dz(dz_mid).unwrap(), 4.0, 1e-9));
        assert!(approx_eq(bull.radius_at_dz(2.0).unwrap(), 5.0, 1e-9));
        assert!(approx_eq(bull.radius_at_dz(5.0).unwrap(), 5.0, 1e-9));

        // 3. BallEndmill (D=6 -> R=3)
        let ball_g = CutterGeometry::BallEndmill {
            diameter_mm: 6.0,
            flute_length_mm: 18.0,
            overall_length_mm: 50.0,
            shank_diameter_mm: 6.0,
            flutes: 2,
        };
        let ball = AnalyticCutterEnvelope::from_geometry(&ball_g);
        assert!(approx_eq(
            ball.profile_dz_at_radius(0.0).unwrap(),
            0.0,
            1e-9
        ));
        assert!(approx_eq(
            ball.profile_dz_at_radius(3.0).unwrap(),
            3.0,
            1e-9
        ));
        let ball_dz = ball.profile_dz_at_radius(1.5).unwrap();
        assert!(approx_eq(ball_dz, 3.0 - (9.0 - 2.25_f64).sqrt(), 1e-9));
        assert!(approx_eq(ball.radius_at_dz(ball_dz).unwrap(), 1.5, 1e-9));

        // 4. ChamferMill (D=6, tip=0.2 -> r_tip=0.1, 90° -> tan(45°)=1, cot=1)
        let cham_g = CutterGeometry::ChamferMill {
            diameter_mm: 6.0,
            tip_diameter_mm: 0.2,
            included_angle_deg: 90.0,
            flute_length_mm: 10.0,
            overall_length_mm: 50.0,
            shank_diameter_mm: 6.0,
            flutes: 2,
        };
        let cham = AnalyticCutterEnvelope::from_geometry(&cham_g);
        assert!(approx_eq(cham.conical_slope_cot().unwrap(), 1.0, 1e-9));
        assert!(approx_eq(
            cham.profile_dz_at_radius(0.05).unwrap(),
            0.0,
            1e-9
        ));
        assert!(approx_eq(
            cham.profile_dz_at_radius(1.1).unwrap(),
            1.0,
            1e-9
        ));
        assert!(approx_eq(cham.radius_at_dz(1.0).unwrap(), 1.1, 1e-9));
        assert!(approx_eq(cham.radius_at_dz(5.0).unwrap(), 3.0, 1e-9));

        // 5. TwistDrill (D=6, 118° -> half=59°)
        let drill_g = CutterGeometry::TwistDrill {
            diameter_mm: 6.0,
            point_angle_deg: 118.0,
            flute_length_mm: 30.0,
            overall_length_mm: 70.0,
            shank_diameter_mm: 6.0,
            flutes: 2,
        };
        let drill = AnalyticCutterEnvelope::from_geometry(&drill_g);
        let expected_cot = 1.0 / 59.0_f64.to_radians().tan();
        assert!(approx_eq(
            drill.conical_slope_cot().unwrap(),
            expected_cot,
            1e-9
        ));
        assert!(approx_eq(
            drill.profile_dz_at_radius(3.0).unwrap(),
            3.0 * expected_cot,
            1e-9
        ));
        assert!(approx_eq(
            drill.radius_at_dz(1.0 * expected_cot).unwrap(),
            1.0,
            1e-9
        ));

        // 6. FaceMill (cutting D=38 -> bottom_r=19, outside D=40 -> outer_r=20, corner R=0.8)
        let face_g = CutterGeometry::FaceMill {
            cutting_diameter_mm: 38.0,
            outside_diameter_mm: 40.0,
            corner_radius_mm: 0.8,
            flute_length_mm: 10.0,
            overall_length_mm: 45.0,
            shank_diameter_mm: 12.0,
            flutes: 4,
        };
        let face = AnalyticCutterEnvelope::from_geometry(&face_g);
        assert!(approx_eq(face.cutting_radius_mm(), 20.0, 1e-9));
        assert!(approx_eq(
            face.profile_dz_at_radius(10.0).unwrap(),
            0.0,
            1e-9
        ));
        assert!(approx_eq(
            face.profile_dz_at_radius(19.0).unwrap(),
            0.8,
            1e-9
        ));
        assert!(approx_eq(
            face.profile_dz_at_radius(19.5).unwrap(),
            1.3,
            1e-9
        ));
        assert!(approx_eq(face.radius_at_dz(1.3).unwrap(), 19.5, 1e-9));
        assert!(approx_eq(face.radius_at_dz(5.0).unwrap(), 20.0, 1e-9));
    }

    #[test]
    fn minkowski_stock_allowance_dilates_ball_bullnose_flat_and_conical_exact() {
        // Ball(R=3) ⊕ Disk(sr=0.5) = BullNose(flat=0.5, corner=3.0) with dz(0.5) = 0, dz(3.5) = 3.0!
        let ball_g = CutterGeometry::BallEndmill {
            diameter_mm: 6.0,
            flute_length_mm: 18.0,
            overall_length_mm: 50.0,
            shank_diameter_mm: 6.0,
            flutes: 2,
        };
        let dilated =
            AnalyticCutterEnvelope::from_geometry(&ball_g).with_stock_allowance(0.5, 0.25);
        assert!(approx_eq(dilated.cutting_radius_mm(), 3.5, 1e-9));
        assert!(approx_eq(dilated.stock_radial_mm(), 0.5, 1e-9));
        assert!(approx_eq(dilated.stock_axial_mm(), 0.25, 1e-9));
        assert!(matches!(
            dilated.shape(),
            CutterEnvelopeShape::BullNose {
                flat_radius_mm,
                corner_radius_mm
            } if approx_eq(flat_radius_mm, 0.5, 1e-9) && approx_eq(corner_radius_mm, 3.0, 1e-9)
        ));
        assert!(approx_eq(
            dilated.profile_dz_at_radius(0.0).unwrap(),
            0.0,
            1e-9
        ));
        assert!(approx_eq(
            dilated.profile_dz_at_radius(0.5).unwrap(),
            0.0,
            1e-9
        ));
        assert!(approx_eq(
            dilated.profile_dz_at_radius(3.5).unwrap(),
            3.0,
            1e-9
        ));
        assert!(approx_eq(dilated.radius_at_dz(0.0).unwrap(), 0.5, 1e-9));
        assert!(approx_eq(dilated.radius_at_dz(3.0).unwrap(), 3.5, 1e-9));

        // Conical(r_tip=0, r_upper=4, 60°) ⊕ Disk(sr=0.4) -> tip=0.4, upper=4.4
        let vbit_g = CutterGeometry::ChamferMill {
            diameter_mm: 8.0,
            tip_diameter_mm: 0.0,
            included_angle_deg: 60.0,
            flute_length_mm: 12.0,
            overall_length_mm: 50.0,
            shank_diameter_mm: 8.0,
            flutes: 2,
        };
        let v_dil = AnalyticCutterEnvelope::from_geometry(&vbit_g).with_stock_allowance(0.4, 0.1);
        assert!(approx_eq(v_dil.cutting_radius_mm(), 4.4, 1e-9));
        assert!(approx_eq(
            v_dil.profile_dz_at_radius(0.4).unwrap(),
            0.0,
            1e-9
        ));
        assert!(approx_eq(
            v_dil.profile_dz_at_radius(1.4).unwrap(),
            3.0_f64.sqrt(),
            1e-9
        ));
        assert!(approx_eq(
            v_dil.radius_at_dz(3.0_f64.sqrt()).unwrap(),
            1.4,
            1e-9
        ));
    }

    #[test]
    fn thread_mill_single_point_and_multiform_non_monotonic_queries_are_exact() {
        // Single-point Ø4.75mm, neck Ø2.85mm, neck_len=16, flute=1.2, 60°
        let sp_g = CutterGeometry::ThreadMill {
            diameter_mm: 4.75,
            neck_diameter_mm: 2.85,
            neck_length_mm: 16.0,
            pitch_mm: 1.0,
            teeth_count: 1,
            thread_angle_deg: 60.0,
            flute_length_mm: 1.2,
            overall_length_mm: 50.0,
            shank_diameter_mm: 6.0,
            flutes: 3,
        };
        let sp = AnalyticCutterEnvelope::from_geometry(&sp_g);
        assert!(sp.is_thread_mill());
        assert!(approx_eq(sp.effective_reach_mm(), 16.0, 1e-9));
        assert!(approx_eq(sp.neck_radius_mm(), 1.425, 1e-9));
        assert!(approx_eq(
            sp.thread_mill_tooth_radius_at_z(0.6),
            2.375,
            1e-9
        ));
        assert!(approx_eq(sp.cutter_radius_at_z(0.6).unwrap(), 2.375, 1e-9));
        assert!(approx_eq(sp.cutter_radius_at_z(8.0).unwrap(), 1.425, 1e-9));
        assert!(approx_eq(sp.cutter_radius_at_z(20.0).unwrap(), 3.0, 1e-9));
        assert!(sp.cutter_radius_at_z_within(25.0, 22.0).is_none());

        // Axial intervals at r <= neck_radius -> full core `(0, 16.0)`
        let core_iv = sp.thread_mill_axial_intervals_at_radius(1.2);
        assert_eq!(core_iv.len(), 1);
        assert!(approx_eq(core_iv[0].0, 0.0, 1e-9));
        assert!(approx_eq(core_iv[0].1, 16.0, 1e-9));

        // Axial intervals in tooth annulus -> 1 centered V-tooth band
        let tooth_iv = sp.thread_mill_axial_intervals_at_radius(2.0);
        assert_eq!(tooth_iv.len(), 1);
        assert!(tooth_iv[0].0 > 0.0 && tooth_iv[0].1 < 1.2);
        assert!(sp.thread_mill_axial_intervals_at_radius(2.5).is_empty());

        // Multi-form 4-tooth M8x1.25 (D=6.0, neck=3.6, flute=5.0, pitch=1.25, teeth=4)
        let mf_g = CutterGeometry::ThreadMill {
            diameter_mm: 6.0,
            neck_diameter_mm: 3.6,
            neck_length_mm: 18.0,
            pitch_mm: 1.25,
            teeth_count: 4,
            thread_angle_deg: 60.0,
            flute_length_mm: 5.0,
            overall_length_mm: 57.0,
            shank_diameter_mm: 6.0,
            flutes: 3,
        };
        let mf = AnalyticCutterEnvelope::from_geometry(&mf_g);
        for k in 0..4 {
            let z_crest = (k as f64 + 0.5) * 1.25;
            assert!(approx_eq(
                mf.thread_mill_tooth_radius_at_z(z_crest),
                3.0,
                1e-9
            ));
        }
        let mf_intervals = mf.thread_mill_axial_intervals_at_radius(2.5);
        assert_eq!(mf_intervals.len(), 4);
        for (k, &(lo, hi)) in mf_intervals.iter().enumerate() {
            let expected_mid = (k as f64 + 0.5) * 1.25;
            assert!(approx_eq(0.5 * (lo + hi), expected_mid, 1e-9));
            assert!(hi > lo);
        }
    }
}
