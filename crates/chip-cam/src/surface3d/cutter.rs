use chip_tools::{AnalyticCutterEnvelope, CutterEnvelopeShape, CutterGeometry};

/// Exact canonical `f64` lower contact profile of a milling cutter for 3-axis Drop-Cutter
/// and Push-Cutter surface contact evaluation (`mm`).
///
/// Radial stock-to-leave (`stock_radial_mm`) is folded analytically via exact horizontal disk
/// Minkowski dilation inside [`AnalyticCutterEnvelope::with_stock_allowance`]:
/// - `Flat(R)` ⊕ `Disk(s_r)` = `Flat(R + s_r)`
/// - `Ball(R)` ⊕ `Disk(s_r)` = `BullNose(flat = s_r, corner = R)` (preserves exact `h(0) = 0` on flats!)
/// - `BullNose(r0, rc)` ⊕ `Disk(s_r)` = `BullNose(r0 + s_r, rc)`
/// - `TaperedVBit(r0, r1, α)` ⊕ `Disk(s_r)` = `TaperedVBit(r0 + s_r, r1 + s_r, α)`
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SurfaceCutterShape {
    Flat {
        radius_mm: f64,
    },
    Ball {
        radius_mm: f64,
    },
    BullNose {
        flat_radius_mm: f64,
        corner_radius_mm: f64,
    },
    TaperedVBit {
        tip_radius_mm: f64,
        upper_radius_mm: f64,
        half_angle_rad: f64,
    },
}

/// Complete analytical cutter contact envelope including axial stock-to-leave and flute height,
/// backed by the shared [`AnalyticCutterEnvelope`] from `chip-tools`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceCutterProfile {
    pub shape: SurfaceCutterShape,
    pub flute_length_mm: f64,
    pub stock_axial_mm: f64,
    envelope: AnalyticCutterEnvelope,
}

impl SurfaceCutterProfile {
    /// Builds an exact analytical [`SurfaceCutterProfile`] from a tool's [`CutterGeometry`]
    /// and radial/axial stock-to-leave allowances (`mm`).
    pub fn from_tool_geometry(
        geom: &CutterGeometry,
        stock_radial_mm: f64,
        stock_axial_mm: f64,
    ) -> Self {
        let envelope = AnalyticCutterEnvelope::from_geometry(geom)
            .with_stock_allowance(stock_radial_mm, stock_axial_mm);
        let flute_length_mm = envelope.flute_length_mm().max(1.0);
        let shape = match envelope.shape() {
            CutterEnvelopeShape::Flat { radius_mm } => SurfaceCutterShape::Flat { radius_mm },
            CutterEnvelopeShape::Ball { radius_mm } => SurfaceCutterShape::Ball { radius_mm },
            CutterEnvelopeShape::BullNose {
                flat_radius_mm,
                corner_radius_mm,
            } => SurfaceCutterShape::BullNose {
                flat_radius_mm,
                corner_radius_mm,
            },
            CutterEnvelopeShape::Conical {
                tip_radius_mm,
                upper_radius_mm,
                half_angle_rad,
                ..
            } => {
                if upper_radius_mm - tip_radius_mm <= 1e-6 {
                    SurfaceCutterShape::Flat {
                        radius_mm: upper_radius_mm,
                    }
                } else {
                    SurfaceCutterShape::TaperedVBit {
                        tip_radius_mm,
                        upper_radius_mm,
                        half_angle_rad,
                    }
                }
            }
            CutterEnvelopeShape::Face {
                flat_radius_mm,
                corner_radius_mm,
                ..
            } => {
                if corner_radius_mm <= 1e-6 {
                    SurfaceCutterShape::Flat {
                        radius_mm: flat_radius_mm,
                    }
                } else {
                    SurfaceCutterShape::BullNose {
                        flat_radius_mm,
                        corner_radius_mm,
                    }
                }
            }
            CutterEnvelopeShape::ThreadMill {
                cutting_radius_mm, ..
            } => SurfaceCutterShape::Flat {
                radius_mm: cutting_radius_mm,
            },
            CutterEnvelopeShape::Lollipop {
                flat_radius_mm,
                sphere_radius_mm,
                ..
            } => {
                if flat_radius_mm <= 1e-9 {
                    SurfaceCutterShape::Ball {
                        radius_mm: sphere_radius_mm,
                    }
                } else {
                    SurfaceCutterShape::BullNose {
                        flat_radius_mm,
                        corner_radius_mm: sphere_radius_mm,
                    }
                }
            }
            CutterEnvelopeShape::SlotMill {
                flat_radius_mm,
                cutting_radius_mm,
                corner_radius_mm,
                ..
            } => {
                if corner_radius_mm <= 1e-6 {
                    SurfaceCutterShape::Flat {
                        radius_mm: cutting_radius_mm,
                    }
                } else {
                    SurfaceCutterShape::BullNose {
                        flat_radius_mm,
                        corner_radius_mm,
                    }
                }
            }
            CutterEnvelopeShape::Dovetail {
                bottom_radius_mm,
                corner_radius_mm,
                ..
            } => {
                if corner_radius_mm <= 1e-6 {
                    SurfaceCutterShape::Flat {
                        radius_mm: bottom_radius_mm,
                    }
                } else {
                    SurfaceCutterShape::BullNose {
                        flat_radius_mm: (bottom_radius_mm - corner_radius_mm).max(0.0),
                        corner_radius_mm,
                    }
                }
            }
        };

        Self {
            shape,
            flute_length_mm,
            stock_axial_mm: envelope.stock_axial_mm(),
            envelope,
        }
    }

    /// Returns a copy of `self` with an additional horizontal radial clearance margin (`extra_radial_mm >= 0`)
    /// folded analytically via horizontal disk Minkowski dilation (leaving flat axial contact heights exact).
    pub fn with_extra_radial_margin(&self, extra_radial_mm: f64) -> Self {
        let sr = extra_radial_mm.max(0.0);
        if sr <= 1e-9 {
            return *self;
        }
        let envelope = self.envelope.with_stock_allowance(sr, 0.0);
        let shape = match self.shape {
            SurfaceCutterShape::Flat { radius_mm } => SurfaceCutterShape::Flat {
                radius_mm: radius_mm + sr,
            },
            SurfaceCutterShape::Ball { radius_mm } => SurfaceCutterShape::BullNose {
                flat_radius_mm: sr,
                corner_radius_mm: radius_mm,
            },
            SurfaceCutterShape::BullNose {
                flat_radius_mm,
                corner_radius_mm,
            } => SurfaceCutterShape::BullNose {
                flat_radius_mm: flat_radius_mm + sr,
                corner_radius_mm,
            },
            SurfaceCutterShape::TaperedVBit {
                tip_radius_mm,
                upper_radius_mm,
                half_angle_rad,
            } => SurfaceCutterShape::TaperedVBit {
                tip_radius_mm: tip_radius_mm + sr,
                upper_radius_mm: upper_radius_mm + sr,
                half_angle_rad,
            },
        };
        Self {
            shape,
            flute_length_mm: self.flute_length_mm,
            stock_axial_mm: self.stock_axial_mm,
            envelope,
        }
    }

    /// Underlying shared [`AnalyticCutterEnvelope`] from `chip-tools`.
    #[inline]
    pub const fn envelope(&self) -> &AnalyticCutterEnvelope {
        &self.envelope
    }

    /// Maximum horizontal contact radius (`mm`) of the cutter envelope (including radial stock).
    #[inline]
    pub fn max_radius_mm(&self) -> f64 {
        self.envelope.cutting_radius_mm()
    }

    /// Height above the cutter tip (`mm`) over which the cutter radius expands from tip to `max_radius_mm`.
    #[inline]
    pub fn tip_profile_height_mm(&self) -> f64 {
        self.envelope
            .tip_profile_height_mm()
            .min(self.flute_length_mm)
    }

    /// Height `h(d) >= 0` of the cutter's lower contact surface above `z_tip` at radial distance `d`
    /// (`0 <= d <= max_radius_mm()`). Returns `None` if `d > max_radius_mm() + 1e-9`.
    #[inline]
    pub fn profile_height_at_radius(&self, d: f64) -> Option<f64> {
        self.envelope.profile_dz_at_radius(d)
    }

    /// Horizontal cutter radius `r(dz)` at height `dz >= 0` above the cutter tip (`mm`).
    #[inline]
    pub fn radius_at_height(&self, dz: f64) -> f64 {
        self.envelope.radius_at_dz(dz.max(0.0)).unwrap_or(0.0)
    }
}
