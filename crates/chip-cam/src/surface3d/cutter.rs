use chip_tools::CutterGeometry;

/// Exact canonical `f64` lower contact profile of a milling cutter for 3-axis Drop-Cutter
/// and Push-Cutter surface contact evaluation (`mm`).
///
/// Radial stock-to-leave (`stock_radial_mm`) is folded analytically via exact horizontal disk
/// Minkowski dilation:
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

/// Complete analytical cutter contact envelope including axial stock-to-leave and flute height.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceCutterProfile {
    pub shape: SurfaceCutterShape,
    pub flute_length_mm: f64,
    pub stock_axial_mm: f64,
}

impl SurfaceCutterProfile {
    /// Builds an exact analytical [`SurfaceCutterProfile`] from a tool's [`CutterGeometry`]
    /// and radial/axial stock-to-leave allowances (`mm`).
    pub fn from_tool_geometry(
        geom: &CutterGeometry,
        stock_radial_mm: f64,
        stock_axial_mm: f64,
    ) -> Self {
        let sr = stock_radial_mm.max(0.0);
        let sz = stock_axial_mm.max(0.0);
        let flute_length_mm = geom.flute_length_mm().max(1.0);

        let shape = match *geom {
            CutterGeometry::FlatEndmill { diameter_mm, .. } => SurfaceCutterShape::Flat {
                radius_mm: (0.5 * diameter_mm).max(1e-4) + sr,
            },
            CutterGeometry::BallEndmill { diameter_mm, .. } => {
                let r = (0.5 * diameter_mm).max(1e-4);
                if sr > 1e-9 {
                    SurfaceCutterShape::BullNose {
                        flat_radius_mm: sr,
                        corner_radius_mm: r,
                    }
                } else {
                    SurfaceCutterShape::Ball { radius_mm: r }
                }
            }
            CutterGeometry::BullNoseEndmill {
                diameter_mm,
                corner_radius_mm,
                ..
            } => {
                let r = (0.5 * diameter_mm).max(1e-4);
                let rc = corner_radius_mm.clamp(0.0, r);
                let r_flat = (r - rc).max(0.0) + sr;
                if rc <= 1e-6 {
                    SurfaceCutterShape::Flat { radius_mm: r_flat }
                } else if r_flat <= 1e-9 {
                    SurfaceCutterShape::Ball { radius_mm: rc }
                } else {
                    SurfaceCutterShape::BullNose {
                        flat_radius_mm: r_flat,
                        corner_radius_mm: rc,
                    }
                }
            }
            CutterGeometry::ChamferMill {
                diameter_mm,
                tip_diameter_mm,
                included_angle_deg,
                ..
            } => {
                let r_upper = (0.5 * diameter_mm).max(1e-4) + sr;
                let r_tip = (0.5 * tip_diameter_mm).clamp(0.0, 0.5 * diameter_mm) + sr;
                let half_angle_rad = (0.5 * included_angle_deg.clamp(5.0, 175.0)).to_radians();
                if r_upper - r_tip <= 1e-6 {
                    SurfaceCutterShape::Flat { radius_mm: r_upper }
                } else {
                    SurfaceCutterShape::TaperedVBit {
                        tip_radius_mm: r_tip,
                        upper_radius_mm: r_upper,
                        half_angle_rad,
                    }
                }
            }
            CutterGeometry::FaceMill {
                cutting_diameter_mm,
                corner_radius_mm,
                ..
            } => {
                let r = (0.5 * cutting_diameter_mm).max(1e-4);
                let rc = corner_radius_mm.clamp(0.0, r);
                let r_flat = (r - rc).max(0.0) + sr;
                if rc <= 1e-6 {
                    SurfaceCutterShape::Flat { radius_mm: r_flat }
                } else {
                    SurfaceCutterShape::BullNose {
                        flat_radius_mm: r_flat,
                        corner_radius_mm: rc,
                    }
                }
            }
            CutterGeometry::TwistDrill {
                diameter_mm,
                point_angle_deg,
                ..
            } => {
                let r_upper = (0.5 * diameter_mm).max(1e-4) + sr;
                let half_angle_rad = (0.5 * point_angle_deg.clamp(30.0, 175.0)).to_radians();
                SurfaceCutterShape::TaperedVBit {
                    tip_radius_mm: sr,
                    upper_radius_mm: r_upper,
                    half_angle_rad,
                }
            }
        };

        Self {
            shape,
            flute_length_mm,
            stock_axial_mm: sz,
        }
    }

    /// Maximum horizontal contact radius (`mm`) of the cutter envelope (including radial stock).
    #[inline]
    pub fn max_radius_mm(&self) -> f64 {
        match self.shape {
            SurfaceCutterShape::Flat { radius_mm } | SurfaceCutterShape::Ball { radius_mm } => {
                radius_mm
            }
            SurfaceCutterShape::BullNose {
                flat_radius_mm,
                corner_radius_mm,
            } => flat_radius_mm + corner_radius_mm,
            SurfaceCutterShape::TaperedVBit {
                upper_radius_mm, ..
            } => upper_radius_mm,
        }
    }

    /// Height above the cutter tip (`mm`) over which the cutter radius expands from tip to `max_radius_mm`.
    #[inline]
    pub fn tip_profile_height_mm(&self) -> f64 {
        match self.shape {
            SurfaceCutterShape::Flat { .. } => 0.0,
            SurfaceCutterShape::Ball { radius_mm } => radius_mm,
            SurfaceCutterShape::BullNose {
                corner_radius_mm, ..
            } => corner_radius_mm,
            SurfaceCutterShape::TaperedVBit {
                tip_radius_mm,
                upper_radius_mm,
                half_angle_rad,
            } => ((upper_radius_mm - tip_radius_mm).max(0.0) / half_angle_rad.tan().max(1e-9))
                .min(self.flute_length_mm),
        }
    }

    /// Height `h(d) >= 0` of the cutter's lower contact surface above `z_tip` at radial distance `d`
    /// (`0 <= d <= max_radius_mm()`). Returns `None` if `d > max_radius_mm() + 1e-9`.
    #[inline]
    pub fn profile_height_at_radius(&self, d: f64) -> Option<f64> {
        let r_max = self.max_radius_mm();
        if !d.is_finite() || d < 0.0 || d > r_max + 1e-9 {
            return None;
        }
        let d_clamped = d.min(r_max);
        let h = match self.shape {
            SurfaceCutterShape::Flat { .. } => 0.0,
            SurfaceCutterShape::Ball { radius_mm } => {
                radius_mm
                    - (radius_mm * radius_mm - d_clamped * d_clamped)
                        .max(0.0)
                        .sqrt()
            }
            SurfaceCutterShape::BullNose {
                flat_radius_mm,
                corner_radius_mm,
            } => {
                if d_clamped <= flat_radius_mm {
                    0.0
                } else {
                    let dr = (d_clamped - flat_radius_mm).min(corner_radius_mm);
                    corner_radius_mm
                        - (corner_radius_mm * corner_radius_mm - dr * dr)
                            .max(0.0)
                            .sqrt()
                }
            }
            SurfaceCutterShape::TaperedVBit {
                tip_radius_mm,
                half_angle_rad,
                ..
            } => {
                if d_clamped <= tip_radius_mm {
                    0.0
                } else {
                    (d_clamped - tip_radius_mm) / half_angle_rad.tan().max(1e-9)
                }
            }
        };
        Some(h)
    }

    /// Horizontal cutter radius `r(dz)` at height `dz >= 0` above the cutter tip (`mm`).
    #[inline]
    pub fn radius_at_height(&self, dz: f64) -> f64 {
        let dz_c = dz.max(0.0);
        match self.shape {
            SurfaceCutterShape::Flat { radius_mm } => radius_mm,
            SurfaceCutterShape::Ball { radius_mm } => {
                if dz_c >= radius_mm {
                    radius_mm
                } else {
                    (2.0 * radius_mm * dz_c - dz_c * dz_c).max(0.0).sqrt()
                }
            }
            SurfaceCutterShape::BullNose {
                flat_radius_mm,
                corner_radius_mm,
            } => {
                if dz_c >= corner_radius_mm {
                    flat_radius_mm + corner_radius_mm
                } else {
                    flat_radius_mm
                        + (2.0 * corner_radius_mm * dz_c - dz_c * dz_c)
                            .max(0.0)
                            .sqrt()
                }
            }
            SurfaceCutterShape::TaperedVBit {
                tip_radius_mm,
                upper_radius_mm,
                half_angle_rad,
            } => (tip_radius_mm + dz_c * half_angle_rad.tan()).min(upper_radius_mm),
        }
    }
}
