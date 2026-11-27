//! Shared canonical `f64` millimeter analytic cutter envelope (`AnalyticCutterEnvelope`)
//! used by both `chip-cam` (3D Drop-Cutter / Push-Cutter contact) and `chip-sim`
//! (Heightmap, Tri-Dexel, and Cylindrical stock removal & 5-rule verification).

use serde::{Deserialize, Serialize};

use crate::model::{CutterGeometry, Tool};

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
    /// Non-monotonic relieved-neck spherical undercut cutter (`LollipopEndmill`).
    Lollipop {
        flat_radius_mm: f64,
        sphere_radius_mm: f64,
        neck_radius_mm: f64,
        neck_length_mm: f64,
        z_neck_join_mm: f64,
    },
    /// Non-monotonic relieved-neck horizontal disc / T-slot / keyway cutter (`SlotMill`).
    SlotMill {
        flat_radius_mm: f64,
        cutting_radius_mm: f64,
        cutting_thickness_mm: f64,
        corner_radius_mm: f64,
        neck_radius_mm: f64,
        neck_length_mm: f64,
    },
    /// Non-monotonic relieved-neck inverted conical dovetail cutter (`DovetailCutter`).
    Dovetail {
        bottom_radius_mm: f64,
        top_radius_mm: f64,
        cutting_height_mm: f64,
        side_angle_rad: f64,
        tan_side_angle: f64,
        corner_radius_mm: f64,
        neck_radius_mm: f64,
        neck_length_mm: f64,
    },
}

/// Exact canonical `f64` millimeter analytic envelope covering all 10 [`CutterGeometry`] variants,
/// including exact horizontal Minkowski disk dilation for radial/axial stock-to-leave and
/// non-monotonic relieved-neck profiles (`ThreadMill`, `LollipopEndmill`, `SlotMill`, `DovetailCutter`).
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
    /// Builds the exact analytic `f64` mm envelope for any of the 10 [`CutterGeometry`] variants.
    pub fn from_geometry(geom: &CutterGeometry) -> Self {
        let mut flute_length_mm = geom.flute_length_mm().max(0.1);
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
            CutterGeometry::LollipopEndmill {
                diameter_mm,
                neck_diameter_mm,
                neck_length_mm,
                ..
            } => {
                let r = (0.5 * diameter_mm).max(1e-4);
                let neck_r = (0.5 * neck_diameter_mm).clamp(0.02_f64.min(r), r);
                let z_geom_join = r + (r * r - neck_r * neck_r).max(0.0).sqrt();
                let z_neck_join_mm = z_geom_join.min(flute_length_mm.max(r));
                flute_length_mm = z_neck_join_mm;
                let neck_len = neck_length_mm.max(z_neck_join_mm);
                (
                    r,
                    CutterEnvelopeShape::Lollipop {
                        flat_radius_mm: 0.0,
                        sphere_radius_mm: r,
                        neck_radius_mm: neck_r,
                        neck_length_mm: neck_len,
                        z_neck_join_mm,
                    },
                )
            }
            CutterGeometry::SlotMill {
                diameter_mm,
                cutting_thickness_mm,
                corner_radius_mm,
                neck_diameter_mm,
                neck_length_mm,
                ..
            } => {
                let r = (0.5 * diameter_mm).max(1e-4);
                let h = cutting_thickness_mm.max(0.05);
                let rc = corner_radius_mm.clamp(0.0, (0.5 * h).min(r));
                let flat_r = (r - rc).max(0.0);
                let neck_r = (0.5 * neck_diameter_mm).clamp(0.02_f64.min(r), r);
                let neck_len = neck_length_mm.max(h);
                flute_length_mm = h;
                (
                    r,
                    CutterEnvelopeShape::SlotMill {
                        flat_radius_mm: flat_r,
                        cutting_radius_mm: r,
                        cutting_thickness_mm: h,
                        corner_radius_mm: rc,
                        neck_radius_mm: neck_r,
                        neck_length_mm: neck_len,
                    },
                )
            }
            CutterGeometry::DovetailCutter {
                diameter_mm,
                side_angle_deg,
                cutting_height_mm,
                corner_radius_mm,
                neck_diameter_mm,
                neck_length_mm,
                ..
            } => {
                let r_bot = (0.5 * diameter_mm).max(1e-4);
                let h = cutting_height_mm.max(0.05);
                let rc = corner_radius_mm.clamp(0.0, (0.5 * h).min(r_bot));
                let angle_clamped = side_angle_deg.clamp(15.0, 85.0);
                let side_angle_rad = angle_clamped.to_radians();
                let tan_side_angle = side_angle_rad.tan().max(1e-6);
                let top_r = (0.5
                    * CutterGeometry::dovetail_top_diameter_mm(diameter_mm, angle_clamped, h, rc))
                .clamp(0.0, r_bot);
                let neck_r = (0.5 * neck_diameter_mm).clamp(0.02_f64.min(r_bot), r_bot);
                let neck_len = neck_length_mm.max(h);
                flute_length_mm = h;
                (
                    r_bot,
                    CutterEnvelopeShape::Dovetail {
                        bottom_radius_mm: r_bot,
                        top_radius_mm: top_r,
                        cutting_height_mm: h,
                        side_angle_rad,
                        tan_side_angle,
                        corner_radius_mm: rc,
                        neck_radius_mm: neck_r,
                        neck_length_mm: neck_len,
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
            CutterEnvelopeShape::Lollipop {
                flat_radius_mm,
                sphere_radius_mm,
                neck_radius_mm,
                neck_length_mm,
                z_neck_join_mm,
            } => {
                let new_flat = flat_radius_mm + sr;
                (
                    new_flat + sphere_radius_mm,
                    CutterEnvelopeShape::Lollipop {
                        flat_radius_mm: new_flat,
                        sphere_radius_mm,
                        neck_radius_mm: neck_radius_mm + sr,
                        neck_length_mm,
                        z_neck_join_mm,
                    },
                )
            }
            CutterEnvelopeShape::SlotMill {
                flat_radius_mm,
                cutting_radius_mm,
                cutting_thickness_mm,
                corner_radius_mm,
                neck_radius_mm,
                neck_length_mm,
            } => {
                let new_cut = cutting_radius_mm + sr;
                (
                    new_cut,
                    CutterEnvelopeShape::SlotMill {
                        flat_radius_mm: flat_radius_mm + sr,
                        cutting_radius_mm: new_cut,
                        cutting_thickness_mm,
                        corner_radius_mm,
                        neck_radius_mm: neck_radius_mm + sr,
                        neck_length_mm,
                    },
                )
            }
            CutterEnvelopeShape::Dovetail {
                bottom_radius_mm,
                top_radius_mm,
                cutting_height_mm,
                side_angle_rad,
                tan_side_angle,
                corner_radius_mm,
                neck_radius_mm,
                neck_length_mm,
            } => {
                let new_bot = bottom_radius_mm + sr;
                (
                    new_bot,
                    CutterEnvelopeShape::Dovetail {
                        bottom_radius_mm: new_bot,
                        top_radius_mm: top_radius_mm + sr,
                        cutting_height_mm,
                        side_angle_rad,
                        tan_side_angle,
                        corner_radius_mm,
                        neck_radius_mm: neck_radius_mm + sr,
                        neck_length_mm,
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

    /// Returns a radially shrunk copy of `self` (`shrink_mm >= 0`) used for grazing-tolerant
    /// overhang checks so flush vertical walls and modeled hole cylinders do not false-trigger.
    pub fn with_radial_shrink(&self, shrink_mm: f64) -> Self {
        let s = shrink_mm.max(0.0);
        if s <= 1e-9 {
            return *self;
        }
        let (cutting_radius_mm, shape) = match self.shape {
            CutterEnvelopeShape::Flat { radius_mm } => {
                let r = (radius_mm - s).max(0.05);
                (r, CutterEnvelopeShape::Flat { radius_mm: r })
            }
            CutterEnvelopeShape::Ball { radius_mm } => {
                let r = (radius_mm - s).max(0.05);
                (r, CutterEnvelopeShape::Ball { radius_mm: r })
            }
            CutterEnvelopeShape::BullNose {
                flat_radius_mm,
                corner_radius_mm,
            } => {
                if flat_radius_mm >= s {
                    let new_flat = flat_radius_mm - s;
                    (
                        new_flat + corner_radius_mm,
                        CutterEnvelopeShape::BullNose {
                            flat_radius_mm: new_flat,
                            corner_radius_mm,
                        },
                    )
                } else {
                    let rem = s - flat_radius_mm;
                    let r = (corner_radius_mm - rem).max(0.05);
                    (r, CutterEnvelopeShape::Ball { radius_mm: r })
                }
            }
            CutterEnvelopeShape::Conical {
                tip_radius_mm,
                upper_radius_mm,
                half_angle_rad,
                tan_half_angle,
            } => {
                let new_upper = (upper_radius_mm - s).max(0.05);
                let new_tip = (tip_radius_mm - s).max(0.0).min(new_upper);
                (
                    new_upper,
                    CutterEnvelopeShape::Conical {
                        tip_radius_mm: new_tip,
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
                let new_outer = (outer_radius_mm - s).max(0.05);
                let new_bottom = (bottom_radius_mm - s).max(0.05).min(new_outer);
                let new_flat = (flat_radius_mm - s).max(0.0).min(new_bottom);
                let new_rc = corner_radius_mm.min(new_bottom - new_flat).max(0.0);
                (
                    new_outer,
                    CutterEnvelopeShape::Face {
                        flat_radius_mm: new_flat,
                        bottom_radius_mm: new_bottom,
                        outer_radius_mm: new_outer,
                        corner_radius_mm: new_rc,
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
                let new_cut = (cutting_radius_mm - s).max(0.05);
                let new_neck = (neck_radius_mm - s).max(0.05).min(new_cut);
                (
                    new_cut,
                    CutterEnvelopeShape::ThreadMill {
                        cutting_radius_mm: new_cut,
                        neck_radius_mm: new_neck,
                        neck_length_mm,
                        pitch_mm,
                        teeth_count,
                        thread_angle_deg,
                    },
                )
            }
            CutterEnvelopeShape::Lollipop {
                flat_radius_mm,
                sphere_radius_mm,
                neck_radius_mm,
                neck_length_mm,
                z_neck_join_mm,
            } => {
                let (new_flat, new_sphere) = if flat_radius_mm >= s {
                    (flat_radius_mm - s, sphere_radius_mm)
                } else {
                    (0.0, (sphere_radius_mm - (s - flat_radius_mm)).max(0.05))
                };
                let new_cut = new_flat + new_sphere;
                let new_neck = (neck_radius_mm - s).max(0.05).min(new_cut);
                (
                    new_cut,
                    CutterEnvelopeShape::Lollipop {
                        flat_radius_mm: new_flat,
                        sphere_radius_mm: new_sphere,
                        neck_radius_mm: new_neck,
                        neck_length_mm,
                        z_neck_join_mm,
                    },
                )
            }
            CutterEnvelopeShape::SlotMill {
                flat_radius_mm,
                cutting_radius_mm,
                cutting_thickness_mm,
                corner_radius_mm,
                neck_radius_mm,
                neck_length_mm,
            } => {
                let new_cut = (cutting_radius_mm - s).max(0.05);
                let new_flat = (flat_radius_mm - s).max(0.0).min(new_cut);
                let new_rc = corner_radius_mm.min(new_cut - new_flat).max(0.0);
                let new_neck = (neck_radius_mm - s).max(0.05).min(new_cut);
                (
                    new_cut,
                    CutterEnvelopeShape::SlotMill {
                        flat_radius_mm: new_flat,
                        cutting_radius_mm: new_cut,
                        cutting_thickness_mm,
                        corner_radius_mm: new_rc,
                        neck_radius_mm: new_neck,
                        neck_length_mm,
                    },
                )
            }
            CutterEnvelopeShape::Dovetail {
                bottom_radius_mm,
                top_radius_mm,
                cutting_height_mm,
                side_angle_rad,
                tan_side_angle,
                corner_radius_mm,
                neck_radius_mm,
                neck_length_mm,
            } => {
                let new_bot = (bottom_radius_mm - s).max(0.05);
                let new_top = (top_radius_mm - s).max(0.05).min(new_bot);
                let new_neck = (neck_radius_mm - s).max(0.05).min(new_top);
                let new_rc = corner_radius_mm.min(new_bot).max(0.0);
                (
                    new_bot,
                    CutterEnvelopeShape::Dovetail {
                        bottom_radius_mm: new_bot,
                        top_radius_mm: new_top,
                        cutting_height_mm,
                        side_angle_rad,
                        tan_side_angle,
                        corner_radius_mm: new_rc,
                        neck_radius_mm: new_neck,
                        neck_length_mm,
                    },
                )
            }
        };

        Self {
            shape,
            cutting_radius_mm,
            flute_length_mm: self.flute_length_mm,
            overall_length_mm: self.overall_length_mm,
            shank_radius_mm: (self.shank_radius_mm - s).max(0.05),
            stock_radial_mm: (self.stock_radial_mm - s).max(0.0),
            stock_axial_mm: self.stock_axial_mm,
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

    /// Returns `true` for non-monotonic undercut cutters with a relieved cylindrical neck
    /// (`ThreadMill`, `LollipopEndmill`, `SlotMill`, `DovetailCutter`).
    #[inline]
    pub const fn has_relieved_neck(&self) -> bool {
        matches!(
            self.shape,
            CutterEnvelopeShape::ThreadMill { .. }
                | CutterEnvelopeShape::Lollipop { .. }
                | CutterEnvelopeShape::SlotMill { .. }
                | CutterEnvelopeShape::Dovetail { .. }
        )
    }

    /// Returns `true` for bounded-head relieved-neck undercut cutters (`Lollipop`, `SlotMill`,
    /// `Dovetail`) whose single continuous cutting head occupies `[dz_lo(r), dz_hi(r)]`.
    #[inline]
    pub const fn has_bounded_undercut_head(&self) -> bool {
        matches!(
            self.shape,
            CutterEnvelopeShape::Lollipop { .. }
                | CutterEnvelopeShape::SlotMill { .. }
                | CutterEnvelopeShape::Dovetail { .. }
        )
    }

    /// Relieved neck radius (`mm`) for relieved-neck undercut cutters; equals `cutting_radius_mm`
    /// for monotonic cutters.
    #[inline]
    pub const fn neck_radius_mm(&self) -> f64 {
        match self.shape {
            CutterEnvelopeShape::ThreadMill { neck_radius_mm, .. }
            | CutterEnvelopeShape::Lollipop { neck_radius_mm, .. }
            | CutterEnvelopeShape::SlotMill { neck_radius_mm, .. }
            | CutterEnvelopeShape::Dovetail { neck_radius_mm, .. } => neck_radius_mm,
            _ => self.cutting_radius_mm,
        }
    }

    /// Usable relieved-neck reach (`mm`) for relieved-neck cutters; equals `flute_length_mm`
    /// for monotonic cutters.
    #[inline]
    pub const fn effective_reach_mm(&self) -> f64 {
        match self.shape {
            CutterEnvelopeShape::ThreadMill { neck_length_mm, .. }
            | CutterEnvelopeShape::Lollipop { neck_length_mm, .. }
            | CutterEnvelopeShape::SlotMill { neck_length_mm, .. }
            | CutterEnvelopeShape::Dovetail { neck_length_mm, .. } => neck_length_mm,
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
            CutterEnvelopeShape::Lollipop {
                sphere_radius_mm, ..
            } => sphere_radius_mm,
            CutterEnvelopeShape::SlotMill {
                corner_radius_mm, ..
            }
            | CutterEnvelopeShape::Dovetail {
                corner_radius_mm, ..
            } => corner_radius_mm,
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
            CutterEnvelopeShape::Lollipop {
                flat_radius_mm,
                sphere_radius_mm,
                ..
            } => {
                if r <= flat_radius_mm {
                    0.0
                } else {
                    let dr = (r - flat_radius_mm).min(sphere_radius_mm);
                    sphere_radius_mm
                        - (sphere_radius_mm * sphere_radius_mm - dr * dr)
                            .max(0.0)
                            .sqrt()
                }
            }
            CutterEnvelopeShape::SlotMill {
                flat_radius_mm,
                corner_radius_mm,
                ..
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
            CutterEnvelopeShape::Dovetail {
                bottom_radius_mm,
                corner_radius_mm,
                ..
            } => {
                let r_c = (bottom_radius_mm - corner_radius_mm).max(0.0);
                if r <= r_c || corner_radius_mm <= 1e-9 {
                    0.0
                } else {
                    let dr = (r - r_c).min(corner_radius_mm);
                    corner_radius_mm
                        - (corner_radius_mm * corner_radius_mm - dr * dr)
                            .max(0.0)
                            .sqrt()
                }
            }
        };
        Some(dz)
    }

    /// Evaluates the exact bounded axial cutting interval `(dz_lo, dz_hi)` above the cutter tip
    /// `(0, 0, 0)` at radial distance `r_mm` (`0 <= r_mm <= cutting_radius_mm`).
    ///
    /// - Monotonic cutters (`Flat`, `BullNose`, `Ball`, `Conical`, `Face`): returns
    ///   `(profile_dz_at_radius(r), flute_length_mm)`.
    /// - Bounded undercut cutters (`Lollipop`, `SlotMill`, `Dovetail`): returns the exact bounded
    ///   cutting-head span `[dz_lo(r), dz_hi(r)]` leaving overhanging roofs above `dz_hi` intact.
    /// - `ThreadMill`: returns the bounding interval spanning its V-tooth band at radius `r`
    ///   (use [`Self::push_thread_mill_axial_intervals_at_radius`] for individual multi-tooth bands).
    #[inline]
    pub fn cutting_z_interval_at_radius(&self, r_mm: f64) -> Option<(f64, f64)> {
        if !r_mm.is_finite() || r_mm < 0.0 || r_mm > self.cutting_radius_mm + 1e-9 {
            return None;
        }
        let r = r_mm.min(self.cutting_radius_mm);
        match self.shape {
            CutterEnvelopeShape::Lollipop {
                flat_radius_mm,
                sphere_radius_mm,
                z_neck_join_mm,
                ..
            } => {
                if r <= flat_radius_mm {
                    Some((0.0, z_neck_join_mm))
                } else {
                    let dr = (r - flat_radius_mm).min(sphere_radius_mm);
                    let half_h = (sphere_radius_mm * sphere_radius_mm - dr * dr)
                        .max(0.0)
                        .sqrt();
                    let dz_lo = (sphere_radius_mm - half_h).max(0.0);
                    let dz_hi = (sphere_radius_mm + half_h).min(z_neck_join_mm);
                    Some((dz_lo, dz_hi.max(dz_lo)))
                }
            }
            CutterEnvelopeShape::SlotMill {
                flat_radius_mm,
                cutting_thickness_mm,
                corner_radius_mm,
                ..
            } => {
                let dz_lo = if r <= flat_radius_mm || corner_radius_mm <= 1e-9 {
                    0.0
                } else {
                    let dr = (r - flat_radius_mm).min(corner_radius_mm);
                    corner_radius_mm
                        - (corner_radius_mm * corner_radius_mm - dr * dr)
                            .max(0.0)
                            .sqrt()
                };
                let dz_hi = (cutting_thickness_mm - dz_lo).max(dz_lo);
                Some((dz_lo, dz_hi))
            }
            CutterEnvelopeShape::Dovetail {
                bottom_radius_mm,
                cutting_height_mm,
                side_angle_rad,
                tan_side_angle,
                corner_radius_mm,
                ..
            } => {
                if corner_radius_mm <= 1e-9 {
                    let dz_hi =
                        ((bottom_radius_mm - r) * tan_side_angle).clamp(0.0, cutting_height_mm);
                    Some((0.0, dz_hi))
                } else {
                    let r_c = (bottom_radius_mm - corner_radius_mm).max(0.0);
                    let dz_lo = if r <= r_c {
                        0.0
                    } else {
                        let dr = (r - r_c).min(corner_radius_mm);
                        corner_radius_mm
                            - (corner_radius_mm * corner_radius_mm - dr * dr)
                                .max(0.0)
                                .sqrt()
                    };
                    let (sin_a, cos_a) = side_angle_rad.sin_cos();
                    let r_tan = r_c + corner_radius_mm * sin_a;
                    let dz_hi = if r >= r_tan {
                        let dr = (r - r_c).min(corner_radius_mm);
                        (corner_radius_mm
                            + (corner_radius_mm * corner_radius_mm - dr * dr)
                                .max(0.0)
                                .sqrt())
                        .min(cutting_height_mm)
                    } else {
                        (corner_radius_mm
                            + (corner_radius_mm - (r - r_c) * sin_a) / cos_a.max(1e-6))
                        .clamp(dz_lo, cutting_height_mm)
                    };
                    Some((dz_lo, dz_hi.max(dz_lo)))
                }
            }
            CutterEnvelopeShape::ThreadMill {
                cutting_radius_mm,
                neck_radius_mm,
                ..
            } => {
                if r <= neck_radius_mm {
                    Some((0.0, self.flute_length_mm))
                } else {
                    let (z_start, tooth_step, n, tan_half) = self.thread_mill_tooth_band_params();
                    let half_w = (cutting_radius_mm - r).max(0.0) * tan_half;
                    let lo = (z_start + 0.5 * tooth_step - half_w).max(z_start);
                    let last_lo = z_start + ((n - 1) as f64) * tooth_step;
                    let hi = (last_lo + 0.5 * tooth_step + half_w).min(last_lo + tooth_step);
                    (hi >= lo - 1e-9).then_some((lo, hi.max(lo)))
                }
            }
            _ => {
                let dz_lo = self.profile_dz_at_radius(r)?;
                Some((dz_lo, self.flute_length_mm.max(dz_lo)))
            }
        }
    }

    /// Evaluates the exact cutting-head horizontal radius `r(dz)` strictly within the cutting
    /// head axial range `dz ∈ [0, flute_length_mm]`. Returns `None` outside the cutting head
    /// (including in the non-cutting relieved neck or shank zones).
    #[inline]
    pub fn cutting_radius_at_dz(&self, dz_mm: f64) -> Option<f64> {
        if !dz_mm.is_finite() || dz_mm < -1e-9 || dz_mm > self.flute_length_mm + 1e-9 {
            return None;
        }
        let dz = dz_mm.clamp(0.0, self.flute_length_mm);
        let r = match self.shape {
            CutterEnvelopeShape::Lollipop {
                flat_radius_mm,
                sphere_radius_mm,
                z_neck_join_mm,
                ..
            } => {
                if dz > z_neck_join_mm + 1e-9 {
                    return None;
                }
                (flat_radius_mm + (2.0 * sphere_radius_mm * dz - dz * dz).max(0.0).sqrt())
                    .min(self.cutting_radius_mm)
            }
            CutterEnvelopeShape::SlotMill {
                flat_radius_mm,
                cutting_radius_mm,
                cutting_thickness_mm,
                corner_radius_mm,
                ..
            } => {
                if dz > cutting_thickness_mm + 1e-9 {
                    return None;
                }
                if corner_radius_mm > 1e-9 && dz < corner_radius_mm {
                    (flat_radius_mm + (2.0 * corner_radius_mm * dz - dz * dz).max(0.0).sqrt())
                        .min(cutting_radius_mm)
                } else if corner_radius_mm > 1e-9 && dz > cutting_thickness_mm - corner_radius_mm {
                    let dtop = (cutting_thickness_mm - dz).max(0.0);
                    (flat_radius_mm
                        + (2.0 * corner_radius_mm * dtop - dtop * dtop)
                            .max(0.0)
                            .sqrt())
                    .min(cutting_radius_mm)
                } else {
                    cutting_radius_mm
                }
            }
            CutterEnvelopeShape::Dovetail {
                bottom_radius_mm,
                top_radius_mm,
                cutting_height_mm,
                side_angle_rad,
                tan_side_angle,
                corner_radius_mm,
                ..
            } => {
                if dz > cutting_height_mm + 1e-9 {
                    return None;
                }
                if corner_radius_mm <= 1e-9 {
                    (bottom_radius_mm - dz / tan_side_angle).clamp(top_radius_mm, bottom_radius_mm)
                } else {
                    let (sin_a, cos_a) = side_angle_rad.sin_cos();
                    let r_c = (bottom_radius_mm - corner_radius_mm).max(0.0);
                    let z_tan = corner_radius_mm * (1.0 + cos_a);
                    if dz <= z_tan {
                        let d = dz - corner_radius_mm;
                        (r_c + (corner_radius_mm * corner_radius_mm - d * d)
                            .max(0.0)
                            .sqrt())
                        .min(bottom_radius_mm)
                    } else {
                        let r_tan = r_c + corner_radius_mm * sin_a;
                        (r_tan - (dz - z_tan) / tan_side_angle)
                            .clamp(top_radius_mm, bottom_radius_mm)
                    }
                }
            }
            CutterEnvelopeShape::ThreadMill { .. } => self.thread_mill_tooth_radius_at_z(dz),
            _ => return self.radius_at_dz(dz),
        };
        Some(r.max(0.0))
    }

    /// Evaluates the full revolved assembly radius `R(dz)` at axial height `dz_mm >= 0` above the
    /// cutter tip across the cutting head `[0, flute_length_mm]`, relieved neck
    /// `(flute_length_mm, neck_length_mm]` (for relieved-neck cutters), and cylindrical shank above.
    #[inline]
    pub fn assembly_radius_at_dz(&self, dz_mm: f64) -> Option<f64> {
        if !dz_mm.is_finite() || dz_mm < -1e-9 {
            return None;
        }
        let dz = dz_mm.max(0.0);
        if dz <= self.flute_length_mm + 1e-9 {
            return self.cutting_radius_at_dz(dz);
        }
        if self.has_relieved_neck() && dz <= self.effective_reach_mm() + 1e-9 {
            return Some(self.neck_radius_mm());
        }
        Some(self.shank_radius_mm)
    }

    /// Evaluates the exact cutting envelope radius `r(dz) >= 0` at axial height `dz_mm >= 0` above
    /// the cutter tip `(0, 0, 0)` (exact inverse of [`Self::profile_dz_at_radius`]).
    ///
    /// Returns `None` if `dz_mm` is non-finite or `< -1e-9`. For relieved-neck cutters
    /// (`ThreadMill`, `Lollipop`, `SlotMill`, `Dovetail`), returns the cutting-head radius in
    /// `[0, flute_length_mm]`, `neck_radius_mm` along `(flute_length_mm, neck_length_mm]`, and
    /// `shank_radius_mm` above `neck_length_mm`.
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
            CutterEnvelopeShape::ThreadMill { .. }
            | CutterEnvelopeShape::Lollipop { .. }
            | CutterEnvelopeShape::SlotMill { .. }
            | CutterEnvelopeShape::Dovetail { .. } => {
                return self.assembly_radius_at_dz(dz);
            }
        };
        Some(r.max(0.0))
    }

    /// Evaluates the height-dependent cutter radius `R(z_local)` at axial height `z_local` above
    /// the cutter tip within `[0, overall_length_mm]`, supporting non-monotonic undercut profiles
    /// such as relieved-neck thread mills, lollipops, slot mills, and dovetail cutters.
    #[inline]
    pub fn cutter_radius_at_z(&self, z_local: f64) -> Option<f64> {
        let max_z = self.overall_length_mm.max(self.effective_reach_mm());
        self.cutter_radius_at_z_within(z_local, max_z)
    }

    /// Evaluates the height-dependent cutter radius `R(z_local)` clipped to `[0, max_z_mm]`
    /// (e.g., `stickout_length_mm` when verifying a mounted tool assembly):
    /// - `[0, flute_length_mm]`: cutting-tooth / cutting-head envelope
    /// - `(flute_length_mm, neck_length_mm]`: relieved neck for relieved-neck undercut cutters
    /// - `(effective_reach_mm, max_z_mm]`: cylindrical shank (`shank_radius_mm`)
    #[inline]
    pub fn cutter_radius_at_z_within(&self, z_local: f64, max_z_mm: f64) -> Option<f64> {
        let max_z = max_z_mm.max(self.effective_reach_mm());
        if !z_local.is_finite() || z_local < -1e-9 || z_local > max_z + 1e-9 {
            return None;
        }
        let z = z_local.clamp(0.0, max_z);
        self.assembly_radius_at_dz(z)
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

/// Physical portion of the revolved cutter assembly (flutes, non-cutting shank/neck, or collet/holder).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ToolAssemblyPart {
    Flute,
    Shank,
    Holder,
}

impl ToolAssemblyPart {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Flute => "Cutter flutes",
            Self::Shank => "Non-cutting shank",
            Self::Holder => "Holder/collet",
        }
    }
}

/// Canonical full revolved tool assembly envelope (cutting flutes, relieved neck, non-cutting
/// shank, collet nut, tapered holder body, effective non-rubbing reach, and plunge limits)
/// shared across `chip-cam`, `chip-stock`, and `chip-sim`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ToolAssemblyEnvelope {
    pub envelope: AnalyticCutterEnvelope,
    pub stickout_length_mm: f64,
    pub holder_nut_radius_mm: f64,
    pub holder_body_radius_mm: f64,
    pub holder_cone_z0: f64,
    pub holder_cone_z1: f64,
    pub effective_non_rubbing_reach_mm: f64,
    pub is_center_cutting: bool,
    pub max_safe_plunge_depth_mm: f64,
}

impl ToolAssemblyEnvelope {
    /// Builds the canonical revolved assembly envelope for `tool` (`nut_r = max(shank_r * 2.2, 12.0, shank_r + 4.0)`).
    pub fn from_tool(tool: &Tool) -> Self {
        let envelope = AnalyticCutterEnvelope::from_geometry(&tool.geometry);
        let flute_length_mm = envelope.flute_length_mm();
        let stickout_length_mm = tool.stickout_length_mm.max(flute_length_mm);
        let shank_radius_mm = envelope.shank_radius_mm();
        let holder_nut_radius_mm = (shank_radius_mm * 2.2).max(12.0).max(shank_radius_mm + 4.0);
        let holder_body_radius_mm = holder_nut_radius_mm * 1.25;
        let holder_cone_z0 = stickout_length_mm + 18.0;
        let holder_cone_z1 = stickout_length_mm + 28.0;

        let raw_flute = tool.geometry.flute_length_mm().max(0.0);
        let raw_stickout = tool.stickout_length_mm.max(raw_flute);
        let d_cut = tool.geometry.cutting_diameter_mm();
        let effective_non_rubbing_reach_mm = match tool.geometry {
            CutterGeometry::ThreadMill {
                neck_diameter_mm,
                neck_length_mm,
                ..
            }
            | CutterGeometry::LollipopEndmill {
                neck_diameter_mm,
                neck_length_mm,
                ..
            }
            | CutterGeometry::SlotMill {
                neck_diameter_mm,
                neck_length_mm,
                ..
            }
            | CutterGeometry::DovetailCutter {
                neck_diameter_mm,
                neck_length_mm,
                ..
            } => {
                if neck_diameter_mm + 1e-3 < d_cut {
                    neck_length_mm.max(raw_flute).min(raw_stickout)
                } else {
                    raw_flute
                }
            }
            _ => {
                let shank = tool.geometry.shank_diameter_mm();
                if shank > 0.0 && shank + 1e-3 < d_cut {
                    raw_stickout
                } else {
                    raw_flute
                }
            }
        };

        let (is_center_cutting, max_safe_plunge_depth_mm) = match tool.geometry {
            CutterGeometry::FlatEndmill { diameter_mm, .. }
            | CutterGeometry::BullNoseEndmill { diameter_mm, .. }
            | CutterGeometry::BallEndmill { diameter_mm, .. } => {
                (true, (1.5 * diameter_mm).min(flute_length_mm))
            }
            CutterGeometry::LollipopEndmill { diameter_mm, .. } => {
                (true, (0.5 * diameter_mm).min(flute_length_mm))
            }
            CutterGeometry::ChamferMill { .. } | CutterGeometry::TwistDrill { .. } => {
                (true, flute_length_mm)
            }
            CutterGeometry::FaceMill { .. }
            | CutterGeometry::ThreadMill { .. }
            | CutterGeometry::SlotMill { .. }
            | CutterGeometry::DovetailCutter { .. } => (false, 0.0),
        };

        Self {
            envelope,
            stickout_length_mm,
            holder_nut_radius_mm,
            holder_body_radius_mm,
            holder_cone_z0,
            holder_cone_z1,
            effective_non_rubbing_reach_mm,
            is_center_cutting,
            max_safe_plunge_depth_mm,
        }
    }

    #[inline]
    pub const fn cutting_radius_mm(&self) -> f64 {
        self.envelope.cutting_radius_mm()
    }

    #[inline]
    pub const fn flute_length_mm(&self) -> f64 {
        self.envelope.flute_length_mm()
    }

    #[inline]
    pub const fn overall_length_mm(&self) -> f64 {
        self.envelope.overall_length_mm()
    }

    #[inline]
    pub const fn shank_radius_mm(&self) -> f64 {
        self.envelope.shank_radius_mm()
    }

    #[inline]
    pub const fn neck_radius_mm(&self) -> f64 {
        self.envelope.neck_radius_mm()
    }

    #[inline]
    pub fn neck_length_mm(&self) -> f64 {
        self.envelope.effective_reach_mm()
    }

    #[inline]
    pub const fn is_thread_mill(&self) -> bool {
        self.envelope.is_thread_mill()
    }

    #[inline]
    pub const fn has_relieved_neck(&self) -> bool {
        self.envelope.has_relieved_neck()
    }

    #[inline]
    pub const fn has_bounded_undercut_head(&self) -> bool {
        self.envelope.has_bounded_undercut_head()
    }

    #[inline]
    pub fn max_assembly_radius_mm(&self) -> f64 {
        self.cutting_radius_mm()
            .max(self.shank_radius_mm())
            .max(self.holder_body_radius_mm)
    }

    /// Evaluates the collet nut / tapered holder cone radius at `dz >= stickout_length_mm`.
    #[inline]
    pub fn holder_radius_at_dz(&self, dz: f64) -> f64 {
        if dz <= self.holder_cone_z0 {
            self.holder_nut_radius_mm
        } else if dz < self.holder_cone_z1 {
            let span = (self.holder_cone_z1 - self.holder_cone_z0).max(1e-6);
            let u = ((dz - self.holder_cone_z0) / span).clamp(0.0, 1.0);
            self.holder_nut_radius_mm + (self.holder_body_radius_mm - self.holder_nut_radius_mm) * u
        } else {
            self.holder_body_radius_mm
        }
    }

    /// Evaluates the full revolved assembly radius at axial height `dz >= 0` above the cutter tip,
    /// spanning cutting flutes, relieved neck, non-cutting shank, collet nut, and tapered holder cone.
    #[inline]
    pub fn radius_at_dz(&self, dz: f64) -> Option<f64> {
        if !dz.is_finite() || dz < -1e-9 {
            return None;
        }
        let z = dz.max(0.0);
        if z < self.stickout_length_mm {
            self.envelope.assembly_radius_at_dz(z)
        } else {
            Some(self.holder_radius_at_dz(z))
        }
    }

    /// Returns the lowest axial height `dz >= 0` above the tip where the assembly reaches radial
    /// distance `r_eff` from the spindle axis, paired with the corresponding [`ToolAssemblyPart`].
    #[inline]
    pub fn min_dz_and_part_for_radius(&self, r_eff: f64) -> Option<(f64, ToolAssemblyPart)> {
        if !r_eff.is_finite() || r_eff < 0.0 {
            return None;
        }
        if r_eff <= self.cutting_radius_mm() + 1e-9
            && let Some(dz) = self.envelope.profile_dz_at_radius(r_eff)
        {
            return Some((dz, ToolAssemblyPart::Flute));
        }
        if self.stickout_length_mm > self.flute_length_mm() + 1e-6
            && r_eff <= self.shank_radius_mm() + 1e-9
        {
            let shank_z = if self.has_relieved_neck() {
                if r_eff <= self.neck_radius_mm() + 1e-9 {
                    self.flute_length_mm()
                } else {
                    self.neck_length_mm()
                }
            } else {
                self.flute_length_mm()
            };
            return Some((shank_z, ToolAssemblyPart::Shank));
        }
        if r_eff <= self.holder_nut_radius_mm + 1e-9 {
            return Some((self.stickout_length_mm, ToolAssemblyPart::Holder));
        }
        if r_eff <= self.holder_body_radius_mm + 1e-9 {
            let dr = (self.holder_body_radius_mm - self.holder_nut_radius_mm).max(1e-9);
            let frac = ((r_eff - self.holder_nut_radius_mm) / dr).clamp(0.0, 1.0);
            let dz = self.holder_cone_z0 + frac * (self.holder_cone_z1 - self.holder_cone_z0);
            return Some((dz, ToolAssemblyPart::Holder));
        }
        None
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

    #[test]
    fn lollipop_slot_and_dovetail_envelopes_and_minkowski_dilation_are_exact() {
        // 1. LollipopEndmill (D=6 -> R=3, neck=3 -> r_neck=1.5, z_join = 3 + sqrt(6.75))
        let z_join_expected = 3.0 + (6.75_f64).sqrt();
        let lolli_g = CutterGeometry::LollipopEndmill {
            diameter_mm: 6.0,
            neck_diameter_mm: 3.0,
            neck_length_mm: 18.0,
            flute_length_mm: z_join_expected,
            overall_length_mm: 55.0,
            shank_diameter_mm: 6.0,
            flutes: 4,
        };
        let lolli = AnalyticCutterEnvelope::from_geometry(&lolli_g);
        assert!(lolli.has_relieved_neck());
        assert!(lolli.has_bounded_undercut_head());
        assert!(!lolli.is_thread_mill());
        assert!(approx_eq(lolli.cutting_radius_mm(), 3.0, 1e-9));
        assert!(approx_eq(lolli.neck_radius_mm(), 1.5, 1e-9));
        assert!(approx_eq(lolli.effective_reach_mm(), 18.0, 1e-9));
        // Equator at dz=3 -> r=3; upper hemisphere at dz=5 -> r=sqrt(30-25)=sqrt(5)
        assert!(approx_eq(
            lolli.cutting_radius_at_dz(3.0).unwrap(),
            3.0,
            1e-9
        ));
        assert!(approx_eq(
            lolli.cutting_radius_at_dz(5.0).unwrap(),
            5.0_f64.sqrt(),
            1e-9
        ));
        assert!(lolli.cutting_radius_at_dz(z_join_expected + 0.5).is_none());
        assert!(approx_eq(
            lolli.assembly_radius_at_dz(10.0).unwrap(),
            1.5,
            1e-9
        ));
        assert!(approx_eq(
            lolli.assembly_radius_at_dz(22.0).unwrap(),
            3.0,
            1e-9
        ));
        // Axial cutting intervals
        let (lo0, hi0) = lolli.cutting_z_interval_at_radius(0.0).unwrap();
        assert!(approx_eq(lo0, 0.0, 1e-9));
        assert!(approx_eq(hi0, z_join_expected, 1e-9));
        let (lo2, hi2) = lolli.cutting_z_interval_at_radius(2.0).unwrap();
        assert!(approx_eq(lo2, 3.0 - 5.0_f64.sqrt(), 1e-9));
        assert!(approx_eq(hi2, 3.0 + 5.0_f64.sqrt(), 1e-9));
        let (lo3, hi3) = lolli.cutting_z_interval_at_radius(3.0).unwrap();
        assert!(approx_eq(lo3, 3.0, 1e-9));
        assert!(approx_eq(hi3, 3.0, 1e-9));
        assert!(lolli.cutting_z_interval_at_radius(3.01).is_none());
        // Horizontal Minkowski dilation by sr=0.5
        let lolli_dil = lolli.with_stock_allowance(0.5, 0.1);
        assert!(approx_eq(lolli_dil.cutting_radius_mm(), 3.5, 1e-9));
        assert!(approx_eq(lolli_dil.neck_radius_mm(), 2.0, 1e-9));
        assert!(approx_eq(
            lolli_dil.cutting_radius_at_dz(0.0).unwrap(),
            0.5,
            1e-9
        ));
        assert!(approx_eq(
            lolli_dil.cutting_radius_at_dz(3.0).unwrap(),
            3.5,
            1e-9
        ));
        assert!(approx_eq(
            lolli_dil.cutting_z_interval_at_radius(0.5).unwrap().0,
            0.0,
            1e-9
        ));

        // 2. SlotMill (D=12 -> R=6, H=4, rc=0.5 -> flat_r=5.5, neck=5 -> r_neck=2.5)
        let slot_g = CutterGeometry::SlotMill {
            diameter_mm: 12.0,
            cutting_thickness_mm: 4.0,
            corner_radius_mm: 0.5,
            neck_diameter_mm: 5.0,
            neck_length_mm: 18.0,
            overall_length_mm: 58.0,
            shank_diameter_mm: 6.0,
            flutes: 4,
        };
        let slot = AnalyticCutterEnvelope::from_geometry(&slot_g);
        assert!(slot.has_relieved_neck());
        assert!(slot.has_bounded_undercut_head());
        assert!(approx_eq(
            slot.cutting_radius_at_dz(2.0).unwrap(),
            6.0,
            1e-9
        ));
        assert!(approx_eq(
            slot.cutting_radius_at_dz(0.0).unwrap(),
            5.5,
            1e-9
        ));
        assert!(approx_eq(
            slot.cutting_radius_at_dz(4.0).unwrap(),
            5.5,
            1e-9
        ));
        assert!(slot.cutting_radius_at_dz(4.5).is_none());
        assert!(approx_eq(
            slot.assembly_radius_at_dz(10.0).unwrap(),
            2.5,
            1e-9
        ));
        assert!(approx_eq(
            slot.assembly_radius_at_dz(22.0).unwrap(),
            3.0,
            1e-9
        ));
        let (slo, shi) = slot.cutting_z_interval_at_radius(4.0).unwrap();
        assert!(approx_eq(slo, 0.0, 1e-9));
        assert!(approx_eq(shi, 4.0, 1e-9));
        let (slo_rim, shi_rim) = slot.cutting_z_interval_at_radius(6.0).unwrap();
        assert!(approx_eq(slo_rim, 0.5, 1e-9));
        assert!(approx_eq(shi_rim, 3.5, 1e-9));
        let slot_dil = slot.with_stock_allowance(0.25, 0.0);
        assert!(approx_eq(slot_dil.cutting_radius_mm(), 6.25, 1e-9));
        assert!(approx_eq(
            slot_dil.cutting_radius_at_dz(0.0).unwrap(),
            5.75,
            1e-9
        ));

        // 3. DovetailCutter (D_bot=12 -> R_bot=6, 60°, H=5, rc=0 -> R_top = 6 - 5/sqrt(3))
        let dove_g = CutterGeometry::DovetailCutter {
            diameter_mm: 12.0,
            side_angle_deg: 60.0,
            cutting_height_mm: 5.0,
            corner_radius_mm: 0.0,
            neck_diameter_mm: 5.5,
            neck_length_mm: 18.0,
            overall_length_mm: 58.0,
            shank_diameter_mm: 6.0,
            flutes: 4,
        };
        let dove = AnalyticCutterEnvelope::from_geometry(&dove_g);
        let tan60 = 3.0_f64.sqrt();
        let expected_r_top = 6.0 - 5.0 / tan60;
        assert!(dove.has_relieved_neck());
        assert!(dove.has_bounded_undercut_head());
        assert!(approx_eq(
            dove.cutting_radius_at_dz(0.0).unwrap(),
            6.0,
            1e-9
        ));
        assert!(approx_eq(
            dove.cutting_radius_at_dz(2.5).unwrap(),
            6.0 - 2.5 / tan60,
            1e-9
        ));
        assert!(approx_eq(
            dove.cutting_radius_at_dz(5.0).unwrap(),
            expected_r_top,
            1e-9
        ));
        assert!(dove.cutting_radius_at_dz(5.2).is_none());
        assert!(approx_eq(
            dove.assembly_radius_at_dz(10.0).unwrap(),
            2.75,
            1e-9
        ));
        assert!(approx_eq(
            dove.assembly_radius_at_dz(22.0).unwrap(),
            3.0,
            1e-9
        ));
        let (dlo, dhi) = dove.cutting_z_interval_at_radius(4.5).unwrap();
        assert!(approx_eq(dlo, 0.0, 1e-9));
        assert!(approx_eq(dhi, 1.5 * tan60, 1e-9));
        let (dlo_core, dhi_core) = dove.cutting_z_interval_at_radius(2.0).unwrap();
        assert!(approx_eq(dlo_core, 0.0, 1e-9));
        assert!(approx_eq(dhi_core, 5.0, 1e-9));
        let dove_dil = dove.with_stock_allowance(0.5, 0.0);
        assert!(approx_eq(dove_dil.cutting_radius_mm(), 6.5, 1e-9));
        assert!(approx_eq(
            dove_dil.cutting_radius_at_dz(5.0).unwrap(),
            expected_r_top + 0.5,
            1e-9
        ));
    }
}
