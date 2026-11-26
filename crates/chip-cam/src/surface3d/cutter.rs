use chip_tools::{AnalyticCutterEnvelope, CutterEnvelopeShape, CutterGeometry, Tool};

use crate::fixture_avoidance::ToolAssemblyProfile;

/// Full-assembly upper shank, collet nut, and tapered holder clearance envelope (`mm` above cutter
/// tip) derived from [`ToolAssemblyProfile::from_tool`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AssemblyClearanceSpec {
    /// Axial height above tip (`mm`) where the non-cutting shank begins (uses relieved-neck reach
    /// for necked tools, `flute_length_mm` for standard tools).
    pub flute_length_mm: f64,
    /// Axial height above tip (`mm`) where the collet nut / tool holder begins.
    pub stickout_length_mm: f64,
    /// Physical non-cutting shank radius (`mm`, before radial stock allowance).
    pub shank_radius_mm: f64,
    /// Physical collet nut cylinder radius (`mm`, before `holder_clearance_mm`).
    pub holder_nut_radius_mm: f64,
    /// Physical upper holder body radius (`mm`, before `holder_clearance_mm`).
    pub holder_body_radius_mm: f64,
    /// Axial height (`mm`) where the tapered holder cone starts expanding from nut to body.
    pub holder_cone_z0: f64,
    /// Axial height (`mm`) where the tapered holder cone reaches `holder_body_radius_mm`.
    pub holder_cone_z1: f64,
    /// Radial & axial safety clearance margin (`mm`) applied around the collet nut and holder cone.
    pub holder_clearance_mm: f64,
    /// Radial stock allowance (`mm`) applied around the wider non-cutting shank tier.
    pub stock_radial_mm: f64,
}

impl AssemblyClearanceSpec {
    /// Effective horizontal shank clearance radius (`mm`), honoring radial stock-to-leave and a
    /// safe non-cutting shank clearance margin (`>= 0.5 * holder_clearance_mm`) so spinning wider
    /// shanks never rub thin uncut roughing skins on unreached overhang walls.
    #[inline]
    pub fn effective_shank_radius_mm(&self) -> f64 {
        (self.shank_radius_mm + self.stock_radial_mm.max(0.5 * self.holder_clearance_mm)).max(0.0)
    }

    /// Effective horizontal collet nut clearance radius (`mm`).
    #[inline]
    pub fn effective_holder_nut_radius_mm(&self) -> f64 {
        (self.holder_nut_radius_mm + self.holder_clearance_mm).max(0.0)
    }

    /// Effective horizontal upper holder body clearance radius (`mm`).
    #[inline]
    pub fn effective_holder_body_radius_mm(&self) -> f64 {
        (self.holder_body_radius_mm + self.holder_clearance_mm).max(0.0)
    }

    /// Maximum radial reach (`mm`) of any shank or holder tier in this assembly spec.
    #[inline]
    pub fn max_radius_mm(&self) -> f64 {
        self.effective_shank_radius_mm()
            .max(self.effective_holder_nut_radius_mm())
            .max(self.effective_holder_body_radius_mm())
    }
}

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

impl SurfaceCutterShape {
    /// Maximum horizontal contact radius (`mm`) of this shape.
    #[inline]
    pub fn max_radius_mm(&self) -> f64 {
        match *self {
            Self::Flat { radius_mm } | Self::Ball { radius_mm } => radius_mm,
            Self::BullNose {
                flat_radius_mm,
                corner_radius_mm,
            } => flat_radius_mm + corner_radius_mm,
            Self::TaperedVBit {
                tip_radius_mm,
                upper_radius_mm,
                ..
            } => upper_radius_mm.max(tip_radius_mm),
        }
    }

    /// Lower contact profile height `h(d) >= 0` above tip at radial distance `d >= 0`.
    /// Returns `None` if `d > self.max_radius_mm() + 1e-9`.
    #[inline]
    pub fn profile_height_at_radius(&self, d: f64) -> Option<f64> {
        if !d.is_finite() || d < -1e-9 {
            return None;
        }
        let r = d.max(0.0);
        match *self {
            Self::Flat { radius_mm } => (r <= radius_mm + 1e-9).then_some(0.0),
            Self::Ball { radius_mm } => {
                if r > radius_mm + 1e-9 {
                    None
                } else {
                    let rc = r.min(radius_mm);
                    Some(radius_mm - (radius_mm * radius_mm - rc * rc).max(0.0).sqrt())
                }
            }
            Self::BullNose {
                flat_radius_mm,
                corner_radius_mm,
            } => {
                let r_max = flat_radius_mm + corner_radius_mm;
                if r > r_max + 1e-9 {
                    None
                } else if r <= flat_radius_mm {
                    Some(0.0)
                } else {
                    let dr = (r - flat_radius_mm).min(corner_radius_mm);
                    Some(
                        corner_radius_mm
                            - (corner_radius_mm * corner_radius_mm - dr * dr)
                                .max(0.0)
                                .sqrt(),
                    )
                }
            }
            Self::TaperedVBit {
                tip_radius_mm,
                upper_radius_mm,
                half_angle_rad,
            } => {
                if r > upper_radius_mm + 1e-9 {
                    None
                } else if r <= tip_radius_mm {
                    Some(0.0)
                } else {
                    let tan_a = half_angle_rad.tan().max(1e-9);
                    Some((r - tip_radius_mm) / tan_a)
                }
            }
        }
    }
}

/// Complete analytical cutter contact envelope including axial stock-to-leave, flute height,
/// and optional full-assembly shank & holder clearance tiers, backed by the shared
/// [`AnalyticCutterEnvelope`] from `chip-tools`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceCutterProfile {
    pub shape: SurfaceCutterShape,
    pub flute_length_mm: f64,
    pub stock_axial_mm: f64,
    /// Optional non-cutting shank, collet nut, and tapered holder clearance envelope (`None` for
    /// flute-only [`SurfaceCutterProfile::from_tool_geometry`], `Some(...)` for
    /// [`SurfaceCutterProfile::from_tool`]).
    pub assembly: Option<AssemblyClearanceSpec>,
    envelope: AnalyticCutterEnvelope,
}

impl SurfaceCutterProfile {
    /// Builds an exact analytical flute-only [`SurfaceCutterProfile`] from a tool's
    /// [`CutterGeometry`] and radial/axial stock-to-leave allowances (`mm`).
    ///
    /// Leaves `assembly: None` for zero-overhead flute-only analytical queries.
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
            assembly: None,
            envelope,
        }
    }

    /// Builds a full-assembly [`SurfaceCutterProfile`] from [`Tool`] including non-cutting shank,
    /// collet nut, and tapered holder clearance derived from [`ToolAssemblyProfile::from_tool`].
    pub fn from_tool(
        tool: &Tool,
        stock_radial_mm: f64,
        stock_axial_mm: f64,
        holder_clearance_mm: f64,
    ) -> Self {
        let mut prof = Self::from_tool_geometry(&tool.geometry, stock_radial_mm, stock_axial_mm);
        let asm = ToolAssemblyProfile::from_tool(tool);
        let shank_start_z = if asm.has_relieved_neck {
            asm.neck_length_mm.max(asm.flute_length_mm)
        } else {
            asm.flute_length_mm
        };
        prof.assembly = Some(AssemblyClearanceSpec {
            flute_length_mm: shank_start_z.max(0.1),
            stickout_length_mm: asm.stickout_length_mm.max(shank_start_z),
            shank_radius_mm: asm.shank_radius_mm,
            holder_nut_radius_mm: asm.holder_nut_radius_mm,
            holder_body_radius_mm: asm.holder_body_radius_mm,
            holder_cone_z0: asm.holder_cone_z0,
            holder_cone_z1: asm.holder_cone_z1,
            holder_clearance_mm: holder_clearance_mm.max(0.0),
            stock_radial_mm: stock_radial_mm.max(0.0),
        });
        prof
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
        let assembly = self.assembly.map(|mut asm| {
            asm.shank_radius_mm += sr;
            asm.holder_nut_radius_mm += sr;
            asm.holder_body_radius_mm += sr;
            asm
        });
        Self {
            shape,
            flute_length_mm: self.flute_length_mm,
            stock_axial_mm: self.stock_axial_mm,
            assembly,
            envelope,
        }
    }

    /// Returns a radially shrunk copy of `self` (`shrink_mm >= 0`) used for grazing-tolerant
    /// 2.5D overhang checks so flush vertical walls and modeled hole cylinders do not false-trigger.
    pub fn with_radial_shrink(&self, shrink_mm: f64) -> Self {
        let s = shrink_mm.max(0.0);
        if s <= 1e-9 {
            return *self;
        }
        let shape = match self.shape {
            SurfaceCutterShape::Flat { radius_mm } => SurfaceCutterShape::Flat {
                radius_mm: (radius_mm - s).max(0.05),
            },
            SurfaceCutterShape::Ball { radius_mm } => SurfaceCutterShape::Ball {
                radius_mm: (radius_mm - s).max(0.05),
            },
            SurfaceCutterShape::BullNose {
                flat_radius_mm,
                corner_radius_mm,
            } => {
                if flat_radius_mm >= s {
                    SurfaceCutterShape::BullNose {
                        flat_radius_mm: flat_radius_mm - s,
                        corner_radius_mm,
                    }
                } else {
                    let rem = s - flat_radius_mm;
                    SurfaceCutterShape::Ball {
                        radius_mm: (corner_radius_mm - rem).max(0.05),
                    }
                }
            }
            SurfaceCutterShape::TaperedVBit {
                tip_radius_mm,
                upper_radius_mm,
                half_angle_rad,
            } => {
                let new_upper = (upper_radius_mm - s).max(0.05);
                let new_tip = (tip_radius_mm - s).max(0.0).min(new_upper);
                SurfaceCutterShape::TaperedVBit {
                    tip_radius_mm: new_tip,
                    upper_radius_mm: new_upper,
                    half_angle_rad,
                }
            }
        };
        let assembly = self.assembly.map(|mut asm| {
            asm.shank_radius_mm = (asm.shank_radius_mm - s).max(0.05);
            asm.holder_nut_radius_mm = (asm.holder_nut_radius_mm - s).max(0.05);
            asm.holder_body_radius_mm =
                (asm.holder_body_radius_mm - s).max(asm.holder_nut_radius_mm);
            asm
        });
        Self {
            shape,
            flute_length_mm: self.flute_length_mm,
            stock_axial_mm: self.stock_axial_mm,
            assembly,
            envelope: self.envelope,
        }
    }

    /// Underlying shared [`AnalyticCutterEnvelope`] from `chip-tools`.
    #[inline]
    pub const fn envelope(&self) -> &AnalyticCutterEnvelope {
        &self.envelope
    }

    /// Maximum horizontal contact radius (`mm`) of the cutting flutes (including radial stock).
    #[inline]
    pub fn max_radius_mm(&self) -> f64 {
        self.shape.max_radius_mm()
    }

    /// Maximum horizontal clearance radius (`mm`) across the full revolved assembly (flutes,
    /// wider shank, collet nut, and upper holder body).
    #[inline]
    pub fn max_assembly_radius_mm(&self) -> f64 {
        match self.assembly {
            Some(asm) => self.max_radius_mm().max(asm.max_radius_mm()),
            None => self.max_radius_mm(),
        }
    }

    /// Returns `true` when any wider-shank or holder tier of `self.assembly` can vertically
    /// overlap `surface_max_z` when the cutter tip descends to `z_floor`.
    #[inline]
    pub fn assembly_can_overlap(&self, surface_max_z: f64, z_floor: f64) -> bool {
        let Some(asm) = self.assembly else {
            return false;
        };
        let r_flute = self.max_radius_mm();
        let wider_shank = asm.effective_shank_radius_mm() > r_flute + 1e-6
            && surface_max_z + self.stock_axial_mm > z_floor + asm.flute_length_mm - 1e-4;
        let holder_can_hit =
            surface_max_z + asm.holder_clearance_mm > z_floor + asm.stickout_length_mm - 1e-4;
        wider_shank || holder_can_hit
    }

    /// Tightest horizontal query/padding radius (`mm`) required across all tiers that can actually
    /// overlap `surface_max_z` at minimum tip height `z_floor`.
    #[inline]
    pub fn effective_query_radius_mm(&self, surface_max_z: f64, z_floor: f64) -> f64 {
        let r_flute = self.max_radius_mm();
        let Some(asm) = self.assembly else {
            return r_flute;
        };
        let mut r = r_flute;
        if asm.effective_shank_radius_mm() > r_flute + 1e-6
            && surface_max_z + self.stock_axial_mm > z_floor + asm.flute_length_mm - 1e-4
        {
            r = r.max(asm.effective_shank_radius_mm());
        }
        if surface_max_z + asm.holder_clearance_mm > z_floor + asm.stickout_length_mm - 1e-4 {
            r = r.max(asm.effective_holder_nut_radius_mm());
        }
        if surface_max_z + asm.holder_clearance_mm > z_floor + asm.holder_cone_z0 - 1e-4 {
            r = r.max(asm.effective_holder_body_radius_mm());
        }
        r
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
        self.shape.profile_height_at_radius(d)
    }
}
