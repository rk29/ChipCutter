use chip_cad::{BodyId, CadDocument, CurveKind};
use chip_math::{Aabb3, DVec3, EvalContext, ParamLength, RigidFrame3};
use serde::{Deserialize, Serialize};

use crate::wcs::CamError;

/// Principal WCS axis along which Cylindrical Stock is oriented.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "lowercase")]
pub enum StockAxis {
    X,
    Y,
    #[default]
    Z,
}

impl StockAxis {
    pub const ALL: [Self; 3] = [Self::Z, Self::X, Self::Y];

    pub const fn label(self) -> &'static str {
        match self {
            Self::X => "WCS X Axis",
            Self::Y => "WCS Y Axis",
            Self::Z => "WCS Z Axis (Vertical Bar)",
        }
    }
}

/// XY alignment of CAD model inside Fixed-Size Box Stock.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum FixedBoxXyAlign {
    #[default]
    CenterXy,
    OffsetFromFrontLeft {
        dx: ParamLength,
        dy: ParamLength,
    },
}

/// Z alignment of CAD model inside Fixed-Size Box Stock.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum FixedBoxZAlign {
    OffsetFromTop { top_allowance: ParamLength },
    OffsetFromBottom { bottom_allowance: ParamLength },
    CenterZ,
}

impl Default for FixedBoxZAlign {
    fn default() -> Self {
        Self::OffsetFromTop {
            top_allowance: ParamLength::from_mm(1.0),
        }
    }
}

fn default_true() -> bool {
    true
}

/// Parametric Stock definition in oriented Setup WCS axes (`03-setups-wcs.md`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum StockConfig {
    RelativeBox {
        side_pos_x: ParamLength,
        side_neg_x: ParamLength,
        side_pos_y: ParamLength,
        side_neg_y: ParamLength,
        #[serde(default = "default_true")]
        symmetric_xy: bool,
        top_offset_z: ParamLength,
        bottom_offset_z: ParamLength,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        round_increment_mm: Option<f64>,
    },
    FixedBox {
        width_x: ParamLength,
        depth_y: ParamLength,
        height_z: ParamLength,
        #[serde(default)]
        xy_align: FixedBoxXyAlign,
        #[serde(default)]
        z_align: FixedBoxZAlign,
    },
    RelativeCylinder {
        #[serde(default)]
        axis: StockAxis,
        radial_offset: ParamLength,
        top_offset: ParamLength,
        bottom_offset: ParamLength,
    },
    FixedCylinder {
        #[serde(default)]
        axis: StockAxis,
        diameter: ParamLength,
        length: ParamLength,
        axial_top_offset: ParamLength,
    },
}

impl Default for StockConfig {
    fn default() -> Self {
        Self::RelativeBox {
            side_pos_x: ParamLength::from_mm(2.0),
            side_neg_x: ParamLength::from_mm(2.0),
            side_pos_y: ParamLength::from_mm(2.0),
            side_neg_y: ParamLength::from_mm(2.0),
            symmetric_xy: true,
            top_offset_z: ParamLength::from_mm(1.0),
            bottom_offset_z: ParamLength::from_mm(0.0),
            round_increment_mm: None,
        }
    }
}

impl StockConfig {
    pub const fn kind_label(&self) -> &'static str {
        match self {
            Self::RelativeBox { .. } => "Relative Box Stock",
            Self::FixedBox { .. } => "Fixed-Size Box Stock",
            Self::RelativeCylinder { .. } => "Relative Cylindrical Stock",
            Self::FixedCylinder { .. } => "Fixed Cylindrical Stock",
        }
    }

    pub fn reevaluate(&mut self, ctx: &EvalContext) {
        match self {
            Self::RelativeBox {
                side_pos_x,
                side_neg_x,
                side_pos_y,
                side_neg_y,
                top_offset_z,
                bottom_offset_z,
                ..
            } => {
                for p in [
                    side_pos_x,
                    side_neg_x,
                    side_pos_y,
                    side_neg_y,
                    top_offset_z,
                    bottom_offset_z,
                ] {
                    let _ = p.reevaluate(ctx);
                }
            }
            Self::FixedBox {
                width_x,
                depth_y,
                height_z,
                xy_align,
                z_align,
            } => {
                let _ = width_x.reevaluate(ctx);
                let _ = depth_y.reevaluate(ctx);
                let _ = height_z.reevaluate(ctx);
                if let FixedBoxXyAlign::OffsetFromFrontLeft { dx, dy } = xy_align {
                    let _ = dx.reevaluate(ctx);
                    let _ = dy.reevaluate(ctx);
                }
                match z_align {
                    FixedBoxZAlign::OffsetFromTop { top_allowance } => {
                        let _ = top_allowance.reevaluate(ctx);
                    }
                    FixedBoxZAlign::OffsetFromBottom { bottom_allowance } => {
                        let _ = bottom_allowance.reevaluate(ctx);
                    }
                    FixedBoxZAlign::CenterZ => {}
                }
            }
            Self::RelativeCylinder {
                radial_offset,
                top_offset,
                bottom_offset,
                ..
            } => {
                let _ = radial_offset.reevaluate(ctx);
                let _ = top_offset.reevaluate(ctx);
                let _ = bottom_offset.reevaluate(ctx);
            }
            Self::FixedCylinder {
                diameter,
                length,
                axial_top_offset,
                ..
            } => {
                let _ = diameter.reevaluate(ctx);
                let _ = length.reevaluate(ctx);
                let _ = axial_top_offset.reevaluate(ctx);
            }
        }
    }

    /// Computes unshifted stock bounds in the rotation-only WCS frame (`origin == ZERO`) around `model_rot_aabb`.
    pub fn compute_oriented_bounds(
        &self,
        model_rot_aabb: &Aabb3,
    ) -> Result<ResolvedStockGeometry, CamError> {
        let model_size = model_rot_aabb.size();
        match self {
            Self::RelativeBox {
                side_pos_x,
                side_neg_x,
                side_pos_y,
                side_neg_y,
                symmetric_xy,
                top_offset_z,
                bottom_offset_z,
                round_increment_mm,
            } => {
                let px = side_pos_x.value_mm.max(0.0);
                let nx = if *symmetric_xy {
                    px
                } else {
                    side_neg_x.value_mm.max(0.0)
                };
                let py = side_pos_y.value_mm.max(0.0);
                let ny = if *symmetric_xy {
                    py
                } else {
                    side_neg_y.value_mm.max(0.0)
                };
                let tz = top_offset_z.value_mm.max(0.0);
                let bz = bottom_offset_z.value_mm.max(0.0);

                let mut min = DVec3::new(
                    model_rot_aabb.min.x - nx,
                    model_rot_aabb.min.y - ny,
                    model_rot_aabb.min.z - bz,
                );
                let mut max = DVec3::new(
                    model_rot_aabb.max.x + px,
                    model_rot_aabb.max.y + py,
                    model_rot_aabb.max.z + tz,
                );

                if let Some(inc) = *round_increment_mm
                    && inc.is_finite()
                    && inc > 1e-4
                {
                    let cur_sz = max - min;
                    let rounded = DVec3::new(
                        (cur_sz.x / inc).ceil() * inc,
                        (cur_sz.y / inc).ceil() * inc,
                        (cur_sz.z / inc).ceil() * inc,
                    );
                    let extra = (rounded - cur_sz).max(DVec3::ZERO);
                    // Expand symmetrically in XY and on bottom Z
                    min.x -= 0.5 * extra.x;
                    max.x += 0.5 * extra.x;
                    min.y -= 0.5 * extra.y;
                    max.y += 0.5 * extra.y;
                    min.z -= extra.z;
                }

                Ok(ResolvedStockGeometry::Box {
                    aabb_wcs: Aabb3::new(min, max),
                })
            }
            Self::FixedBox {
                width_x,
                depth_y,
                height_z,
                xy_align,
                z_align,
            } => {
                let wx = width_x.value_mm;
                let dy = depth_y.value_mm;
                let hz = height_z.value_mm;
                for (name, v) in [("width_x", wx), ("depth_y", dy), ("height_z", hz)] {
                    if !v.is_finite() || v <= 0.0 {
                        return Err(CamError::InvalidStockDimension {
                            field: name,
                            value_mm: v,
                        });
                    }
                }
                if wx + 1e-4 < model_size.x {
                    return Err(CamError::StockSmallerThanModel {
                        axis: "X",
                        stock_mm: wx,
                        model_mm: model_size.x,
                    });
                }
                if dy + 1e-4 < model_size.y {
                    return Err(CamError::StockSmallerThanModel {
                        axis: "Y",
                        stock_mm: dy,
                        model_mm: model_size.y,
                    });
                }
                if hz + 1e-4 < model_size.z {
                    return Err(CamError::StockSmallerThanModel {
                        axis: "Z",
                        stock_mm: hz,
                        model_mm: model_size.z,
                    });
                }

                let (min_x, min_y) = match xy_align {
                    FixedBoxXyAlign::CenterXy => {
                        let c = model_rot_aabb.center();
                        (c.x - 0.5 * wx, c.y - 0.5 * dy)
                    }
                    FixedBoxXyAlign::OffsetFromFrontLeft { dx, dy: off_y } => (
                        model_rot_aabb.min.x - dx.value_mm,
                        model_rot_aabb.min.y - off_y.value_mm,
                    ),
                };
                let max_z = match z_align {
                    FixedBoxZAlign::OffsetFromTop { top_allowance } => {
                        model_rot_aabb.max.z + top_allowance.value_mm
                    }
                    FixedBoxZAlign::OffsetFromBottom { bottom_allowance } => {
                        model_rot_aabb.min.z - bottom_allowance.value_mm + hz
                    }
                    FixedBoxZAlign::CenterZ => model_rot_aabb.center().z + 0.5 * hz,
                };
                let min_z = max_z - hz;

                Ok(ResolvedStockGeometry::Box {
                    aabb_wcs: Aabb3::new(
                        DVec3::new(min_x, min_y, min_z),
                        DVec3::new(min_x + wx, min_y + dy, max_z),
                    ),
                })
            }
            Self::RelativeCylinder {
                axis,
                radial_offset,
                top_offset,
                bottom_offset,
            } => {
                let ro = radial_offset.value_mm.max(0.0);
                let to = top_offset.value_mm.max(0.0);
                let bo = bottom_offset.value_mm.max(0.0);
                let c = model_rot_aabb.center();
                let (rad_model, min_ax, max_ax) = match axis {
                    StockAxis::Z => (
                        0.5 * model_size.x.hypot(model_size.y),
                        model_rot_aabb.min.z - bo,
                        model_rot_aabb.max.z + to,
                    ),
                    StockAxis::X => (
                        0.5 * model_size.y.hypot(model_size.z),
                        model_rot_aabb.min.x - bo,
                        model_rot_aabb.max.x + to,
                    ),
                    StockAxis::Y => (
                        0.5 * model_size.x.hypot(model_size.z),
                        model_rot_aabb.min.y - bo,
                        model_rot_aabb.max.y + to,
                    ),
                };
                let radius_mm = (rad_model + ro).max(0.5);
                Ok(build_cylinder_stock(*axis, c, radius_mm, min_ax, max_ax))
            }
            Self::FixedCylinder {
                axis,
                diameter,
                length,
                axial_top_offset,
            } => {
                let d = diameter.value_mm;
                let len = length.value_mm;
                if !d.is_finite() || d <= 0.0 {
                    return Err(CamError::InvalidStockDimension {
                        field: "diameter",
                        value_mm: d,
                    });
                }
                if !len.is_finite() || len <= 0.0 {
                    return Err(CamError::InvalidStockDimension {
                        field: "length",
                        value_mm: len,
                    });
                }
                let radius_mm = 0.5 * d;
                let c = model_rot_aabb.center();
                let top_allow = axial_top_offset.value_mm;
                let max_ax = match axis {
                    StockAxis::Z => model_rot_aabb.max.z + top_allow,
                    StockAxis::X => model_rot_aabb.max.x + top_allow,
                    StockAxis::Y => model_rot_aabb.max.y + top_allow,
                };
                let min_ax = max_ax - len;
                Ok(build_cylinder_stock(*axis, c, radius_mm, min_ax, max_ax))
            }
        }
    }
}

fn build_cylinder_stock(
    axis: StockAxis,
    center: DVec3,
    radius_mm: f64,
    min_axial_mm: f64,
    max_axial_mm: f64,
) -> ResolvedStockGeometry {
    let aabb_wcs = match axis {
        StockAxis::Z => Aabb3::new(
            DVec3::new(center.x - radius_mm, center.y - radius_mm, min_axial_mm),
            DVec3::new(center.x + radius_mm, center.y + radius_mm, max_axial_mm),
        ),
        StockAxis::X => Aabb3::new(
            DVec3::new(min_axial_mm, center.y - radius_mm, center.z - radius_mm),
            DVec3::new(max_axial_mm, center.y + radius_mm, center.z + radius_mm),
        ),
        StockAxis::Y => Aabb3::new(
            DVec3::new(center.x - radius_mm, min_axial_mm, center.z - radius_mm),
            DVec3::new(center.x + radius_mm, max_axial_mm, center.z + radius_mm),
        ),
    };
    ResolvedStockGeometry::Cylinder {
        axis,
        center_wcs: aabb_wcs.center(),
        radius_mm,
        min_axial_mm,
        max_axial_mm,
        aabb_wcs,
    }
}

/// Concrete evaluated 3D stock shape in active Setup WCS coordinates (`mm`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ResolvedStockGeometry {
    Box {
        aabb_wcs: Aabb3,
    },
    Cylinder {
        axis: StockAxis,
        center_wcs: DVec3,
        radius_mm: f64,
        min_axial_mm: f64,
        max_axial_mm: f64,
        aabb_wcs: Aabb3,
    },
}

impl ResolvedStockGeometry {
    #[inline]
    pub fn aabb_wcs(&self) -> Aabb3 {
        match *self {
            Self::Box { aabb_wcs } | Self::Cylinder { aabb_wcs, .. } => aabb_wcs,
        }
    }

    pub fn shifted_by(self, shift_wcs: DVec3) -> Self {
        match self {
            Self::Box { aabb_wcs } => Self::Box {
                aabb_wcs: Aabb3::new(aabb_wcs.min + shift_wcs, aabb_wcs.max + shift_wcs),
            },
            Self::Cylinder {
                axis,
                center_wcs,
                radius_mm,
                min_axial_mm,
                max_axial_mm,
                aabb_wcs,
            } => {
                let ax_shift = match axis {
                    StockAxis::Z => shift_wcs.z,
                    StockAxis::X => shift_wcs.x,
                    StockAxis::Y => shift_wcs.y,
                };
                Self::Cylinder {
                    axis,
                    center_wcs: center_wcs + shift_wcs,
                    radius_mm,
                    min_axial_mm: min_axial_mm + ax_shift,
                    max_axial_mm: max_axial_mm + ax_shift,
                    aabb_wcs: Aabb3::new(aabb_wcs.min + shift_wcs, aabb_wcs.max + shift_wcs),
                }
            }
        }
    }
}

/// Computes the exact tight axis-aligned bounding box of `target_bodies` (or all bodies if empty)
/// projected along the orthonormal axes of `rot_frame` (`origin == ZERO`).
///
/// Combines analytic `CircleArc` axis-extrema with exact `f64` B-Rep triangle/edge vertices so
/// rotated/tilted solids never suffer loose world-AABB overestimation!
pub fn compute_tight_oriented_model_aabb(
    doc: &CadDocument,
    target_bodies: &[BodyId],
    rot_frame: &RigidFrame3,
) -> Result<Aabb3, CamError> {
    if doc.bodies.is_empty() {
        return Err(CamError::EmptyCadDocument);
    }

    let include_body = |bid: BodyId| target_bodies.is_empty() || target_bodies.contains(&bid);
    let mut aabb = Aabb3::EMPTY;

    // 1. Exact `f64` triangle mesh vertices belonging to target bodies
    for tri in &doc.render_mesh.pick_triangles {
        if let Some(face) = doc.face(tri.face_id)
            && include_body(face.body_id)
        {
            aabb.extend_point(rot_frame.world_to_local_point(tri.v0_mm));
            aabb.extend_point(rot_frame.world_to_local_point(tri.v1_mm));
            aabb.extend_point(rot_frame.world_to_local_point(tri.v2_mm));
        }
    }

    // 2. Exact analytic full-circle extrema along each WCS axis `(X_wcs, Y_wcs, Z_wcs)`
    for edge in &doc.edges {
        if !include_body(edge.body_id) {
            continue;
        }
        match edge.curve {
            CurveKind::Line { start_mm, end_mm } => {
                aabb.extend_point(rot_frame.world_to_local_point(start_mm));
                aabb.extend_point(rot_frame.world_to_local_point(end_mm));
            }
            CurveKind::CircleArc {
                center_mm,
                normal,
                radius_mm,
                is_full_circle: true,
                ..
            } => {
                let c_loc = rot_frame.world_to_local_point(center_mm);
                let n_loc = rot_frame.world_to_local_dir(normal.normalize_or_zero());
                // Along axis i, a circle of radius R with unit normal n spans c_i ± R * sqrt(1 - n_i^2)
                let dx = radius_mm * (1.0 - n_loc.x * n_loc.x).max(0.0).sqrt();
                let dy = radius_mm * (1.0 - n_loc.y * n_loc.y).max(0.0).sqrt();
                let dz = radius_mm * (1.0 - n_loc.z * n_loc.z).max(0.0).sqrt();
                aabb.extend_point(c_loc - DVec3::new(dx, dy, dz));
                aabb.extend_point(c_loc + DVec3::new(dx, dy, dz));
            }
            _ => {}
        }
    }

    if !aabb.is_valid() {
        return Err(CamError::EmptyCadDocument);
    }
    Ok(aabb)
}
