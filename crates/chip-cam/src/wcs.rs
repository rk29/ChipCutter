use chip_cad::{
    CadDocument, CurveKind, EdgeId, FaceId, SelectionBinding, SurfaceKind, analyze_cylindrical_hole,
};
use chip_math::{Aabb3, DVec3, EvalContext, FrameError, ParamLength, RigidFrame3};
use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Errors raised during WCS orientation, Stock computation, Feature extraction, or Height validation.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum CamError {
    #[error("CAD document contains no solid bodies")]
    EmptyCadDocument,
    #[error(transparent)]
    Frame(#[from] FrameError),
    #[error(
        "invalid WCS axis reference: CAD entity #{index} is missing or has incompatible geometry"
    )]
    InvalidWcsAxisRef { index: u32 },
    #[error("invalid WCS origin feature reference: CAD entity #{index} is missing or incompatible")]
    InvalidWcsOriginRef { index: u32 },
    #[error("stock dimension '{field}' resolved to {value_mm:.4} mm (must be finite and > 0)")]
    InvalidStockDimension { field: &'static str, value_mm: f64 },
    #[error(
        "fixed stock {axis} ({stock_mm:.3} mm) is smaller than model {axis} ({model_mm:.3} mm)"
    )]
    StockSmallerThanModel {
        axis: &'static str,
        stock_mm: f64,
        model_mm: f64,
    },
    #[error(
        "selected face #{face_id} is not planar horizontal relative to Setup WCS +Z (|n·Z| = {cos_z:.4})"
    )]
    FaceNotHorizontalInWcs { face_id: u32, cos_z: f64 },
    #[error(
        "selected hole #{face_id} axis is not parallel to Setup WCS +Z (|axis·Z| = {cos_z:.4})"
    )]
    HoleNotAlignedWithWcsZ { face_id: u32, cos_z: f64 },
    #[error("selected entity #{index} could not be resolved in the current CAD document")]
    MissingSelectedEntity { index: u32 },
    #[error("selected sketch item '{sketch_id}/{item_id}' could not be resolved")]
    MissingSelectedSketch { sketch_id: String, item_id: String },
    #[error(
        "selected sketch '{sketch_name}' normal is not aligned with Setup WCS +Z (n·Z = {cos_z:.4})"
    )]
    SketchNotAlignedWithSetupZ { sketch_name: String, cos_z: f64 },
    #[error(
        "pure-2D setup (no 3D STEP model) cannot reference 3D B-Rep faces/edges for WCS orientation or origin"
    )]
    Pure2dRequiresBoxWcs,
    #[error(
        "height hierarchy violation: {upper_name} ({upper_z:.3} mm) must be >= {lower_name} ({lower_z:.3} mm)"
    )]
    HeightHierarchyViolation {
        upper_name: &'static str,
        upper_z: f64,
        lower_name: &'static str,
        lower_z: f64,
    },
    #[error("toolpath generation cancelled")]
    Cancelled,
    #[error("operation '{op_name}' has no cutting tool assigned")]
    MissingTool { op_name: String },
    #[error(
        "cutter '{tool_name}' ({tool_kind}) is incompatible with operation '{op_name}' ({op_kind})"
    )]
    IncompatibleTool {
        op_name: String,
        op_kind: &'static str,
        tool_name: String,
        tool_kind: &'static str,
    },
    #[error(
        "operation '{op_name}' ({op_kind}) requires at least one compatible geometry selection"
    )]
    EmptySelection {
        op_name: String,
        op_kind: &'static str,
    },
    #[error("invalid parameter '{param}' = {value:.4}: {reason}")]
    InvalidParameter {
        param: &'static str,
        value: f64,
        reason: &'static str,
    },
    #[error(
        "pocket cavity cannot accommodate cutter Ø{tool_diameter_mm:.3} mm (no collision-free interior offset)"
    )]
    PocketTooNarrowForTool { tool_diameter_mm: f64 },
    #[error("hole Ø{hole_diameter_mm:.3} mm is smaller than cutter Ø{tool_diameter_mm:.3} mm")]
    HoleSmallerThanTool {
        hole_diameter_mm: f64,
        tool_diameter_mm: f64,
    },
    #[error("invalid chamfer configuration: {reason}")]
    InvalidChamferGeometry { reason: String },
    #[error("invalid thread mill configuration: {reason}")]
    InvalidThreadMillGeometry { reason: String },
    #[error("invalid undercut configuration: {reason}")]
    InvalidUndercutGeometry { reason: String },
    #[error(
        "chamfer tip Z ({tip_z_wcs:.3} mm) collides with adjacent lower floor at Z = {floor_z_wcs:.3} mm (reduce tip_offset)"
    )]
    ChamferTipCollision { tip_z_wcs: f64, floor_z_wcs: f64 },
    #[error(
        "operation '{op_name}' produced an empty toolpath (check heights, stepover, and stock-to-leave)"
    )]
    EmptyToolpath { op_name: String },
    #[error("invalid 4th-axis rotary configuration for '{op_name}': {reason}")]
    InvalidRotarySetup { op_name: String, reason: String },
    #[error("invalid workholding fixture '{fixture_name}': {reason}")]
    InvalidFixture {
        fixture_name: String,
        reason: String,
    },
}

pub use chip_cl::WorkOffsetSlot;

/// 1D anchor selector along a bounding box axis (`Min`, `Center`, `Max`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum AxisAnchor {
    Min,
    #[default]
    Center,
    Max,
}

impl AxisAnchor {
    pub const ALL: [Self; 3] = [Self::Min, Self::Center, Self::Max];

    #[inline]
    pub fn interp(self, min: f64, max: f64) -> f64 {
        match self {
            Self::Min => min,
            Self::Center => 0.5 * (min + max),
            Self::Max => max,
        }
    }
}

/// 27-point grid selector (`3 × 3 × 3`) on a Stock or Model axis-aligned bounding box in WCS space.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct BoxAnchor27 {
    pub x: AxisAnchor,
    pub y: AxisAnchor,
    pub z: AxisAnchor,
}

impl Default for BoxAnchor27 {
    fn default() -> Self {
        Self::TOP_FRONT_LEFT
    }
}

impl BoxAnchor27 {
    pub const TOP_CENTER: Self = Self {
        x: AxisAnchor::Center,
        y: AxisAnchor::Center,
        z: AxisAnchor::Max,
    };

    pub const TOP_FRONT_LEFT: Self = Self {
        x: AxisAnchor::Min,
        y: AxisAnchor::Min,
        z: AxisAnchor::Max,
    };

    pub const BOTTOM_FRONT_LEFT: Self = Self {
        x: AxisAnchor::Min,
        y: AxisAnchor::Min,
        z: AxisAnchor::Min,
    };

    pub const VOLUMETRIC_CENTER: Self = Self {
        x: AxisAnchor::Center,
        y: AxisAnchor::Center,
        z: AxisAnchor::Center,
    };

    /// Returns all 27 anchor combinations in deterministic `(z, y, x)` order.
    pub fn all_27() -> [Self; 27] {
        let mut out = [Self::VOLUMETRIC_CENTER; 27];
        let mut idx = 0;
        for z in [AxisAnchor::Max, AxisAnchor::Center, AxisAnchor::Min] {
            for y in [AxisAnchor::Max, AxisAnchor::Center, AxisAnchor::Min] {
                for x in [AxisAnchor::Min, AxisAnchor::Center, AxisAnchor::Max] {
                    out[idx] = Self { x, y, z };
                    idx += 1;
                }
            }
        }
        out
    }

    #[inline]
    pub fn eval_on_aabb(self, aabb: &Aabb3) -> DVec3 {
        DVec3::new(
            self.x.interp(aabb.min.x, aabb.max.x),
            self.y.interp(aabb.min.y, aabb.max.y),
            self.z.interp(aabb.min.z, aabb.max.z),
        )
    }

    pub fn label(self) -> String {
        let z_lbl = match self.z {
            AxisAnchor::Max => "Top",
            AxisAnchor::Center => "Mid-Z",
            AxisAnchor::Min => "Bottom",
        };
        let y_lbl = match self.y {
            AxisAnchor::Min => "Front",
            AxisAnchor::Center => "Center",
            AxisAnchor::Max => "Back",
        };
        let x_lbl = match self.x {
            AxisAnchor::Min => "Left",
            AxisAnchor::Center => "Center",
            AxisAnchor::Max => "Right",
        };
        if self.x == AxisAnchor::Center && self.y == AxisAnchor::Center {
            format!("{z_lbl} Center")
        } else {
            format!("{z_lbl} {y_lbl}-{x_lbl}")
        }
    }
}

/// B-Rep entity supplying a WCS axis direction vector.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum WcsAxisRef {
    /// Uses planar face outward normal or cylinder/cone axis direction.
    FaceAxis(SelectionBinding),
    /// Uses straight line edge direction (`end - start`).
    EdgeDirection(SelectionBinding),
}

impl WcsAxisRef {
    pub fn resolve_world_dir(&self, doc: &CadDocument) -> Result<DVec3, CamError> {
        match self {
            Self::FaceAxis(b) => {
                let face = doc
                    .face(FaceId(b.last_index))
                    .ok_or(CamError::InvalidWcsAxisRef {
                        index: b.last_index,
                    })?;
                match face.surface {
                    SurfaceKind::Plane { normal, .. } => Ok(normal),
                    SurfaceKind::Cylinder { axis_dir, .. }
                    | SurfaceKind::Cone { axis_dir, .. }
                    | SurfaceKind::Torus { axis_dir, .. } => Ok(axis_dir),
                    SurfaceKind::OtherNurbs => Err(CamError::InvalidWcsAxisRef {
                        index: b.last_index,
                    }),
                }
            }
            Self::EdgeDirection(b) => {
                let edge = doc
                    .edge(EdgeId(b.last_index))
                    .ok_or(CamError::InvalidWcsAxisRef {
                        index: b.last_index,
                    })?;
                match edge.curve {
                    CurveKind::Line { start_mm, end_mm } => Ok(end_mm - start_mm),
                    CurveKind::CircleArc { normal, .. } => Ok(normal),
                    CurveKind::EllipseArc | CurveKind::OtherNurbs => {
                        Err(CamError::InvalidWcsAxisRef {
                            index: b.last_index,
                        })
                    }
                }
            }
        }
    }
}

fn is_zero_angle(v: &f64) -> bool {
    v.abs() <= 1e-12
}

fn apply_extra_z_rot(frame: RigidFrame3, extra_z_rot_deg: f64) -> RigidFrame3 {
    if !extra_z_rot_deg.is_finite() || extra_z_rot_deg.abs() <= 1e-12 {
        return frame;
    }
    let (s, c) = extra_z_rot_deg.to_radians().sin_cos();
    let new_x = (frame.x_axis * c + frame.y_axis * s).normalize_or_zero();
    let new_y = frame.z_axis.cross(new_x).normalize_or_zero();
    RigidFrame3 {
        origin_mm: frame.origin_mm,
        x_axis: new_x,
        y_axis: new_y,
        z_axis: frame.z_axis,
    }
}

/// WCS machine axis alignment mode (`T_body_to_wcs` rotation).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum WcsOrientationMode {
    ModelOrientation {
        #[serde(default)]
        flip_z: bool,
        #[serde(default)]
        flip_x: bool,
        #[serde(default)]
        clock_90: u8,
        #[serde(default, skip_serializing_if = "is_zero_angle")]
        extra_z_rot_deg: f64,
    },
    SelectZAndX {
        z_ref: WcsAxisRef,
        #[serde(default)]
        flip_z: bool,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        x_ref: Option<WcsAxisRef>,
        #[serde(default)]
        flip_x: bool,
        #[serde(default)]
        clock_90: u8,
        #[serde(default, skip_serializing_if = "is_zero_angle")]
        extra_z_rot_deg: f64,
    },
    SelectXYPlane {
        face_ref: SelectionBinding,
        #[serde(default)]
        flip_z: bool,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        x_edge_ref: Option<SelectionBinding>,
        #[serde(default)]
        clock_90: u8,
        #[serde(default, skip_serializing_if = "is_zero_angle")]
        extra_z_rot_deg: f64,
    },
}

impl Default for WcsOrientationMode {
    fn default() -> Self {
        Self::ModelOrientation {
            flip_z: false,
            flip_x: false,
            clock_90: 0,
            extra_z_rot_deg: 0.0,
        }
    }
}

impl WcsOrientationMode {
    /// Resolves the pure rotational basis (`origin == ZERO`) in world coordinates.
    pub fn resolve_rotation(&self, doc: &CadDocument) -> Result<RigidFrame3, CamError> {
        match self {
            Self::ModelOrientation {
                flip_z,
                flip_x,
                clock_90,
                extra_z_rot_deg,
            } => Ok(apply_extra_z_rot(
                RigidFrame3::from_z_and_x_hint(
                    DVec3::ZERO,
                    DVec3::Z,
                    Some(DVec3::X),
                    *flip_z,
                    *flip_x,
                    *clock_90,
                )?,
                *extra_z_rot_deg,
            )),
            Self::SelectZAndX {
                z_ref,
                flip_z,
                x_ref,
                flip_x,
                clock_90,
                extra_z_rot_deg,
            } => {
                let raw_z = z_ref.resolve_world_dir(doc)?;
                let raw_x = match x_ref {
                    Some(xr) => Some(xr.resolve_world_dir(doc)?),
                    None => None,
                };
                Ok(apply_extra_z_rot(
                    RigidFrame3::from_z_and_x_hint(
                        DVec3::ZERO,
                        raw_z,
                        raw_x,
                        *flip_z,
                        *flip_x,
                        *clock_90,
                    )?,
                    *extra_z_rot_deg,
                ))
            }
            Self::SelectXYPlane {
                face_ref,
                flip_z,
                x_edge_ref,
                clock_90,
                extra_z_rot_deg,
            } => {
                let face =
                    doc.face(FaceId(face_ref.last_index))
                        .ok_or(CamError::InvalidWcsAxisRef {
                            index: face_ref.last_index,
                        })?;
                let SurfaceKind::Plane { normal, .. } = face.surface else {
                    return Err(CamError::InvalidWcsAxisRef {
                        index: face_ref.last_index,
                    });
                };
                let x_hint = match x_edge_ref {
                    Some(eb) => {
                        let edge =
                            doc.edge(EdgeId(eb.last_index))
                                .ok_or(CamError::InvalidWcsAxisRef {
                                    index: eb.last_index,
                                })?;
                        match edge.curve {
                            CurveKind::Line { start_mm, end_mm } => Some(end_mm - start_mm),
                            _ => {
                                return Err(CamError::InvalidWcsAxisRef {
                                    index: eb.last_index,
                                });
                            }
                        }
                    }
                    None => None,
                };
                Ok(apply_extra_z_rot(
                    RigidFrame3::from_z_and_x_hint(
                        DVec3::ZERO,
                        normal,
                        x_hint,
                        *flip_z,
                        false,
                        *clock_90,
                    )?,
                    *extra_z_rot_deg,
                ))
            }
        }
    }
}

/// Specific B-Rep point/feature anchor for WCS origin (`X0 Y0 Z0`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum WcsFeaturePointRef {
    EdgeStart(SelectionBinding),
    EdgeMidpoint(SelectionBinding),
    EdgeEnd(SelectionBinding),
    CircleCenter(SelectionBinding),
    HoleAxisCenter {
        hole_face: SelectionBinding,
        top_rim: bool,
    },
}

impl WcsFeaturePointRef {
    pub fn resolve_world_point(
        &self,
        doc: &CadDocument,
        spindle_up_world: DVec3,
    ) -> Result<DVec3, CamError> {
        match self {
            Self::EdgeStart(b) | Self::EdgeMidpoint(b) | Self::EdgeEnd(b) => {
                let edge = doc
                    .edge(EdgeId(b.last_index))
                    .ok_or(CamError::InvalidWcsOriginRef {
                        index: b.last_index,
                    })?;
                match edge.curve {
                    CurveKind::Line { start_mm, end_mm } => Ok(match self {
                        Self::EdgeStart(_) => start_mm,
                        Self::EdgeMidpoint(_) => 0.5 * (start_mm + end_mm),
                        Self::EdgeEnd(_) => end_mm,
                        _ => start_mm,
                    }),
                    CurveKind::CircleArc {
                        start_mm,
                        end_mm,
                        center_mm,
                        ..
                    } => Ok(match self {
                        Self::EdgeStart(_) => start_mm,
                        Self::EdgeMidpoint(_) => center_mm,
                        Self::EdgeEnd(_) => end_mm,
                        _ => center_mm,
                    }),
                    _ => Err(CamError::InvalidWcsOriginRef {
                        index: b.last_index,
                    }),
                }
            }
            Self::CircleCenter(b) => {
                let edge = doc
                    .edge(EdgeId(b.last_index))
                    .ok_or(CamError::InvalidWcsOriginRef {
                        index: b.last_index,
                    })?;
                if let CurveKind::CircleArc { center_mm, .. } = edge.curve {
                    Ok(center_mm)
                } else {
                    Err(CamError::InvalidWcsOriginRef {
                        index: b.last_index,
                    })
                }
            }
            Self::HoleAxisCenter { hole_face, top_rim } => {
                let fid = FaceId(hole_face.last_index);
                let face = doc.face(fid).ok_or(CamError::InvalidWcsOriginRef {
                    index: hole_face.last_index,
                })?;
                let SurfaceKind::Cylinder {
                    axis_origin_mm,
                    axis_dir,
                    is_internal: true,
                    ..
                } = face.surface
                else {
                    return Err(CamError::InvalidWcsOriginRef {
                        index: hole_face.last_index,
                    });
                };
                let span = analyze_cylindrical_hole(doc, fid, spindle_up_world).ok_or(
                    CamError::InvalidWcsOriginRef {
                        index: hole_face.last_index,
                    },
                )?;
                let target_proj = if *top_rim {
                    span.top_proj_mm
                } else {
                    span.bottom_proj_mm
                };
                let u = axis_dir.normalize_or_zero();
                let up = spindle_up_world.normalize_or_zero();
                let denom = u.dot(up);
                if denom.abs() < 1e-6 {
                    return Err(CamError::InvalidWcsOriginRef {
                        index: hole_face.last_index,
                    });
                }
                let s = (target_proj - axis_origin_mm.dot(up)) / denom;
                Ok(axis_origin_mm + u * s)
            }
        }
    }
}

/// WCS origin (`X0 Y0 Z0`) anchor mode.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum WcsOriginMode {
    StockBoxPoint { anchor: BoxAnchor27 },
    ModelBoxPoint { anchor: BoxAnchor27 },
    SelectedCadFeature { feature: WcsFeaturePointRef },
}

impl Default for WcsOriginMode {
    fn default() -> Self {
        Self::StockBoxPoint {
            anchor: BoxAnchor27::TOP_FRONT_LEFT,
        }
    }
}

/// Complete Work Coordinate System specification for a [`crate::setup::Setup`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WcsConfig {
    #[serde(default)]
    pub orientation: WcsOrientationMode,
    #[serde(default)]
    pub origin: WcsOriginMode,
    #[serde(default = "default_zero_offsets")]
    pub additional_offset_wcs: [ParamLength; 3],
}

fn default_zero_offsets() -> [ParamLength; 3] {
    [
        ParamLength::from_mm(0.0),
        ParamLength::from_mm(0.0),
        ParamLength::from_mm(0.0),
    ]
}

impl Default for WcsConfig {
    fn default() -> Self {
        Self {
            orientation: WcsOrientationMode::default(),
            origin: WcsOriginMode::default(),
            additional_offset_wcs: default_zero_offsets(),
        }
    }
}

impl WcsConfig {
    pub fn reevaluate_offsets(&mut self, ctx: &EvalContext) {
        for off in &mut self.additional_offset_wcs {
            let _ = off.reevaluate(ctx);
        }
    }

    /// Formats a concise human-readable WCS Zero summary for operator headers (`07-post-processor.md`).
    pub fn describe_origin(&self) -> String {
        let base = match &self.origin {
            WcsOriginMode::StockBoxPoint { anchor } => format!("Stock {}", anchor.label()),
            WcsOriginMode::ModelBoxPoint { anchor } => format!("Model {}", anchor.label()),
            WcsOriginMode::SelectedCadFeature { .. } => "Selected CAD Feature".to_string(),
        };
        let dx = self.additional_offset_wcs[0].value_mm;
        let dy = self.additional_offset_wcs[1].value_mm;
        let dz = self.additional_offset_wcs[2].value_mm;
        if dx.abs() > 1e-4 || dy.abs() > 1e-4 || dz.abs() > 1e-4 {
            format!("{base} (+Offset {dx:+.2}, {dy:+.2}, {dz:+.2} mm)")
        } else {
            base
        }
    }
}

/// Formats a concise human-readable WCS Zero summary for operator headers (`07-post-processor.md`).
pub fn describe_wcs_origin(wcs: &WcsConfig) -> String {
    wcs.describe_origin()
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn wcs_extra_z_rot_zero_is_identity_and_rotates_xy_around_z() {
        let doc = CadDocument::empty();

        let zero = WcsOrientationMode::default();
        let f_zero = zero.resolve_rotation(&doc).unwrap();
        assert_eq!(f_zero, RigidFrame3::IDENTITY);

        let rot45 = WcsOrientationMode::ModelOrientation {
            flip_z: false,
            flip_x: false,
            clock_90: 0,
            extra_z_rot_deg: 45.0,
        };
        let f_45 = rot45.resolve_rotation(&doc).unwrap();
        let s2 = std::f64::consts::FRAC_1_SQRT_2;
        assert!((f_45.x_axis - DVec3::new(s2, s2, 0.0)).length() < 1e-12);
        assert!((f_45.y_axis - DVec3::new(-s2, s2, 0.0)).length() < 1e-12);
        assert!((f_45.z_axis - DVec3::Z).length() < 1e-12);

        // 90° extra_z_rot matches clock_90 = 1
        let rot90 = WcsOrientationMode::ModelOrientation {
            flip_z: false,
            flip_x: false,
            clock_90: 0,
            extra_z_rot_deg: 90.0,
        };
        let clk1 = WcsOrientationMode::ModelOrientation {
            flip_z: false,
            flip_x: false,
            clock_90: 1,
            extra_z_rot_deg: 0.0,
        };
        let f_90 = rot90.resolve_rotation(&doc).unwrap();
        let f_clk = clk1.resolve_rotation(&doc).unwrap();
        assert!((f_90.x_axis - f_clk.x_axis).length() < 1e-12);
        assert!((f_90.y_axis - f_clk.y_axis).length() < 1e-12);
        assert!((f_90.z_axis - f_clk.z_axis).length() < 1e-12);
    }
}
