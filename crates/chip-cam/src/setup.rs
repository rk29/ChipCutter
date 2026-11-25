use chip_cad::{BodyId, CadDocument, FaceId, SurfaceKind};
use chip_math::{Aabb2, Aabb3, DVec2, DVec3, DisplayUnit, EvalContext, RigidFrame3};
use chip_sketch::{
    FaceSketchOriginMode, SketchAnchor, SketchFaceBinding, SketchLayer, SketchPlaneRef,
};
use serde::{Deserialize, Serialize};

use crate::{
    heights::SetupHeights,
    operation::Operation,
    rotary::{ResolvedRotaryConfig, RotaryConfig},
    stock::{ResolvedStockGeometry, StockConfig, compute_tight_oriented_model_aabb},
    wcs::{CamError, WcsConfig, WcsOriginMode, WorkOffsetSlot},
    workholding::FixtureBody,
};

fn default_post_profile() -> String {
    "grbl".to_string()
}

fn default_true() -> bool {
    true
}

fn is_true(v: &bool) -> bool {
    *v
}

/// Single physical CNC clamping setup (`03-setups-wcs.md`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Setup {
    pub id: String,
    pub name: String,
    #[serde(default)]
    pub target_bodies: Vec<BodyId>,
    #[serde(default)]
    pub wcs_slot: WorkOffsetSlot,
    #[serde(default = "default_post_profile")]
    pub post_profile: String,
    #[serde(default)]
    pub wcs: WcsConfig,
    #[serde(default)]
    pub stock: StockConfig,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub carryover_from_setup_id: Option<String>,
    #[serde(default, skip_serializing_if = "RotaryConfig::is_disabled")]
    pub rotary: RotaryConfig,
    #[serde(default)]
    pub heights: SetupHeights,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub fixtures: Vec<FixtureBody>,
    #[serde(default = "default_true", skip_serializing_if = "is_true")]
    pub auto_avoid_fixtures_on_rapids: bool,
    #[serde(default)]
    pub operations: Vec<Operation>,
}

impl Setup {
    pub fn new(id: impl Into<String>, name: impl Into<String>, wcs_slot: WorkOffsetSlot) -> Self {
        Self {
            id: id.into(),
            name: name.into(),
            target_bodies: Vec::new(),
            wcs_slot,
            post_profile: default_post_profile(),
            wcs: WcsConfig::default(),
            stock: StockConfig::default(),
            carryover_from_setup_id: None,
            rotary: RotaryConfig::default(),
            heights: SetupHeights::default(),
            fixtures: Vec::new(),
            auto_avoid_fixtures_on_rapids: true,
            operations: Vec::new(),
        }
    }

    /// Resolves this Setup's orthonormal WCS [`RigidFrame3`], tight oriented model AABB,
    /// concrete [`ResolvedStockGeometry`], and Setup Z planes against `doc`.
    pub fn resolve(
        &mut self,
        doc: &CadDocument,
        display_unit: DisplayUnit,
    ) -> Result<ResolvedSetup, CamError> {
        self.resolve_immutable(doc, display_unit)
    }

    pub fn resolve_immutable(
        &self,
        doc: &CadDocument,
        display_unit: DisplayUnit,
    ) -> Result<ResolvedSetup, CamError> {
        self.resolve_with_sketches(doc, &[], display_unit)
    }

    /// Resolves this Setup against either a 3D [`CadDocument`] OR, when `doc.bodies.is_empty()`,
    /// a Pure-2D virtual sheet bounded by `sketches` and `self.stock`'s sheet Z thickness.
    pub fn resolve_with_sketches(
        &self,
        doc: &CadDocument,
        sketches: &[SketchLayer],
        display_unit: DisplayUnit,
    ) -> Result<ResolvedSetup, CamError> {
        // 1. Orthonormal rotation frame at world origin (0, 0, 0)
        let rot_frame = match self.wcs.orientation.resolve_rotation(doc) {
            Ok(f) => f,
            Err(_) if !doc.has_3d_geometry() => return Err(CamError::Pure2dRequiresBoxWcs),
            Err(e) => return Err(e),
        };

        // 2–3. Resolve model bounds & oriented stock bounds in rotated axes.
        // In Pure-2D mode (`!doc.has_3d_geometry()`) with `StockConfig::FixedBox`, anchor the fixed
        // stock directly to the Setup WCS origin rather than the sketch entities' AABB so:
        //   1. Moving a hole or sketch entity never drags the stock, WCS, or fixtures around, and
        //   2. 2D sketch `(X, Y)` maps 1:1 to Setup WCS `(X, Y)` (matching Hybrid 3D mode).
        let is_pure_2d_fixed_box =
            !doc.has_3d_geometry() && matches!(self.stock, StockConfig::FixedBox { .. });
        let (model_rot_aabb, stock_rot) = if is_pure_2d_fixed_box {
            let mut stock_clone = self.stock.clone();
            let base_ctx = EvalContext::new(display_unit);
            stock_clone.reevaluate(&base_ctx);
            let (wx, dy, hz) = match &stock_clone {
                StockConfig::FixedBox {
                    width_x,
                    depth_y,
                    height_z,
                    ..
                } => (width_x.value_mm, depth_y.value_mm, height_z.value_mm),
                _ => (0.0, 0.0, 0.0),
            };
            for (name, v) in [("width_x", wx), ("depth_y", dy), ("height_z", hz)] {
                if !v.is_finite() || v <= 0.0 {
                    return Err(CamError::InvalidStockDimension {
                        field: name,
                        value_mm: v,
                    });
                }
            }
            let raw_aabb = Aabb3::new(DVec3::new(0.0, 0.0, -hz), DVec3::new(wx, dy, 0.0));
            let anchor_raw = match &self.wcs.origin {
                WcsOriginMode::StockBoxPoint { anchor }
                | WcsOriginMode::ModelBoxPoint { anchor } => anchor.eval_on_aabb(&raw_aabb),
                WcsOriginMode::SelectedCadFeature { .. } => {
                    return Err(CamError::Pure2dRequiresBoxWcs);
                }
            };
            let stock_aabb_rot = Aabb3::new(raw_aabb.min - anchor_raw, raw_aabb.max - anchor_raw);
            (
                stock_aabb_rot,
                ResolvedStockGeometry::Box {
                    aabb_wcs: stock_aabb_rot,
                },
            )
        } else {
            let model_rot_aabb = if doc.has_3d_geometry() {
                compute_tight_oriented_model_aabb(doc, &self.target_bodies, &rot_frame)?
            } else {
                compute_pure_2d_sheet_rot_aabb(
                    &self.id,
                    sketches,
                    self.stock.pure_2d_sheet_thickness_mm(display_unit),
                    &rot_frame,
                )
            };

            let mut stock_clone = self.stock.clone();
            let base_ctx = EvalContext::new(display_unit)
                .with_var("model_top_z", model_rot_aabb.max.z)
                .with_var("model_bottom_z", model_rot_aabb.min.z);
            stock_clone.reevaluate(&base_ctx);
            let stock_rot = stock_clone.compute_oriented_bounds(&model_rot_aabb)?;
            (model_rot_aabb, stock_rot)
        };
        let stock_rot_aabb = stock_rot.aabb_wcs();

        // 4. Compute anchor point in rotated coordinates (before additional offset)
        let base_anchor_rot = match &self.wcs.origin {
            WcsOriginMode::StockBoxPoint { anchor } => anchor.eval_on_aabb(&stock_rot_aabb),
            WcsOriginMode::ModelBoxPoint { anchor } => anchor.eval_on_aabb(&model_rot_aabb),
            WcsOriginMode::SelectedCadFeature { feature } => {
                if doc.bodies.is_empty() {
                    return Err(CamError::Pure2dRequiresBoxWcs);
                }
                let pt_world = feature.resolve_world_point(doc, rot_frame.z_axis)?;
                rot_frame.world_to_local_point(pt_world)
            }
        };

        let mut wcs_clone = self.wcs.clone();
        let stock_sz = stock_rot_aabb.size();
        let wcs_ctx = EvalContext::new(display_unit)
            .with_var("stock_x", stock_sz.x)
            .with_var("stock_y", stock_sz.y)
            .with_var("stock_z", stock_sz.z);
        wcs_clone.reevaluate_offsets(&wcs_ctx);

        let extra_rot = DVec3::new(
            wcs_clone.additional_offset_wcs[0].value_mm,
            wcs_clone.additional_offset_wcs[1].value_mm,
            wcs_clone.additional_offset_wcs[2].value_mm,
        );

        let origin_rot = base_anchor_rot + extra_rot;
        let origin_world = rot_frame.local_to_world_point(origin_rot);
        let frame = rot_frame.with_origin(origin_world);

        // Shift rotated model & stock bounds by `-origin_rot` so `(0, 0, 0)` is the WCS origin!
        let shift_wcs = -origin_rot;
        let model_aabb_wcs = Aabb3::new(
            model_rot_aabb.min + shift_wcs,
            model_rot_aabb.max + shift_wcs,
        );
        let stock = stock_rot.shifted_by(shift_wcs);
        let stock_aabb_wcs = stock.aabb_wcs();

        // 5. Resolve Setup Clearance & Retract heights and optional 4th-axis rotary kinematics
        let mut heights_clone = self.heights.clone();
        let setup_eval = build_setup_eval_context(display_unit, &model_aabb_wcs, &stock_aabb_wcs);
        heights_clone.reevaluate(&setup_eval);
        let (clearance_z_wcs, retract_z_wcs) =
            heights_clone.resolve(&model_aabb_wcs, &stock_aabb_wcs)?;
        let rotary = self.rotary.resolve(&stock, &setup_eval);

        Ok(ResolvedSetup {
            frame,
            model_aabb_wcs,
            stock,
            rotary,
            clearance_z_wcs,
            retract_z_wcs,
        })
    }
}

/// Concrete evaluated Setup WCS frame, bounds, stock geometry, optional rotary axis, and Z planes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ResolvedSetup {
    pub frame: RigidFrame3,
    pub model_aabb_wcs: Aabb3,
    pub stock: ResolvedStockGeometry,
    pub rotary: Option<ResolvedRotaryConfig>,
    pub clearance_z_wcs: f64,
    pub retract_z_wcs: f64,
}

impl ResolvedSetup {
    /// Builds an [`EvalContext`] populated with Setup-scoped variables (`stock_x`, `stock_y`,
    /// `stock_z`, `model_top_z`, `model_bottom_z`, plus `rotary_radius` when rotary is active).
    pub fn eval_context(&self, display_unit: DisplayUnit) -> EvalContext {
        let mut ctx =
            build_setup_eval_context(display_unit, &self.model_aabb_wcs, &self.stock.aabb_wcs());
        if let Some(rot) = self.rotary {
            ctx.set_var("rotary_radius", rot.stock_outer_radius_mm);
            ctx.set_var("rotary_clearance_r", rot.clearance_radius_mm);
        }
        ctx
    }
}

fn build_setup_eval_context(
    display_unit: DisplayUnit,
    model_aabb_wcs: &Aabb3,
    stock_aabb_wcs: &Aabb3,
) -> EvalContext {
    let sz = stock_aabb_wcs.size();
    EvalContext::new(display_unit)
        .with_var("stock_x", sz.x)
        .with_var("stock_y", sz.y)
        .with_var("stock_z", sz.z)
        .with_var("model_top_z", model_aabb_wcs.max.z)
        .with_var("model_bottom_z", model_aabb_wcs.min.z)
}

fn compute_pure_2d_sheet_rot_aabb(
    setup_id: &str,
    sketches: &[SketchLayer],
    sheet_thickness_mm: f64,
    rot_frame: &RigidFrame3,
) -> Aabb3 {
    let mut xy = Aabb2::EMPTY;
    for sk in sketches {
        if let SketchAnchor::SetupXy {
            setup_id: sk_setup_id,
            ..
        } = &sk.anchor
            && sk.visible
            && (sk_setup_id == setup_id || sk_setup_id.is_empty())
        {
            let b = sk.exact_aabb();
            if b.is_valid() {
                xy.extend_point(b.min);
                xy.extend_point(b.max);
            }
        }
    }
    if !xy.is_valid() {
        for sk in sketches {
            if sk.visible {
                let b = sk.exact_aabb();
                if b.is_valid() {
                    xy.extend_point(b.min);
                    xy.extend_point(b.max);
                }
            }
        }
    }
    if !xy.is_valid() {
        xy = Aabb2 {
            min: DVec2::ZERO,
            max: DVec2::new(100.0, 100.0),
        };
    }
    let thick = sheet_thickness_mm.max(0.1);
    let mut aabb = Aabb3::EMPTY;
    for z in [-thick, 0.0] {
        for p2 in [
            xy.min,
            DVec2::new(xy.max.x, xy.min.y),
            xy.max,
            DVec2::new(xy.min.x, xy.max.y),
        ] {
            aabb.extend_point(rot_frame.world_to_local_point(DVec3::new(p2.x, p2.y, z)));
        }
    }
    aabb
}

/// Resolves the canonical 3D World [`RigidFrame3`] of a [`SketchLayer`] inside `setup`,
/// querying `doc` directly when anchored to a 3D planar B-Rep face ([`SketchAnchor::PlanarFace`]).
pub fn resolve_sketch_world_frame_with_doc(
    doc: &CadDocument,
    sketch: &SketchLayer,
    setup: &ResolvedSetup,
) -> RigidFrame3 {
    resolve_sketch_world_frame_inner(doc.has_3d_geometry(), Some(doc), sketch, setup)
}

/// Resolves the canonical 3D World [`RigidFrame3`] of a [`SketchLayer`] inside `setup`!
///
/// - Mapping `(x_sketch, y_sketch, 0.0)` via `frame.local_to_world_point(...)` yields the 3D World
///   point for viewport rendering and ray-plane hit testing.
/// - Mapping that 3D World point via `setup.frame.world_to_local_point(...)` yields the exact
///   Setup WCS `(x_wcs, y_wcs, z_wcs)` coordinate for CAM toolpaths!
pub fn resolve_sketch_world_frame(
    doc_has_bodies: bool,
    sketch: &SketchLayer,
    setup: &ResolvedSetup,
) -> RigidFrame3 {
    resolve_sketch_world_frame_inner(doc_has_bodies, None, sketch, setup)
}

fn resolve_sketch_world_frame_inner(
    doc_has_bodies: bool,
    doc: Option<&CadDocument>,
    sketch: &SketchLayer,
    setup: &ResolvedSetup,
) -> RigidFrame3 {
    match &sketch.anchor {
        SketchAnchor::SetupXy {
            plane_ref,
            z_offset,
            ..
        } => {
            let stock_aabb = setup.stock.aabb_wcs();
            let base_z_wcs = match plane_ref {
                SketchPlaneRef::StockTop => stock_aabb.max.z,
                SketchPlaneRef::ModelTop => setup.model_aabb_wcs.max.z,
                SketchPlaneRef::StockBottom => stock_aabb.min.z,
                SketchPlaneRef::WcsOrigin => 0.0,
            };
            let z_wcs = base_z_wcs + z_offset.value_mm;

            if !doc_has_bodies {
                // Pure-2D mode: `sketch` XY coordinates define the virtual sheet in rotated world XY!
                let rot_only = setup.frame.with_origin(DVec3::ZERO);
                let origin_rot = rot_only.world_to_local_point(setup.frame.origin_mm);
                let world_origin = setup.frame.local_to_world_point(DVec3::new(
                    -origin_rot.x,
                    -origin_rot.y,
                    z_wcs,
                ));
                setup.frame.with_origin(world_origin)
            } else {
                // Hybrid 3D STEP + SetupXY mode: `sketch` XY coordinates are anchored directly at Setup WCS `(0, 0)`!
                let world_origin = setup
                    .frame
                    .local_to_world_point(DVec3::new(0.0, 0.0, z_wcs));
                setup.frame.with_origin(world_origin)
            }
        }
        SketchAnchor::PlanarFace {
            face_binding,
            origin_mode,
            x_axis_clock_deg,
            z_offset,
            ..
        } => resolve_planar_face_sketch_frame(
            doc,
            face_binding,
            *origin_mode,
            *x_axis_clock_deg,
            z_offset.value_mm,
            setup,
        ),
    }
}

fn resolve_planar_face_sketch_frame(
    doc: Option<&CadDocument>,
    binding: &SketchFaceBinding,
    origin_mode: FaceSketchOriginMode,
    x_axis_clock_deg: f64,
    z_offset_mm: f64,
    setup: &ResolvedSetup,
) -> RigidFrame3 {
    let mut normal_world = binding.normal_world;
    let mut centroid_world = binding.centroid_world_mm;

    if let Some(cad_doc) = doc
        && let Some(face) = cad_doc.face(FaceId(binding.last_face_index))
        && let SurfaceKind::Plane { normal, .. } = face.surface
        && normal.dot(binding.normal_world.normalize_or_zero()) > 0.90
    {
        normal_world = normal;
        if let Some(live_bind) = cad_doc.bind_face(face.id) {
            centroid_world = live_bind.signature.centroid_mm;
        } else if face.aabb_mm.is_valid() {
            centroid_world = face.aabb_mm.center();
        }
    }

    let z_axis = normal_world.normalize_or_zero();
    let z_axis = if z_axis.length_squared() > 0.5 {
        z_axis
    } else {
        setup.frame.z_axis
    };

    // Choose deterministic in-plane reference `+X` axis: align with `setup.frame.x_axis` when
    // parallel to Setup XY, otherwise project world `+X` (or `+Y` when normal ≈ `±X`).
    let base_cand = if z_axis.dot(setup.frame.z_axis).abs() >= 0.999 {
        setup.frame.x_axis
    } else if z_axis.dot(DVec3::X).abs() < 0.9 {
        DVec3::X
    } else {
        DVec3::Y
    };
    let mut ref_x = (base_cand - z_axis * base_cand.dot(z_axis)).normalize_or_zero();
    if ref_x.length_squared() < 0.5 {
        let alt = if z_axis.dot(DVec3::Y).abs() < 0.9 {
            DVec3::Y
        } else {
            DVec3::Z
        };
        ref_x = (alt - z_axis * alt.dot(z_axis)).normalize_or_zero();
    }
    let ref_y = z_axis.cross(ref_x).normalize_or_zero();
    let (sin_c, cos_c) = x_axis_clock_deg.to_radians().sin_cos();
    let x_axis = (ref_x * cos_c + ref_y * sin_c).normalize_or_zero();
    let y_axis = z_axis.cross(x_axis).normalize_or_zero();

    let base_origin = match origin_mode {
        FaceSketchOriginMode::FaceCentroid => centroid_world,
        FaceSketchOriginMode::SetupWcsProjected => {
            setup.frame.origin_mm + z_axis * (centroid_world - setup.frame.origin_mm).dot(z_axis)
        }
    };
    let origin_mm = base_origin + z_axis * z_offset_mm;

    let _ = y_axis;
    RigidFrame3::from_z_and_x_hint(origin_mm, z_axis, Some(x_axis), false, false, 0)
        .unwrap_or_else(|_| setup.frame.with_origin(origin_mm))
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::{AxisAnchor, BoxAnchor27, FixedBoxXyAlign, FixedBoxZAlign};
    use chip_math::ParamLength;
    use chip_sketch::{SketchEntity, SketchEntityEntry};

    #[test]
    fn pure_2d_fixed_box_keeps_stock_stationary_and_maps_sketch_1_to_1_with_wcs() {
        let doc = CadDocument::empty();
        let mut setup = Setup::new("setup_1", "Bar Setup", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27 {
                x: AxisAnchor::Center,
                y: AxisAnchor::Max,
                z: AxisAnchor::Max,
            },
        };
        setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(20.0),
            depth_y: ParamLength::from_mm(120.0),
            height_z: ParamLength::from_mm(20.0),
            xy_align: FixedBoxXyAlign::OffsetFromFrontLeft {
                dx: ParamLength::from_mm(0.0),
                dy: ParamLength::from_mm(5.0),
            },
            z_align: FixedBoxZAlign::OffsetFromTop {
                top_allowance: ParamLength::from_mm(0.0),
            },
        };

        let mut sk = SketchLayer::new("sketch_1", "Through Holes", "setup_1");
        sk.entities.push(SketchEntityEntry {
            id: "hole_5mm".to_string(),
            name: "5mm Hole".to_string(),
            sub_layer: None,
            is_construction: false,
            entity: SketchEntity::new_circle(
                ParamLength::from_mm(0.0),
                ParamLength::from_mm(-110.0),
                ParamLength::from_mm(5.0),
            ),
        });
        sk.rebuild_regions(None);

        let rs1 = setup
            .resolve_with_sketches(&doc, std::slice::from_ref(&sk), DisplayUnit::Millimeter)
            .expect("resolve Pure-2D FixedBox");
        let aabb1 = rs1.stock.aabb_wcs();
        assert!((aabb1.min.x - (-10.0)).abs() < 1e-6);
        assert!((aabb1.max.x - 10.0).abs() < 1e-6);
        assert!((aabb1.min.y - (-120.0)).abs() < 1e-6);
        assert!((aabb1.max.y - 0.0).abs() < 1e-6);
        assert!((aabb1.min.z - (-20.0)).abs() < 1e-6);
        assert!((aabb1.max.z - 0.0).abs() < 1e-6);

        let sk_frame1 = resolve_sketch_world_frame(false, &sk, &rs1);
        let hole_wcs1 = rs1
            .frame
            .world_to_local_point(sk_frame1.local_to_world_point(DVec3::new(0.0, -110.0, 0.0)));
        assert!(
            (hole_wcs1.x - 0.0).abs() < 1e-6 && (hole_wcs1.y - (-110.0)).abs() < 1e-6,
            "hole at sketch (0, -110) must sit on WCS centerline (0, -110), got {hole_wcs1:?}"
        );

        // Move hole in sketch -> stock & WCS frame must stay completely stationary!
        sk.entities[0].entity = SketchEntity::new_circle(
            ParamLength::from_mm(4.5),
            ParamLength::from_mm(-60.0),
            ParamLength::from_mm(5.0),
        );
        sk.rebuild_regions(None);

        let rs2 = setup
            .resolve_with_sketches(&doc, std::slice::from_ref(&sk), DisplayUnit::Millimeter)
            .expect("resolve after moving hole");
        assert_eq!(rs1.frame.origin_mm, rs2.frame.origin_mm);
        assert_eq!(rs1.stock.aabb_wcs(), rs2.stock.aabb_wcs());

        let sk_frame2 = resolve_sketch_world_frame(false, &sk, &rs2);
        let hole_wcs2 = rs2
            .frame
            .world_to_local_point(sk_frame2.local_to_world_point(DVec3::new(4.5, -60.0, 0.0)));
        assert!(
            (hole_wcs2.x - 4.5).abs() < 1e-6 && (hole_wcs2.y - (-60.0)).abs() < 1e-6,
            "moved hole must land at exact WCS (4.5, -60), got {hole_wcs2:?}"
        );
    }
}
