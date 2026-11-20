use chip_cad::{BodyId, CadDocument, FaceId, SurfaceKind};
use chip_math::{Aabb2, Aabb3, DVec2, DVec3, DisplayUnit, EvalContext, RigidFrame3};
use chip_sketch::{
    FaceSketchOriginMode, SketchAnchor, SketchFaceBinding, SketchLayer, SketchPlaneRef,
};
use serde::{Deserialize, Serialize};

use crate::{
    heights::SetupHeights,
    operation::Operation,
    stock::{ResolvedStockGeometry, StockConfig, compute_tight_oriented_model_aabb},
    wcs::{CamError, WcsConfig, WcsOriginMode, WorkOffsetSlot},
};

fn default_post_profile() -> String {
    "grbl".to_string()
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
    #[serde(default)]
    pub heights: SetupHeights,
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
            heights: SetupHeights::default(),
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

        // 2. Tight bounding box of target B-Rep/mesh body/bodies OR Pure-2D virtual sheet in rotated axes
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

        // 3. Re-evaluate stock expressions with model dimensions
        let mut stock_clone = self.stock.clone();
        let base_ctx = EvalContext::new(display_unit)
            .with_var("model_top_z", model_rot_aabb.max.z)
            .with_var("model_bottom_z", model_rot_aabb.min.z);
        stock_clone.reevaluate(&base_ctx);
        let stock_rot = stock_clone.compute_oriented_bounds(&model_rot_aabb)?;
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

        // 5. Resolve Setup Clearance & Retract heights
        let mut heights_clone = self.heights.clone();
        let setup_eval = build_setup_eval_context(display_unit, &model_aabb_wcs, &stock_aabb_wcs);
        heights_clone.reevaluate(&setup_eval);
        let (clearance_z_wcs, retract_z_wcs) =
            heights_clone.resolve(&model_aabb_wcs, &stock_aabb_wcs)?;

        Ok(ResolvedSetup {
            frame,
            model_aabb_wcs,
            stock,
            clearance_z_wcs,
            retract_z_wcs,
        })
    }
}

/// Concrete evaluated Setup WCS frame, bounds, stock geometry, and Z planes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ResolvedSetup {
    pub frame: RigidFrame3,
    pub model_aabb_wcs: Aabb3,
    pub stock: ResolvedStockGeometry,
    pub clearance_z_wcs: f64,
    pub retract_z_wcs: f64,
}

impl ResolvedSetup {
    /// Builds an [`EvalContext`] populated with Setup-scoped variables (`stock_x`, `stock_y`,
    /// `stock_z`, `model_top_z`, `model_bottom_z`).
    pub fn eval_context(&self, display_unit: DisplayUnit) -> EvalContext {
        build_setup_eval_context(display_unit, &self.model_aabb_wcs, &self.stock.aabb_wcs())
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
