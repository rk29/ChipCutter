use std::collections::HashMap;

use chip_math::{Aabb3, DMat3, DQuat, DVec3};
use serde::{Deserialize, Serialize};

use crate::{
    kernel::CadError,
    mesh::{MeshTriangle, TriangleMeshBody, build_render_mesh_for_triangle_mesh},
    model::{
        BRepEdge, BRepFace, BRepLoop, CadDocument, CurveKind, EdgeId, EdgePolyline,
        EntitySignatureKind, FaceId, FaceTriangle, GeometricSignature, MESH_FACE_SENTINEL,
        RenderMesh, RenderVertex, SolidBody, SurfaceKind,
    },
};

/// Minimum allowed per-axis scale factor (`1e-4`).
pub const MIN_MODEL_SCALE: f64 = 1e-4;
/// Maximum allowed per-axis scale factor (`1e4`).
pub const MAX_MODEL_SCALE: f64 = 1e4;

fn default_unit_scale() -> [f64; 3] {
    [1.0, 1.0, 1.0]
}

fn is_zero_vec3(v: &[f64; 3]) -> bool {
    v.iter().all(|x| x.abs() <= 1e-9)
}

fn is_unit_scale(v: &[f64; 3]) -> bool {
    v.iter().all(|x| (*x - 1.0).abs() <= 1e-9)
}

/// Non-destructive per-model affine transform (`Scale -> Rotate -> Translate`) stored on
/// `ModelEntry` in `.chip.toml` (`p' = Rz(rz) * Ry(ry) * Rx(rx) * (S * p) + t`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ModelTransform {
    #[serde(default, skip_serializing_if = "is_zero_vec3")]
    pub translate_mm: [f64; 3],
    #[serde(default, skip_serializing_if = "is_zero_vec3")]
    pub rotate_deg: [f64; 3],
    #[serde(default = "default_unit_scale", skip_serializing_if = "is_unit_scale")]
    pub scale: [f64; 3],
}

impl Default for ModelTransform {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl ModelTransform {
    pub const IDENTITY: Self = Self {
        translate_mm: [0.0, 0.0, 0.0],
        rotate_deg: [0.0, 0.0, 0.0],
        scale: [1.0, 1.0, 1.0],
    };

    /// Returns `true` when translation is zero, rotation is zero, and scale is `[1, 1, 1]`.
    pub fn is_identity(&self) -> bool {
        is_zero_vec3(&self.translate_mm)
            && is_zero_vec3(&self.rotate_deg)
            && is_unit_scale(&self.scale)
    }

    /// Validates that all transform components are finite and scale factors lie within
    /// `[MIN_MODEL_SCALE, MAX_MODEL_SCALE]`.
    pub fn validate(&self) -> Result<(), CadError> {
        for v in self
            .translate_mm
            .iter()
            .chain(self.rotate_deg.iter())
            .chain(self.scale.iter())
        {
            if !v.is_finite() {
                return Err(CadError::InvalidTransform(
                    "transform contains non-finite (NaN or Inf) component".to_string(),
                ));
            }
        }
        for (axis, &s) in ['X', 'Y', 'Z'].iter().zip(self.scale.iter()) {
            if !(MIN_MODEL_SCALE..=MAX_MODEL_SCALE).contains(&s) {
                return Err(CadError::InvalidTransform(format!(
                    "scale {axis} ({s}) must lie within [{MIN_MODEL_SCALE}, {MAX_MODEL_SCALE}]"
                )));
            }
        }
        Ok(())
    }

    /// Returns `Some(s)` when all three scale axes match within `1e-6` relative tolerance.
    pub fn uniform_scale(&self) -> Option<f64> {
        let [sx, sy, sz] = self.scale;
        let tol = 1e-6 * sx.abs().max(1.0);
        if (sx - sy).abs() <= tol && (sx - sz).abs() <= tol {
            Some((sx + sy + sz) / 3.0)
        } else {
            None
        }
    }

    /// Canonical rotation matrix `R = Rz(rz) * Ry(ry) * Rx(rx)`.
    pub fn rotation_matrix(&self) -> DMat3 {
        let rx = self.rotate_deg[0].to_radians();
        let ry = self.rotate_deg[1].to_radians();
        let rz = self.rotate_deg[2].to_radians();
        DMat3::from_rotation_z(rz) * DMat3::from_rotation_y(ry) * DMat3::from_rotation_x(rx)
    }

    /// Linear part `L = R * diag(sx, sy, sz)`.
    pub fn linear_matrix(&self) -> DMat3 {
        let r = self.rotation_matrix();
        let s = DMat3::from_diagonal(DVec3::from_array(self.scale));
        r * s
    }

    /// Inverse-transpose linear matrix `(R * S)^(-T) = R * diag(1/sx, 1/sy, 1/sz)` for normals.
    pub fn normal_matrix(&self) -> DMat3 {
        let r = self.rotation_matrix();
        let inv_s = DMat3::from_diagonal(DVec3::new(
            1.0 / self.scale[0].max(MIN_MODEL_SCALE),
            1.0 / self.scale[1].max(MIN_MODEL_SCALE),
            1.0 / self.scale[2].max(MIN_MODEL_SCALE),
        ));
        r * inv_s
    }

    /// Applies `p' = R * (S * p) + t` to a 3D point in canonical millimeters.
    #[inline]
    pub fn transform_point(&self, p: DVec3) -> DVec3 {
        let scaled = DVec3::new(
            p.x * self.scale[0],
            p.y * self.scale[1],
            p.z * self.scale[2],
        );
        self.rotation_matrix() * scaled + DVec3::from_array(self.translate_mm)
    }

    /// Transforms a surface/plane normal via normalized `R * S^-1 * n`.
    #[inline]
    pub fn transform_normal(&self, n: DVec3) -> DVec3 {
        let v = self.normal_matrix() * n;
        if v.length_squared() > 1e-18 {
            v.normalize()
        } else {
            DVec3::Z
        }
    }

    /// Transforms a direction / tangent vector via normalized `R * S * d`.
    #[inline]
    pub fn transform_dir_unit(&self, d: DVec3) -> DVec3 {
        let v = self.linear_matrix() * d;
        if v.length_squared() > 1e-18 {
            v.normalize()
        } else {
            DVec3::X
        }
    }

    /// Standard Blender/glTF/OBJ `Y-Up -> Z-Up` preset (`+90°` around `+X`).
    pub const fn y_up_to_z_up() -> Self {
        Self {
            translate_mm: [0.0, 0.0, 0.0],
            rotate_deg: [90.0, 0.0, 0.0],
            scale: [1.0, 1.0, 1.0],
        }
    }

    /// Composes an import transform from rotation, uniform scale, optional `Y-Up -> Z-Up` (`+90° X`),
    /// optional XY centering on `(0, 0)`, and optional drop-to-`Z = 0` against `vertices_mm`,
    /// returning `(transform, transformed_aabb)` for both CLI `import-mesh` and GUI preview modal.
    pub fn compose_for_mesh_import(
        vertices_mm: &[DVec3],
        rotate_deg: [f64; 3],
        uniform_scale: f64,
        y_up_to_z_up: bool,
        center_xy: bool,
        drop_z0: bool,
    ) -> (Self, Aabb3) {
        let mut xf = Self {
            translate_mm: [0.0; 3],
            rotate_deg: [
                rotate_deg[0] + if y_up_to_z_up { 90.0 } else { 0.0 },
                rotate_deg[1],
                rotate_deg[2],
            ],
            scale: [uniform_scale; 3],
        };
        let mut aabb = Aabb3::EMPTY;
        for v in vertices_mm {
            aabb.extend_point(xf.transform_point(*v));
        }
        if aabb.is_valid() {
            if center_xy {
                xf = xf.centered_xy_for_aabb(aabb);
                let shift = DVec3::new(
                    -0.5 * (aabb.min.x + aabb.max.x),
                    -0.5 * (aabb.min.y + aabb.max.y),
                    0.0,
                );
                aabb = Aabb3::new(aabb.min + shift, aabb.max + shift);
            }
            if drop_z0 {
                xf = xf.dropped_bottom_to_z0_for_aabb(aabb);
                let shift = DVec3::new(0.0, 0.0, -aabb.min.z);
                aabb = Aabb3::new(aabb.min + shift, aabb.max + shift);
            }
        }
        (xf, aabb)
    }

    /// Composes a world-space rotation `delta_rot` around `pivot_mm` onto `self`:
    /// `R' = delta_rot * R`, `S' = S`, `t' = delta_rot * (t - pivot) + pivot`.
    pub fn rotated_world(&self, delta_rot: DMat3, pivot_mm: DVec3) -> Self {
        let new_r = delta_rot * self.rotation_matrix();
        let t = DVec3::from_array(self.translate_mm);
        let new_t = delta_rot * (t - pivot_mm) + pivot_mm;
        Self {
            translate_mm: clean_vec3_array(new_t),
            rotate_deg: rotation_matrix_to_xyz_euler_deg(new_r),
            scale: self.scale,
        }
    }

    /// Rotates around a world axis (`DVec3::X`, `Y`, `Z`, or arbitrary unit axis) by `angle_deg`
    /// about `pivot_mm`.
    pub fn rotated_world_axis_deg(&self, axis: DVec3, angle_deg: f64, pivot_mm: DVec3) -> Self {
        let unit = if axis.length_squared() > 1e-12 {
            axis.normalize()
        } else {
            DVec3::Z
        };
        let delta = DMat3::from_axis_angle(unit, angle_deg.to_radians());
        self.rotated_world(delta, pivot_mm)
    }

    /// Rotates the model about `pivot_mm` by the shortest arc mapping `world_face_normal` to `-Z`
    /// so the clicked face sits flat on the CNC table/spoilboard.
    pub fn laid_flat_normal_to_neg_z(&self, world_face_normal: DVec3, pivot_mm: DVec3) -> Self {
        let n = if world_face_normal.length_squared() > 1e-12 {
            world_face_normal.normalize()
        } else {
            return *self;
        };
        let target = DVec3::NEG_Z;
        let dot = n.dot(target).clamp(-1.0, 1.0);
        if dot > 1.0 - 1e-9 {
            return *self;
        }
        let delta = if dot < -1.0 + 1e-9 {
            // Antiparallel (+Z -> -Z): rotate 180° around +X
            DMat3::from_rotation_x(std::f64::consts::PI)
        } else {
            let q = DQuat::from_rotation_arc(n, target);
            DMat3::from_quat(q)
        };
        self.rotated_world(delta, pivot_mm)
    }

    /// Sets a uniform scale factor across X, Y, and Z (clamped to valid range).
    pub fn with_uniform_scale(&self, factor: f64) -> Self {
        let f = factor.clamp(MIN_MODEL_SCALE, MAX_MODEL_SCALE);
        Self {
            scale: [f, f, f],
            ..*self
        }
    }

    /// Multiplies the current scale uniformly so that `axis` (`0=X, 1=Y, 2=Z`) of `current_aabb`
    /// matches `target_mm`.
    pub fn scaled_uniformly_to_fit_axis(
        &self,
        current_aabb: Aabb3,
        axis: usize,
        target_mm: f64,
    ) -> Self {
        if !current_aabb.is_valid() || !target_mm.is_finite() || target_mm <= 1e-4 {
            return *self;
        }
        let sz = current_aabb.size();
        let cur = match axis {
            0 => sz.x,
            1 => sz.y,
            _ => sz.z,
        };
        if cur <= 1e-6 {
            return *self;
        }
        let ratio = target_mm / cur;
        let new_scale = [
            (self.scale[0] * ratio).clamp(MIN_MODEL_SCALE, MAX_MODEL_SCALE),
            (self.scale[1] * ratio).clamp(MIN_MODEL_SCALE, MAX_MODEL_SCALE),
            (self.scale[2] * ratio).clamp(MIN_MODEL_SCALE, MAX_MODEL_SCALE),
        ];
        Self {
            scale: new_scale,
            ..*self
        }
    }

    /// Translates in XY so `transformed_aabb`'s center lies at `(0, 0)`.
    pub fn centered_xy_for_aabb(&self, transformed_aabb: Aabb3) -> Self {
        if !transformed_aabb.is_valid() {
            return *self;
        }
        let c = transformed_aabb.center();
        let mut out = *self;
        out.translate_mm[0] = clean_scalar(out.translate_mm[0] - c.x);
        out.translate_mm[1] = clean_scalar(out.translate_mm[1] - c.y);
        out
    }

    /// Translates in Z so `transformed_aabb.min.z == 0.0` (rests on spoilboard `Z = 0`).
    pub fn dropped_bottom_to_z0_for_aabb(&self, transformed_aabb: Aabb3) -> Self {
        if !transformed_aabb.is_valid() {
            return *self;
        }
        let mut out = *self;
        out.translate_mm[2] = clean_scalar(out.translate_mm[2] - transformed_aabb.min.z);
        out
    }

    /// Translates in Z so `transformed_aabb.max.z == 0.0` (top face sits at `Z = 0`).
    pub fn aligned_top_to_z0_for_aabb(&self, transformed_aabb: Aabb3) -> Self {
        if !transformed_aabb.is_valid() {
            return *self;
        }
        let mut out = *self;
        out.translate_mm[2] = clean_scalar(out.translate_mm[2] - transformed_aabb.max.z);
        out
    }
}

fn clean_scalar(v: f64) -> f64 {
    if v.abs() <= 1e-9 { 0.0 } else { v }
}

fn clean_vec3_array(v: DVec3) -> [f64; 3] {
    [clean_scalar(v.x), clean_scalar(v.y), clean_scalar(v.z)]
}

pub use chip_math::rotation_matrix_to_xyz_euler_deg;

fn transform_aabb_corners(aabb: Aabb3, xf: &ModelTransform) -> Aabb3 {
    if !aabb.is_valid() {
        return Aabb3::EMPTY;
    }
    let xs = [aabb.min.x, aabb.max.x];
    let ys = [aabb.min.y, aabb.max.y];
    let zs = [aabb.min.z, aabb.max.z];
    let mut out = Aabb3::EMPTY;
    for &x in &xs {
        for &y in &ys {
            for &z in &zs {
                out.extend_point(xf.transform_point(DVec3::new(x, y, z)));
            }
        }
    }
    out
}

fn polyline_length_mm(pts: &[DVec3]) -> f64 {
    pts.windows(2).map(|w| w[0].distance(w[1])).sum()
}

impl TriangleMeshBody {
    /// Returns a new [`TriangleMeshBody`] with `xf` applied to every `f64` vertex, recomputing
    /// exact geometric triangle normals, per-triangle AABBs, overall `aabb_mm`, and scaled volume.
    pub fn transformed(&self, xf: &ModelTransform) -> Self {
        if xf.is_identity() {
            return self.clone();
        }
        let vertices_mm: Vec<DVec3> = self
            .vertices_mm
            .iter()
            .map(|&v| xf.transform_point(v))
            .collect();

        let mut aabb_mm = Aabb3::EMPTY;
        for &v in &vertices_mm {
            aabb_mm.extend_point(v);
        }

        let mut triangles = Vec::with_capacity(self.triangles.len());
        let mut signed_volume_6x = 0.0_f64;
        for tri in &self.triangles {
            let [i0, i1, i2] = tri.indices;
            let v0 = vertices_mm[i0 as usize];
            let v1 = vertices_mm[i1 as usize];
            let v2 = vertices_mm[i2 as usize];
            let cross = (v1 - v0).cross(v2 - v0);
            let normal = if cross.length_squared() > 1e-24 {
                cross.normalize()
            } else {
                xf.transform_normal(tri.normal)
            };
            let mut taabb = Aabb3::EMPTY;
            taabb.extend_point(v0);
            taabb.extend_point(v1);
            taabb.extend_point(v2);
            signed_volume_6x += v0.dot(v1.cross(v2));
            triangles.push(MeshTriangle {
                indices: tri.indices,
                normal,
                aabb: taabb,
            });
        }

        let mut diagnostics = self.diagnostics.clone();
        diagnostics.signed_volume_mm3 = signed_volume_6x / 6.0;

        Self {
            id: self.id,
            name: self.name.clone(),
            format: self.format,
            detected_unit: self.detected_unit,
            applied_unit: self.applied_unit,
            vertices_mm,
            triangles,
            aabb_mm,
            diagnostics,
        }
    }
}

impl CadDocument {
    /// Returns a new [`CadDocument`] transformed non-destructively by `xf` in exact `f64` mm.
    ///
    /// - Uniform scaling preserves all analytic B-Rep surfaces and curves (`Plane`, `Cylinder`,
    ///   `Cone`, `Torus`, `Line`, `CircleArc`) exactly.
    /// - Non-uniform scaling preserves exact `Plane` normals, `Line` edges, polylines, and
    ///   triangle meshes, while safely converting stretched circular/cylindrical analytic entities
    ///   to `EllipseArc` / `OtherNurbs`.
    pub fn transformed(&self, xf: &ModelTransform) -> Self {
        if xf.is_identity() {
            return self.clone();
        }
        let uniform_s = xf.uniform_scale();
        let det_scale = (xf.scale[0] * xf.scale[1] * xf.scale[2]).abs();
        let approx_area_scale = det_scale.powf(2.0 / 3.0);

        // 1. Transform triangle meshes (exact for both uniform and non-uniform scaling)
        let meshes: Vec<TriangleMeshBody> = self.meshes.iter().map(|m| m.transformed(xf)).collect();

        // 2. Transform RenderMesh (or rebuild cleanly from transformed mesh for mesh-only docs)
        let render_mesh = if self.bodies.is_empty() && meshes.len() == 1 {
            build_render_mesh_for_triangle_mesh(&meshes[0], 35.0, MESH_FACE_SENTINEL)
        } else {
            let vertices: Vec<RenderVertex> = self
                .render_mesh
                .vertices
                .iter()
                .map(|v| {
                    let p = xf.transform_point(DVec3::new(
                        f64::from(v.position_mm[0]),
                        f64::from(v.position_mm[1]),
                        f64::from(v.position_mm[2]),
                    ));
                    let n = xf.transform_normal(DVec3::new(
                        f64::from(v.normal[0]),
                        f64::from(v.normal[1]),
                        f64::from(v.normal[2]),
                    ));
                    RenderVertex {
                        position_mm: [p.x as f32, p.y as f32, p.z as f32],
                        normal: [n.x as f32, n.y as f32, n.z as f32],
                        face_id: v.face_id,
                    }
                })
                .collect();
            let pick_triangles: Vec<FaceTriangle> = self
                .render_mesh
                .pick_triangles
                .iter()
                .map(|t| FaceTriangle {
                    face_id: t.face_id,
                    v0_mm: xf.transform_point(t.v0_mm),
                    v1_mm: xf.transform_point(t.v1_mm),
                    v2_mm: xf.transform_point(t.v2_mm),
                })
                .collect();
            let edge_polylines: Vec<EdgePolyline> = self
                .render_mesh
                .edge_polylines
                .iter()
                .map(|ep| EdgePolyline {
                    edge_id: ep.edge_id,
                    points_mm: ep
                        .points_mm
                        .iter()
                        .map(|&p| xf.transform_point(p))
                        .collect(),
                })
                .collect();
            RenderMesh {
                vertices,
                indices: self.render_mesh.indices.clone(),
                pick_triangles,
                edge_polylines,
            }
        };

        // Precompute tight transformed AABBs per FaceId and per EdgeId from exact pick geometry
        let mut face_aabbs: HashMap<FaceId, Aabb3> = HashMap::new();
        let mut face_areas: HashMap<FaceId, f64> = HashMap::new();
        for t in &render_mesh.pick_triangles {
            let entry = face_aabbs.entry(t.face_id).or_insert(Aabb3::EMPTY);
            entry.extend_point(t.v0_mm);
            entry.extend_point(t.v1_mm);
            entry.extend_point(t.v2_mm);
            let tri_area = 0.5 * (t.v1_mm - t.v0_mm).cross(t.v2_mm - t.v0_mm).length();
            *face_areas.entry(t.face_id).or_insert(0.0) += tri_area;
        }
        let mut edge_aabbs: HashMap<EdgeId, Aabb3> = HashMap::new();
        let mut edge_lengths: HashMap<EdgeId, f64> = HashMap::new();
        for ep in &render_mesh.edge_polylines {
            let entry = edge_aabbs.entry(ep.edge_id).or_insert(Aabb3::EMPTY);
            for &p in &ep.points_mm {
                entry.extend_point(p);
            }
            edge_lengths.insert(ep.edge_id, polyline_length_mm(&ep.points_mm));
        }

        // 3. Transform B-Rep edges
        let edges: Vec<BRepEdge> = self
            .edges
            .iter()
            .map(|e| {
                let (curve, length_mm, sig_kind, sig_axis, sig_measure) = match e.curve {
                    CurveKind::Line { start_mm, end_mm } => {
                        let s = xf.transform_point(start_mm);
                        let t = xf.transform_point(end_mm);
                        let len = s.distance(t);
                        let dir = if len > 1e-9 { (t - s) / len } else { DVec3::X };
                        (
                            CurveKind::Line {
                                start_mm: s,
                                end_mm: t,
                            },
                            len,
                            EntitySignatureKind::LineEdge,
                            dir,
                            len,
                        )
                    }
                    CurveKind::CircleArc {
                        center_mm,
                        normal,
                        radius_mm,
                        start_mm,
                        end_mm,
                        is_full_circle,
                    } => {
                        if let Some(us) = uniform_s {
                            let new_r = radius_mm * us;
                            let new_n = xf.transform_normal(normal);
                            (
                                CurveKind::CircleArc {
                                    center_mm: xf.transform_point(center_mm),
                                    normal: new_n,
                                    radius_mm: new_r,
                                    start_mm: xf.transform_point(start_mm),
                                    end_mm: xf.transform_point(end_mm),
                                    is_full_circle,
                                },
                                e.length_mm * us,
                                EntitySignatureKind::CircleEdge,
                                new_n,
                                new_r,
                            )
                        } else {
                            let len = edge_lengths
                                .get(&e.id)
                                .copied()
                                .unwrap_or(e.length_mm * det_scale.cbrt());
                            (
                                CurveKind::EllipseArc,
                                len,
                                EntitySignatureKind::OtherEdge,
                                xf.transform_normal(normal),
                                len,
                            )
                        }
                    }
                    CurveKind::EllipseArc => {
                        let len = edge_lengths
                            .get(&e.id)
                            .copied()
                            .unwrap_or(e.length_mm * det_scale.cbrt());
                        (
                            CurveKind::EllipseArc,
                            len,
                            EntitySignatureKind::OtherEdge,
                            xf.transform_dir_unit(e.signature.normal_or_axis),
                            len,
                        )
                    }
                    CurveKind::OtherNurbs => {
                        let len = edge_lengths
                            .get(&e.id)
                            .copied()
                            .unwrap_or(e.length_mm * det_scale.cbrt());
                        (
                            CurveKind::OtherNurbs,
                            len,
                            EntitySignatureKind::OtherEdge,
                            xf.transform_dir_unit(e.signature.normal_or_axis),
                            len,
                        )
                    }
                };
                let aabb_mm = edge_aabbs
                    .get(&e.id)
                    .copied()
                    .filter(|b| b.is_valid())
                    .unwrap_or_else(|| transform_aabb_corners(e.aabb_mm, xf));
                BRepEdge {
                    id: e.id,
                    body_id: e.body_id,
                    curve,
                    length_mm,
                    aabb_mm,
                    adjacent_faces: e.adjacent_faces.clone(),
                    loops: e.loops.clone(),
                    signature: GeometricSignature {
                        kind: sig_kind,
                        normal_or_axis: sig_axis,
                        centroid_mm: xf.transform_point(e.signature.centroid_mm),
                        characteristic_measure: sig_measure,
                        aabb_mm,
                    },
                }
            })
            .collect();

        // 4. Transform B-Rep faces
        let faces: Vec<BRepFace> = self
            .faces
            .iter()
            .map(|f| {
                let area_mm2 = if let Some(us) = uniform_s {
                    f.area_mm2 * us * us
                } else {
                    face_areas
                        .get(&f.id)
                        .copied()
                        .filter(|&a| a > 0.0)
                        .unwrap_or(f.area_mm2 * approx_area_scale)
                };
                let (surface, sig_kind, sig_axis, sig_measure) = match f.surface {
                    SurfaceKind::Plane { origin_mm, normal } => {
                        let n = xf.transform_normal(normal);
                        (
                            SurfaceKind::Plane {
                                origin_mm: xf.transform_point(origin_mm),
                                normal: n,
                            },
                            EntitySignatureKind::PlanarFace,
                            n,
                            area_mm2,
                        )
                    }
                    SurfaceKind::Cylinder {
                        axis_origin_mm,
                        axis_dir,
                        radius_mm,
                        is_internal,
                    } => {
                        if let Some(us) = uniform_s {
                            let new_r = radius_mm * us;
                            let new_axis = xf.transform_dir_unit(axis_dir);
                            let kind = if is_internal {
                                EntitySignatureKind::InternalCylinderFace
                            } else {
                                EntitySignatureKind::ExternalCylinderFace
                            };
                            (
                                SurfaceKind::Cylinder {
                                    axis_origin_mm: xf.transform_point(axis_origin_mm),
                                    axis_dir: new_axis,
                                    radius_mm: new_r,
                                    is_internal,
                                },
                                kind,
                                new_axis,
                                new_r,
                            )
                        } else {
                            (
                                SurfaceKind::OtherNurbs,
                                EntitySignatureKind::OtherFace,
                                xf.transform_dir_unit(axis_dir),
                                area_mm2,
                            )
                        }
                    }
                    SurfaceKind::Cone {
                        apex_mm,
                        axis_dir,
                        half_angle_rad,
                    } => {
                        if uniform_s.is_some() {
                            let new_axis = xf.transform_dir_unit(axis_dir);
                            (
                                SurfaceKind::Cone {
                                    apex_mm: xf.transform_point(apex_mm),
                                    axis_dir: new_axis,
                                    half_angle_rad,
                                },
                                EntitySignatureKind::ConicalFace,
                                new_axis,
                                area_mm2,
                            )
                        } else {
                            (
                                SurfaceKind::OtherNurbs,
                                EntitySignatureKind::OtherFace,
                                xf.transform_dir_unit(axis_dir),
                                area_mm2,
                            )
                        }
                    }
                    SurfaceKind::Torus {
                        center_mm,
                        axis_dir,
                        major_radius_mm,
                        minor_radius_mm,
                    } => {
                        if let Some(us) = uniform_s {
                            let new_axis = xf.transform_dir_unit(axis_dir);
                            (
                                SurfaceKind::Torus {
                                    center_mm: xf.transform_point(center_mm),
                                    axis_dir: new_axis,
                                    major_radius_mm: major_radius_mm * us,
                                    minor_radius_mm: minor_radius_mm * us,
                                },
                                EntitySignatureKind::OtherFace,
                                new_axis,
                                area_mm2,
                            )
                        } else {
                            (
                                SurfaceKind::OtherNurbs,
                                EntitySignatureKind::OtherFace,
                                xf.transform_dir_unit(axis_dir),
                                area_mm2,
                            )
                        }
                    }
                    SurfaceKind::OtherNurbs => (
                        SurfaceKind::OtherNurbs,
                        EntitySignatureKind::OtherFace,
                        xf.transform_normal(f.signature.normal_or_axis),
                        area_mm2,
                    ),
                };
                let aabb_mm = face_aabbs
                    .get(&f.id)
                    .copied()
                    .filter(|b| b.is_valid())
                    .unwrap_or_else(|| transform_aabb_corners(f.aabb_mm, xf));
                BRepFace {
                    id: f.id,
                    body_id: f.body_id,
                    surface,
                    area_mm2,
                    aabb_mm,
                    outer_loop: f.outer_loop,
                    inner_loops: f.inner_loops.clone(),
                    signature: GeometricSignature {
                        kind: sig_kind,
                        normal_or_axis: sig_axis,
                        centroid_mm: xf.transform_point(f.signature.centroid_mm),
                        characteristic_measure: sig_measure,
                        aabb_mm,
                    },
                }
            })
            .collect();

        // 5. Transform B-Rep loops
        let loops: Vec<BRepLoop> = self
            .loops
            .iter()
            .map(|lp| {
                let mut loop_aabb = Aabb3::EMPTY;
                let mut total_len = 0.0_f64;
                for eu in &lp.edges {
                    if let Some(edge) = edges.get(eu.edge_id.0 as usize) {
                        loop_aabb = loop_aabb.union(&edge.aabb_mm);
                        total_len += edge.length_mm;
                    }
                }
                if !loop_aabb.is_valid() {
                    loop_aabb = transform_aabb_corners(lp.signature.aabb_mm, xf);
                }
                BRepLoop {
                    id: lp.id,
                    face_id: lp.face_id,
                    is_outer: lp.is_outer,
                    edges: lp.edges.clone(),
                    signature: GeometricSignature {
                        kind: EntitySignatureKind::EdgeLoop,
                        normal_or_axis: xf.transform_normal(lp.signature.normal_or_axis),
                        centroid_mm: xf.transform_point(lp.signature.centroid_mm),
                        characteristic_measure: if total_len > 0.0 {
                            total_len
                        } else {
                            lp.signature.characteristic_measure * det_scale.cbrt()
                        },
                        aabb_mm: loop_aabb,
                    },
                }
            })
            .collect();

        // 6. Recompute SolidBody and overall CadDocument AABBs
        let mut doc_aabb = Aabb3::EMPTY;
        let bodies: Vec<SolidBody> = self
            .bodies
            .iter()
            .map(|b| {
                let mut b_aabb = Aabb3::EMPTY;
                for &fid in &b.faces {
                    if let Some(f) = faces.get(fid.0 as usize) {
                        b_aabb = b_aabb.union(&f.aabb_mm);
                    }
                }
                if !b_aabb.is_valid() {
                    b_aabb = transform_aabb_corners(b.aabb_mm, xf);
                }
                doc_aabb = doc_aabb.union(&b_aabb);
                SolidBody {
                    id: b.id,
                    name: b.name.clone(),
                    faces: b.faces.clone(),
                    edges: b.edges.clone(),
                    aabb_mm: b_aabb,
                }
            })
            .collect();
        for m in &meshes {
            doc_aabb = doc_aabb.union(&m.aabb_mm);
        }

        Self {
            source_name: self.source_name.clone(),
            detected_unit: self.detected_unit,
            bodies,
            meshes,
            faces,
            loops,
            edges,
            aabb_mm: doc_aabb,
            render_mesh,
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::approx_eq;

    use super::*;
    use crate::{
        CadKernel, MeshImportOptions, MonstertruckKernel, load_mesh_cad_document,
        sample_multi_feature_bracket_step, sample_sculpted_dome_stl_bytes,
    };

    #[test]
    fn model_transform_euler_roundtrip_lay_flat_scale_fit_and_toml_omission() {
        assert!(ModelTransform::IDENTITY.is_identity());
        assert!(ModelTransform::IDENTITY.validate().is_ok());

        // Invalid scale rejected
        let bad = ModelTransform {
            scale: [0.0, 1.0, 1.0],
            ..ModelTransform::IDENTITY
        };
        assert!(bad.validate().is_err());

        // Euler composition preserves exact world rotation across repeated 90° clicks
        let pivot = DVec3::new(10.0, 20.0, 5.0);
        let r1 = ModelTransform::IDENTITY.rotated_world_axis_deg(DVec3::X, 90.0, pivot);
        let r2 = r1.rotated_world_axis_deg(DVec3::Y, 90.0, pivot);
        let r3 = r2.rotated_world_axis_deg(DVec3::Z, -90.0, pivot);
        let expected_r = DMat3::from_rotation_z(-std::f64::consts::FRAC_PI_2)
            * DMat3::from_rotation_y(std::f64::consts::FRAC_PI_2)
            * DMat3::from_rotation_x(std::f64::consts::FRAC_PI_2);
        let got_r = r3.rotation_matrix();
        for axis in [DVec3::X, DVec3::Y, DVec3::Z] {
            assert!((expected_r * axis - got_r * axis).length() < 1e-9);
        }

        // Lay flat maps cardinal, diagonal, and +Z normals cleanly to -Z
        for raw_n in [
            DVec3::Y,
            DVec3::NEG_X,
            DVec3::Z,
            DVec3::new(1.0, 2.0, 3.0).normalize(),
        ] {
            let laid = ModelTransform::IDENTITY.laid_flat_normal_to_neg_z(raw_n, DVec3::ZERO);
            let mapped = laid.transform_normal(raw_n);
            assert!(
                mapped.dot(DVec3::NEG_Z) > 1.0 - 1e-9,
                "failed to lay normal {raw_n:?} flat -> got {mapped:?}"
            );
        }

        // Fit-to-axis uniform scale
        let aabb = Aabb3::new(DVec3::new(0.0, 0.0, 0.0), DVec3::new(50.0, 25.0, 10.0));
        let fit = ModelTransform::IDENTITY.scaled_uniformly_to_fit_axis(aabb, 0, 150.0);
        assert!(approx_eq(fit.uniform_scale().unwrap(), 3.0, 1e-9));
    }

    #[test]
    fn cad_document_transformed_preserves_exact_step_brep_and_stl_mesh_geometry() {
        // 1. STEP B-Rep: uniform 2x scale + 90° X rotation + drop to Z=0
        let step_text = sample_multi_feature_bracket_step(false).unwrap();
        let raw_step = MonstertruckKernel
            .load_step_bytes("bracket.step", step_text.as_bytes(), 0.05)
            .unwrap();
        let orig_size = raw_step.aabb_mm.size();
        let orig_hole_r = raw_step
            .faces
            .iter()
            .find_map(|f| match f.surface {
                SurfaceKind::Cylinder {
                    radius_mm,
                    is_internal: true,
                    ..
                } => Some(radius_mm),
                _ => None,
            })
            .unwrap();

        let xf = ModelTransform {
            translate_mm: [0.0, 0.0, 0.0],
            rotate_deg: [90.0, 0.0, 0.0],
            scale: [2.0, 2.0, 2.0],
        };
        let t_step = raw_step.transformed(&xf);
        let xf_drop = xf
            .centered_xy_for_aabb(t_step.aabb_mm)
            .dropped_bottom_to_z0_for_aabb(t_step.aabb_mm);
        let placed = raw_step.transformed(&xf_drop);

        assert!(approx_eq(placed.aabb_mm.size().x, orig_size.x * 2.0, 1e-4));
        assert!(approx_eq(placed.aabb_mm.size().y, orig_size.z * 2.0, 1e-4));
        assert!(approx_eq(placed.aabb_mm.size().z, orig_size.y * 2.0, 1e-4));
        assert!(approx_eq(placed.aabb_mm.min.z, 0.0, 1e-6));
        assert!(approx_eq(placed.aabb_mm.center().x, 0.0, 1e-6));
        assert!(approx_eq(placed.aabb_mm.center().y, 0.0, 1e-6));

        let scaled_hole_r = placed
            .faces
            .iter()
            .find_map(|f| match f.surface {
                SurfaceKind::Cylinder {
                    radius_mm,
                    is_internal: true,
                    ..
                } => Some(radius_mm),
                _ => None,
            })
            .unwrap();
        assert!(approx_eq(scaled_hole_r, orig_hole_r * 2.0, 1e-6));

        // Non-uniform scale degrades cylinders safely to OtherNurbs without bogus radii
        let non_unif = ModelTransform {
            scale: [1.5, 1.0, 1.0],
            ..ModelTransform::IDENTITY
        };
        let stretched = raw_step.transformed(&non_unif);
        assert!(
            !stretched
                .faces
                .iter()
                .any(|f| matches!(f.surface, SurfaceKind::Cylinder { .. }))
        );

        // 2. STL Mesh: non-uniform scale + rotation preserves watertightness and exact scaled volume
        let stl_bytes = sample_sculpted_dome_stl_bytes();
        let raw_mesh =
            load_mesh_cad_document("dome.stl", &stl_bytes, &MeshImportOptions::default()).unwrap();
        let orig_vol = raw_mesh.meshes[0].diagnostics.signed_volume_mm3;
        let mesh_xf = ModelTransform {
            translate_mm: [15.0, -10.0, 5.0],
            rotate_deg: [90.0, 0.0, 45.0],
            scale: [1.5, 2.0, 0.5],
        };
        let t_mesh = raw_mesh.transformed(&mesh_xf);
        let new_vol = t_mesh.meshes[0].diagnostics.signed_volume_mm3;
        assert!(t_mesh.meshes[0].diagnostics.is_watertight);
        assert!(approx_eq(new_vol, orig_vol * (1.5 * 2.0 * 0.5), 1e-2));
    }
}
