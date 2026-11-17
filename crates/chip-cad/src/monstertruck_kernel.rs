use chip_math::{ANALYTIC_CLASSIFY_TOL_MM, Aabb3, COINCIDENCE_TOL_MM, DVec3};
use monstertruck_io::step::load::{
    Table,
    step_geometry::{
        BoundedCurve, Curve3D, ElementarySurface, ParametricCurve, Point3, Surface, Vector3,
    },
};
use monstertruck_meshing::prelude::*;

use crate::{
    kernel::{CadError, CadKernel},
    model::{
        BRepEdge, BRepFace, BRepLoop, BodyId, CadDocument, CurveKind, EdgeId, EdgePolyline,
        EntitySignatureKind, FaceId, FaceTriangle, GeometricSignature, LoopId, OrientedEdgeUse,
        RenderMesh, RenderVertex, SolidBody, StepUnitKind, SurfaceKind,
    },
};

/// Pure-Rust ISO 10303-21 STEP kernel powered by `monstertruck`.
#[derive(Debug, Default, Clone, Copy)]
pub struct MonstertruckKernel;

impl CadKernel for MonstertruckKernel {
    fn load_step_bytes(
        &self,
        source_name: &str,
        step_bytes: &[u8],
        chord_tol_mm: f64,
    ) -> Result<CadDocument, CadError> {
        if step_bytes.is_empty() {
            return Err(CadError::EmptyInput);
        }

        let detected_unit = detect_step_length_unit(step_bytes);
        let unit_scale = detected_unit.scale_to_mm();
        let tol_mm = chord_tol_mm.max(1e-3);
        let tol_file_units = (tol_mm / unit_scale).max(1e-5);

        let table =
            Table::from_step_bytes(step_bytes).map_err(|e| CadError::StepParse(e.to_string()))?;

        if table.shell.is_empty() {
            return Err(CadError::NoGeometry);
        }

        // Sort STEP shell entity IDs so BodyId / FaceId / LoopId / EdgeId assignment is 100% deterministic.
        let mut sorted_shells: Vec<_> = table.shell.iter().collect();
        sorted_shells.sort_by_key(|(entity_id, _)| **entity_id);

        let mut bodies = Vec::with_capacity(sorted_shells.len());
        let mut faces = Vec::new();
        let mut loops = Vec::new();
        let mut edges = Vec::new();
        let mut render_mesh = RenderMesh::default();
        let mut doc_aabb = Aabb3::EMPTY;

        for (body_idx, (&step_shell_id, step_shell)) in sorted_shells.into_iter().enumerate() {
            let body_id = BodyId(body_idx as u32);
            let trimmed = table.to_compressed_trimmed_shell(step_shell).map_err(|e| {
                CadError::TopologyConversion {
                    shell_id: step_shell_id,
                    detail: e.to_string(),
                }
            })?;

            let meshed = trimmed.robust_triangulation(tol_file_units);

            let base_edge_offset = edges.len();
            let base_face_offset = faces.len();

            let mut body_edge_ids = Vec::with_capacity(trimmed.edges.len());
            for (local_e_idx, comp_edge) in trimmed.edges.iter().enumerate() {
                let edge_id = EdgeId((base_edge_offset + local_e_idx) as u32);
                body_edge_ids.push(edge_id);

                let poly_pts: Vec<DVec3> = meshed
                    .edges
                    .get(local_e_idx)
                    .map(|me| {
                        me.curve
                            .iter()
                            .map(|pt| pt_to_dvec3(*pt) * unit_scale)
                            .collect()
                    })
                    .unwrap_or_default();

                let curve_kind = classify_curve_3d(&comp_edge.curve, unit_scale);
                let length_mm = polyline_length(&poly_pts);
                let edge_aabb = Aabb3::from_points(poly_pts.iter().copied());

                if !poly_pts.is_empty() {
                    render_mesh.edge_polylines.push(EdgePolyline {
                        edge_id,
                        points_mm: poly_pts.clone(),
                    });
                }

                let sig = build_edge_signature(&curve_kind, &poly_pts, length_mm, edge_aabb);
                edges.push(BRepEdge {
                    id: edge_id,
                    body_id,
                    curve: curve_kind,
                    length_mm,
                    aabb_mm: edge_aabb,
                    adjacent_faces: Vec::new(),
                    loops: Vec::new(),
                    signature: sig,
                });
            }

            let mut body_face_ids = Vec::with_capacity(trimmed.faces.len());
            let mut body_aabb = Aabb3::EMPTY;

            for (local_f_idx, comp_face) in trimmed.faces.iter().enumerate() {
                let face_id = FaceId((base_face_offset + local_f_idx) as u32);
                body_face_ids.push(face_id);

                // Extract tessellated polygon mesh for this specific face
                let mut face_aabb = Aabb3::EMPTY;
                let mut face_area_mm2 = 0.0_f64;
                let mut weighted_centroid = DVec3::ZERO;
                let mut exact_samples: Vec<(DVec3, DVec3)> = Vec::new();

                if let Some(Some(mut poly)) =
                    meshed.faces.get(local_f_idx).map(|f| f.surface.clone())
                {
                    if !comp_face.orientation {
                        poly.invert();
                    }
                    let positions = poly.positions();
                    let normals = poly.normals();

                    for tri in poly.faces().triangle_iter() {
                        let p0 = pt_to_dvec3(positions[tri[0].pos]) * unit_scale;
                        let p1 = pt_to_dvec3(positions[tri[1].pos]) * unit_scale;
                        let p2 = pt_to_dvec3(positions[tri[2].pos]) * unit_scale;

                        face_aabb.extend_point(p0);
                        face_aabb.extend_point(p1);
                        face_aabb.extend_point(p2);

                        let cross = (p1 - p0).cross(p2 - p0);
                        let tri_area = 0.5 * cross.length();
                        face_area_mm2 += tri_area;
                        weighted_centroid += (p0 + p1 + p2) * (tri_area / 3.0);

                        let fallback_n = cross.try_normalize().unwrap_or(DVec3::Z);
                        let n0 = tri[0]
                            .nor
                            .and_then(|idx| normals.as_slice().get(idx).copied())
                            .and_then(|v| vec_to_dvec3(v).try_normalize())
                            .unwrap_or(fallback_n);
                        let n1 = tri[1]
                            .nor
                            .and_then(|idx| normals.as_slice().get(idx).copied())
                            .and_then(|v| vec_to_dvec3(v).try_normalize())
                            .unwrap_or(fallback_n);
                        let n2 = tri[2]
                            .nor
                            .and_then(|idx| normals.as_slice().get(idx).copied())
                            .and_then(|v| vec_to_dvec3(v).try_normalize())
                            .unwrap_or(fallback_n);

                        if exact_samples.len() < 48 {
                            exact_samples.push((p0, n0));
                            exact_samples.push((p1, n1));
                            exact_samples.push((p2, n2));
                        }

                        let base_v = render_mesh.vertices.len() as u32;
                        render_mesh.vertices.push(RenderVertex {
                            position_mm: [p0.x as f32, p0.y as f32, p0.z as f32],
                            normal: [n0.x as f32, n0.y as f32, n0.z as f32],
                            face_id: face_id.0,
                        });
                        render_mesh.vertices.push(RenderVertex {
                            position_mm: [p1.x as f32, p1.y as f32, p1.z as f32],
                            normal: [n1.x as f32, n1.y as f32, n1.z as f32],
                            face_id: face_id.0,
                        });
                        render_mesh.vertices.push(RenderVertex {
                            position_mm: [p2.x as f32, p2.y as f32, p2.z as f32],
                            normal: [n2.x as f32, n2.y as f32, n2.z as f32],
                            face_id: face_id.0,
                        });
                        render_mesh
                            .indices
                            .extend_from_slice(&[base_v, base_v + 1, base_v + 2]);

                        render_mesh.pick_triangles.push(FaceTriangle {
                            face_id,
                            v0_mm: p0,
                            v1_mm: p1,
                            v2_mm: p2,
                        });
                    }
                }

                let centroid_mm = if face_area_mm2 > 1e-12 {
                    weighted_centroid / face_area_mm2
                } else {
                    face_aabb.center()
                };

                let surface_kind = classify_surface(
                    &comp_face.surface,
                    comp_face.orientation,
                    unit_scale,
                    centroid_mm,
                    &exact_samples,
                );

                // Extract outer & inner boundary loops; identify outer loop by largest bounding box diagonal.
                let mut wire_loop_ids = Vec::with_capacity(comp_face.boundaries.len());
                let mut max_diag = -1.0_f64;
                let mut outer_idx = 0_usize;

                let mut pending_loops = Vec::with_capacity(comp_face.boundaries.len());
                for (w_idx, wire) in comp_face.boundaries.iter().enumerate() {
                    let mut loop_aabb = Aabb3::EMPTY;
                    let mut loop_len = 0.0_f64;
                    let mut uses = Vec::with_capacity(wire.len());

                    for edge_use in wire {
                        let global_edge_idx = base_edge_offset + edge_use.index;
                        let edge_id = EdgeId(global_edge_idx as u32);
                        uses.push(OrientedEdgeUse {
                            edge_id,
                            forward: edge_use.orientation,
                        });
                        if let Some(edge_rec) = edges.get(global_edge_idx) {
                            loop_aabb = loop_aabb.union(&edge_rec.aabb_mm);
                            loop_len += edge_rec.length_mm;
                        }
                    }
                    let diag = loop_aabb.diagonal();
                    if diag > max_diag {
                        max_diag = diag;
                        outer_idx = w_idx;
                    }
                    pending_loops.push((uses, loop_aabb, loop_len));
                }

                let normal_hint = match surface_kind {
                    SurfaceKind::Plane { normal, .. } => normal,
                    SurfaceKind::Cylinder { axis_dir, .. } => axis_dir,
                    SurfaceKind::Cone { axis_dir, .. } => axis_dir,
                    SurfaceKind::Torus { axis_dir, .. } => axis_dir,
                    SurfaceKind::OtherNurbs => DVec3::Z,
                };

                for (w_idx, (uses, loop_aabb, loop_len)) in pending_loops.into_iter().enumerate() {
                    let loop_id = LoopId(loops.len() as u32);
                    let is_outer = w_idx == outer_idx;
                    for u in &uses {
                        if let Some(edge_rec) = edges.get_mut(u.edge_id.0 as usize) {
                            if !edge_rec.adjacent_faces.contains(&face_id) {
                                edge_rec.adjacent_faces.push(face_id);
                            }
                            if !edge_rec.loops.contains(&loop_id) {
                                edge_rec.loops.push(loop_id);
                            }
                        }
                    }
                    loops.push(BRepLoop {
                        id: loop_id,
                        face_id,
                        is_outer,
                        edges: uses,
                        signature: GeometricSignature {
                            kind: EntitySignatureKind::EdgeLoop,
                            normal_or_axis: normal_hint,
                            centroid_mm: loop_aabb.center(),
                            characteristic_measure: loop_len,
                            aabb_mm: loop_aabb,
                        },
                    });
                    wire_loop_ids.push((loop_id, is_outer));
                }

                let outer_loop = wire_loop_ids
                    .iter()
                    .find(|(_, is_out)| *is_out)
                    .map(|(id, _)| *id);
                let inner_loops = wire_loop_ids
                    .into_iter()
                    .filter(|(_, is_out)| !*is_out)
                    .map(|(id, _)| id)
                    .collect();

                body_aabb = body_aabb.union(&face_aabb);
                let face_sig =
                    build_face_signature(&surface_kind, centroid_mm, face_area_mm2, face_aabb);

                faces.push(BRepFace {
                    id: face_id,
                    body_id,
                    surface: surface_kind,
                    area_mm2: face_area_mm2,
                    aabb_mm: face_aabb,
                    outer_loop,
                    inner_loops,
                    signature: face_sig,
                });
            }

            doc_aabb = doc_aabb.union(&body_aabb);
            bodies.push(SolidBody {
                id: body_id,
                name: format!("Solid Body {} (#{step_shell_id})", body_idx + 1),
                faces: body_face_ids,
                edges: body_edge_ids,
                aabb_mm: body_aabb,
            });
        }

        if render_mesh.indices.is_empty() {
            return Err(CadError::EmptyTessellation);
        }

        Ok(CadDocument {
            source_name: source_name.to_string(),
            detected_unit,
            bodies,
            faces,
            loops,
            edges,
            aabb_mm: doc_aabb,
            render_mesh,
        })
    }
}

/// Scans ISO 10303-21 STEP text for `CONVERSION_BASED_UNIT('INCH'...)` or `SI_UNIT(<prefix>, .METRE.)`.
fn detect_step_length_unit(step_bytes: &[u8]) -> StepUnitKind {
    let ascii_upper: String = step_bytes
        .iter()
        .take(256 * 1024)
        .map(|&b| (b as char).to_ascii_uppercase())
        .collect();

    if ascii_upper.contains("CONVERSION_BASED_UNIT") && ascii_upper.contains("'INCH'") {
        return StepUnitKind::Inch;
    }
    if let Some(pos) = ascii_upper.find(".METRE.") {
        let start = pos.saturating_sub(48);
        let window = &ascii_upper[start..pos];
        if window.contains(".MILLI.") {
            return StepUnitKind::Millimeter;
        }
        if window.contains(".CENTI.") {
            return StepUnitKind::Centimeter;
        }
        if window.contains("SI_UNIT($,") || window.contains("SI_UNIT( $,") {
            return StepUnitKind::Meter;
        }
    }
    StepUnitKind::Millimeter
}

#[inline]
fn pt_to_dvec3(p: Point3) -> DVec3 {
    DVec3::new(p.x, p.y, p.z)
}

#[inline]
fn vec_to_dvec3(v: Vector3) -> DVec3 {
    DVec3::new(v.x, v.y, v.z)
}

fn polyline_length(pts: &[DVec3]) -> f64 {
    pts.windows(2).map(|w| w[0].distance(w[1])).sum()
}

/// Recognizes analytic `Line` and `CircleArc` curves within `ANALYTIC_CLASSIFY_TOL_MM`,
/// even when represented as `SurfaceCurve`s or rational degree-2 B-splines/NURBS.
fn classify_curve_3d(curve: &Curve3D, unit_scale: f64) -> CurveKind {
    let (t0, t1) = curve.range_tuple();
    if !t0.is_finite() || !t1.is_finite() || (t1 - t0).abs() < 1e-12 {
        return CurveKind::OtherNurbs;
    }

    const SAMPLES: usize = 17;
    let mut pts = [DVec3::ZERO; SAMPLES];
    for (i, slot) in pts.iter_mut().enumerate() {
        let u = i as f64 / (SAMPLES - 1) as f64;
        let t = t0 + (t1 - t0) * u;
        *slot = pt_to_dvec3(curve.subs(t)) * unit_scale;
    }

    let start_mm = pts[0];
    let end_mm = pts[SAMPLES - 1];
    let chord = end_mm - start_mm;
    let chord_len = chord.length();

    // 1. Line test (only if endpoints aren't closed onto each other)
    if chord_len > COINCIDENCE_TOL_MM {
        let dir = chord / chord_len;
        let is_line = pts[1..SAMPLES - 1].iter().all(|&p| {
            let v = p - start_mm;
            let perp = v - dir * v.dot(dir);
            perp.length() <= ANALYTIC_CLASSIFY_TOL_MM
        });
        if is_line {
            return CurveKind::Line { start_mm, end_mm };
        }
    }

    // 2. Circle / Circular Arc test via 3 well-separated sample points
    let p_a = pts[0];
    let p_b = pts[5];
    let p_c = pts[11];
    if let Some((center_mm, normal, radius_mm)) = circumcircle_3d(p_a, p_b, p_c)
        && radius_mm > COINCIDENCE_TOL_MM
        && radius_mm < 1e7
    {
        let all_on_circle = pts.iter().all(|&p| {
            let rel = p - center_mm;
            let plane_err = rel.dot(normal).abs();
            let rad_err = (rel.length() - radius_mm).abs();
            plane_err <= ANALYTIC_CLASSIFY_TOL_MM * 5.0 && rad_err <= ANALYTIC_CLASSIFY_TOL_MM * 5.0
        });
        if all_on_circle {
            let is_full_circle = start_mm.distance(end_mm) <= COINCIDENCE_TOL_MM;
            // Orient normal consistently with curve traversal `(p_a - center) x (p_b - center)`
            let oriented_normal = ((p_a - center_mm).cross(p_b - center_mm))
                .try_normalize()
                .unwrap_or(normal);
            return CurveKind::CircleArc {
                center_mm,
                normal: oriented_normal,
                radius_mm,
                start_mm,
                end_mm,
                is_full_circle,
            };
        }
    }

    CurveKind::OtherNurbs
}

/// Computes the 3D circumcircle `(center, unit_normal, radius)` of three non-collinear points.
fn circumcircle_3d(a: DVec3, b: DVec3, c: DVec3) -> Option<(DVec3, DVec3, f64)> {
    let ab = b - a;
    let ac = c - a;
    let cross = ab.cross(ac);
    let cross_sq = cross.length_squared();
    if cross_sq < 1e-18 {
        return None;
    }
    let normal = cross / cross_sq.sqrt();
    let to_center = (cross.cross(ab) * ac.length_squared() + ac.cross(cross) * ab.length_squared())
        / (2.0 * cross_sq);
    let center = a + to_center;
    let radius = to_center.length();
    Some((center, normal, radius))
}

/// Classifies a B-Rep surface into `Plane`, `Cylinder` (with internal hole vs external boss flag!),
/// `Cone`, `Torus`, or `OtherNurbs`.
fn classify_surface(
    surface: &Surface,
    face_orientation: bool,
    unit_scale: f64,
    centroid_mm: DVec3,
    samples: &[(DVec3, DVec3)],
) -> SurfaceKind {
    let orient_sign = if face_orientation { 1.0 } else { -1.0 };

    // Fast path for explicit STEP elementary Plane
    if let Surface::ElementarySurface(ElementarySurface::Plane(plane)) = surface {
        let n = vec_to_dvec3(plane.normal()) * orient_sign;
        if let Some(normal) = n.try_normalize() {
            return SurfaceKind::Plane {
                origin_mm: centroid_mm,
                normal,
            };
        }
    }

    if samples.len() >= 3 {
        // 1. Analytic Plane test over exact surface-evaluated trimmed vertices & normals
        let (p0, n0) = samples[0];
        let is_planar = samples.iter().all(|&(p, n)| {
            n.dot(n0) >= 1.0 - 1e-6 && (p - p0).dot(n0).abs() <= ANALYTIC_CLASSIFY_TOL_MM * 10.0
        });
        if is_planar {
            return SurfaceKind::Plane {
                origin_mm: centroid_mm,
                normal: n0,
            };
        }

        // 2. Analytic Cylinder test (handles CylindricalSurface, ExtrusionSurface of arcs, and NURBS cylinders)
        if let Some(cyl) = try_fit_cylinder(samples) {
            return cyl;
        }
    }

    // 3. Explicit STEP Conical / Toroidal elementary tags
    match surface {
        Surface::ElementarySurface(ElementarySurface::ConicalSurface(cone)) => {
            let mat = cone.transform();
            let axis_dir = DVec3::new(mat[2][0], mat[2][1], mat[2][2])
                .try_normalize()
                .unwrap_or(DVec3::Z);
            let apex_mm = DVec3::new(mat[3][0], mat[3][1], mat[3][2]) * unit_scale;
            return SurfaceKind::Cone {
                apex_mm,
                axis_dir,
                half_angle_rad: std::f64::consts::FRAC_PI_4,
            };
        }
        Surface::ElementarySurface(ElementarySurface::ToroidalSurface(torus)) => {
            let mat = torus.transform();
            let axis_dir = DVec3::new(mat[2][0], mat[2][1], mat[2][2])
                .try_normalize()
                .unwrap_or(DVec3::Z);
            let center_mm = DVec3::new(mat[3][0], mat[3][1], mat[3][2]) * unit_scale;
            let inner = torus.entity();
            return SurfaceKind::Torus {
                center_mm,
                axis_dir,
                major_radius_mm: inner.large_radius() * unit_scale,
                minor_radius_mm: inner.small_radius() * unit_scale,
            };
        }
        _ => {}
    }

    SurfaceKind::OtherNurbs
}

/// Parameterization-independent 3D cylinder recognizer.
///
/// On a cylinder of radius `R` and unit axis `A`:
/// - Every outward normal `n_i` is perpendicular to `A` (`n_i · A == 0`).
/// - For an **internal hole** (outward solid normal points *inward* toward axis), `p_i + R * n_i`
///   collapses onto the axis line!
/// - For an **external boss/fillet** (outward solid normal points *away* from axis), `p_i - R * n_i`
///   collapses onto the axis line!
fn try_fit_cylinder(samples: &[(DVec3, DVec3)]) -> Option<SurfaceKind> {
    // Find a pair of normals with maximal cross product to determine cylinder axis A
    let mut best_cross = DVec3::ZERO;
    let mut best_len_sq = 0.0_f64;
    for i in 0..samples.len() {
        for j in (i + 1)..samples.len() {
            let c = samples[i].1.cross(samples[j].1);
            let lsq = c.length_squared();
            if lsq > best_len_sq {
                best_len_sq = lsq;
                best_cross = c;
            }
        }
    }
    if best_len_sq < 1e-6 {
        return None;
    }
    let mut axis_dir = best_cross / best_len_sq.sqrt();
    // Canonicalize axis sign (+Z preferred, then +X, then +Y) for deterministic signatures!
    if axis_dir.z < -1e-6
        || (axis_dir.z.abs() <= 1e-6 && axis_dir.x < -1e-6)
        || (axis_dir.z.abs() <= 1e-6 && axis_dir.x.abs() <= 1e-6 && axis_dir.y < 0.0)
    {
        axis_dir = -axis_dir;
    }

    // All normals must be orthogonal to `axis_dir`
    if samples.iter().any(|&(_, n)| n.dot(axis_dir).abs() > 1e-4) {
        return None;
    }

    // Project points & normals onto the 2D plane orthogonal to `axis_dir`:
    // Solve `q_i = p_perp_i + signed_r * n_i ≈ c_perp` via least squares over `signed_r`:
    // For pairs (0, i): `(p_perp_i - p_perp_0) + signed_r * (n_i - n_0) = 0`
    // => `signed_r = - sum((dp · dn)) / sum((dn · dn))`
    let project_perp = |v: DVec3| v - axis_dir * v.dot(axis_dir);
    let p0 = project_perp(samples[0].0);
    let n0 = project_perp(samples[0].1);

    let mut num = 0.0_f64;
    let mut den = 0.0_f64;
    for &(pi, ni) in &samples[1..] {
        let dp = project_perp(pi) - p0;
        let dn = project_perp(ni) - n0;
        num -= dp.dot(dn);
        den += dn.dot(dn);
    }
    if den < 1e-8 {
        return None;
    }
    let signed_r = num / den;
    let radius_mm = signed_r.abs();
    if !(COINCIDENCE_TOL_MM..=1e7).contains(&radius_mm) {
        return None;
    }

    // Compute mean axis point in 3D (projected onto centroid height along axis)
    let mut axis_perp_sum = DVec3::ZERO;
    let mut axial_t_sum = 0.0_f64;
    for &(pi, ni) in samples {
        axis_perp_sum += project_perp(pi) + project_perp(ni) * signed_r;
        axial_t_sum += pi.dot(axis_dir);
    }
    let inv_len = 1.0 / (samples.len() as f64);
    let axis_perp = axis_perp_sum * inv_len;
    let mean_t = axial_t_sum * inv_len;

    let max_scatter = samples
        .iter()
        .map(|&(pi, ni)| {
            let cand = project_perp(pi) + project_perp(ni) * signed_r;
            cand.distance(axis_perp)
        })
        .fold(0.0_f64, f64::max);

    if max_scatter > ANALYTIC_CLASSIFY_TOL_MM * 10.0 {
        return None;
    }

    let axis_origin_mm = axis_perp + axis_dir * mean_t;
    let is_internal = signed_r > 0.0;

    Some(SurfaceKind::Cylinder {
        axis_origin_mm,
        axis_dir,
        radius_mm,
        is_internal,
    })
}

fn build_face_signature(
    surface: &SurfaceKind,
    centroid_mm: DVec3,
    area_mm2: f64,
    aabb_mm: Aabb3,
) -> GeometricSignature {
    match *surface {
        SurfaceKind::Plane { normal, .. } => GeometricSignature {
            kind: EntitySignatureKind::PlanarFace,
            normal_or_axis: normal,
            centroid_mm,
            characteristic_measure: area_mm2,
            aabb_mm,
        },
        SurfaceKind::Cylinder {
            axis_dir,
            radius_mm,
            is_internal,
            ..
        } => GeometricSignature {
            kind: if is_internal {
                EntitySignatureKind::InternalCylinderFace
            } else {
                EntitySignatureKind::ExternalCylinderFace
            },
            normal_or_axis: axis_dir,
            centroid_mm,
            characteristic_measure: radius_mm,
            aabb_mm,
        },
        SurfaceKind::Cone { axis_dir, .. } => GeometricSignature {
            kind: EntitySignatureKind::ConicalFace,
            normal_or_axis: axis_dir,
            centroid_mm,
            characteristic_measure: area_mm2,
            aabb_mm,
        },
        SurfaceKind::Torus {
            axis_dir,
            minor_radius_mm,
            ..
        } => GeometricSignature {
            kind: EntitySignatureKind::OtherFace,
            normal_or_axis: axis_dir,
            centroid_mm,
            characteristic_measure: minor_radius_mm,
            aabb_mm,
        },
        SurfaceKind::OtherNurbs => GeometricSignature {
            kind: EntitySignatureKind::OtherFace,
            normal_or_axis: DVec3::Z,
            centroid_mm,
            characteristic_measure: area_mm2,
            aabb_mm,
        },
    }
}

fn build_edge_signature(
    curve: &CurveKind,
    poly_pts: &[DVec3],
    length_mm: f64,
    aabb_mm: Aabb3,
) -> GeometricSignature {
    let centroid_mm = if poly_pts.is_empty() {
        aabb_mm.center()
    } else {
        poly_pts.iter().copied().sum::<DVec3>() / (poly_pts.len() as f64)
    };

    match *curve {
        CurveKind::Line { start_mm, end_mm } => {
            let dir = (end_mm - start_mm).try_normalize().unwrap_or(DVec3::X);
            GeometricSignature {
                kind: EntitySignatureKind::LineEdge,
                normal_or_axis: dir,
                centroid_mm,
                characteristic_measure: length_mm,
                aabb_mm,
            }
        }
        CurveKind::CircleArc {
            normal, radius_mm, ..
        } => GeometricSignature {
            kind: EntitySignatureKind::CircleEdge,
            normal_or_axis: normal,
            centroid_mm,
            characteristic_measure: radius_mm,
            aabb_mm,
        },
        CurveKind::EllipseArc | CurveKind::OtherNurbs => GeometricSignature {
            kind: EntitySignatureKind::OtherEdge,
            normal_or_axis: DVec3::Z,
            centroid_mm,
            characteristic_measure: length_mm,
            aabb_mm,
        },
    }
}
