//! Thin CAD/CAM -> `chip-stock` geometric adapter isolating `chip-cad` and `chip-cam` types
//! from the pure volumetric `chip-stock` kernels.

use chip_bvh::TriMesh3D;
use chip_cad::{BodyId, CadDocument, SurfaceKind};
use chip_cam::{
    ExtractedFeature25D, GeometrySelectionItem, ResolvedRotaryConfig, ResolvedSetup,
    ResolvedStockGeometry, RotaryAxisKind, StockAxis, extract_selection_features,
};
use chip_math::DVec3;
use chip_stock::{
    AnalyticHorizFace, Axis3, RotaryStockSpec, StockShapeSpec, TargetTriangle, TriDexelStock,
};

/// Converts `chip-cam`'s [`StockAxis`] into `chip-stock`'s [`Axis3`].
#[inline]
pub const fn axis3_from_stock_axis(axis: StockAxis) -> Axis3 {
    match axis {
        StockAxis::X => Axis3::X,
        StockAxis::Y => Axis3::Y,
        StockAxis::Z => Axis3::Z,
    }
}

/// Converts `chip-cam`'s [`RotaryAxisKind`] into `chip-stock`'s [`Axis3`].
#[inline]
pub const fn axis3_from_rotary_axis(axis: RotaryAxisKind) -> Axis3 {
    match axis {
        RotaryAxisKind::XAxis => Axis3::X,
        RotaryAxisKind::YAxis => Axis3::Y,
        RotaryAxisKind::ZAxis => Axis3::Z,
    }
}

/// Converts `chip-cam`'s [`ResolvedStockGeometry`] into `chip-stock`'s pure [`StockShapeSpec`].
pub const fn stock_shape_spec_from_cam(stock: &ResolvedStockGeometry) -> StockShapeSpec {
    match *stock {
        ResolvedStockGeometry::Box { aabb_wcs } => StockShapeSpec::Box { aabb_wcs },
        ResolvedStockGeometry::Cylinder {
            axis,
            center_wcs,
            radius_mm,
            min_axial_mm,
            max_axial_mm,
            aabb_wcs,
        } => StockShapeSpec::Cylinder {
            axis: axis3_from_stock_axis(axis),
            center_wcs,
            radius_mm,
            min_axial_mm,
            max_axial_mm,
            aabb_wcs,
        },
    }
}

/// Converts `chip-cam`'s [`ResolvedRotaryConfig`] into `chip-stock`'s pure [`RotaryStockSpec`].
pub const fn rotary_stock_spec_from_cam(rotary: ResolvedRotaryConfig) -> RotaryStockSpec {
    RotaryStockSpec {
        axis: axis3_from_rotary_axis(rotary.axis),
        direction_sign: rotary.direction.sign(),
        axis_letter: rotary.axis_letter,
        center_wcs: rotary.center_wcs,
        stock_u_min: rotary.stock_u_min,
        stock_u_max: rotary.stock_u_max,
        stock_outer_radius_mm: rotary.stock_outer_radius_mm,
        chuck_u_min: rotary.chuck_u_min,
        chuck_u_max: rotary.chuck_u_max,
        chuck_radius_mm: rotary.chuck_radius_mm,
    }
}

/// Collects exact analytic horizontal B-Rep planar faces in Setup WCS together with a face-ID
/// handled mask so tessellated `pick_triangles` on those faces can be skipped cleanly.
///
/// When `upward_only` is `true`, only upward-facing planes (`n_wcs.z > 0.999`, `dir_sign = -1`)
/// are collected (2.5D heightfield). When `false`, both `+Z` and `-Z` horizontal planes
/// (`|n_wcs.z| > 0.999`) are collected with signed Z-ray crossing directions (3D Tri-Dexel).
fn collect_analytic_horiz_faces(
    doc: &CadDocument,
    setup: &ResolvedSetup,
    target_bodies: &[BodyId],
    upward_only: bool,
) -> (Vec<AnalyticHorizFace>, Vec<bool>) {
    let include_body = |bid: BodyId| target_bodies.is_empty() || target_bodies.contains(&bid);
    let mut horiz_faces: Vec<AnalyticHorizFace> = Vec::new();
    let mut handled_horiz_face_ids = vec![false; doc.faces.len()];

    for face in &doc.faces {
        if !include_body(face.body_id) {
            continue;
        }
        if let SurfaceKind::Plane { normal, .. } = face.surface {
            let n_wcs = setup.frame.world_to_local_dir(normal.normalize_or_zero());
            let matches_normal = if upward_only {
                n_wcs.z > 0.999
            } else {
                n_wcs.z.abs() > 0.999
            };
            if matches_normal
                && let Some(binding) = doc.bind_face(face.id)
                && let Ok(summary) = extract_selection_features(
                    doc,
                    setup,
                    &[GeometrySelectionItem::PlanarFace { binding }],
                )
            {
                let dir_sign: i8 = if n_wcs.z < 0.0 { 1 } else { -1 };
                for feat in summary.features {
                    if let ExtractedFeature25D::PlanarFace {
                        source_face_id,
                        outer_boundary_wcs,
                        inner_islands_wcs,
                        face_z_wcs,
                        ..
                    } = feat
                        && source_face_id == Some(face.id)
                        && !outer_boundary_wcs.segments.is_empty()
                    {
                        let aabb = outer_boundary_wcs.exact_aabb();
                        if aabb.is_valid() {
                            horiz_faces.push(AnalyticHorizFace {
                                outer: outer_boundary_wcs,
                                inners: inner_islands_wcs,
                                aabb,
                                z_wcs: face_z_wcs,
                                dir_sign,
                            });
                            if (face.id.0 as usize) < handled_horiz_face_ids.len() {
                                handled_horiz_face_ids[face.id.0 as usize] = true;
                            }
                        }
                    }
                }
            }
        }
    }

    (horiz_faces, handled_horiz_face_ids)
}

/// Extracts upward-facing exact analytic horizontal B-Rep faces (`n_wcs.z > 0.999`) and
/// sloped/freeform/mesh target triangles in Setup WCS for [`chip_stock::HeightfieldStock::rasterize_target`].
///
/// Filters both STEP B-Rep faces (`doc.faces`) and imported STL/OBJ/3MF mesh bodies (`doc.meshes`)
/// accurately by `target_bodies` (or includes all bodies when `target_bodies` is empty).
pub fn extract_heightfield_target_from_cad(
    doc: &CadDocument,
    setup: &ResolvedSetup,
    target_bodies: &[BodyId],
) -> (Vec<AnalyticHorizFace>, Vec<TargetTriangle>) {
    let include_body = |bid: BodyId| target_bodies.is_empty() || target_bodies.contains(&bid);
    let (horiz_faces, handled_horiz_face_ids) =
        collect_analytic_horiz_faces(doc, setup, target_bodies, true);

    let mut triangles: Vec<TargetTriangle> = Vec::new();

    // 1. Sloped / conical / toroidal / freeform STEP B-Rep triangles (strictly filtered by body_id)
    for tri in &doc.render_mesh.pick_triangles {
        let fid_idx = tri.face_id.0 as usize;
        if fid_idx < handled_horiz_face_ids.len() && handled_horiz_face_ids[fid_idx] {
            continue;
        }
        let Some(face) = doc.face(tri.face_id) else {
            continue;
        };
        if !include_body(face.body_id) {
            continue;
        }
        let v0 = setup.frame.world_to_local_point(tri.v0_mm);
        let v1 = setup.frame.world_to_local_point(tri.v1_mm);
        let v2 = setup.frame.world_to_local_point(tri.v2_mm);
        let analytic_plane = if let SurfaceKind::Plane { origin_mm, normal } = face.surface {
            let o_wcs = setup.frame.world_to_local_point(origin_mm);
            let n_wcs = setup.frame.world_to_local_dir(normal.normalize_or_zero());
            (n_wcs.z.abs() > 1e-4).then_some((o_wcs, n_wcs))
        } else {
            None
        };
        triangles.push(TargetTriangle {
            v: [v0, v1, v2],
            analytic_plane,
        });
    }

    // 2. Imported triangle mesh bodies (`STL`/`OBJ`/`3MF`), filtered accurately by `mesh.id`
    for mesh in &doc.meshes {
        if !include_body(mesh.id) {
            continue;
        }
        for tri in &mesh.triangles {
            let [i0, i1, i2] = tri.indices;
            let (Some(&p0), Some(&p1), Some(&p2)) = (
                mesh.vertices_mm.get(i0 as usize),
                mesh.vertices_mm.get(i1 as usize),
                mesh.vertices_mm.get(i2 as usize),
            ) else {
                continue;
            };
            triangles.push(TargetTriangle {
                v: [
                    setup.frame.world_to_local_point(p0),
                    setup.frame.world_to_local_point(p1),
                    setup.frame.world_to_local_point(p2),
                ],
                analytic_plane: None,
            });
        }
    }

    (horiz_faces, triangles)
}

/// Voxelizes the targeted 3D CAD solid(s) from `doc` (both STEP B-Rep solids AND imported
/// STL/OBJ/3MF mesh bodies in `doc.meshes`, accurately filtered by `target_bodies`) onto a
/// [`TriDexelStock`] lattice matching `template`.
///
/// Returns `None` if no targeted 3D geometry exists in `doc`.
pub fn extract_tridexel_target_from_cad(
    doc: &CadDocument,
    setup: &ResolvedSetup,
    target_bodies: &[BodyId],
    template: &TriDexelStock,
) -> Option<TriDexelStock> {
    if !doc.has_3d_geometry() {
        return None;
    }
    let include_body = |bid: BodyId| target_bodies.is_empty() || target_bodies.contains(&bid);

    // 1. Collect horizontal B-Rep planar faces (`±Z`) analytically for exact Z-ray hole/island cuts
    let (horiz_faces, handled_horiz_face_ids) =
        collect_analytic_horiz_faces(doc, setup, target_bodies, false);

    let mut tris_wcs_all: Vec<[DVec3; 3]> = Vec::new();
    let mut tris_wcs_z: Vec<[DVec3; 3]> = Vec::new();

    // 2. STEP B-Rep `pick_triangles` (ignore `MESH_FACE_SENTINEL` so `doc.meshes` are filtered below)
    for tri in &doc.render_mesh.pick_triangles {
        let Some(face) = doc.face(tri.face_id) else {
            continue;
        };
        if !include_body(face.body_id) {
            continue;
        }
        let v0 = setup.frame.world_to_local_point(tri.v0_mm);
        let v1 = setup.frame.world_to_local_point(tri.v1_mm);
        let v2 = setup.frame.world_to_local_point(tri.v2_mm);
        if (v1 - v0).cross(v2 - v0).length_squared() > 1e-14 {
            tris_wcs_all.push([v0, v1, v2]);
            let fid_idx = tri.face_id.0 as usize;
            let handled_z =
                fid_idx < handled_horiz_face_ids.len() && handled_horiz_face_ids[fid_idx];
            if !handled_z {
                tris_wcs_z.push([v0, v1, v2]);
            }
        }
    }

    // 3. Imported triangle mesh bodies (`STL`/`OBJ`/`3MF`) filtered accurately by `mesh.id`!
    for mesh in &doc.meshes {
        if !include_body(mesh.id) {
            continue;
        }
        for tri in &mesh.triangles {
            let [i0, i1, i2] = tri.indices;
            let (Some(&p0), Some(&p1), Some(&p2)) = (
                mesh.vertices_mm.get(i0 as usize),
                mesh.vertices_mm.get(i1 as usize),
                mesh.vertices_mm.get(i2 as usize),
            ) else {
                continue;
            };
            let v0 = setup.frame.world_to_local_point(p0);
            let v1 = setup.frame.world_to_local_point(p1);
            let v2 = setup.frame.world_to_local_point(p2);
            if (v1 - v0).cross(v2 - v0).length_squared() > 1e-14 {
                tris_wcs_all.push([v0, v1, v2]);
                tris_wcs_z.push([v0, v1, v2]);
            }
        }
    }

    if tris_wcs_all.is_empty() && horiz_faces.is_empty() {
        return None;
    }

    Some(TriDexelStock::from_target_geometry(
        &tris_wcs_all,
        &tris_wcs_z,
        &horiz_faces,
        template,
    ))
}

/// Builds a WCS-space [`TriMesh3D`] target solid for GPU BVH and cylindrical target rasterization,
/// accurately filtering both `doc.meshes` and B-Rep `doc.faces` by `target_bodies`.
pub fn extract_target_trimesh_from_cad(
    doc: &CadDocument,
    setup: &ResolvedSetup,
    target_bodies: &[BodyId],
) -> Option<TriMesh3D> {
    if !doc.has_3d_geometry() {
        return None;
    }
    let mesh = doc.build_wcs_tri_mesh(target_bodies, &setup.frame);
    (!mesh.is_empty()).then_some(mesh)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cad::{
        BodyId, CadDocument, MESH_FACE_SENTINEL, MeshDiagnostics, MeshFormat, MeshTriangle,
        MeshUnitKind, TriangleMeshBody, build_render_mesh_for_triangle_mesh,
    };
    use chip_cam::{ResolvedSetup, ResolvedStockGeometry};
    use chip_math::{Aabb3, DVec3, RigidFrame3};
    use chip_stock::{HeightfieldStock, SimResolution, TriDexelStock};

    use super::*;

    fn make_box_mesh_body(id: BodyId, name: &str, min: DVec3, max: DVec3) -> TriangleMeshBody {
        let vertices_mm = vec![
            DVec3::new(min.x, min.y, min.z), // 0
            DVec3::new(max.x, min.y, min.z), // 1
            DVec3::new(max.x, max.y, min.z), // 2
            DVec3::new(min.x, max.y, min.z), // 3
            DVec3::new(min.x, min.y, max.z), // 4
            DVec3::new(max.x, min.y, max.z), // 5
            DVec3::new(max.x, max.y, max.z), // 6
            DVec3::new(min.x, max.y, max.z), // 7
        ];
        // Outward-facing CCW triangles for all 6 faces
        let raw_tris: [[u32; 3]; 12] = [
            // Bottom (-Z)
            [0, 2, 1],
            [0, 3, 2],
            // Top (+Z)
            [4, 5, 6],
            [4, 6, 7],
            // Front (-Y)
            [0, 1, 5],
            [0, 5, 4],
            // Back (+Y)
            [3, 7, 6],
            [3, 6, 2],
            // Left (-X)
            [0, 4, 7],
            [0, 7, 3],
            // Right (+X)
            [1, 2, 6],
            [1, 6, 5],
        ];
        let triangles = raw_tris
            .into_iter()
            .map(|indices| {
                let v0 = vertices_mm[indices[0] as usize];
                let v1 = vertices_mm[indices[1] as usize];
                let v2 = vertices_mm[indices[2] as usize];
                MeshTriangle {
                    indices,
                    normal: (v1 - v0).cross(v2 - v0).normalize_or_zero(),
                    aabb: Aabb3::from_points([v0, v1, v2]),
                }
            })
            .collect();
        let size = max - min;
        TriangleMeshBody {
            id,
            name: name.to_string(),
            format: MeshFormat::ThreeMf,
            detected_unit: Some(MeshUnitKind::Millimeter),
            applied_unit: MeshUnitKind::Millimeter,
            vertices_mm,
            triangles,
            aabb_mm: Aabb3::new(min, max),
            diagnostics: MeshDiagnostics {
                raw_triangle_count: 12,
                valid_triangle_count: 12,
                degenerate_triangles_removed: 0,
                welded_vertex_count: 8,
                open_boundary_edges: 0,
                non_manifold_edges: 0,
                is_watertight: true,
                signed_volume_mm3: size.x * size.y * size.z,
            },
        }
    }

    #[test]
    fn multibody_stl_mesh_target_bodies_filter_isolates_selected_mesh_across_tridexel_and_heightfield()
     {
        // Two disjoint 20 × 20 × 10 mm STL/OBJ mesh bodies inside the same CadDocument:
        // - Body 0 (`mesh_left`):  X ∈ [0, 20],  Y ∈ [0, 20], Z ∈ [-10, 0] -> vol = 4,000 mm³
        // - Body 1 (`mesh_right`): X ∈ [30, 50], Y ∈ [0, 20], Z ∈ [-10, 0] -> vol = 4,000 mm³
        let mesh_left = make_box_mesh_body(
            BodyId(0),
            "Left Part",
            DVec3::new(0.0, 0.0, -10.0),
            DVec3::new(20.0, 20.0, 0.0),
        );
        let mesh_right = make_box_mesh_body(
            BodyId(1),
            "Right Part",
            DVec3::new(30.0, 0.0, -10.0),
            DVec3::new(50.0, 20.0, 0.0),
        );

        // Populate BOTH `doc.meshes` AND `doc.render_mesh.pick_triangles` with `MESH_FACE_SENTINEL`
        // (matching how real multi-body STL/OBJ/3MF documents populate the render mesh!).
        let mut render_mesh =
            build_render_mesh_for_triangle_mesh(&mesh_left, 35.0, MESH_FACE_SENTINEL);
        let rm_right = build_render_mesh_for_triangle_mesh(&mesh_right, 35.0, MESH_FACE_SENTINEL);
        let v_off = render_mesh.vertices.len() as u32;
        render_mesh.vertices.extend(rm_right.vertices);
        render_mesh
            .indices
            .extend(rm_right.indices.into_iter().map(|idx| idx + v_off));
        render_mesh.pick_triangles.extend(rm_right.pick_triangles);

        let full_aabb = Aabb3::new(DVec3::new(0.0, 0.0, -10.0), DVec3::new(50.0, 20.0, 0.0));
        let mut doc = CadDocument::empty();
        doc.source_name = "two_meshes.3mf".to_string();
        doc.meshes = vec![mesh_left, mesh_right];
        doc.aabb_mm = full_aabb;
        doc.render_mesh = render_mesh;

        let setup = ResolvedSetup {
            frame: RigidFrame3::IDENTITY,
            model_aabb_wcs: full_aabb,
            stock: ResolvedStockGeometry::Box {
                aabb_wcs: full_aabb,
            },
            rotary: None,
            clearance_z_wcs: 15.0,
            retract_z_wcs: 5.0,
        };

        let stock_spec = stock_shape_spec_from_cam(&setup.stock);
        let td_template =
            TriDexelStock::from_stock_spec(&stock_spec, SimResolution::Standard).unwrap();

        // 1. Empty `target_bodies`: BOTH bodies included (~8,000 mm³)
        let td_all = extract_tridexel_target_from_cad(&doc, &setup, &[], &td_template).unwrap();
        assert!(
            (td_all.compute_volume_mm3() - 8_000.0).abs() < 120.0,
            "all meshes vol expected ~8000, got {:.1}",
            td_all.compute_volume_mm3()
        );
        assert!(td_all.sample_top_z(10.0, 10.0).is_some());
        assert!(td_all.sample_top_z(40.0, 10.0).is_some());

        // 2. Filter to ONLY BodyId(1) (`mesh_right`): ONLY right body included (~4,000 mm³)!
        //    Previously `from_target_cad` leaked BodyId(0) through `MESH_FACE_SENTINEL` pick_triangles!
        let td_right =
            extract_tridexel_target_from_cad(&doc, &setup, &[BodyId(1)], &td_template).unwrap();
        assert!(
            (td_right.compute_volume_mm3() - 4_000.0).abs() < 80.0,
            "filtered right mesh vol expected ~4000, got {:.1}",
            td_right.compute_volume_mm3()
        );
        assert!(
            td_right.sample_top_z(10.0, 10.0).is_none(),
            "excluded BodyId(0) must NOT appear in filtered TriDexel target"
        );
        assert!(
            td_right.sample_top_z(40.0, 10.0).is_some(),
            "selected BodyId(1) must appear in filtered TriDexel target"
        );

        // 3. Heightfield target adapter filters `doc.meshes` identically!
        let mut hf =
            HeightfieldStock::from_stock_spec(&stock_spec, SimResolution::Standard).unwrap();
        let (hf_horiz, hf_tris) = extract_heightfield_target_from_cad(&doc, &setup, &[BodyId(0)]);
        hf.rasterize_target(&hf_horiz, &hf_tris);
        let i_left = ((10.0 - hf.origin_wcs.x) / hf.dx_mm).floor() as usize;
        let i_right = ((40.0 - hf.origin_wcs.x) / hf.dx_mm).floor() as usize;
        let j_mid = ((10.0 - hf.origin_wcs.y) / hf.dy_mm).floor() as usize;
        assert!(
            hf.z_target[j_mid * hf.nx + i_left].is_finite(),
            "selected BodyId(0) must be rasterized into Heightfield z_target"
        );
        assert!(
            !hf.z_target[j_mid * hf.nx + i_right].is_finite(),
            "excluded BodyId(1) must NOT be rasterized into Heightfield z_target"
        );

        // 4. TriMesh3D target adapter filters `doc.meshes` identically!
        let cyl_mesh = extract_target_trimesh_from_cad(&doc, &setup, &[BodyId(1)]).unwrap();
        assert_eq!(cyl_mesh.len(), 12);
    }
}
