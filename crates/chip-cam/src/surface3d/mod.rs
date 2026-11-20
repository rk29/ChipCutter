//! Exact analytical `f64` 3-Axis Surface Machining Kernel (`Drop-Cutter` & `Push-Cutter`).
//!
//! Operates in active Setup WCS coordinates over [`SurfaceMesh3D`], built seamlessly from either
//! imported triangle meshes (`STL`/`OBJ`/`3MF`) or tessellated STEP B-Rep solids.

pub mod cutter;
pub mod drop_cutter;
pub mod levels;
pub mod push_cutter;

use chip_cad::{BodyId, CadDocument, MeshBvh, MeshTriangle, TriangleMeshBody};
use chip_math::{Aabb3, DVec3, RigidFrame3};

pub use cutter::{SurfaceCutterProfile, SurfaceCutterShape};
pub use drop_cutter::{drop_cutter_points_parallel, drop_cutter_z, drop_cutter_z_with_scratch};
pub use levels::{
    clip_polyline_to_region, detect_flat_plateaus, rect_region_2d, resolve_3d_containment_region,
    sample_surface_slope_deg, stock_xy_region,
};
pub use push_cutter::{push_cutter_region_z, slice_surface_contours_z};

/// WCS-space canonical `f64` triangle surface mesh accelerated by a [`MeshBvh`].
#[derive(Debug, Clone)]
pub struct SurfaceMesh3D {
    pub vertices_wcs: Vec<DVec3>,
    pub triangles: Vec<MeshTriangle>,
    pub aabb_wcs: Aabb3,
    pub bvh: MeshBvh,
}

impl SurfaceMesh3D {
    /// Builds a [`SurfaceMesh3D`] from a single [`TriangleMeshBody`] transformed into `setup_frame`.
    pub fn from_triangle_mesh(mesh: &TriangleMeshBody, setup_frame: &RigidFrame3) -> Self {
        let vertices_wcs: Vec<DVec3> = mesh
            .vertices_mm
            .iter()
            .map(|&v| setup_frame.world_to_local_point(v))
            .collect();

        let mut aabb_wcs = Aabb3::EMPTY;
        let mut triangles = Vec::with_capacity(mesh.triangles.len());
        for tri in &mesh.triangles {
            let [i0, i1, i2] = tri.indices;
            let v0 = vertices_wcs[i0 as usize];
            let v1 = vertices_wcs[i1 as usize];
            let v2 = vertices_wcs[i2 as usize];
            let n = (v1 - v0).cross(v2 - v0).normalize_or_zero();
            let tri_aabb = Aabb3::new(v0.min(v1).min(v2), v0.max(v1).max(v2));
            aabb_wcs = aabb_wcs.union(&tri_aabb);
            triangles.push(MeshTriangle {
                indices: tri.indices,
                normal: n,
                aabb: tri_aabb,
            });
        }
        let bvh = MeshBvh::build(&triangles);
        Self {
            vertices_wcs,
            triangles,
            aabb_wcs,
            bvh,
        }
    }

    /// Builds a unified [`SurfaceMesh3D`] in Setup WCS from all targeted bodies (`doc.meshes`
    /// and/or `doc.render_mesh.pick_triangles` for STEP B-Rep solids).
    pub fn from_cad_document(
        doc: &CadDocument,
        target_bodies: &[BodyId],
        setup_frame: &RigidFrame3,
    ) -> Self {
        let include_body = |bid: BodyId| target_bodies.is_empty() || target_bodies.contains(&bid);

        let mut vertices_wcs: Vec<DVec3> = Vec::new();
        let mut triangles: Vec<MeshTriangle> = Vec::new();
        let mut aabb_wcs = Aabb3::EMPTY;

        // 1. Imported triangle meshes (`STL` / `OBJ` / `3MF`)
        for mesh in &doc.meshes {
            if !include_body(mesh.id) {
                continue;
            }
            let base = vertices_wcs.len() as u32;
            vertices_wcs.extend(
                mesh.vertices_mm
                    .iter()
                    .map(|&v| setup_frame.world_to_local_point(v)),
            );
            for tri in &mesh.triangles {
                let i0 = base + tri.indices[0];
                let i1 = base + tri.indices[1];
                let i2 = base + tri.indices[2];
                let v0 = vertices_wcs[i0 as usize];
                let v1 = vertices_wcs[i1 as usize];
                let v2 = vertices_wcs[i2 as usize];
                let n = (v1 - v0).cross(v2 - v0).normalize_or_zero();
                let tri_aabb = Aabb3::new(v0.min(v1).min(v2), v0.max(v1).max(v2));
                aabb_wcs = aabb_wcs.union(&tri_aabb);
                triangles.push(MeshTriangle {
                    indices: [i0, i1, i2],
                    normal: n,
                    aabb: tri_aabb,
                });
            }
        }

        // 2. STEP B-Rep `pick_triangles` (exact `f64` surface tessellation)
        for tri in &doc.render_mesh.pick_triangles {
            let Some(face) = doc.face(tri.face_id) else {
                continue;
            };
            if !include_body(face.body_id) {
                continue;
            }
            let base = vertices_wcs.len() as u32;
            let v0 = setup_frame.world_to_local_point(tri.v0_mm);
            let v1 = setup_frame.world_to_local_point(tri.v1_mm);
            let v2 = setup_frame.world_to_local_point(tri.v2_mm);
            vertices_wcs.push(v0);
            vertices_wcs.push(v1);
            vertices_wcs.push(v2);
            let n = (v1 - v0).cross(v2 - v0).normalize_or_zero();
            let tri_aabb = Aabb3::new(v0.min(v1).min(v2), v0.max(v1).max(v2));
            aabb_wcs = aabb_wcs.union(&tri_aabb);
            triangles.push(MeshTriangle {
                indices: [base, base + 1, base + 2],
                normal: n,
                aabb: tri_aabb,
            });
        }

        let bvh = MeshBvh::build(&triangles);
        Self {
            vertices_wcs,
            triangles,
            aabb_wcs,
            bvh,
        }
    }

    /// Builds a minimal [`SurfaceMesh3D`] directly from raw WCS triangles (useful for analytical unit tests).
    pub fn from_raw_triangles(raw_tris: &[[DVec3; 3]]) -> Self {
        let mut vertices_wcs = Vec::with_capacity(raw_tris.len() * 3);
        let mut triangles = Vec::with_capacity(raw_tris.len());
        let mut aabb_wcs = Aabb3::EMPTY;
        for &[v0, v1, v2] in raw_tris {
            let base = vertices_wcs.len() as u32;
            vertices_wcs.push(v0);
            vertices_wcs.push(v1);
            vertices_wcs.push(v2);
            let n = (v1 - v0).cross(v2 - v0).normalize_or_zero();
            let tri_aabb = Aabb3::new(v0.min(v1).min(v2), v0.max(v1).max(v2));
            aabb_wcs = aabb_wcs.union(&tri_aabb);
            triangles.push(MeshTriangle {
                indices: [base, base + 1, base + 2],
                normal: n,
                aabb: tri_aabb,
            });
        }
        let bvh = MeshBvh::build(&triangles);
        Self {
            vertices_wcs,
            triangles,
            aabb_wcs,
            bvh,
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cad::{MeshImportOptions, load_mesh_cad_document, sample_sculpted_dome_stl_bytes};
    use chip_math::{DVec2, DVec3, DisplayUnit, RigidFrame3, approx_eq};
    use chip_tools::CutterGeometry;

    use crate::{
        BoxAnchor27, RegionSet2D, Setup, WcsOriginMode, WorkOffsetSlot,
        surface3d::{
            SurfaceCutterProfile, SurfaceMesh3D, drop_cutter_points_parallel, drop_cutter_z,
            push_cutter_region_z, slice_surface_contours_z,
        },
    };

    fn make_flat(d: f64) -> CutterGeometry {
        CutterGeometry::FlatEndmill {
            diameter_mm: d,
            flute_length_mm: 24.0,
            overall_length_mm: 60.0,
            shank_diameter_mm: d,
            flutes: 3,
        }
    }

    fn make_ball(d: f64) -> CutterGeometry {
        CutterGeometry::BallEndmill {
            diameter_mm: d,
            flute_length_mm: 24.0,
            overall_length_mm: 60.0,
            shank_diameter_mm: d,
            flutes: 2,
        }
    }

    fn make_bull(d: f64, rc: f64) -> CutterGeometry {
        CutterGeometry::BullNoseEndmill {
            diameter_mm: d,
            corner_radius_mm: rc,
            flute_length_mm: 24.0,
            overall_length_mm: 60.0,
            shank_diameter_mm: d,
            flutes: 3,
        }
    }

    fn make_vbit(d: f64, tip_d: f64, angle_deg: f64) -> CutterGeometry {
        CutterGeometry::ChamferMill {
            diameter_mm: d,
            tip_diameter_mm: tip_d,
            included_angle_deg: angle_deg,
            flute_length_mm: 16.0,
            overall_length_mm: 50.0,
            shank_diameter_mm: d,
            flutes: 2,
        }
    }

    #[test]
    fn drop_cutter_exact_plane_edge_and_vertex_contacts_to_sub_micron() {
        // 45° ramp triangle `z = x` over `[0..20] x [-10..10]`
        let ramp = SurfaceMesh3D::from_raw_triangles(&[
            [
                DVec3::new(0.0, -10.0, 0.0),
                DVec3::new(20.0, -10.0, 20.0),
                DVec3::new(20.0, 10.0, 20.0),
            ],
            [
                DVec3::new(0.0, -10.0, 0.0),
                DVec3::new(20.0, 10.0, 20.0),
                DVec3::new(0.0, 10.0, 0.0),
            ],
        ]);

        let x = 10.0;
        let y = 0.0;

        // 1. Flat Ø6 (R=3): uphill rim touches at x + 3 -> z_tip = 13.0 exact!
        let flat = SurfaceCutterProfile::from_tool_geometry(&make_flat(6.0), 0.0, 0.0);
        let z_flat = drop_cutter_z(&ramp, &flat, x, y, -100.0).unwrap();
        assert!(
            (z_flat - 13.0).abs() < 1e-9,
            "flat on 45° ramp expected 13.0, got {z_flat}"
        );

        // 2. Ball Ø6 (R=3): on 45° ramp (nz = 1/sqrt(2)), z_center = x + R*sqrt(2), so z_tip = x + R*(sqrt(2) - 1)
        let ball = SurfaceCutterProfile::from_tool_geometry(&make_ball(6.0), 0.0, 0.0);
        let expected_ball = x + 3.0 * (2.0_f64.sqrt() - 1.0);
        let z_ball = drop_cutter_z(&ramp, &ball, x, y, -100.0).unwrap();
        assert!(
            (z_ball - expected_ball).abs() < 1e-9,
            "ball on 45° ramp expected {expected_ball}, got {z_ball}"
        );

        // 3. Bull-Nose Ø10, Rc=2 (r_flat=3, rc=2): z_tip = x + r_flat*1 + rc*(sqrt(2) - 1)
        let bull = SurfaceCutterProfile::from_tool_geometry(&make_bull(10.0, 2.0), 0.0, 0.0);
        let expected_bull = x + 3.0 + 2.0 * (2.0_f64.sqrt() - 1.0);
        let z_bull = drop_cutter_z(&ramp, &bull, x, y, -100.0).unwrap();
        assert!(
            (z_bull - expected_bull).abs() < 1e-9,
            "bull-nose on 45° ramp expected {expected_bull}, got {z_bull}"
        );

        // 4. 60° V-Bit (Ø8, tip=0, half_angle=30°, cone slope = cot(30°) = sqrt(3) > 1):
        // Ramp slope (1.0) < cone slope (1.732), so sharp tip contacts ramp directly at `z_tip = 10.0`!
        let v60 = SurfaceCutterProfile::from_tool_geometry(&make_vbit(8.0, 0.0, 60.0), 0.0, 0.0);
        let z_v60 = drop_cutter_z(&ramp, &v60, x, y, -100.0).unwrap();
        assert!(
            (z_v60 - 10.0).abs() < 1e-9,
            "60° V-bit on 45° ramp expected 10.0, got {z_v60}"
        );

        // 5. 120° V-Bit (Ø8, tip=0, half_angle=60°, cone slope = cot(60°) = 1/sqrt(3) < 1):
        // Ramp slope (1.0) > cone slope (0.577), so upper rim `R=4` contacts at `x+4`, `h(4) = 4/sqrt(3)`!
        let v120 = SurfaceCutterProfile::from_tool_geometry(&make_vbit(8.0, 0.0, 120.0), 0.0, 0.0);
        let expected_v120 = (x + 4.0) - 4.0 / 3.0_f64.sqrt();
        let z_v120 = drop_cutter_z(&ramp, &v120, x, y, -100.0).unwrap();
        assert!(
            (z_v120 - expected_v120).abs() < 1e-9,
            "120° V-bit on 45° ramp expected {expected_v120}, got {z_v120}"
        );

        // 6. Pure horizontal ridge edge test at `z = 10` along `(0, 0, 10) -> (10, 0, 10)`
        // Ball R=3 at `(5, 1.5)`: distance to edge `d = 1.5`, exact `z_tip = 10 - (3 - sqrt(9 - 2.25))`!
        let ridge = SurfaceMesh3D::from_raw_triangles(&[[
            DVec3::new(0.0, 0.0, 10.0),
            DVec3::new(10.0, 0.0, 10.0),
            DVec3::new(5.0, 0.0, 0.0),
        ]]);
        let expected_edge_ball = 10.0 - (3.0 - (9.0 - 2.25_f64).sqrt());
        let z_edge = drop_cutter_z(&ridge, &ball, 5.0, 1.5, -100.0).unwrap();
        assert!(
            (z_edge - expected_edge_ball).abs() < 1e-9,
            "ball on horizontal edge expected {expected_edge_ball}, got {z_edge}"
        );

        // Bull-Nose Ø10, Rc=2 (r_flat=3, rc=2) at `(5, 4.0)`: `d = 4.0 = r_flat + 1.0`,
        // exact `z_tip = 10 - (2 - sqrt(4 - 1)) = 8 + sqrt(3)`!
        let expected_edge_bull = 10.0 - (2.0 - 3.0_f64.sqrt());
        let z_edge_bull = drop_cutter_z(&ridge, &bull, 5.0, 4.0, -100.0).unwrap();
        assert!(
            (z_edge_bull - expected_edge_bull).abs() < 1e-8,
            "bull-nose on horizontal edge expected {expected_edge_bull}, got {z_edge_bull}"
        );

        // 7. Pure single-vertex spike test at `(0, 0, 15)`
        let spike = SurfaceMesh3D::from_raw_triangles(&[[
            DVec3::new(0.0, 0.0, 15.0),
            DVec3::new(0.0, 0.0, 0.0),
            DVec3::new(0.1, 0.0, 0.0),
        ]]);
        let z_vtx_ball = drop_cutter_z(&spike, &ball, -2.0, 0.0, -100.0).unwrap();
        let expected_vtx = 15.0 - (3.0 - (9.0 - 4.0_f64).sqrt());
        assert!(
            (z_vtx_ball - expected_vtx).abs() < 1e-9,
            "ball on vertex expected {expected_vtx}, got {z_vtx_ball}"
        );
    }

    #[test]
    fn drop_cutter_and_push_cutter_match_analytical_hemispherical_dome_oracle() {
        let stl = sample_sculpted_dome_stl_bytes();
        let doc = load_mesh_cad_document("sculpted_dome.stl", &stl, &MeshImportOptions::default())
            .unwrap();

        // Also verify Setup Stock & 27-point WCS resolution on imported triangle mesh!
        let mut setup = Setup::new("setup_1", "Dome Setup", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
            anchor: BoxAnchor27::BOTTOM_FRONT_LEFT,
        };
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
        let m_sz = resolved.model_aabb_wcs.size();
        assert!(approx_eq(m_sz.x, 70.0, 1e-3));
        assert!(approx_eq(m_sz.y, 70.0, 1e-3));
        assert!(approx_eq(m_sz.z, 33.0, 1e-3));
        assert!(approx_eq(resolved.model_aabb_wcs.min.z, 0.0, 1e-6));
        assert!(approx_eq(resolved.model_aabb_wcs.max.z, 33.0, 1e-3));

        // Build WCS SurfaceMesh3D (dome R = 25.0 centered at (0, 0, 8.0), pedestal top Z = 8.0)
        let surface = SurfaceMesh3D::from_cad_document(&doc, &[], &RigidFrame3::IDENTITY);
        let r_dome = 25.0f64;
        let z_base = 8.0f64;

        // 1. Ball Endmill Ø6 (R = 3.0) across radial stations from apex to pedestal:
        // Exact analytical formula: for r <= R_dome + R_ball on hemisphere,
        // `z_tip = max(z_base, z_base + sqrt((R_dome + R_ball)^2 - r^2) - R_ball)`.
        let r_ball = 3.0f64;
        let ball = SurfaceCutterProfile::from_tool_geometry(&make_ball(6.0), 0.0, 0.0);
        for &(x, y) in &[
            (0.0_f64, 0.0_f64),
            (5.0, 0.0),
            (10.0, 10.0),
            (18.0, 0.0),
            (24.0, 0.0),
            (27.0, 0.0),
            (31.0, 0.0),
        ] {
            let r = x.hypot(y);
            let sphere_z = if r < r_dome + r_ball {
                z_base + ((r_dome + r_ball).powi(2) - r * r).sqrt() - r_ball
            } else {
                z_base
            };
            let expected = sphere_z.max(z_base);
            let actual = drop_cutter_z(&surface, &ball, x, y, 0.0).unwrap();
            assert!(
                (actual - expected).abs() < 0.05,
                "Ball R=3 at ({x}, {y}): expected {expected:.4}, got {actual:.4} (err={:.4})",
                (actual - expected).abs()
            );
        }

        // 2. Ball Endmill with radial stock 0.5 mm + axial stock 0.25 mm
        // Exact analytical contact on sphere: BullNose(flat=0.5, rc=3.0) + axial 0.25!
        let ball_stock = SurfaceCutterProfile::from_tool_geometry(&make_ball(6.0), 0.5, 0.25);
        // At apex (0,0): must be `33.0 + 0.25 = 33.25` (zero spurious axial shift from radial stock!)
        let z_apex_stock = drop_cutter_z(&surface, &ball_stock, 0.0, 0.0, 0.0).unwrap();
        assert!(
            (z_apex_stock - 33.25).abs() < 0.02,
            "Apex with radial=0.5, axial=0.25 expected 33.25, got {z_apex_stock}"
        );
        // On pedestal at (31, 0): must be `8.0 + 0.25 = 8.25`
        let z_ped_stock = drop_cutter_z(&surface, &ball_stock, 31.0, 0.0, 0.0).unwrap();
        assert!((z_ped_stock - 8.25).abs() < 1e-6);

        // 3. Flat Endmill Ø8 (R = 4.0) on hemisphere:
        // Nearest point on disk to origin is at `r_min = max(0, r - R_flat)`, so
        // `z_tip = max(z_base, z_base + sqrt(R_dome^2 - r_min^2))`!
        let r_flat = 4.0f64;
        let flat = SurfaceCutterProfile::from_tool_geometry(&make_flat(8.0), 0.0, 0.0);
        for &(x, y) in &[
            (0.0_f64, 0.0_f64),
            (10.0, 0.0),
            (20.0, 0.0),
            (28.0, 0.0),
            (31.0, 0.0),
        ] {
            let r = x.hypot(y);
            let r_min = (r - r_flat).max(0.0);
            let expected = if r_min < r_dome {
                z_base + (r_dome * r_dome - r_min * r_min).sqrt()
            } else {
                z_base
            };
            let actual = drop_cutter_z(&surface, &flat, x, y, 0.0).unwrap();
            assert!(
                (actual - expected).abs() < 0.05,
                "Flat R=4 at ({x}, {y}): expected {expected:.4}, got {actual:.4}"
            );
        }

        // 4. Parallel batch Drop-Cutter with 2D circular containment boundary (`R = 15 mm`)
        let bounds = RegionSet2D::from_circle(DVec2::ZERO, 15.0);
        let pts = vec![
            DVec2::new(0.0, 0.0),
            DVec2::new(10.0, 0.0),
            DVec2::new(20.0, 0.0), // outside containment!
        ];
        let zs = drop_cutter_points_parallel(&surface, &ball, &pts, 0.0, Some(&bounds));
        assert!(zs[0].is_some());
        assert!(zs[1].is_some());
        assert!(
            zs[2].is_none(),
            "point outside containment must be filtered out"
        );

        // 5. Push-Cutter / Waterline Slice Oracle on dome at Z = 18.0 mm (dz = 10.0 mm above base):
        let z_slice = 18.0f64;
        let raw_contours = slice_surface_contours_z(&surface, z_slice);
        assert_eq!(raw_contours.len(), 1);
        assert!(raw_contours[0].closed);
        let expected_raw_r = (r_dome * r_dome - (z_slice - z_base).powi(2)).sqrt();
        for seg in &raw_contours[0].segments {
            let r_pt = seg.start().length();
            assert!(
                (r_pt - expected_raw_r).abs() < 0.05,
                "raw slice radius expected {expected_raw_r:.4}, got {r_pt:.4}"
            );
        }

        // Flat Endmill R=4 push-cutter region at Z = 18.0: exact radius = expected_raw_r + 4.0
        let push_flat = push_cutter_region_z(&surface, &flat, z_slice);
        assert_eq!(push_flat.regions.len(), 1);
        let expected_flat_r = expected_raw_r + r_flat;
        let flat_area_r = (push_flat.total_area_mm2() / std::f64::consts::PI).sqrt();
        assert!(
            (flat_area_r - expected_flat_r).abs() < 0.06,
            "Flat push-cutter radius expected {expected_flat_r:.4}, got {flat_area_r:.4}"
        );

        // Ball Endmill R=3 push-cutter region at Z = 18.0:
        // Ball center is at Z = 21.0 (`dz = 13.0` above base), exact contact radius = `sqrt(28^2 - 13^2) = 24.7992`!
        let push_ball = push_cutter_region_z(&surface, &ball, z_slice);
        assert_eq!(push_ball.regions.len(), 1);
        let expected_ball_r =
            ((r_dome + r_ball).powi(2) - (z_slice + r_ball - z_base).powi(2)).sqrt();
        let ball_area_r = (push_ball.total_area_mm2() / std::f64::consts::PI).sqrt();
        assert!(
            (ball_area_r - expected_ball_r).abs() < 0.06,
            "Ball push-cutter radius expected {expected_ball_r:.4}, got {ball_area_r:.4}"
        );
    }
}
