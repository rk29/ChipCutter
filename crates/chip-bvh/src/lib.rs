//! UI-agnostic flat-array binned-SAH BVH and canonical `f64` triangle/segment spatial indexes
//! (`chip-bvh`).
//!
//! Provides GPU-uploadable 32-byte [`FlatBvhNode`] and 48-byte [`GpuTriangle`] layouts alongside
//! exact `f64` `O(log N)` CPU queries:
//! - Front-to-back raycasting (`raycast`, `raycast_all_hits`)
//! - Exact point-to-triangle distance (`closest_point`)
//! - Drop-Cutter vertical cylinder queries (`query_xy_circle_into`)
//! - Push-Cutter / Waterline Z-slab queries (`query_z_slab_into`)
//! - 3D line-segment picking (`SegmentBvh`)

#![deny(unsafe_code)]

pub mod bvh;
pub mod mesh;
pub mod segment;

pub use bvh::{
    ClosestPointHit, FlatBvh, FlatBvhNode, GpuTriangle, RayHit, clip_triangle_to_z_slab,
    closest_point_on_segment, closest_point_on_triangle, closest_point_on_triangle_at_z,
    ray_triangle_intersect,
};
pub use mesh::{Tri3D, TriMesh3D};
pub use segment::{BvhSegment, SegmentBvh, SegmentClosestHit, SegmentPickHit};

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::f64::consts::TAU;

    use chip_math::{Aabb3, DVec3, Ray3, approx_eq};

    use super::*;

    /// Deterministic PRNG (xorshift64*) so oracle tests are 100% reproducible.
    struct Rng(u64);

    impl Rng {
        fn next_u64(&mut self) -> u64 {
            self.0 ^= self.0 >> 12;
            self.0 ^= self.0 << 25;
            self.0 ^= self.0 >> 27;
            self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
        }

        fn next_f64(&mut self) -> f64 {
            (self.next_u64() >> 11) as f64 / ((1u64 << 53) as f64)
        }

        fn range(&mut self, lo: f64, hi: f64) -> f64 {
            lo + (hi - lo) * self.next_f64()
        }
    }

    fn make_box_mesh(min: DVec3, max: DVec3) -> TriMesh3D {
        let v = vec![
            DVec3::new(min.x, min.y, min.z), // 0
            DVec3::new(max.x, min.y, min.z), // 1
            DVec3::new(max.x, max.y, min.z), // 2
            DVec3::new(min.x, max.y, min.z), // 3
            DVec3::new(min.x, min.y, max.z), // 4
            DVec3::new(max.x, min.y, max.z), // 5
            DVec3::new(max.x, max.y, max.z), // 6
            DVec3::new(min.x, max.y, max.z), // 7
        ];
        let tris = [
            // Bottom (-Z)
            ([0, 2, 1], 1),
            ([0, 3, 2], 1),
            // Top (+Z)
            ([4, 5, 6], 2),
            ([4, 6, 7], 2),
            // Front (-Y)
            ([0, 1, 5], 3),
            ([0, 5, 4], 3),
            // Back (+Y)
            ([3, 7, 6], 4),
            ([3, 6, 2], 4),
            // Left (-X)
            ([0, 4, 7], 5),
            ([0, 7, 3], 5),
            // Right (+X)
            ([1, 2, 6], 6),
            ([1, 6, 5], 6),
        ];
        TriMesh3D::from_indexed_with_faces(v, &tris)
    }

    fn make_cylinder_mesh(radius: f64, z_min: f64, z_max: f64, segs: usize) -> TriMesh3D {
        let mut raw = Vec::with_capacity(segs * 4);
        let c_bot = DVec3::new(0.0, 0.0, z_min);
        let c_top = DVec3::new(0.0, 0.0, z_max);
        for i in 0..segs {
            let a0 = TAU * (i as f64) / (segs as f64);
            let a1 = TAU * ((i + 1) as f64) / (segs as f64);
            let b0 = DVec3::new(radius * a0.cos(), radius * a0.sin(), z_min);
            let b1 = DVec3::new(radius * a1.cos(), radius * a1.sin(), z_min);
            let t0 = DVec3::new(radius * a0.cos(), radius * a0.sin(), z_max);
            let t1 = DVec3::new(radius * a1.cos(), radius * a1.sin(), z_max);
            // Side wall
            raw.push(([b0, b1, t1], 10));
            raw.push(([b0, t1, t0], 10));
            // Caps
            raw.push(([c_bot, b1, b0], 11));
            raw.push(([c_top, t0, t1], 12));
        }
        TriMesh3D::from_raw_triangles_with_faces(&raw)
    }

    fn make_dome_mesh(radius: f64, z_base: f64, rings: usize, sectors: usize) -> TriMesh3D {
        let mut raw = Vec::new();
        for r in 0..rings {
            let phi0 = 0.5 * std::f64::consts::PI * (r as f64) / (rings as f64);
            let phi1 = 0.5 * std::f64::consts::PI * ((r + 1) as f64) / (rings as f64);
            for s in 0..sectors {
                let th0 = TAU * (s as f64) / (sectors as f64);
                let th1 = TAU * ((s + 1) as f64) / (sectors as f64);
                let pt = |phi: f64, th: f64| {
                    let ring_r = radius * phi.cos();
                    DVec3::new(
                        ring_r * th.cos(),
                        ring_r * th.sin(),
                        z_base + radius * phi.sin(),
                    )
                };
                let p00 = pt(phi0, th0);
                let p10 = pt(phi0, th1);
                let p01 = pt(phi1, th0);
                let p11 = pt(phi1, th1);
                raw.push([p00, p10, p11]);
                if r + 1 < rings {
                    raw.push([p00, p11, p01]);
                }
            }
        }
        TriMesh3D::from_raw_triangles(&raw)
    }

    fn make_random_mesh(count: usize, seed: u64) -> TriMesh3D {
        let mut rng = Rng(seed);
        let mut raw = Vec::with_capacity(count);
        for i in 0..count {
            let cx = rng.range(-100.0, 100.0);
            let cy = rng.range(-100.0, 100.0);
            let cz = rng.range(-50.0, 50.0);
            let c = DVec3::new(cx, cy, cz);
            let d0 = DVec3::new(
                rng.range(-8.0, 8.0),
                rng.range(-8.0, 8.0),
                rng.range(-8.0, 8.0),
            );
            let d1 = DVec3::new(
                rng.range(-8.0, 8.0),
                rng.range(-8.0, 8.0),
                rng.range(-8.0, 8.0),
            );
            let d2 = DVec3::new(
                rng.range(-8.0, 8.0),
                rng.range(-8.0, 8.0),
                rng.range(-8.0, 8.0),
            );
            raw.push(([c + d0, c + d1, c + d2], (i % 17) as u32));
        }
        TriMesh3D::from_raw_triangles_with_faces(&raw)
    }

    fn brute_force_raycast(
        mesh: &TriMesh3D,
        origin: DVec3,
        dir: DVec3,
        max_t: f64,
    ) -> Option<RayHit> {
        let mut best: Option<RayHit> = None;
        for (ti, tri) in mesh.triangles.iter().enumerate() {
            let v0 = mesh.vertices[tri.indices[0] as usize];
            let v1 = mesh.vertices[tri.indices[1] as usize];
            let v2 = mesh.vertices[tri.indices[2] as usize];
            if let Some((t, u, v)) = ray_triangle_intersect(origin, dir, v0, v1, v2, max_t)
                && best.is_none_or(|b| t < b.t)
            {
                best = Some(RayHit {
                    tri_index: ti,
                    face_id: tri.face_id,
                    t,
                    point: origin + dir * t,
                    normal: tri.normal,
                    bary_u: u,
                    bary_v: v,
                });
            }
        }
        best
    }

    fn brute_force_raycast_all(
        mesh: &TriMesh3D,
        origin: DVec3,
        dir: DVec3,
        max_t: f64,
    ) -> Vec<RayHit> {
        let mut hits = Vec::new();
        for (ti, tri) in mesh.triangles.iter().enumerate() {
            let v0 = mesh.vertices[tri.indices[0] as usize];
            let v1 = mesh.vertices[tri.indices[1] as usize];
            let v2 = mesh.vertices[tri.indices[2] as usize];
            if let Some((t, u, v)) = ray_triangle_intersect(origin, dir, v0, v1, v2, max_t) {
                hits.push(RayHit {
                    tri_index: ti,
                    face_id: tri.face_id,
                    t,
                    point: origin + dir * t,
                    normal: tri.normal,
                    bary_u: u,
                    bary_v: v,
                });
            }
        }
        hits.sort_by(|a, b| {
            a.t.total_cmp(&b.t)
                .then_with(|| a.tri_index.cmp(&b.tri_index))
        });
        hits
    }

    fn brute_force_closest_point(mesh: &TriMesh3D, point: DVec3) -> Option<ClosestPointHit> {
        let mut best: Option<(f64, ClosestPointHit)> = None;
        for (ti, tri) in mesh.triangles.iter().enumerate() {
            let v0 = mesh.vertices[tri.indices[0] as usize];
            let v1 = mesh.vertices[tri.indices[1] as usize];
            let v2 = mesh.vertices[tri.indices[2] as usize];
            let (q, d2, u, v) = closest_point_on_triangle(point, v0, v1, v2);
            if best.is_none_or(|(bd2, _)| d2 < bd2) {
                best = Some((
                    d2,
                    ClosestPointHit {
                        tri_index: ti,
                        face_id: tri.face_id,
                        point: q,
                        normal: tri.normal,
                        distance: d2.sqrt(),
                        bary_u: u,
                        bary_v: v,
                    },
                ));
            }
        }
        best.map(|(_, h)| h)
    }

    fn verify_mesh_against_oracles(mesh: &TriMesh3D, seed: u64, n_queries: usize) {
        let mut rng = Rng(seed);
        let b = mesh.bounds();
        let pad = b.size() * 0.5 + DVec3::splat(10.0);
        let q_min = b.min - pad;
        let q_max = b.max + pad;

        let mut all_buf = Vec::new();
        let mut circle_buf = Vec::new();
        let mut slab_buf = Vec::new();

        for q in 0..n_queries {
            // 1. Raycast & raycast_all_hits
            let origin = DVec3::new(
                rng.range(q_min.x, q_max.x),
                rng.range(q_min.y, q_max.y),
                rng.range(q_min.z, q_max.z),
            );
            let target = DVec3::new(
                rng.range(b.min.x, b.max.x),
                rng.range(b.min.y, b.max.y),
                rng.range(b.min.z, b.max.z),
            );
            let dir = if q % 7 == 0 {
                // Axis-aligned rays to stress zero-direction slab handling
                match q % 3 {
                    0 => DVec3::NEG_Z,
                    1 => DVec3::X,
                    _ => DVec3::NEG_Y,
                }
            } else {
                (target - origin).normalize_or_zero()
            };
            if dir.length_squared() < 1e-12 {
                continue;
            }
            let max_t = if q % 5 == 0 {
                origin.distance(target) * 0.75
            } else {
                f64::INFINITY
            };

            let bvh_hit = mesh.raycast(origin, dir, max_t);
            let ref_hit = brute_force_raycast(mesh, origin, dir, max_t);
            match (bvh_hit, ref_hit) {
                (Some(a), Some(r)) => {
                    assert!(
                        approx_eq(a.t, r.t, 1e-11),
                        "raycast t mismatch on q={q}: bvh={} ref={}",
                        a.t,
                        r.t
                    );
                }
                (None, None) => {}
                (a, r) => panic!("raycast hit/miss mismatch on q={q}: bvh={a:?} ref={r:?}"),
            }

            mesh.raycast_all_hits(origin, dir, max_t, &mut all_buf);
            let ref_all = brute_force_raycast_all(mesh, origin, dir, max_t);
            assert_eq!(
                all_buf.len(),
                ref_all.len(),
                "raycast_all_hits count mismatch on q={q}"
            );
            for (a, r) in all_buf.iter().zip(ref_all.iter()) {
                assert_eq!(a.tri_index, r.tri_index);
                assert!(approx_eq(a.t, r.t, 1e-12));
            }

            // 2. Closest point
            let pt = DVec3::new(
                rng.range(q_min.x, q_max.x),
                rng.range(q_min.y, q_max.y),
                rng.range(q_min.z, q_max.z),
            );
            let bvh_cp = mesh.closest_point(pt).unwrap();
            let ref_cp = brute_force_closest_point(mesh, pt).unwrap();
            assert!(
                approx_eq(bvh_cp.distance, ref_cp.distance, 1e-11),
                "closest_point distance mismatch on q={q}: bvh={} ref={}",
                bvh_cp.distance,
                ref_cp.distance
            );

            // 3. XY circle query
            let cx = rng.range(q_min.x, q_max.x);
            let cy = rng.range(q_min.y, q_max.y);
            let radius = rng.range(0.0, 25.0);
            mesh.query_xy_circle_into(cx, cy, radius, &mut circle_buf);
            circle_buf.sort_unstable();
            let r_sq = radius * radius;
            let ref_circle: Vec<usize> = mesh
                .triangles
                .iter()
                .enumerate()
                .filter(|(_, tri)| {
                    crate::bvh::xy_circle_intersects_aabb_f64(&tri.aabb, cx, cy, r_sq)
                })
                .map(|(i, _)| i)
                .collect();
            assert_eq!(
                circle_buf, ref_circle,
                "query_xy_circle_into mismatch on q={q}"
            );

            // 4. Z slab query
            let z0 = rng.range(q_min.z, q_max.z);
            let z1 = z0 + rng.range(0.0, 15.0);
            mesh.query_z_slab_into(z0, z1, &mut slab_buf);
            slab_buf.sort_unstable();
            let ref_slab: Vec<usize> = mesh
                .triangles
                .iter()
                .enumerate()
                .filter(|(_, tri)| tri.aabb.min.z <= z1 && tri.aabb.max.z >= z0)
                .map(|(i, _)| i)
                .collect();
            assert_eq!(slab_buf, ref_slab, "query_z_slab_into mismatch on q={q}");
        }
    }

    #[test]
    fn gpu_struct_sizes_and_bytemuck_pod_layouts() {
        assert_eq!(std::mem::size_of::<FlatBvhNode>(), 32);
        assert_eq!(std::mem::align_of::<FlatBvhNode>(), 4);
        assert_eq!(std::mem::size_of::<GpuTriangle>(), 48);
        assert_eq!(std::mem::align_of::<GpuTriangle>(), 4);

        let mesh = make_box_mesh(DVec3::ZERO, DVec3::new(10.0, 20.0, 30.0));
        let node_bytes: &[u8] = bytemuck::cast_slice(mesh.nodes());
        assert_eq!(node_bytes.len(), mesh.nodes().len() * 32);

        let gpu_tris = mesh.gpu_triangles();
        assert_eq!(gpu_tris.len(), 12);
        let tri_bytes: &[u8] = bytemuck::cast_slice(&gpu_tris);
        assert_eq!(tri_bytes.len(), 12 * 48);
    }

    #[test]
    fn conservative_f32_node_bounds_strictly_contain_all_exact_f64_triangles() {
        let mesh = make_random_mesh(500, 0xCAFE_BABE_1234_5678);
        let nodes = mesh.nodes();
        let prims = mesh.prim_indices();

        // Check root contains overall mesh bounds
        let root = &nodes[0];
        let b = mesh.bounds();
        for axis in 0..3 {
            assert!(f64::from(root.aabb_min[axis]) <= b.min[axis]);
            assert!(f64::from(root.aabb_max[axis]) >= b.max[axis]);
        }

        // Check every leaf strictly contains all its exact f64 triangles
        for node in nodes {
            if node.is_leaf() {
                let first = node.left_or_first as usize;
                let end = first + node.tri_count as usize;
                for &pi in &prims[first..end] {
                    let tri = &mesh.triangles[pi as usize];
                    for axis in 0..3 {
                        assert!(f64::from(node.aabb_min[axis]) <= tri.aabb.min[axis]);
                        assert!(f64::from(node.aabb_max[axis]) >= tri.aabb.max[axis]);
                    }
                }
            }
        }
    }

    #[test]
    fn flat_bvh_matches_brute_force_oracles_on_box_cylinder_dome_and_random_meshes() {
        let box_mesh = make_box_mesh(DVec3::new(-10.0, -20.0, 0.0), DVec3::new(30.0, 40.0, 15.0));
        verify_mesh_against_oracles(&box_mesh, 101, 200);

        let cyl_mesh = make_cylinder_mesh(18.5, -25.0, 45.0, 64);
        verify_mesh_against_oracles(&cyl_mesh, 202, 200);

        let dome_mesh = make_dome_mesh(25.0, 8.0, 20, 48);
        verify_mesh_against_oracles(&dome_mesh, 303, 200);

        let rand_mesh = make_random_mesh(1200, 404);
        verify_mesh_against_oracles(&rand_mesh, 505, 250);
    }

    #[test]
    fn parallel_batch_queries_match_serial() {
        let dome = make_dome_mesh(25.0, 8.0, 16, 32);
        let rays: Vec<(DVec3, DVec3, f64)> = (-5..=5)
            .map(|i| {
                (
                    DVec3::new(i as f64 * 4.0, 0.0, 100.0),
                    DVec3::NEG_Z,
                    f64::INFINITY,
                )
            })
            .collect();
        let par_hits = dome.raycast_batch_par(&rays);
        for (ray, par_hit) in rays.iter().zip(par_hits.iter()) {
            let ser_hit = dome.raycast(ray.0, ray.1, ray.2);
            assert_eq!(par_hit, &ser_hit);
        }

        let pts: Vec<DVec3> = (-5..=5)
            .map(|i| DVec3::new(i as f64 * 6.0, 3.0, 40.0))
            .collect();
        let par_cp = dome.closest_points_batch_par(&pts);
        for (&pt, cp) in pts.iter().zip(par_cp.iter()) {
            assert_eq!(cp, &dome.closest_point(pt));
        }
    }

    #[test]
    fn segment_bvh_matches_brute_force_picking_and_closest_point_oracles() {
        let mut rng = Rng(999_888_777);
        let mut segs = Vec::with_capacity(600);
        // Include both random 3D segments and collinear polyline chains
        for i in 0..500 {
            let a = DVec3::new(
                rng.range(-50.0, 50.0),
                rng.range(-50.0, 50.0),
                rng.range(-20.0, 40.0),
            );
            let b = a + DVec3::new(
                rng.range(-12.0, 12.0),
                rng.range(-12.0, 12.0),
                rng.range(-12.0, 12.0),
            );
            segs.push(BvhSegment {
                a,
                b,
                user_id: (i % 40) as u32,
            });
        }
        // Collinear chain along X axis (tests 1D degenerate SAH fallback)
        for i in 0..100 {
            segs.push(BvhSegment {
                a: DVec3::new(i as f64 * 0.5, 0.0, 10.0),
                b: DVec3::new((i + 1) as f64 * 0.5, 0.0, 10.0),
                user_id: 999,
            });
        }

        let seg_bvh = SegmentBvh::build(segs.clone());

        for q in 0..250 {
            let origin = DVec3::new(
                rng.range(-70.0, 70.0),
                rng.range(-70.0, 70.0),
                rng.range(45.0, 90.0),
            );
            let target = DVec3::new(
                rng.range(-50.0, 50.0),
                rng.range(-50.0, 50.0),
                rng.range(-20.0, 40.0),
            );
            let ray = Ray3::new(origin, target - origin).unwrap();
            let radius = rng.range(0.2, 6.0);
            let max_t = if q % 3 == 0 {
                origin.distance(target)
            } else {
                f64::INFINITY
            };

            let bvh_hit = seg_bvh.pick_ray(&ray, radius, max_t);

            // Brute-force oracle
            let mut ref_hit: Option<SegmentPickHit> = None;
            for (idx, seg) in segs.iter().enumerate() {
                let hit = ray.closest_to_segment(seg.a, seg.b);
                if hit.distance_mm <= radius && hit.ray_t <= max_t {
                    let score = hit.distance_mm + hit.ray_t * 1e-3;
                    if ref_hit.is_none_or(|b| score < b.score) {
                        ref_hit = Some(SegmentPickHit {
                            seg_index: idx,
                            user_id: seg.user_id,
                            score,
                            hit,
                        });
                    }
                }
            }

            match (bvh_hit, ref_hit) {
                (Some(a), Some(r)) => {
                    assert!(
                        approx_eq(a.score, r.score, 1e-11),
                        "SegmentBvh pick score mismatch q={q}: bvh={} ref={}",
                        a.score,
                        r.score
                    );
                }
                (None, None) => {}
                (a, r) => panic!("SegmentBvh pick hit/miss mismatch q={q}: bvh={a:?} ref={r:?}"),
            }

            // Closest point oracle
            let pt = DVec3::new(
                rng.range(-60.0, 60.0),
                rng.range(-60.0, 60.0),
                rng.range(-30.0, 50.0),
            );
            let bvh_cp = seg_bvh.closest_point(pt).unwrap();
            let mut ref_d2 = f64::INFINITY;
            for seg in &segs {
                let (_, d2, _) = closest_point_on_segment(pt, seg.a, seg.b);
                if d2 < ref_d2 {
                    ref_d2 = d2;
                }
            }
            assert!(approx_eq(bvh_cp.distance, ref_d2.sqrt(), 1e-11));
        }
    }

    #[test]
    fn empty_and_degenerate_inputs_do_not_panic() {
        let empty = TriMesh3D::default();
        assert!(empty.is_empty());
        assert!(
            empty
                .raycast(DVec3::ZERO, DVec3::Z, f64::INFINITY)
                .is_none()
        );
        assert!(empty.closest_point(DVec3::ZERO).is_none());
        let mut buf = vec![1, 2, 3];
        empty.query_xy_circle_into(0.0, 0.0, 10.0, &mut buf);
        assert!(buf.is_empty());
        empty.query_z_slab_into(0.0, 10.0, &mut buf);
        assert!(buf.is_empty());

        // Single degenerate triangle + NaN triangle
        let degen = TriMesh3D::from_raw_triangles(&[
            [DVec3::ZERO, DVec3::ZERO, DVec3::ZERO],
            [DVec3::NAN, DVec3::X, DVec3::Y],
        ]);
        assert_eq!(degen.len(), 1);
        let cp = degen.closest_point(DVec3::new(3.0, 4.0, 0.0)).unwrap();
        assert!(approx_eq(cp.distance, 5.0, 1e-12));

        // Coincident triangles (> MAX_LEAF_PRIMS)
        let dup_tris = vec![[DVec3::ZERO, DVec3::X, DVec3::Y]; 32];
        let dup = TriMesh3D::from_raw_triangles(&dup_tris);
        let hit = dup
            .raycast(DVec3::new(0.25, 0.25, 5.0), DVec3::NEG_Z, f64::INFINITY)
            .unwrap();
        assert!(approx_eq(hit.t, 5.0, 1e-12));

        let aabb = Aabb3::new(DVec3::splat(-1.0), DVec3::splat(2.0));
        dup.query_aabb_into(&aabb, &mut buf);
        assert_eq!(buf.len(), 32);
    }
}
