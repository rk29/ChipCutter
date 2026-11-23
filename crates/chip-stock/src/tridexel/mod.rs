//! Full-3D orthogonal **Tri-Dexel** volumetric stock kernel (`TriDexelStock`) with
//! axis-generic [`crate::Axis3`] parameterization and GPU-ready flat fixed-`K` span pools.

pub mod grid;
pub mod mesher;
pub mod sweep;
pub mod transfer;

pub use grid::{
    DexelPlane, DexelSpan, FlatDexelSpan, INLINE_DEXEL_SLOTS, RaySpans, TriDexelStock,
    pack_span_ops, query_overlap_length, query_target_gouge_depth, spans_contain_coord,
    spans_difference_length, subtract_dexel_interval, unpack_max_op, unpack_min_op,
};
pub use mesher::{
    tessellate_tridexel_stock_mesh, tessellate_tridexel_stock_mesh_colored,
    tessellate_tridexel_stock_mesh_with_options,
};

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::{Aabb3, DVec3};

    use crate::types::{Axis3, SimResolution, StockShapeSpec, UNCUT_OP_IDX};

    use super::*;

    #[test]
    fn flat_dexel_span_has_exact_16_byte_pod_gpu_layout() {
        assert_eq!(std::mem::size_of::<FlatDexelSpan>(), 16);
        assert_eq!(std::mem::align_of::<FlatDexelSpan>(), 4);
        assert_eq!(std::mem::offset_of!(FlatDexelSpan, min_local), 0);
        assert_eq!(std::mem::offset_of!(FlatDexelSpan, max_local), 4);
        assert_eq!(std::mem::offset_of!(FlatDexelSpan, op_packed), 8);
        assert_eq!(std::mem::offset_of!(FlatDexelSpan, _pad), 12);

        let packed = pack_span_ops(UNCUT_OP_IDX, 7);
        assert_eq!(unpack_min_op(packed), UNCUT_OP_IDX);
        assert_eq!(unpack_max_op(packed), 7);

        let spans = [
            FlatDexelSpan::from_wcs(DexelSpan::new(10.0, 25.0, 3, UNCUT_OP_IDX), 10.0),
            bytemuck::Zeroable::zeroed(),
        ];
        let bytes: &[u8] = bytemuck::cast_slice(&spans);
        assert_eq!(bytes.len(), 32);
        let roundtrip: &[FlatDexelSpan] = bytemuck::cast_slice(bytes);
        assert_eq!(roundtrip, &spans);
        let decoded = roundtrip[0].to_wcs(10.0);
        assert!((decoded.min_mm - 10.0).abs() < 1e-6);
        assert!((decoded.max_mm - 25.0).abs() < 1e-6);
        assert_eq!(decoded.min_op_idx, 3);
        assert_eq!(decoded.max_op_idx, UNCUT_OP_IDX);
    }

    #[test]
    fn tridexel_box_and_cylinder_volumes_match_analytical_formulas_across_all_3_axes() {
        let box_stock = StockShapeSpec::Box {
            aabb_wcs: Aabb3::new(DVec3::new(0.0, 0.0, -20.0), DVec3::new(80.0, 60.0, 0.0)),
        };
        let td_box = TriDexelStock::from_stock_spec(&box_stock, SimResolution::Standard).unwrap();
        let (vx, vy, vz) = td_box.per_axis_volumes_mm3();
        let expected_box = 80.0 * 60.0 * 20.0;
        for v in [vx, vy, vz, td_box.compute_volume_mm3()] {
            assert!(
                (v - expected_box).abs() < 1e-2,
                "box volume mismatch: got {v}, expected {expected_box}"
            );
        }
        // Zero overflow rays for box stock
        for axis in Axis3::ALL {
            assert_eq!(td_box.plane(axis).overflow_ray_count(), 0);
        }

        let r = 20.0;
        let len = 80.0;
        for cyl_axis in Axis3::ALL {
            let aabb = match cyl_axis {
                Axis3::X => Aabb3::new(DVec3::new(-40.0, -r, -r), DVec3::new(40.0, r, r)),
                Axis3::Y => Aabb3::new(DVec3::new(-r, -40.0, -r), DVec3::new(r, 40.0, r)),
                Axis3::Z => Aabb3::new(DVec3::new(-r, -r, -40.0), DVec3::new(r, r, 40.0)),
            };
            let cyl_stock = StockShapeSpec::Cylinder {
                axis: cyl_axis,
                center_wcs: DVec3::ZERO,
                radius_mm: r,
                min_axial_mm: -40.0,
                max_axial_mm: 40.0,
                aabb_wcs: aabb,
            };
            let td_cyl =
                TriDexelStock::from_stock_spec(&cyl_stock, SimResolution::Standard).unwrap();
            let expected_cyl = std::f64::consts::PI * r * r * len;
            let rel_err = (td_cyl.compute_volume_mm3() - expected_cyl).abs() / expected_cyl;
            assert!(
                rel_err < 0.005,
                "axis {cyl_axis:?} cylinder volume err {:.4}% > 0.5% (got {:.1}, expected {:.1})",
                rel_err * 100.0,
                td_cyl.compute_volume_mm3(),
                expected_cyl
            );
        }
    }

    #[test]
    fn flat_pool_inline_and_overflow_stacked_grooves_preserve_exact_volume_and_mesh_parity() {
        // 20 × 20 × 20 mm block (`X, Y, Z ∈ [0, 20]`), exact volume = 8,000 mm³
        let aabb = Aabb3::new(DVec3::ZERO, DVec3::splat(20.0));
        let mut stock = TriDexelStock::from_stock_spec(
            &StockShapeSpec::Box { aabb_wcs: aabb },
            SimResolution::Standard,
        )
        .unwrap();

        // Carve 6 stacked horizontal grooves (like a 6-tooth multi-form thread mill / stacked cross-grooves)
        // across all Z-rays in the interior X ∈ [5, 15], Y ∈ [5, 15] -> each interior Z-ray splits into
        // 7 disjoint spans (> INLINE_DEXEL_SLOTS = 4), triggering the overflow side-table!
        let grooves: [(f64, f64); 6] = [
            (2.0, 3.0),
            (5.0, 6.0),
            (8.0, 9.0),
            (11.0, 12.0),
            (14.0, 15.0),
            (17.0, 18.0),
        ];

        let mut expected_removed_z_mm3 = 0.0;
        let z_cell_area = stock.dx_mm() * stock.dy_mm();
        let (nx, ny, nz) = (stock.nx(), stock.ny(), stock.nz());

        for j in 0..ny {
            let py = stock.z_plane().center_v(j);
            if !(5.0..=15.0).contains(&py) {
                continue;
            }
            for i in 0..nx {
                let px = stock.z_plane().center_u(i);
                if !(5.0..=15.0).contains(&px) {
                    continue;
                }
                for (g_idx, &(z_lo, z_hi)) in grooves.iter().enumerate() {
                    let rem = stock.plane_mut(Axis3::Z).subtract_ray_interval(
                        i,
                        j,
                        z_lo,
                        z_hi,
                        g_idx as u32,
                    );
                    expected_removed_z_mm3 += rem * z_cell_area;
                }
                // Verify exact 7 spans (> K = 4) and exact endpoints + op tags
                let spans = stock.z_plane().ray_spans(i, j);
                assert_eq!(spans.len(), 7, "6 grooves must split Z-ray into 7 spans");
                assert!((spans[0].min_mm - 0.0).abs() < 1e-5);
                assert!((spans[0].max_mm - 2.0).abs() < 1e-5);
                assert_eq!(spans[0].min_op_idx, UNCUT_OP_IDX);
                assert_eq!(spans[0].max_op_idx, 0);
                assert!((spans[6].min_mm - 18.0).abs() < 1e-5);
                assert!((spans[6].max_mm - 20.0).abs() < 1e-5);
                assert_eq!(spans[6].min_op_idx, 5);
                assert_eq!(spans[6].max_op_idx, UNCUT_OP_IDX);
            }
        }

        // Also carve matching X and Y planes for those 6 grooves so 3-axis majority vote sees all 6 cavities
        for k in 0..nz {
            let pz = stock.x_plane().center_v(k);
            let in_groove = grooves.iter().any(|&(lo, hi)| pz >= lo && pz <= hi);
            if !in_groove {
                continue;
            }
            for j in 0..ny {
                let py = stock.x_plane().center_u(j);
                if (5.0..=15.0).contains(&py) {
                    stock
                        .plane_mut(Axis3::X)
                        .subtract_ray_interval(j, k, 5.0, 15.0, 1);
                }
            }
            for i in 0..nx {
                let px = stock.y_plane().center_u(i);
                if (5.0..=15.0).contains(&px) {
                    stock
                        .plane_mut(Axis3::Y)
                        .subtract_ray_interval(i, k, 5.0, 15.0, 1);
                }
            }
        }

        assert!(
            stock.z_plane().overflow_ray_count() > 100,
            "interior 7-span rays must reside in overflow side-table"
        );
        // X and Y planes only have <= 2 spans per ray -> zero overflow!
        assert_eq!(stock.x_plane().overflow_ray_count(), 0);
        assert_eq!(stock.y_plane().overflow_ray_count(), 0);

        // Exact Z-plane volume matches `8000 - expected_removed_z_mm3`
        let z_vol = stock.z_plane().volume_mm3();
        assert!(
            (z_vol - (8_000.0 - expected_removed_z_mm3)).abs() < 1e-2,
            "z_vol={z_vol:.3} vs expected {:.3}",
            8_000.0 - expected_removed_z_mm3
        );

        // Meshing across overflow + inline rays produces valid watertight outward triangles
        let mesh = tessellate_tridexel_stock_mesh(&stock, true);
        assert!(mesh.vertices.len() > 500);
        assert_eq!(mesh.indices.len() % 3, 0);

        // Now clear out the middle `[1.5, 18.5]` on one overflow ray -> shrinks from 7 spans down to 2 spans!
        // Must transparently migrate BACK from `overflow` into the inline fixed-K pool!
        let mid_i = nx / 2;
        let mid_j = ny / 2;
        let before_ov = stock.z_plane().overflow_ray_count();
        stock
            .plane_mut(Axis3::Z)
            .subtract_ray_interval(mid_i, mid_j, 1.5, 18.5, 2);
        let after_ov = stock.z_plane().overflow_ray_count();
        assert_eq!(
            after_ov,
            before_ov - 1,
            "shrinking <= K spans must migrate ray back out of overflow table"
        );
        let shrunk = stock.z_plane().ray_spans(mid_i, mid_j);
        assert_eq!(shrunk.len(), 2);
        assert!((shrunk[0].max_mm - 1.5).abs() < 1e-5);
        assert!((shrunk[1].min_mm - 18.5).abs() < 1e-5);
    }

    #[test]
    fn rigid_frame_180_flip_transfer_is_exact_zero_blur_and_preserves_op_tags() {
        use chip_math::{Aabb2, DVec2, RigidFrame3};

        // 80 x 60 x 20 mm block in Setup 1 WCS (`X ∈ [-40, 40], Y ∈ [-30, 30], Z ∈ [-20, 0]`)
        let aabb = Aabb3::new(DVec3::new(-40.0, -30.0, -20.0), DVec3::new(40.0, 30.0, 0.0));
        let mut stock1 = TriDexelStock::from_stock_spec(
            &StockShapeSpec::Box { aabb_wcs: aabb },
            SimResolution::Standard,
        )
        .unwrap();

        // Carve an asymmetric L-shaped pocket (`op_idx = 2`) and circular counterbore (`op_idx = 5`)
        let (nx, ny, nz) = (stock1.nx(), stock1.ny(), stock1.nz());
        let in_l_pocket = |x: f64, y: f64| -> bool {
            ((-25.0..=15.0).contains(&x) && (-18.0..=0.0).contains(&y))
                || ((-25.0..=-5.0).contains(&x) && (0.0..=18.0).contains(&y))
        };
        let in_bore = |x: f64, y: f64| -> bool { (x - 20.0).hypot(y - 12.0) <= 8.0 };

        for j in 0..ny {
            let py = stock1.z_plane().center_v(j);
            for i in 0..nx {
                let px = stock1.z_plane().center_u(i);
                if in_l_pocket(px, py) {
                    stock1
                        .plane_mut(Axis3::Z)
                        .subtract_ray_interval(i, j, -7.5, 0.0, 2);
                }
                if in_bore(px, py) {
                    stock1
                        .plane_mut(Axis3::Z)
                        .subtract_ray_interval(i, j, -12.0, 0.0, 5);
                }
            }
        }
        // Mirror the same cuts on X and Y planes so 3-axis majority vote and volumes agree
        for k in 0..nz {
            let pz = stock1.x_plane().center_v(k);
            if pz >= -7.5 {
                for j in 0..ny {
                    let py = stock1.x_plane().center_u(j);
                    if (-18.0..=0.0).contains(&py) {
                        stock1
                            .plane_mut(Axis3::X)
                            .subtract_ray_interval(j, k, -25.0, 15.0, 2);
                    } else if (0.0..=18.0).contains(&py) {
                        stock1
                            .plane_mut(Axis3::X)
                            .subtract_ray_interval(j, k, -25.0, -5.0, 2);
                    }
                }
                for i in 0..nx {
                    let px = stock1.y_plane().center_u(i);
                    if (-25.0..=-5.0).contains(&px) {
                        stock1
                            .plane_mut(Axis3::Y)
                            .subtract_ray_interval(i, k, -18.0, 18.0, 2);
                    } else if (-5.0..=15.0).contains(&px) {
                        stock1
                            .plane_mut(Axis3::Y)
                            .subtract_ray_interval(i, k, -18.0, 0.0, 2);
                    }
                }
            }
            if pz >= -12.0 {
                for j in 0..ny {
                    let py = stock1.x_plane().center_u(j);
                    let dy = py - 12.0;
                    if dy.abs() <= 8.0 {
                        let dx = (64.0 - dy * dy).max(0.0).sqrt();
                        stock1.plane_mut(Axis3::X).subtract_ray_interval(
                            j,
                            k,
                            20.0 - dx,
                            20.0 + dx,
                            5,
                        );
                    }
                }
                for i in 0..nx {
                    let px = stock1.y_plane().center_u(i);
                    let dx = px - 20.0;
                    if dx.abs() <= 8.0 {
                        let dy = (64.0 - dx * dx).max(0.0).sqrt();
                        stock1.plane_mut(Axis3::Y).subtract_ray_interval(
                            i,
                            k,
                            12.0 - dy,
                            12.0 + dy,
                            5,
                        );
                    }
                }
            }
        }

        let v_src = stock1.compute_volume_mm3();
        let src_frame = RigidFrame3::IDENTITY.with_origin(DVec3::new(0.0, 0.0, 20.0));

        // Test both 180° X-flip (X' = +X, Y' = -Y, Z' = -Z) and 180° Y-flip (X' = -X, Y' = +Y, Z' = -Z)
        let x_flip_frame = RigidFrame3 {
            origin_mm: DVec3::new(0.0, 0.0, 0.0),
            x_axis: DVec3::X,
            y_axis: DVec3::NEG_Y,
            z_axis: DVec3::NEG_Z,
        };
        let y_flip_frame = RigidFrame3 {
            origin_mm: DVec3::new(0.0, 0.0, 0.0),
            x_axis: DVec3::NEG_X,
            y_axis: DVec3::Y,
            z_axis: DVec3::NEG_Z,
        };

        for (label, dst_frame, expect_bore_xy) in [
            ("180° X-flip", x_flip_frame, DVec2::new(20.0, -12.0)),
            ("180° Y-flip", y_flip_frame, DVec2::new(-20.0, 12.0)),
        ] {
            let stock2 = stock1
                .transform_to_frame(src_frame, dst_frame, SimResolution::Standard)
                .unwrap();
            let v_dst = stock2.compute_volume_mm3();
            assert!(
                (v_dst - v_src).abs() < 1e-3,
                "{label}: exact volume drift {:.6} mm³ >= 1e-3",
                (v_dst - v_src).abs()
            );

            // Original bore at Z ∈ [-12, 0] in Setup 1 (world Z ∈ [8, 20]) becomes Z ∈ [-20, -8]
            // in Setup 2! So remaining solid at bore center in Setup 2 is Z ∈ [-8, 0] (min_mm = -8,
            // max_mm = 0, min_op_idx = 5, max_op_idx = UNCUT_OP_IDX)!
            let i_b = ((expect_bore_xy.x - stock2.origin_wcs.x) / stock2.dx_mm()).floor() as usize;
            let j_b = ((expect_bore_xy.y - stock2.origin_wcs.y) / stock2.dy_mm()).floor() as usize;
            let spans = stock2.z_plane().ray_spans(i_b, j_b);
            assert_eq!(spans.len(), 1, "{label}: expected 1 span at flipped bore");
            assert!(
                (spans[0].min_mm - (-8.0)).abs() < 1e-4,
                "{label}: expected min_mm = -8.0, got {}",
                spans[0].min_mm
            );
            assert!(
                (spans[0].max_mm - 0.0).abs() < 1e-4,
                "{label}: expected max_mm = 0.0, got {}",
                spans[0].max_mm
            );
            assert_eq!(
                spans[0].min_op_idx, 5,
                "{label}: bore op_idx preserved on flipped bottom"
            );
            assert_eq!(spans[0].max_op_idx, UNCUT_OP_IDX);

            // Pure-CPU rest-stock queries
            assert!(stock2.has_material_in_z_slab(-5.0, 0.0, 0.1));
            assert!(!stock2.has_material_in_z_slab(0.5, 5.0, 0.1));
            let bore_box = Aabb2 {
                min: expect_bore_xy - DVec2::splat(3.0),
                max: expect_bore_xy + DVec2::splat(3.0),
            };
            let max_z = stock2.max_stock_z_in_xy_aabb(bore_box).unwrap();
            assert!((max_z - 0.0).abs() < 1e-4);

            // At Z ∈ [-19, -15] (deep inside Setup 1's machined pocket & bore), the bore is AIR,
            // so extracted material region should have a hole at `expect_bore_xy`!
            let regions = stock2.extract_remaining_material_regions_in_z_slab(-19.0, -15.0, 0.0);
            assert!(
                regions.len() >= 2,
                "{label}: expected outer + hole contours in pocketed Z slab, got {}",
                regions.len()
            );

            // Heightfield projection preserves exact top Z envelope
            let hf2 = crate::HeightfieldStock::from_tridexel_projection(&stock2);
            assert_eq!(hf2.nx, stock2.nx());
            assert_eq!(hf2.ny, stock2.ny());
            let top_at_bore = hf2
                .sample_stock_z(expect_bore_xy.x, expect_bore_xy.y)
                .unwrap();
            assert!((top_at_bore - 0.0).abs() < 1e-4);
        }
    }

    #[test]
    fn rigid_frame_35_deg_oblique_transfer_conserves_volume_within_half_percent() {
        use chip_math::RigidFrame3;

        let aabb = Aabb3::new(DVec3::new(-30.0, -20.0, -15.0), DVec3::new(30.0, 20.0, 0.0));
        let mut stock1 = TriDexelStock::from_stock_spec(
            &StockShapeSpec::Box { aabb_wcs: aabb },
            SimResolution::Standard,
        )
        .unwrap();

        // Carve a central step/pocket (`X ∈ [-10, 10], Y ∈ [-8, 8], Z ∈ [-6, 0]`) across all 3 planes
        let (nx, ny, nz) = (stock1.nx(), stock1.ny(), stock1.nz());
        for j in 0..ny {
            let py = stock1.z_plane().center_v(j);
            if (-8.0..=8.0).contains(&py) {
                for i in 0..nx {
                    let px = stock1.z_plane().center_u(i);
                    if (-10.0..=10.0).contains(&px) {
                        stock1
                            .plane_mut(Axis3::Z)
                            .subtract_ray_interval(i, j, -6.0, 0.0, 1);
                    }
                }
            }
        }
        for k in 0..nz {
            let pz = stock1.x_plane().center_v(k);
            if pz >= -6.0 {
                for j in 0..ny {
                    let py = stock1.x_plane().center_u(j);
                    if (-8.0..=8.0).contains(&py) {
                        stock1
                            .plane_mut(Axis3::X)
                            .subtract_ray_interval(j, k, -10.0, 10.0, 1);
                    }
                }
                for i in 0..nx {
                    let px = stock1.y_plane().center_u(i);
                    if (-10.0..=10.0).contains(&px) {
                        stock1
                            .plane_mut(Axis3::Y)
                            .subtract_ray_interval(i, k, -8.0, 8.0, 1);
                    }
                }
            }
        }

        let v_src = stock1.compute_volume_mm3();
        let src_frame = RigidFrame3::IDENTITY;
        let angle = 35.0_f64.to_radians();
        let tilted_z = DVec3::new(angle.sin(), 0.0, angle.cos());
        let dst_frame = RigidFrame3::from_z_and_x_hint(
            DVec3::new(10.0, -5.0, 4.0),
            tilted_z,
            Some(DVec3::new(angle.cos(), 0.25, -angle.sin())),
            false,
            false,
            0,
        )
        .unwrap();

        let stock2 = stock1
            .transform_to_frame(src_frame, dst_frame, SimResolution::Standard)
            .unwrap();
        let v_dst = stock2.compute_volume_mm3();
        let rel_err = (v_dst - v_src).abs() / v_src;
        assert!(
            rel_err < 0.005,
            "35° oblique BVH transfer volume err {:.3}% >= 0.5% (src={v_src:.1}, dst={v_dst:.1})",
            rel_err * 100.0
        );

        // Verify a point deep inside the solid maps to solid, and the carved pocket center maps to air
        let solid_pt_dst = dst_frame.world_to_local_point(DVec3::new(-22.0, -12.0, -7.5));
        let air_pt_dst = dst_frame.world_to_local_point(DVec3::new(0.0, 0.0, -2.5));
        let solid_i = ((solid_pt_dst.x - stock2.origin_wcs.x) / stock2.dx_mm()).floor() as usize;
        let solid_j = ((solid_pt_dst.y - stock2.origin_wcs.y) / stock2.dy_mm()).floor() as usize;
        let solid_k = ((solid_pt_dst.z - stock2.origin_wcs.z) / stock2.dz_mm()).floor() as usize;
        assert!(
            stock2.is_voxel_solid(solid_i, solid_j, solid_k),
            "interior solid point must remain solid after 35° oblique transfer"
        );
        let air_i = ((air_pt_dst.x - stock2.origin_wcs.x) / stock2.dx_mm()).floor() as usize;
        let air_j = ((air_pt_dst.y - stock2.origin_wcs.y) / stock2.dy_mm()).floor() as usize;
        let air_k = ((air_pt_dst.z - stock2.origin_wcs.z) / stock2.dz_mm()).floor() as usize;
        assert!(
            !stock2.is_voxel_solid(air_i, air_j, air_k),
            "carved pocket center must remain air after 35° oblique transfer"
        );
    }
}
