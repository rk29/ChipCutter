//! Optional `wgpu` GPU compute volumetric stock simulation backend (`#[cfg(feature = "gpu-compute")]`).

pub mod backend;
pub mod bvh;
pub mod checkpoints;
pub mod context;
pub mod cylindrical;
pub mod heightfield;
pub mod metrics;
pub mod render_handle;
pub mod tridexel;
pub mod types;

pub use backend::WgpuComputeBackend;
pub use bvh::GpuMeshBvh;
pub use checkpoints::{DEFAULT_GPU_CHECKPOINT_BUDGET_BYTES, GpuCheckpointCache};
pub use context::GpuSimContext;
pub use metrics::GpuErrorHistogram;
pub use render_handle::{
    GpuCylindricalRenderData, GpuHeightfieldRenderData, GpuStockRenderHandle,
    GpuTriDexelRenderData, MAX_TRIDEXEL_EMIT_QUADS, MAX_TRIDEXEL_EMIT_VERTICES, PackedTriDexelQuad,
    TRIDEXEL_PACKED_QUAD_BYTES,
};
pub use types::{
    BvhDispatchUniforms, CylindricalUniforms, GPU_METRICS_HIST_BINS, GPU_METRICS_MAX_WORKGROUPS,
    GPU_TILE_SIZE, GpuMetricsPartial, GpuRotaryPose, GpuSweepChord, GpuToolProfile,
    GpuViolationAccum, GpuViolationSlot, HeightfieldUniforms, MetricsUniforms,
    TriDexelPlaneUniforms,
};

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::f64::consts::PI;

    use chip_math::{Aabb3, DVec2, DVec3, DisplayUnit};
    use chip_tools::{CoolantMode, CutterGeometry, SpindleDirection, Tool, ToolId};

    use crate::{
        backend::{CpuExactBackend, SimStockBackend, SimulatedStock, StockCheckpointSnapshot},
        cylindrical::CylindricalStock,
        heightfield::HeightfieldStock,
        types::{
            Axis3, RadialCutterProfile, RotaryStockSpec, SimResolution, StockShapeSpec,
            StockVerifyTolerances, SweepChord, SweepMotion, SweepToolSpec,
        },
    };

    use super::*;

    fn make_tool(id: &str, stickout_mm: f64, geometry: CutterGeometry) -> SweepToolSpec {
        let is_flat_endmill = matches!(geometry, CutterGeometry::FlatEndmill { .. });
        let tool = Tool {
            id: ToolId(id.to_string()),
            tool_number: 1,
            length_offset_number: 1,
            diameter_offset_number: 1,
            name: id.to_string(),
            vendor: None,
            product_id: None,
            native_unit: DisplayUnit::Millimeter,
            spindle_direction: SpindleDirection::Cw,
            default_coolant: CoolantMode::Mist,
            stickout_length_mm: stickout_mm,
            geometry,
            presets: Vec::new(),
        };
        SweepToolSpec {
            profile: RadialCutterProfile::from_tool(&tool),
            is_flat_endmill,
        }
    }

    fn all_seven_tools() -> Vec<SweepToolSpec> {
        vec![
            // 0: FlatEndmill
            make_tool(
                "flat",
                22.0,
                CutterGeometry::FlatEndmill {
                    diameter_mm: 6.0,
                    flute_length_mm: 14.0,
                    overall_length_mm: 50.0,
                    shank_diameter_mm: 6.0,
                    flutes: 3,
                },
            ),
            // 1: BallEndmill
            make_tool(
                "ball",
                24.0,
                CutterGeometry::BallEndmill {
                    diameter_mm: 6.0,
                    flute_length_mm: 12.0,
                    overall_length_mm: 50.0,
                    shank_diameter_mm: 6.0,
                    flutes: 2,
                },
            ),
            // 2: BullNoseEndmill
            make_tool(
                "bullnose",
                24.0,
                CutterGeometry::BullNoseEndmill {
                    diameter_mm: 8.0,
                    corner_radius_mm: 1.5,
                    flute_length_mm: 16.0,
                    overall_length_mm: 55.0,
                    shank_diameter_mm: 8.0,
                    flutes: 4,
                },
            ),
            // 3: ChamferMill / V-Bit
            make_tool(
                "vbit",
                22.0,
                CutterGeometry::ChamferMill {
                    diameter_mm: 10.0,
                    tip_diameter_mm: 0.2,
                    included_angle_deg: 90.0,
                    flute_length_mm: 12.0,
                    overall_length_mm: 50.0,
                    shank_diameter_mm: 10.0,
                    flutes: 2,
                },
            ),
            // 4: TwistDrill
            make_tool(
                "drill",
                32.0,
                CutterGeometry::TwistDrill {
                    diameter_mm: 5.0,
                    point_angle_deg: 118.0,
                    flute_length_mm: 28.0,
                    overall_length_mm: 65.0,
                    shank_diameter_mm: 5.0,
                    flutes: 2,
                },
            ),
            // 5: FaceMill
            make_tool(
                "facemill",
                30.0,
                CutterGeometry::FaceMill {
                    cutting_diameter_mm: 16.0,
                    outside_diameter_mm: 20.0,
                    corner_radius_mm: 1.0,
                    flute_length_mm: 8.0,
                    overall_length_mm: 45.0,
                    shank_diameter_mm: 12.0,
                    flutes: 4,
                },
            ),
            // 6: ThreadMill (multi-form)
            make_tool(
                "threadmill",
                25.0,
                CutterGeometry::ThreadMill {
                    diameter_mm: 6.0,
                    neck_diameter_mm: 3.5,
                    neck_length_mm: 18.0,
                    pitch_mm: 1.25,
                    teeth_count: 4,
                    thread_angle_deg: 60.0,
                    flute_length_mm: 6.0,
                    overall_length_mm: 50.0,
                    shank_diameter_mm: 6.0,
                    flutes: 3,
                },
            ),
        ]
    }

    #[test]
    fn gpu_heightfield_all_seven_cutters_match_cpu_within_tolerance() {
        let ctx = GpuSimContext::shared_headless().expect("expected Vulkan/wgpu compute adapter");
        let stock_spec = StockShapeSpec::Box {
            aabb_wcs: Aabb3::new(DVec3::new(0.0, 0.0, -15.0), DVec3::new(100.0, 80.0, 0.0)),
        };
        let hf = HeightfieldStock::from_stock_spec(&stock_spec, SimResolution::Standard).unwrap();
        let tools = all_seven_tools();
        let tol = StockVerifyTolerances::default();

        let chords = vec![
            // 0: Flat sloped 3D ramp (checks v0.14.1 interval fix #15)
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(10.0, 12.0, -1.0),
                    p1: DVec3::new(28.0, 20.0, -6.5),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 0,
                is_rapid: false,
                profile_idx: 0,
                verify_tol: tol,
            },
            // 0b: Flat closed-form PlanarArcXY
            SweepChord {
                motion: SweepMotion::PlanarArcXY {
                    start: DVec3::new(35.0, 15.0, -4.0),
                    end: DVec3::new(25.0, 25.0, -4.0),
                    center_xy: DVec2::new(25.0, 15.0),
                    radius_mm: 10.0,
                    a0_rad: 0.0,
                    signed_sweep_rad: PI * 0.5,
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 0,
                is_rapid: false,
                profile_idx: 0,
                verify_tol: tol,
            },
            // 1: Ball sloped 3D ramp
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(45.0, 12.0, -2.0),
                    p1: DVec3::new(62.0, 24.0, -7.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 1,
                is_rapid: false,
                profile_idx: 1,
                verify_tol: tol,
            },
            // 2: BullNose horizontal + ramp
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(72.0, 14.0, -3.5),
                    p1: DVec3::new(88.0, 24.0, -6.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 2,
                is_rapid: false,
                profile_idx: 2,
                verify_tol: tol,
            },
            // 3: V-Bit / ChamferMill sloped corner climb (closed-form conical t*)
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(12.0, 42.0, -4.8),
                    p1: DVec3::new(30.0, 52.0, -0.5),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 3,
                is_rapid: false,
                profile_idx: 3,
                verify_tol: tol,
            },
            // 4: TwistDrill through-hole (z_cut <= z_bottom -> -inf empty cells!)
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(48.0, 50.0, 2.0),
                    p1: DVec3::new(48.0, 50.0, -18.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 4,
                is_rapid: false,
                profile_idx: 4,
                verify_tol: tol,
            },
            // 5: FaceMill skim pass
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(62.0, 62.0, -1.8),
                    p1: DVec3::new(86.0, 62.0, -1.8),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 5,
                is_rapid: false,
                profile_idx: 5,
                verify_tol: tol,
            },
            // 6: ThreadMill sweep
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(18.0, 66.0, -8.0),
                    p1: DVec3::new(28.0, 66.0, -6.75),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 6,
                is_rapid: false,
                profile_idx: 6,
                verify_tol: tol,
            },
        ];

        let mut cpu = CpuExactBackend::new(SimulatedStock::Heightfield(hf.clone()), None);
        let mut gpu = WgpuComputeBackend::new(ctx, SimulatedStock::Heightfield(hf), None);
        assert!(gpu.backend_info().is_gpu);

        let cp0 = gpu.capture_checkpoint(0);
        let out_cpu = cpu.sweep_batch(&chords, &tools, true);
        let out_gpu = gpu.sweep_batch(&chords, &tools, true);
        assert_eq!(out_cpu.mutated, out_gpu.mutated);

        let vol_cpu = cpu.compute_volume_mm3();
        let vol_gpu = gpu.compute_volume_mm3();
        let vol_rel_err = (vol_gpu - vol_cpu).abs() / vol_cpu.max(1.0);
        assert!(
            vol_rel_err < 0.0005,
            "Heightfield volume parity failed: cpu={vol_cpu:.4}, gpu={vol_gpu:.4}, rel_err={:.5}%",
            vol_rel_err * 100.0
        );

        let (SimulatedStock::Heightfield(hf_cpu), _) = cpu.into_parts() else {
            panic!("expected heightfield");
        };

        // Test checkpoint restore + re-sweep parity on GPU before consuming
        gpu.reset_from_checkpoint(&cp0);
        let _ = gpu.sweep_batch(&chords, &tools, true);
        let (SimulatedStock::Heightfield(hf_gpu), _) = gpu.into_parts() else {
            panic!("expected heightfield");
        };

        let mut max_delta = 0.0_f64;
        let mut through_count_cpu = 0usize;
        let mut through_count_gpu = 0usize;
        for idx in 0..hf_cpu.z_stock.len() {
            let zc = hf_cpu.z_stock[idx];
            let zg = hf_gpu.z_stock[idx];
            if !zc.is_finite() {
                through_count_cpu += 1;
            }
            if !zg.is_finite() {
                through_count_gpu += 1;
            }
            assert_eq!(
                zc.is_finite(),
                zg.is_finite(),
                "Through-drill empty-cell mismatch at cell {idx}: cpu={zc}, gpu={zg}"
            );
            if zc.is_finite() {
                let d = (f64::from(zg) - f64::from(zc)).abs();
                max_delta = max_delta.max(d);
            }
        }
        assert!(
            through_count_cpu > 50,
            "expected through-drilled empty cells"
        );
        assert_eq!(through_count_cpu, through_count_gpu);
        assert!(
            max_delta <= 2e-3,
            "Heightfield max cell delta {max_delta:.6} mm exceeded 2e-3 mm"
        );
    }

    #[test]
    fn gpu_heightfield_fault_injection_matches_cpu_rules_1_to_4_and_neck() {
        let ctx = GpuSimContext::shared_headless().expect("expected Vulkan/wgpu compute adapter");
        let stock_spec = StockShapeSpec::Box {
            aabb_wcs: Aabb3::new(DVec3::new(0.0, 0.0, -30.0), DVec3::new(80.0, 80.0, 0.0)),
        };
        let mut hf =
            HeightfieldStock::from_stock_spec(&stock_spec, SimResolution::Standard).unwrap();
        // Flat target plane at Z = -5.0 WCS (stock-local Z = 25.0)
        let target_local = hf.z_wcs_to_local(-5.0);
        hf.z_target.fill(target_local);

        // Short-flute / short-stickout flat endmill to trigger flute rub + holder collision
        let short_flat = make_tool(
            "short_flat",
            12.0,
            CutterGeometry::FlatEndmill {
                diameter_mm: 6.0,
                flute_length_mm: 6.0,
                overall_length_mm: 40.0,
                shank_diameter_mm: 6.0,
                flutes: 2,
            },
        );
        // ThreadMill for relieved-neck collision
        let tm = make_tool(
            "tm",
            22.0,
            CutterGeometry::ThreadMill {
                diameter_mm: 6.0,
                neck_diameter_mm: 3.5,
                neck_length_mm: 16.0,
                pitch_mm: 1.0,
                teeth_count: 1,
                thread_angle_deg: 60.0,
                flute_length_mm: 3.0,
                overall_length_mm: 50.0,
                shank_diameter_mm: 6.0,
                flutes: 3,
            },
        );
        let tools = vec![short_flat, tm];
        let tol = StockVerifyTolerances::default();

        let chords = vec![
            // Rapid crash + target gouge + flute rub + holder collision at Z = -18 (stickout=12 -> holder Z=-6 < 0)
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(20.0, 20.0, -18.0),
                    p1: DVec3::new(35.0, 20.0, -18.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 0,
                is_rapid: true,
                profile_idx: 0,
                verify_tol: tol,
            },
            // Non-rapid ThreadMill deep cut triggering relieved-neck collision
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(55.0, 55.0, -10.0),
                    p1: DVec3::new(62.0, 55.0, -10.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 1,
                is_rapid: false,
                profile_idx: 1,
                verify_tol: tol,
            },
        ];

        let mut cpu = CpuExactBackend::new(SimulatedStock::Heightfield(hf.clone()), None);
        let mut gpu = WgpuComputeBackend::new(ctx, SimulatedStock::Heightfield(hf), None);

        let out_cpu = cpu.sweep_batch(&chords, &tools, true);
        let out_gpu = gpu.sweep_batch(&chords, &tools, true);

        for (name, c, g) in [
            ("rapid", out_cpu.violations.rapid, out_gpu.violations.rapid),
            ("gouge", out_cpu.violations.gouge, out_gpu.violations.gouge),
            ("flute", out_cpu.violations.flute, out_gpu.violations.flute),
            ("neck", out_cpu.violations.neck, out_gpu.violations.neck),
            (
                "holder",
                out_cpu.violations.holder,
                out_gpu.violations.holder,
            ),
        ] {
            let (dc, _) = c.unwrap_or_else(|| panic!("CPU should detect {name} violation"));
            let (dg, _) = g.unwrap_or_else(|| panic!("GPU should detect {name} violation"));
            assert!(
                (dg - dc).abs() <= 2e-3,
                "{name} violation depth mismatch: cpu={dc:.5}, gpu={dg:.5}"
            );
        }
    }

    #[test]
    fn gpu_cylindrical_rotary_and_fault_injection_match_cpu_within_tolerance() {
        let ctx = GpuSimContext::shared_headless().expect("expected Vulkan/wgpu compute adapter");
        let rotary = RotaryStockSpec {
            axis: Axis3::X,
            direction_sign: 1.0,
            axis_letter: 'A',
            center_wcs: DVec3::ZERO,
            stock_u_min: -40.0,
            stock_u_max: 40.0,
            stock_outer_radius_mm: 22.0,
            chuck_u_min: -55.0,
            chuck_u_max: -38.0,
            chuck_radius_mm: 32.5,
        };
        let stock_spec = StockShapeSpec::Cylinder {
            axis: Axis3::X,
            center_wcs: DVec3::ZERO,
            radius_mm: 22.0,
            min_axial_mm: -40.0,
            max_axial_mm: 40.0,
            aabb_wcs: Aabb3::new(
                DVec3::new(-40.0, -22.0, -22.0),
                DVec3::new(40.0, 22.0, 22.0),
            ),
        };
        let mut cyl =
            CylindricalStock::from_stock_spec(&stock_spec, rotary, SimResolution::Standard)
                .unwrap();
        // Set target cylinder radius at 15.0 mm
        cyl.r_target.fill(15.0_f32);

        let tools = all_seven_tools();
        let tol = StockVerifyTolerances::default();
        let chords = vec![
            // Turning spiral (flat endmill, 360° wrap)
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(-15.0, 0.0, 18.0),
                    p1: DVec3::new(10.0, 0.0, 18.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 360.0,
                },
                op_idx: 0,
                is_rapid: false,
                profile_idx: 0,
                verify_tol: tol,
            },
            // Off-centerline simultaneous 4-axis helical flute (ball endmill)
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(-10.0, 7.0, 15.5),
                    p1: DVec3::new(15.0, 7.0, 15.5),
                    rotary_deg_0: 15.0,
                    rotary_deg_1: 120.0,
                },
                op_idx: 1,
                is_rapid: false,
                profile_idx: 1,
                verify_tol: tol,
            },
            // Rapid crash + target gouge (r < 15.0)
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(20.0, 0.0, 25.0),
                    p1: DVec3::new(20.0, 0.0, 12.0),
                    rotary_deg_0: 180.0,
                    rotary_deg_1: 180.0,
                },
                op_idx: 2,
                is_rapid: true,
                profile_idx: 0,
                verify_tol: tol,
            },
            // Chuck keep-out violation
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(-42.0, 0.0, 25.0),
                    p1: DVec3::new(-42.0, 0.0, 20.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 3,
                is_rapid: false,
                profile_idx: 0,
                verify_tol: tol,
            },
        ];

        let mut cpu = CpuExactBackend::new(SimulatedStock::Cylindrical(cyl.clone()), None);
        let mut gpu = WgpuComputeBackend::new(ctx, SimulatedStock::Cylindrical(cyl), None);

        let out_cpu = cpu.sweep_batch(&chords, &tools, true);
        let out_gpu = gpu.sweep_batch(&chords, &tools, true);
        assert_eq!(out_cpu.mutated, out_gpu.mutated);

        assert!(out_cpu.violations.rapid.is_some());
        assert!(out_gpu.violations.rapid.is_some());
        assert!(out_cpu.violations.gouge.is_some());
        assert!(out_gpu.violations.gouge.is_some());
        assert!(out_cpu.violations.holder.is_some());
        assert!(out_gpu.violations.holder.is_some());

        let vol_cpu = cpu.compute_volume_mm3();
        let vol_gpu = gpu.compute_volume_mm3();
        let vol_rel_err = (vol_gpu - vol_cpu).abs() / vol_cpu.max(1.0);
        assert!(
            vol_rel_err < 0.001,
            "Cylindrical volume parity failed: cpu={vol_cpu:.4}, gpu={vol_gpu:.4}, rel_err={:.5}%",
            vol_rel_err * 100.0
        );

        let (SimulatedStock::Cylindrical(cyl_cpu), _) = cpu.into_parts() else {
            panic!("expected cylindrical");
        };
        let (SimulatedStock::Cylindrical(cyl_gpu), _) = gpu.into_parts() else {
            panic!("expected cylindrical");
        };

        let mut max_delta = 0.0_f64;
        let mut worst = (0usize, 0usize, 0.0_f32, 0.0_f32, 0u32, 0u32);
        let mut diff_gt_5um = 0usize;
        for idx in 0..cyl_cpu.r_stock.len() {
            let rc = cyl_cpu.r_stock[idx];
            let rg = cyl_gpu.r_stock[idx];
            let d = (f64::from(rg) - f64::from(rc)).abs();
            if d > 5e-3 {
                diff_gt_5um += 1;
            }
            if d > max_delta {
                max_delta = d;
                worst = (
                    idx / cyl_cpu.ntheta,
                    idx % cyl_cpu.ntheta,
                    rc,
                    rg,
                    cyl_cpu.last_op_idx[idx],
                    cyl_gpu.last_op_idx[idx],
                );
            }
        }
        assert!(
            max_delta <= 5e-3,
            "Cylindrical max cell delta {max_delta:.6} mm exceeded 5e-3 mm ({diff_gt_5um} cells > 5um) at (iu={}, it={}): cpu={} (op {}), gpu={} (op {})",
            worst.0,
            worst.1,
            worst.2,
            worst.4,
            worst.3,
            worst.5
        );
    }

    fn make_box_trimesh(min: DVec3, max: DVec3) -> chip_bvh::TriMesh3D {
        let verts = vec![
            DVec3::new(min.x, min.y, min.z),
            DVec3::new(max.x, min.y, min.z),
            DVec3::new(max.x, max.y, min.z),
            DVec3::new(min.x, max.y, min.z),
            DVec3::new(min.x, min.y, max.z),
            DVec3::new(max.x, min.y, max.z),
            DVec3::new(max.x, max.y, max.z),
            DVec3::new(min.x, max.y, max.z),
        ];
        let tris: [[u32; 3]; 12] = [
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [3, 7, 6],
            [3, 6, 2],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
        ];
        chip_bvh::TriMesh3D::from_indexed(verts, &tris)
    }

    fn make_cylinder_trimesh_x(
        u_min: f64,
        u_max: f64,
        radius: f64,
        segs: usize,
    ) -> chip_bvh::TriMesh3D {
        let mut verts = Vec::with_capacity(2 + 2 * segs);
        verts.push(DVec3::new(u_min, 0.0, 0.0)); // 0: min cap center
        verts.push(DVec3::new(u_max, 0.0, 0.0)); // 1: max cap center
        for i in 0..segs {
            let ang = (i as f64 / segs as f64) * std::f64::consts::TAU;
            let (s, c) = ang.sin_cos();
            verts.push(DVec3::new(u_min, radius * s, radius * c));
            verts.push(DVec3::new(u_max, radius * s, radius * c));
        }
        let mut tris = Vec::with_capacity(segs * 4);
        for i in 0..segs {
            let j = (i + 1) % segs;
            let a0 = (2 + 2 * i) as u32;
            let b0 = (2 + 2 * i + 1) as u32;
            let a1 = (2 + 2 * j) as u32;
            let b1 = (2 + 2 * j + 1) as u32;
            // Side quads (outward CCW)
            tris.push([a0, b0, b1]);
            tris.push([a0, b1, a1]);
            // End caps
            tris.push([0, a0, a1]);
            tris.push([1, b1, b0]);
        }
        chip_bvh::TriMesh3D::from_indexed(verts, &tris)
    }

    #[test]
    fn gpu_tridexel_box_and_cylinder_analytical_volumes_across_all_3_axes() {
        let ctx = GpuSimContext::shared_headless().expect("expected Vulkan/wgpu compute adapter");
        let box_spec = StockShapeSpec::Box {
            aabb_wcs: Aabb3::new(DVec3::new(0.0, 0.0, -20.0), DVec3::new(80.0, 60.0, 0.0)),
        };
        let td_box =
            crate::tridexel::TriDexelStock::from_stock_spec(&box_spec, SimResolution::Standard)
                .unwrap();
        let expected_box = 80.0 * 60.0 * 20.0;
        let gpu_box = WgpuComputeBackend::new(
            std::sync::Arc::clone(&ctx),
            SimulatedStock::TriDexel(Box::new(td_box)),
            None,
        );
        assert!(gpu_box.backend_info().is_gpu);
        let vol_gpu = gpu_box.compute_volume_mm3();
        assert!(
            (vol_gpu - expected_box).abs() / expected_box < 1e-4,
            "GPU TriDexel box volume mismatch: gpu={vol_gpu:.3} vs expected={expected_box:.3}"
        );

        let r = 20.0;
        let len = 80.0;
        let expected_cyl = PI * r * r * len;
        for cyl_axis in Axis3::ALL {
            let aabb = match cyl_axis {
                Axis3::X => Aabb3::new(DVec3::new(-40.0, -r, -r), DVec3::new(40.0, r, r)),
                Axis3::Y => Aabb3::new(DVec3::new(-r, -40.0, -r), DVec3::new(r, 40.0, r)),
                Axis3::Z => Aabb3::new(DVec3::new(-r, -r, -40.0), DVec3::new(r, r, 40.0)),
            };
            let cyl_spec = StockShapeSpec::Cylinder {
                axis: cyl_axis,
                center_wcs: DVec3::ZERO,
                radius_mm: r,
                min_axial_mm: -40.0,
                max_axial_mm: 40.0,
                aabb_wcs: aabb,
            };
            let td_cyl =
                crate::tridexel::TriDexelStock::from_stock_spec(&cyl_spec, SimResolution::Standard)
                    .unwrap();
            let vol_cpu = td_cyl.compute_volume_mm3();
            let gpu_cyl = WgpuComputeBackend::new(
                std::sync::Arc::clone(&ctx),
                SimulatedStock::TriDexel(Box::new(td_cyl)),
                None,
            );
            let vol_gpu = gpu_cyl.compute_volume_mm3();
            assert!(
                (vol_gpu - expected_cyl).abs() / expected_cyl < 0.005,
                "GPU TriDexel {cyl_axis:?} cylinder analytical err: gpu={vol_gpu:.2}, exp={expected_cyl:.2}"
            );
            assert!(
                (vol_gpu - vol_cpu).abs() / vol_cpu < 1e-4,
                "GPU vs CPU TriDexel {cyl_axis:?} cylinder volume mismatch: gpu={vol_gpu:.2}, cpu={vol_cpu:.2}"
            );
        }
    }

    #[test]
    fn gpu_tridexel_all_seven_cutters_and_fault_injection_match_cpu() {
        let ctx = GpuSimContext::shared_headless().expect("expected Vulkan/wgpu compute adapter");
        let stock_aabb = Aabb3::new(DVec3::new(0.0, 0.0, -25.0), DVec3::new(100.0, 80.0, 0.0));
        let target_aabb = Aabb3::new(DVec3::new(5.0, 5.0, -22.0), DVec3::new(95.0, 75.0, -8.0));
        let stock_spec = StockShapeSpec::Box {
            aabb_wcs: stock_aabb,
        };
        let td =
            crate::tridexel::TriDexelStock::from_stock_spec(&stock_spec, SimResolution::Standard)
                .unwrap();
        let target_mesh = make_box_trimesh(target_aabb.min, target_aabb.max);
        let raw_tris: Vec<[DVec3; 3]> = (0..target_mesh.len())
            .filter_map(|i| target_mesh.triangle_vertices(i).map(|(a, b, c)| [a, b, c]))
            .collect();
        let target_td =
            crate::tridexel::TriDexelStock::from_target_geometry(&raw_tris, &raw_tris, &[], &td);

        let mut tools = all_seven_tools();
        // Add short-flute flat endmill (idx 7) for flute + holder collision fault injection
        tools.push(make_tool(
            "short_flat",
            10.0,
            CutterGeometry::FlatEndmill {
                diameter_mm: 6.0,
                flute_length_mm: 5.0,
                overall_length_mm: 40.0,
                shank_diameter_mm: 6.0,
                flutes: 2,
            },
        ));
        let tol = StockVerifyTolerances::default();

        let chords = vec![
            // 0: Flat sloped ramp
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(10.0, 12.0, -1.0),
                    p1: DVec3::new(28.0, 20.0, -6.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 0,
                is_rapid: false,
                profile_idx: 0,
                verify_tol: tol,
            },
            // 0b: Flat PlanarArcXY
            SweepChord {
                motion: SweepMotion::PlanarArcXY {
                    start: DVec3::new(35.0, 15.0, -4.0),
                    end: DVec3::new(25.0, 25.0, -4.0),
                    center_xy: DVec2::new(25.0, 15.0),
                    radius_mm: 10.0,
                    a0_rad: 0.0,
                    signed_sweep_rad: PI * 0.5,
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 0,
                is_rapid: false,
                profile_idx: 0,
                verify_tol: tol,
            },
            // 1: Ball sloped ramp
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(45.0, 12.0, -2.0),
                    p1: DVec3::new(62.0, 24.0, -7.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 1,
                is_rapid: false,
                profile_idx: 1,
                verify_tol: tol,
            },
            // 2: BullNose ramp
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(72.0, 14.0, -3.5),
                    p1: DVec3::new(88.0, 24.0, -6.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 2,
                is_rapid: false,
                profile_idx: 2,
                verify_tol: tol,
            },
            // 3: Chamfer / V-Bit sloped climb
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(12.0, 42.0, -4.8),
                    p1: DVec3::new(30.0, 52.0, -0.5),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 3,
                is_rapid: false,
                profile_idx: 3,
                verify_tol: tol,
            },
            // 4: TwistDrill through-hole
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(48.0, 50.0, 2.0),
                    p1: DVec3::new(48.0, 50.0, -26.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 4,
                is_rapid: false,
                profile_idx: 4,
                verify_tol: tol,
            },
            // 5: FaceMill skim
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(62.0, 62.0, -1.8),
                    p1: DVec3::new(86.0, 62.0, -1.8),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 5,
                is_rapid: false,
                profile_idx: 5,
                verify_tol: tol,
            },
            // 6: Multi-form ThreadMill deep cut triggering relieved-neck collision
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(18.0, 66.0, -14.0),
                    p1: DVec3::new(28.0, 66.0, -12.75),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 6,
                is_rapid: false,
                profile_idx: 6,
                verify_tol: tol,
            },
            // 7: Short flat rapid crash + target gouge + holder crash + non-rapid flute rub
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(70.0, 42.0, -15.0),
                    p1: DVec3::new(82.0, 42.0, -15.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 7,
                is_rapid: true,
                profile_idx: 7,
                verify_tol: tol,
            },
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(70.0, 48.0, -15.0),
                    p1: DVec3::new(82.0, 48.0, -15.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 7,
                is_rapid: false,
                profile_idx: 7,
                verify_tol: tol,
            },
        ];

        let initial_vol = td.compute_volume_mm3();
        let mut cpu = CpuExactBackend::new(
            SimulatedStock::TriDexel(Box::new(td.clone())),
            Some(target_td.clone()),
        );
        let mut gpu =
            WgpuComputeBackend::new(ctx, SimulatedStock::TriDexel(Box::new(td)), Some(target_td));

        let cp0 = gpu.capture_checkpoint(0);
        let out_cpu = cpu.sweep_batch(&chords, &tools, true);
        let out_gpu = gpu.sweep_batch(&chords, &tools, true);
        assert_eq!(out_cpu.mutated, out_gpu.mutated);

        // Rules 1-4 + relieved neck must match between CPU and GPU
        for (name, c, g) in [
            ("rapid", out_cpu.violations.rapid, out_gpu.violations.rapid),
            ("gouge", out_cpu.violations.gouge, out_gpu.violations.gouge),
            ("flute", out_cpu.violations.flute, out_gpu.violations.flute),
            ("neck", out_cpu.violations.neck, out_gpu.violations.neck),
            (
                "holder",
                out_cpu.violations.holder,
                out_gpu.violations.holder,
            ),
        ] {
            let (dc, _) = c.unwrap_or_else(|| panic!("CPU should detect {name} violation"));
            let (dg, _) = g.unwrap_or_else(|| panic!("GPU should detect {name} violation"));
            assert!(
                (dg - dc).abs() <= 5e-3,
                "TriDexel {name} violation depth mismatch: cpu={dc:.5}, gpu={dg:.5}"
            );
        }

        // Volume parity < 0.1%
        let vol_cpu = cpu.compute_volume_mm3();
        let vol_gpu = gpu.compute_volume_mm3();
        let vol_rel_err = (vol_gpu - vol_cpu).abs() / vol_cpu.max(1.0);
        assert!(
            vol_rel_err < 0.001,
            "TriDexel volume parity failed: cpu={vol_cpu:.3}, gpu={vol_gpu:.3}, rel_err={:.4}%",
            vol_rel_err * 100.0
        );

        // GPU vs CPU StockErrorReport parity (RMS & MAE within 0.005 mm)
        let rep_cpu = cpu
            .compute_error_report(initial_vol, 0.05, Vec::new())
            .unwrap();
        let rep_gpu = gpu
            .compute_error_report(initial_vol, 0.05, Vec::new())
            .unwrap();
        assert!(
            (rep_gpu.rms_error_mm - rep_cpu.rms_error_mm).abs() < 0.005,
            "TriDexel RMS mismatch: cpu={:.5}, gpu={:.5}",
            rep_cpu.rms_error_mm,
            rep_gpu.rms_error_mm
        );
        assert!(
            (rep_gpu.mean_abs_error_mm - rep_cpu.mean_abs_error_mm).abs() < 0.005,
            "TriDexel MAE mismatch: cpu={:.5}, gpu={:.5}",
            rep_cpu.mean_abs_error_mm,
            rep_gpu.mean_abs_error_mm
        );
        assert!(
            (rep_gpu.in_tolerance_surface_pct - rep_cpu.in_tolerance_surface_pct).abs() < 0.5,
            "TriDexel in_tol mismatch: cpu={:.2}%, gpu={:.2}%",
            rep_cpu.in_tolerance_surface_pct,
            rep_gpu.in_tolerance_surface_pct
        );
        assert!(
            (rep_gpu.max_excess_mm - rep_cpu.max_excess_mm).abs() < 0.01,
            "TriDexel max_excess mismatch: cpu={:.4}, gpu={:.4}",
            rep_cpu.max_excess_mm,
            rep_gpu.max_excess_mm
        );
        assert!(
            (rep_gpu.max_gouge_mm - rep_cpu.max_gouge_mm).abs() < 0.01,
            "TriDexel max_gouge mismatch: cpu={:.4}, gpu={:.4}",
            rep_cpu.max_gouge_mm,
            rep_gpu.max_gouge_mm
        );

        // Verify checkpoint reset + re-sweep yields identical volume
        gpu.reset_from_checkpoint(&cp0);
        let _ = gpu.sweep_batch(&chords, &tools, true);
        let vol_gpu_reswept = gpu.compute_volume_mm3();
        assert!(
            (vol_gpu_reswept - vol_gpu).abs() < 1e-2,
            "TriDexel checkpoint restore mismatch: before={vol_gpu:.3}, after={vol_gpu_reswept:.3}"
        );
    }

    #[test]
    fn gpu_tridexel_overflow_safety_valve_six_stacked_grooves_and_migration_back() {
        let ctx = GpuSimContext::shared_headless().expect("expected Vulkan/wgpu compute adapter");
        let stock_aabb = Aabb3::new(DVec3::new(0.0, 0.0, 0.0), DVec3::new(24.0, 24.0, 24.0));
        let td = crate::tridexel::TriDexelStock::from_stock_spec(
            &StockShapeSpec::Box {
                aabb_wcs: stock_aabb,
            },
            SimResolution::Standard,
        )
        .unwrap();

        // 6-tooth multi-form ThreadMill (teeth_count = 6) carves 6 stacked annular grooves in a
        // single undercut zone -> interior rays split into 7 disjoint Z-spans (> K = 4)!
        let six_tooth_tm = make_tool(
            "tm_6tooth",
            26.0,
            CutterGeometry::ThreadMill {
                diameter_mm: 8.0,
                neck_diameter_mm: 2.0,
                neck_length_mm: 22.0,
                pitch_mm: 2.5,
                teeth_count: 6,
                thread_angle_deg: 60.0,
                flute_length_mm: 15.0,
                overall_length_mm: 60.0,
                shank_diameter_mm: 6.0,
                flutes: 3,
            },
        );
        // Clearing flat endmill to merge/clear the stacked grooves back to <= K = 4 spans
        let clear_flat = make_tool(
            "clear_flat",
            30.0,
            CutterGeometry::FlatEndmill {
                diameter_mm: 14.0,
                flute_length_mm: 26.0,
                overall_length_mm: 60.0,
                shank_diameter_mm: 10.0,
                flutes: 4,
            },
        );
        let tools = vec![six_tooth_tm, clear_flat];
        let tol = StockVerifyTolerances::default();

        // Sweep 6-tooth ThreadMill horizontally across center at Z = 3.0 -> teeth span Z in [3..18],
        // leaving solid material below Z=3 and above Z=18, plus 5 ridges between the 6 grooves -> 7 spans!
        let groove_chords = vec![SweepChord {
            motion: SweepMotion::Linear {
                p0: DVec3::new(8.0, 12.0, 3.0),
                p1: DVec3::new(16.0, 12.0, 3.0),
                rotary_deg_0: 0.0,
                rotary_deg_1: 0.0,
            },
            op_idx: 0,
            is_rapid: false,
            profile_idx: 0,
            verify_tol: tol,
        }];

        let mut cpu = CpuExactBackend::new(SimulatedStock::TriDexel(Box::new(td.clone())), None);
        let mut gpu = WgpuComputeBackend::new(ctx, SimulatedStock::TriDexel(Box::new(td)), None);

        let _ = cpu.sweep_batch(&groove_chords, &tools, false);
        let _ = gpu.sweep_batch(&groove_chords, &tools, false);

        let vol_cpu_ov = cpu.compute_volume_mm3();
        let vol_gpu_ov = gpu.compute_volume_mm3();
        let rel_err_ov = (vol_gpu_ov - vol_cpu_ov).abs() / vol_cpu_ov.max(1.0);
        assert!(
            rel_err_ov < 0.0005,
            "Overflow 7-span volume parity failed: cpu={vol_cpu_ov:.3}, gpu={vol_gpu_ov:.3}, rel_err={:.5}%",
            rel_err_ov * 100.0
        );

        // Inspect checkpoint snapshot to verify overflow_ray_count > 0 and exact 7-span rays on GPU!
        let cp_ov = gpu.capture_checkpoint(1);
        let StockCheckpointSnapshot::TriDexel { stock: td_ov, .. } = &cp_ov else {
            panic!("expected TriDexel checkpoint");
        };
        let z_ov_count = td_ov.z_plane().overflow_ray_count();
        assert!(
            z_ov_count > 20,
            "expected > 20 overflow Z-rays (> K = 4 spans), got {z_ov_count}"
        );
        let max_spans = td_ov
            .z_plane()
            .overflow
            .values()
            .map(Vec::len)
            .max()
            .unwrap_or(0);
        assert_eq!(
            max_spans, 7,
            "6 stacked grooves must produce exact 7-span rays with zero dropped spans"
        );

        // Now sweep clearing flat endmill across the entire grooved zone -> shrinks all overflow
        // rays from 7 spans down to <= 2 spans, cleanly migrating them BACK into GPU inline slots!
        let merge_chords = vec![SweepChord {
            motion: SweepMotion::Linear {
                p0: DVec3::new(6.0, 12.0, 2.0),
                p1: DVec3::new(18.0, 12.0, 2.0),
                rotary_deg_0: 0.0,
                rotary_deg_1: 0.0,
            },
            op_idx: 1,
            is_rapid: false,
            profile_idx: 1,
            verify_tol: tol,
        }];
        let _ = cpu.sweep_batch(&merge_chords, &tools, false);
        let _ = gpu.sweep_batch(&merge_chords, &tools, false);

        let vol_cpu_after = cpu.compute_volume_mm3();
        let vol_gpu_after = gpu.compute_volume_mm3();
        let rel_err_after = (vol_gpu_after - vol_cpu_after).abs() / vol_cpu_after.max(1.0);
        assert!(
            rel_err_after < 0.0005,
            "Post-migration volume parity failed: cpu={vol_cpu_after:.3}, gpu={vol_gpu_after:.3}"
        );

        let (SimulatedStock::TriDexel(final_td_gpu), _) = gpu.into_parts() else {
            panic!("expected TriDexel");
        };
        assert_eq!(
            final_td_gpu.z_plane().overflow_ray_count(),
            0,
            "all overflow rays must cleanly migrate back to GPU inline pool once <= K spans"
        );
    }

    #[test]
    fn gpu_bvh_target_rasterization_and_closest_point_match_cpu() {
        let ctx = GpuSimContext::shared_headless().expect("expected Vulkan/wgpu compute adapter");

        // 1. Tri-Dexel GPU BVH voxelization vs CPU `from_target_geometry`
        let stock_aabb = Aabb3::new(DVec3::new(0.0, 0.0, -15.0), DVec3::new(50.0, 40.0, 0.0));
        let target_aabb = Aabb3::new(DVec3::new(5.0, 5.0, -12.0), DVec3::new(45.0, 35.0, -2.0));
        let box_mesh = make_box_trimesh(target_aabb.min, target_aabb.max);
        let raw_tris: Vec<[DVec3; 3]> = (0..box_mesh.len())
            .filter_map(|i| box_mesh.triangle_vertices(i).map(|(a, b, c)| [a, b, c]))
            .collect();

        let td_template = crate::tridexel::TriDexelStock::from_stock_spec(
            &StockShapeSpec::Box {
                aabb_wcs: stock_aabb,
            },
            SimResolution::Standard,
        )
        .unwrap();
        let td_cpu = crate::tridexel::TriDexelStock::from_target_geometry(
            &raw_tris,
            &raw_tris,
            &[],
            &td_template,
        );
        let td_gpu = ctx.rasterize_tridexel_target_mesh(&td_template, &box_mesh);
        let vol_expected = 40.0 * 30.0 * 10.0;
        let vol_cpu = td_cpu.compute_volume_mm3();
        let vol_gpu = td_gpu.compute_volume_mm3();
        assert!(
            (vol_gpu - vol_expected).abs() / vol_expected < 0.01,
            "GPU BVH TriDexel vol={vol_gpu:.2} vs analytical={vol_expected:.2}"
        );
        assert!(
            (vol_gpu - vol_cpu).abs() / vol_cpu < 0.005,
            "GPU vs CPU BVH TriDexel vol mismatch: gpu={vol_gpu:.2}, cpu={vol_cpu:.2}"
        );

        // 2. Cylindrical GPU BVH inward radial raycast vs CPU `rasterize_target_mesh`
        let rotary = RotaryStockSpec {
            axis: Axis3::X,
            direction_sign: 1.0,
            axis_letter: 'A',
            center_wcs: DVec3::ZERO,
            stock_u_min: -30.0,
            stock_u_max: 30.0,
            stock_outer_radius_mm: 22.0,
            chuck_u_min: -45.0,
            chuck_u_max: -30.0,
            chuck_radius_mm: 30.0,
        };
        let cyl_spec = StockShapeSpec::Cylinder {
            axis: Axis3::X,
            center_wcs: DVec3::ZERO,
            radius_mm: 22.0,
            min_axial_mm: -30.0,
            max_axial_mm: 30.0,
            aabb_wcs: Aabb3::new(
                DVec3::new(-30.0, -22.0, -22.0),
                DVec3::new(30.0, 22.0, 22.0),
            ),
        };
        let cyl_mesh = make_cylinder_trimesh_x(-20.0, 20.0, 15.0, 48);
        let mut cyl_cpu =
            CylindricalStock::from_stock_spec(&cyl_spec, rotary, SimResolution::Standard).unwrap();
        let mut cyl_gpu = cyl_cpu.clone();
        cyl_cpu.rasterize_target_mesh(&cyl_mesh);
        ctx.rasterize_cylindrical_target_mesh(&mut cyl_gpu, &cyl_mesh);

        let mut cyl_max_diff = 0.0_f64;
        let mut cyl_hits_both = 0usize;
        let mut cyl_hit_mismatch = 0usize;
        for (a, b) in cyl_cpu.r_target.iter().zip(cyl_gpu.r_target.iter()) {
            if (*a >= 0.0) != (*b >= 0.0) {
                cyl_hit_mismatch += 1;
            } else if *a >= 0.0 {
                cyl_hits_both += 1;
                cyl_max_diff = cyl_max_diff.max((f64::from(*a) - f64::from(*b)).abs());
            }
        }
        assert!(cyl_hits_both > 50_000);
        assert!(
            (cyl_hit_mismatch as f64) / (cyl_cpu.r_target.len() as f64) < 0.002,
            "GPU vs CPU BVH Cylindrical hit/miss mismatch too high: {cyl_hit_mismatch} / {}",
            cyl_cpu.r_target.len()
        );
        assert!(
            cyl_max_diff < 2e-3,
            "GPU BVH Cylindrical r_target max diff {cyl_max_diff:.6} mm > 2e-3 mm"
        );

        // 3. Heightfield GPU BVH top-down raycast vs CPU `rasterize_target`
        let mut hf_cpu = HeightfieldStock::from_stock_spec(
            &StockShapeSpec::Box {
                aabb_wcs: stock_aabb,
            },
            SimResolution::Standard,
        )
        .unwrap();
        let mut hf_gpu = hf_cpu.clone();
        let target_tris: Vec<crate::types::TargetTriangle> = raw_tris
            .iter()
            .map(|&v| crate::types::TargetTriangle {
                v,
                analytic_plane: None,
            })
            .collect();
        hf_cpu.rasterize_target(&[], &target_tris);
        ctx.rasterize_heightfield_target_mesh(&mut hf_gpu, &box_mesh);

        let mut hf_matches = 0usize;
        let mut hf_max_diff = 0.0_f64;
        for (a, b) in hf_cpu.z_target.iter().zip(hf_gpu.z_target.iter()) {
            if a.is_finite() && b.is_finite() {
                hf_matches += 1;
                hf_max_diff = hf_max_diff.max((f64::from(*a) - f64::from(*b)).abs());
            }
        }
        assert!(hf_matches > 10_000);
        assert!(
            hf_max_diff < 2e-3,
            "GPU BVH Heightfield z_target max diff {hf_max_diff:.6} mm > 2e-3 mm"
        );

        // 4. GPU point-to-mesh closest-point distance vs CPU `TriMesh3D::closest_point`
        let queries = vec![
            DVec3::new(0.0, 20.0, -7.0),  // 5 mm outside -X face
            DVec3::new(25.0, 20.0, 3.0),  // 5 mm above +Z face
            DVec3::new(50.0, 40.0, 0.0),  // outside corner
            DVec3::new(25.0, 20.0, -7.0), // interior point
            DVec3::new(15.0, 15.0, -2.0), // on +Z face (dist = 0)
        ];
        let gpu_closest = ctx.query_mesh_closest_points(&box_mesh, &queries);
        for (q, g_res) in queries.iter().zip(gpu_closest.iter()) {
            let c_hit = box_mesh.closest_point(*q).unwrap();
            let (g_pt, g_dist) = g_res.expect("expected GPU closest point hit");
            assert!(
                (g_dist - c_hit.distance).abs() < 1e-3,
                "closest distance mismatch at {q:?}: cpu={:.5}, gpu={:.5}",
                c_hit.distance,
                g_dist
            );
            assert!(
                (g_pt - c_hit.point).length() < 1e-3,
                "closest point mismatch at {q:?}: cpu={:?}, gpu={:?}",
                c_hit.point,
                g_pt
            );
        }
    }

    #[test]
    fn gpu_parallel_reduction_error_report_and_quick_summary_match_cpu_all_three_engines() {
        let ctx = GpuSimContext::shared_headless().expect("expected Vulkan/wgpu compute adapter");
        let tools = all_seven_tools();
        let tol = StockVerifyTolerances::default();

        // === 1. Heightfield GPU vs CPU metrology ===
        {
            let stock_aabb = Aabb3::new(DVec3::new(0.0, 0.0, -15.0), DVec3::new(60.0, 60.0, 0.0));
            let mut hf = HeightfieldStock::from_stock_spec(
                &StockShapeSpec::Box {
                    aabb_wcs: stock_aabb,
                },
                SimResolution::Standard,
            )
            .unwrap();
            let target_mesh =
                make_box_trimesh(DVec3::new(5.0, 5.0, -15.0), DVec3::new(55.0, 55.0, -4.0));
            ctx.rasterize_heightfield_target_mesh(&mut hf, &target_mesh);
            let init_vol = hf.compute_volume_mm3();

            // Cut with both excess (uncut areas) and a deliberate 1.5 mm gouge down to Z = -5.5
            let chords = vec![
                SweepChord {
                    motion: SweepMotion::Linear {
                        p0: DVec3::new(10.0, 20.0, -3.5),
                        p1: DVec3::new(50.0, 20.0, -3.5),
                        rotary_deg_0: 0.0,
                        rotary_deg_1: 0.0,
                    },
                    op_idx: 0,
                    is_rapid: false,
                    profile_idx: 5,
                    verify_tol: tol,
                },
                SweepChord {
                    motion: SweepMotion::Linear {
                        p0: DVec3::new(20.0, 40.0, -5.5),
                        p1: DVec3::new(40.0, 40.0, -5.5),
                        rotary_deg_0: 0.0,
                        rotary_deg_1: 0.0,
                    },
                    op_idx: 1,
                    is_rapid: false,
                    profile_idx: 0,
                    verify_tol: tol,
                },
            ];
            let mut cpu = CpuExactBackend::new(SimulatedStock::Heightfield(hf.clone()), None);
            let mut gpu = WgpuComputeBackend::new(
                std::sync::Arc::clone(&ctx),
                SimulatedStock::Heightfield(hf),
                None,
            );
            let _ = cpu.sweep_batch(&chords, &tools, false);
            let _ = gpu.sweep_batch(&chords, &tools, false);

            let (ce, cg, crms) = cpu.compute_quick_error_summary().unwrap();
            let (ge, gg, grms) = gpu.compute_quick_error_summary().unwrap();
            assert!((ge - ce).abs() / ce.max(1.0) < 0.002);
            assert!((gg - cg).abs() / cg.max(1.0) < 0.005);
            assert!((grms - crms).abs() < 0.005);

            let rc = cpu
                .compute_error_report(init_vol, 0.05, Vec::new())
                .unwrap();
            let rg = gpu
                .compute_error_report(init_vol, 0.05, Vec::new())
                .unwrap();
            assert!((rg.rms_error_mm - rc.rms_error_mm).abs() < 0.005);
            assert!((rg.mean_abs_error_mm - rc.mean_abs_error_mm).abs() < 0.005);
            assert!((rg.mean_signed_error_mm - rc.mean_signed_error_mm).abs() < 0.005);
            assert!((rg.max_excess_mm - rc.max_excess_mm).abs() < 0.005);
            assert!((rg.max_gouge_mm - rc.max_gouge_mm).abs() < 0.005);
            assert!((rg.in_tolerance_surface_pct - rc.in_tolerance_surface_pct).abs() < 0.2);
        }

        // === 2. Cylindrical GPU vs CPU metrology ===
        {
            let rotary = RotaryStockSpec {
                axis: Axis3::X,
                direction_sign: 1.0,
                axis_letter: 'A',
                center_wcs: DVec3::ZERO,
                stock_u_min: -30.0,
                stock_u_max: 30.0,
                stock_outer_radius_mm: 20.0,
                chuck_u_min: -45.0,
                chuck_u_max: -30.0,
                chuck_radius_mm: 30.0,
            };
            let cyl_spec = StockShapeSpec::Cylinder {
                axis: Axis3::X,
                center_wcs: DVec3::ZERO,
                radius_mm: 20.0,
                min_axial_mm: -30.0,
                max_axial_mm: 30.0,
                aabb_wcs: Aabb3::new(
                    DVec3::new(-30.0, -20.0, -20.0),
                    DVec3::new(30.0, 20.0, 20.0),
                ),
            };
            let mut cyl =
                CylindricalStock::from_stock_spec(&cyl_spec, rotary, SimResolution::Standard)
                    .unwrap();
            cyl.r_target.fill(16.0);
            let init_vol = cyl.current_volume_mm3();

            let chords = vec![
                SweepChord {
                    motion: SweepMotion::Linear {
                        p0: DVec3::new(-15.0, 0.0, 17.0),
                        p1: DVec3::new(5.0, 0.0, 17.0),
                        rotary_deg_0: 0.0,
                        rotary_deg_1: 360.0,
                    },
                    op_idx: 0,
                    is_rapid: false,
                    profile_idx: 0,
                    verify_tol: tol,
                },
                SweepChord {
                    motion: SweepMotion::Linear {
                        p0: DVec3::new(10.0, 0.0, 14.5),
                        p1: DVec3::new(18.0, 0.0, 14.5),
                        rotary_deg_0: 0.0,
                        rotary_deg_1: 180.0,
                    },
                    op_idx: 1,
                    is_rapid: false,
                    profile_idx: 0,
                    verify_tol: tol,
                },
            ];
            let mut cpu = CpuExactBackend::new(SimulatedStock::Cylindrical(cyl.clone()), None);
            let mut gpu = WgpuComputeBackend::new(
                std::sync::Arc::clone(&ctx),
                SimulatedStock::Cylindrical(cyl),
                None,
            );
            let _ = cpu.sweep_batch(&chords, &tools, false);
            let _ = gpu.sweep_batch(&chords, &tools, false);

            let (ce, cg, crms) = cpu.compute_quick_error_summary().unwrap();
            let (ge, gg, grms) = gpu.compute_quick_error_summary().unwrap();
            assert!((ge - ce).abs() / ce.max(1.0) < 0.005);
            assert!((gg - cg).abs() / cg.max(1.0) < 0.01);
            assert!((grms - crms).abs() < 0.005);

            let rc = cpu
                .compute_error_report(init_vol, 0.05, Vec::new())
                .unwrap();
            let rg = gpu
                .compute_error_report(init_vol, 0.05, Vec::new())
                .unwrap();
            assert!((rg.rms_error_mm - rc.rms_error_mm).abs() < 0.005);
            assert!((rg.mean_abs_error_mm - rc.mean_abs_error_mm).abs() < 0.005);
            assert!((rg.max_excess_mm - rc.max_excess_mm).abs() < 0.005);
            assert!((rg.max_gouge_mm - rc.max_gouge_mm).abs() < 0.005);
            assert!((rg.in_tolerance_surface_pct - rc.in_tolerance_surface_pct).abs() < 0.2);
        }

        // === 3. Tri-Dexel GPU vs CPU metrology (including rasterize_target_mesh_gpu) ===
        {
            let stock_aabb = Aabb3::new(DVec3::ZERO, DVec3::new(42.0, 32.0, 12.0));
            let target_mesh = make_box_trimesh(DVec3::splat(1.0), DVec3::new(41.0, 31.0, 11.0));
            let td = crate::tridexel::TriDexelStock::from_stock_spec(
                &StockShapeSpec::Box {
                    aabb_wcs: stock_aabb,
                },
                SimResolution::Standard,
            )
            .unwrap();
            let init_vol = td.compute_volume_mm3();

            let mut gpu = WgpuComputeBackend::new(
                std::sync::Arc::clone(&ctx),
                SimulatedStock::TriDexel(Box::new(td.clone())),
                None,
            );
            gpu.rasterize_target_mesh_gpu(&target_mesh);
            let target_for_cpu = ctx.rasterize_tridexel_target_mesh(&td, &target_mesh);
            let cpu =
                CpuExactBackend::new(SimulatedStock::TriDexel(Box::new(td)), Some(target_for_cpu));

            let rc = cpu
                .compute_error_report(init_vol, 0.05, Vec::new())
                .unwrap();
            let rg = gpu
                .compute_error_report(init_vol, 0.05, Vec::new())
                .unwrap();
            assert!(
                (rg.rms_error_mm - rc.rms_error_mm).abs() < 0.005,
                "TriDexel uniform-offset RMS: cpu={:.5}, gpu={:.5}",
                rc.rms_error_mm,
                rg.rms_error_mm
            );
            assert!((rg.mean_abs_error_mm - rc.mean_abs_error_mm).abs() < 0.005);
            assert!((rg.mean_signed_error_mm - rc.mean_signed_error_mm).abs() < 0.005);
            assert!((rg.max_excess_mm - rc.max_excess_mm).abs() < 0.005);
            assert!((rg.max_gouge_mm - rc.max_gouge_mm).abs() < 0.005);
            assert!((rg.p95_abs_error_mm - rc.p95_abs_error_mm).abs() < 0.05);
            assert!((rg.p99_abs_error_mm - rc.p99_abs_error_mm).abs() < 0.05);
            assert!((rg.in_tolerance_surface_pct - rc.in_tolerance_surface_pct).abs() < 0.2);
            assert!(
                (rg.excess_volume_mm3 - rc.excess_volume_mm3).abs() / rc.excess_volume_mm3.max(1.0)
                    < 0.001
            );
        }
    }

    #[test]
    fn gpu_checkpoint_cache_enforces_vram_budget_lru_eviction_and_cpu_spill_restore() {
        let ctx = GpuSimContext::shared_headless().expect("expected Vulkan/wgpu compute adapter");
        let stock_spec = StockShapeSpec::Box {
            aabb_wcs: Aabb3::new(DVec3::new(0.0, 0.0, -20.0), DVec3::new(80.0, 60.0, 0.0)),
        };
        let hf = HeightfieldStock::from_stock_spec(&stock_spec, SimResolution::Standard).unwrap();
        let tools = all_seven_tools();
        let tol = StockVerifyTolerances::default();

        let mut gpu = WgpuComputeBackend::new(ctx, SimulatedStock::Heightfield(hf), None);
        let cp_0_cpu = gpu.capture_checkpoint(0);
        let single_bytes = GpuCheckpointCache::snapshot_byte_size(&cp_0_cpu);
        assert!(single_bytes > 0);

        // Tight budget: fits at most 2 GPU checkpoints simultaneously
        let budget = single_bytes * 2;
        let mut cache = GpuCheckpointCache::new(budget);
        assert!(cache.capture_from_backend(&gpu, 0));
        let vol_0 = gpu.compute_volume_mm3();

        // Cut #1 -> state at seg 10
        let _ = gpu.sweep_batch(
            &[SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(10.0, 20.0, -6.0),
                    p1: DVec3::new(60.0, 20.0, -6.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 0,
                is_rapid: false,
                profile_idx: 0,
                verify_tol: tol,
            }],
            &tools,
            false,
        );
        let cp_10_cpu = gpu.capture_checkpoint(10);
        let vol_10 = gpu.compute_volume_mm3();
        assert!(vol_10 < vol_0 - 100.0);
        assert!(cache.capture_from_backend(&gpu, 10));
        assert_eq!(cache.len(), 2);
        assert_eq!(cache.used_bytes(), budget);
        assert_eq!(cache.eviction_count(), 0);

        // Touch seg 0 via VRAM restore so seg 10 becomes the LRU entry
        assert!(cache.restore_into_backend(&mut gpu, 0, Some(&cp_0_cpu)));
        assert_eq!(cache.hit_count(), 1);
        assert!((gpu.compute_volume_mm3() - vol_0).abs() < 1e-2);

        // Restore seg 10 (wait: instead sweep directly from seg 0 to create distinct seg 20 state!)
        let _ = gpu.sweep_batch(
            &[SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(15.0, 42.0, -10.0),
                    p1: DVec3::new(65.0, 42.0, -10.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 1,
                is_rapid: false,
                profile_idx: 0,
                verify_tol: tol,
            }],
            &tools,
            false,
        );
        let vol_20 = gpu.compute_volume_mm3();
        assert!((vol_20 - vol_10).abs() > 100.0);

        // Capturing seg 20 must evict LRU entry (seg 10) while keeping seg 0 and seg 20 in VRAM
        assert!(cache.capture_from_backend(&gpu, 20));
        assert_eq!(cache.eviction_count(), 1);
        assert_eq!(cache.len(), 2);
        assert!(cache.used_bytes() <= budget);
        assert!(cache.contains(0));
        assert!(cache.contains(20));
        assert!(!cache.contains(10));

        // Restoring evicted seg 10 transparently spills back from CPU snapshot and restores exact volume
        assert!(cache.restore_into_backend(&mut gpu, 10, Some(&cp_10_cpu)));
        assert_eq!(cache.miss_count(), 1);
        assert!(cache.used_bytes() <= budget);
        assert!(cache.contains(10));
        assert!(
            (gpu.compute_volume_mm3() - vol_10).abs() < 1e-2,
            "restored evicted checkpoint volume mismatch: got={:.3}, expected={:.3}",
            gpu.compute_volume_mm3(),
            vol_10
        );

        // Zero-budget cache gracefully refuses VRAM storage and still restores from CPU fallback
        let mut zero_cache = GpuCheckpointCache::new(0);
        assert!(!zero_cache.capture_from_backend(&gpu, 10));
        assert!(zero_cache.is_empty());
        assert!(zero_cache.restore_into_backend(&mut gpu, 0, Some(&cp_0_cpu)));
        assert!((gpu.compute_volume_mm3() - vol_0).abs() < 1e-2);
    }

    #[test]
    fn m64_tridexel_undercut_tslot_dovetail_lollipop_cpu_vs_gpu_parity_and_fault_injection() {
        let ctx = GpuSimContext::shared_headless().expect("expected Vulkan/wgpu compute adapter");
        let tol = StockVerifyTolerances::default();

        // Stock block: X in [0, 40], Y in [-15, +15], Z in [-18, 0] (40 x 30 x 18 = 21,600 mm^3)
        let stock_aabb = Aabb3::new(DVec3::new(0.0, -15.0, -18.0), DVec3::new(40.0, 15.0, 0.0));
        let base_td = crate::tridexel::TriDexelStock::from_stock_spec(
            &StockShapeSpec::Box {
                aabb_wcs: stock_aabb,
            },
            SimResolution::Standard,
        )
        .unwrap();

        let flat_6 = make_tool(
            "flat_6",
            24.0,
            CutterGeometry::FlatEndmill {
                diameter_mm: 6.0,
                flute_length_mm: 18.0,
                overall_length_mm: 55.0,
                shank_diameter_mm: 6.0,
                flutes: 3,
            },
        );
        // Sharp-corner 12x4 T-slot mill (neck D=5 -> r_neck=2.5 < pre-slot half-width 3.0)
        let slot_12 = make_tool(
            "slot_12",
            26.0,
            CutterGeometry::SlotMill {
                diameter_mm: 12.0,
                cutting_thickness_mm: 4.0,
                corner_radius_mm: 0.0,
                neck_diameter_mm: 5.0,
                neck_length_mm: 18.0,
                overall_length_mm: 58.0,
                shank_diameter_mm: 6.0,
                flutes: 4,
            },
        );
        // Sharp-corner 12mm 60deg Dovetail (H=5 -> R_top = 6 - 5/sqrt(3) = 3.1132 mm, neck D=5 -> r_neck=2.5)
        let dove_12 = make_tool(
            "dove_12",
            26.0,
            CutterGeometry::DovetailCutter {
                diameter_mm: 12.0,
                side_angle_deg: 60.0,
                cutting_height_mm: 5.0,
                corner_radius_mm: 0.0,
                neck_diameter_mm: 5.0,
                neck_length_mm: 18.0,
                overall_length_mm: 58.0,
                shank_diameter_mm: 6.0,
                flutes: 4,
            },
        );
        // 8mm Lollipop (R=4, neck D=4 -> r_neck=2 < pre-slot half-width 3.0)
        let lolli_8 = make_tool(
            "lolli_8",
            26.0,
            CutterGeometry::LollipopEndmill {
                diameter_mm: 8.0,
                neck_diameter_mm: 4.0,
                neck_length_mm: 18.0,
                flute_length_mm: CutterGeometry::lollipop_sphere_height_mm(8.0, 4.0),
                overall_length_mm: 58.0,
                shank_diameter_mm: 6.0,
                flutes: 4,
            },
        );
        let tools = vec![flat_6, slot_12, dove_12, lolli_8];

        // Pre-cut through-slot along X in [-10, 50] at Y = 0, Z = -12 (width 6 mm -> Y in [-3, +3], Z in [-12, 0])
        let preslot_chords = vec![SweepChord {
            motion: SweepMotion::Linear {
                p0: DVec3::new(-10.0, 0.0, -12.0),
                p1: DVec3::new(50.0, 0.0, -12.0),
                rotary_deg_0: 0.0,
                rotary_deg_1: 0.0,
            },
            op_idx: 0,
            is_rapid: false,
            profile_idx: 0,
            verify_tol: tol,
        }];

        let mut preslot_td = base_td.clone();
        let _ = preslot_td.sweep_chords(&preslot_chords, &tools, None, false);
        let preslot_vol = preslot_td.compute_volume_mm3();

        // === 1. Horizontal T-Slot with SlotMill at Z_tip = -12.0 (cavity Z in [-12, -8], Y in [-6, +6]) ===
        {
            let tslot_chords = vec![SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(-10.0, 0.0, -12.0),
                    p1: DVec3::new(50.0, 0.0, -12.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 1,
                is_rapid: false,
                profile_idx: 1,
                verify_tol: tol,
            }];
            let mut cpu =
                CpuExactBackend::new(SimulatedStock::TriDexel(Box::new(preslot_td.clone())), None);
            let mut gpu = WgpuComputeBackend::new(
                std::sync::Arc::clone(&ctx),
                SimulatedStock::TriDexel(Box::new(preslot_td.clone())),
                None,
            );
            let out_cpu = cpu.sweep_batch(&tslot_chords, &tools, true);
            let out_gpu = gpu.sweep_batch(&tslot_chords, &tools, true);
            assert!(
                out_cpu.violations.neck.is_none(),
                "clean T-slot must have zero neck collision on CPU"
            );
            assert!(
                out_gpu.violations.neck.is_none(),
                "clean T-slot must have zero neck collision on GPU"
            );
            assert!(out_cpu.violations.flute.is_none());
            assert!(out_gpu.violations.flute.is_none());

            // Exact undercut removed volume: 2 wings of width (6 - 3) = 3 mm each x 4 mm tall x 40 mm long = 960 mm^3!
            let removed_cpu = preslot_vol - cpu.compute_volume_mm3();
            let removed_gpu = preslot_vol - gpu.compute_volume_mm3();
            let expected_removed = 2.0 * 3.0 * 4.0 * 40.0;
            assert!(
                (removed_cpu - expected_removed).abs() / expected_removed < 0.015,
                "T-slot CPU removed vol {removed_cpu:.2} vs exact {expected_removed:.2}"
            );
            assert!(
                (removed_gpu - removed_cpu).abs() / expected_removed < 0.002,
                "T-slot CPU/GPU volume parity: cpu={removed_cpu:.2}, gpu={removed_gpu:.2}"
            );

            // Verify overhanging roof preservation on BOTH CPU & GPU at (X=20, Y=+4.5):
            // Z-ray must have 2 disjoint spans: floor [-18, -12] and intact overhanging roof [-8, 0]!
            let (SimulatedStock::TriDexel(td_cpu), _) = cpu.into_parts() else {
                panic!("expected TriDexel");
            };
            let (SimulatedStock::TriDexel(td_gpu), _) = gpu.into_parts() else {
                panic!("expected TriDexel");
            };
            for (label, td) in [("CPU", &td_cpu), ("GPU", &td_gpu)] {
                let z_plane = td.z_plane();
                let ix = ((20.0 - z_plane.origin_u_mm) / z_plane.du_mm).floor() as usize;
                let iy = ((4.5 - z_plane.origin_v_mm) / z_plane.dv_mm).floor() as usize;
                let spans = z_plane.ray_spans(ix, iy);
                assert_eq!(
                    spans.len(),
                    2,
                    "{label} T-slot overhang at Y=4.5 must have 2 Z-spans (floor + roof), got {spans:?}"
                );
                assert!(
                    (spans[0].max_mm - (-12.0)).abs() < 0.05,
                    "{label} floor z={}",
                    spans[0].max_mm
                );
                assert!(
                    (spans[1].min_mm - (-8.0)).abs() < 0.05,
                    "{label} roof underside z={}",
                    spans[1].min_mm
                );
                assert!(
                    (spans[1].max_mm - 0.0).abs() < 0.05,
                    "{label} roof top z={}",
                    spans[1].max_mm
                );
            }
        }

        // === 2. 60-deg Dovetail Slot at Z_tip = -12.0 (cavity Z in [-12, -7]) ===
        {
            let dove_chords = vec![SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(-10.0, 0.0, -12.0),
                    p1: DVec3::new(50.0, 0.0, -12.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 2,
                is_rapid: false,
                profile_idx: 2,
                verify_tol: tol,
            }];
            let mut cpu =
                CpuExactBackend::new(SimulatedStock::TriDexel(Box::new(preslot_td.clone())), None);
            let mut gpu = WgpuComputeBackend::new(
                std::sync::Arc::clone(&ctx),
                SimulatedStock::TriDexel(Box::new(preslot_td.clone())),
                None,
            );
            let out_cpu = cpu.sweep_batch(&dove_chords, &tools, true);
            let out_gpu = gpu.sweep_batch(&dove_chords, &tools, true);
            assert!(out_cpu.violations.neck.is_none());
            assert!(out_gpu.violations.neck.is_none());

            // Exact trapezoidal undercut wing area outside Y in [-3, +3]:
            // At dz=0: r=6.0 (wing=3.0); at dz=5: r=6 - 5/sqrt(3)=3.11324865 (wing=0.11324865)
            // Single wing area = 0.5 * (3.0 + 0.11324865) * 5.0 = 7.7831216 mm^2 -> 2 wings * 40 mm = 622.6497 mm^3!
            let tan60 = 3.0_f64.sqrt();
            let wing_top = (6.0 - 5.0 / tan60) - 3.0;
            let expected_removed = 2.0 * 0.5 * (3.0 + wing_top) * 5.0 * 40.0;
            let removed_cpu = preslot_vol - cpu.compute_volume_mm3();
            let removed_gpu = preslot_vol - gpu.compute_volume_mm3();
            assert!(
                (removed_cpu - expected_removed).abs() / expected_removed < 0.015,
                "Dovetail CPU removed vol {removed_cpu:.2} vs exact {expected_removed:.2}"
            );
            assert!(
                (removed_gpu - removed_cpu).abs() / expected_removed < 0.002,
                "Dovetail CPU/GPU parity: cpu={removed_cpu:.2}, gpu={removed_gpu:.2}"
            );

            // Verify exact angled dovetail cross-section width on Y-rays AND roof overhang on Z-rays!
            let (SimulatedStock::TriDexel(td_cpu), _) = cpu.into_parts() else {
                panic!("expected TriDexel");
            };
            let (SimulatedStock::TriDexel(td_gpu), _) = gpu.into_parts() else {
                panic!("expected TriDexel");
            };
            for (label, td) in [("CPU", &td_cpu), ("GPU", &td_gpu)] {
                let y_plane = td.plane(Axis3::Y);
                let ix = ((20.0 - y_plane.origin_u_mm) / y_plane.du_mm).floor() as usize;
                // Sample at Z = -9.5 (dz = 2.5 above tip) -> expected half-width = 6.0 - 2.5/sqrt(3) = 4.5566 mm
                let iz_mid = ((-9.5 - y_plane.origin_v_mm) / y_plane.dv_mm).floor() as usize;
                let pz_actual = y_plane.origin_v_mm + (iz_mid as f64 + 0.5) * y_plane.dv_mm;
                let expected_half_w = 6.0 - (pz_actual - (-12.0)) / tan60;
                let y_spans = y_plane.ray_spans(ix, iz_mid);
                assert_eq!(
                    y_spans.len(),
                    2,
                    "{label} dovetail Y-ray at mid-height must split into 2 walls"
                );
                let actual_half_w = 0.5 * (y_spans[1].min_mm - y_spans[0].max_mm);
                assert!(
                    (actual_half_w - expected_half_w).abs() < 0.05,
                    "{label} dovetail angled wall half-width at Z={pz_actual:.2}: got {actual_half_w:.3}, expected {expected_half_w:.3}"
                );

                // Above Z = -7.0 (e.g. Z = -4.0), pre-slot roof remains at half-width = 3.0 mm!
                let iz_roof = ((-4.0 - y_plane.origin_v_mm) / y_plane.dv_mm).floor() as usize;
                let y_roof_spans = y_plane.ray_spans(ix, iz_roof);
                assert_eq!(y_roof_spans.len(), 2);
                let roof_half_w = 0.5 * (y_roof_spans[1].min_mm - y_roof_spans[0].max_mm);
                assert!(
                    (roof_half_w - 3.0).abs() < 0.05,
                    "{label} dovetail roof half-width must remain 3.0 mm, got {roof_half_w:.3}"
                );
            }
        }

        // === 3. Concave Spherical Undercut Groove with LollipopEndmill at Z_tip = -12.0 ===
        {
            let lolli_chords = vec![SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(-10.0, 0.0, -12.0),
                    p1: DVec3::new(50.0, 0.0, -12.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 3,
                is_rapid: false,
                profile_idx: 3,
                verify_tol: tol,
            }];
            let mut cpu =
                CpuExactBackend::new(SimulatedStock::TriDexel(Box::new(preslot_td.clone())), None);
            let mut gpu = WgpuComputeBackend::new(
                std::sync::Arc::clone(&ctx),
                SimulatedStock::TriDexel(Box::new(preslot_td.clone())),
                None,
            );
            let out_cpu = cpu.sweep_batch(&lolli_chords, &tools, true);
            let out_gpu = gpu.sweep_batch(&lolli_chords, &tools, true);
            assert!(out_cpu.violations.neck.is_none());
            assert!(out_gpu.violations.neck.is_none());

            // Exact circular segment area outside |y| <= 3.0 of circle R=4.0:
            // Two circular caps with chord at y=3.0 (half-angle alpha = acos(3/4)):
            // Each cap area = R^2 * (alpha - sin(alpha)*cos(alpha)) = 16 * (acos(0.75) - 0.75 * sqrt(7)/4)
            let alpha = (0.75_f64).acos();
            let cap_area = 16.0 * (alpha - 0.75 * (1.0 - 0.75 * 0.75_f64).sqrt());
            let expected_removed = 2.0 * cap_area * 40.0;
            let removed_cpu = preslot_vol - cpu.compute_volume_mm3();
            let removed_gpu = preslot_vol - gpu.compute_volume_mm3();
            assert!(
                (removed_cpu - expected_removed).abs() / expected_removed < 0.015,
                "Lollipop CPU removed vol {removed_cpu:.2} vs exact {expected_removed:.2}"
            );
            assert!(
                (removed_gpu - removed_cpu).abs() / expected_removed < 0.005,
                "Lollipop CPU/GPU parity: cpu={removed_cpu:.2}, gpu={removed_gpu:.2}"
            );

            // Verify concave spherical roof preservation at Y = 3.5 (within sphere R=4, outside pre-slot 3.0)
            let (SimulatedStock::TriDexel(td_cpu), _) = cpu.into_parts() else {
                panic!("expected TriDexel");
            };
            let (SimulatedStock::TriDexel(td_gpu), _) = gpu.into_parts() else {
                panic!("expected TriDexel");
            };
            for (label, td) in [("CPU", &td_cpu), ("GPU", &td_gpu)] {
                let z_plane = td.z_plane();
                let ix = ((20.0 - z_plane.origin_u_mm) / z_plane.du_mm).floor() as usize;
                let iy = ((3.5 - z_plane.origin_v_mm) / z_plane.dv_mm).floor() as usize;
                let py_actual = z_plane.origin_v_mm + (iy as f64 + 0.5) * z_plane.dv_mm;
                let half_h = (16.0 - py_actual * py_actual).sqrt();
                let spans = z_plane.ray_spans(ix, iy);
                assert_eq!(
                    spans.len(),
                    2,
                    "{label} Lollipop overhang at Y={py_actual:.2} must have 2 Z-spans"
                );
                assert!((spans[0].max_mm - (-12.0 + 4.0 - half_h)).abs() < 0.05);
                assert!((spans[1].min_mm - (-12.0 + 4.0 + half_h)).abs() < 0.05);
                assert!((spans[1].max_mm - 0.0).abs() < 0.05);
            }
        }

        // === 4. Fault-Injection (A): Vertical plunge of SlotMill / Dovetail into uncut solid roof ===
        for (label, prof_idx) in [("SlotMill", 1usize), ("DovetailCutter", 2usize)] {
            let plunge_chords = vec![SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(20.0, 0.0, 2.0),
                    p1: DVec3::new(20.0, 0.0, -12.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 4,
                is_rapid: false,
                profile_idx: prof_idx,
                verify_tol: tol,
            }];
            let mut cpu =
                CpuExactBackend::new(SimulatedStock::TriDexel(Box::new(base_td.clone())), None);
            let mut gpu = WgpuComputeBackend::new(
                std::sync::Arc::clone(&ctx),
                SimulatedStock::TriDexel(Box::new(base_td.clone())),
                None,
            );
            let out_cpu = cpu.sweep_batch(&plunge_chords, &tools, true);
            let out_gpu = gpu.sweep_batch(&plunge_chords, &tools, true);
            assert!(
                out_cpu.violations.neck.is_some(),
                "{label} vertical plunge into uncut solid must trigger neck collision on CPU"
            );
            assert!(
                out_gpu.violations.neck.is_some(),
                "{label} vertical plunge into uncut solid must trigger neck collision on GPU"
            );
            let d_cpu = out_cpu.violations.neck.unwrap().0;
            let d_gpu = out_gpu.violations.neck.unwrap().0;
            assert!(
                (d_cpu - d_gpu).abs() < 0.2,
                "{label} vertical plunge neck depth parity: cpu={d_cpu:.3}, gpu={d_gpu:.3}"
            );
        }

        // === 5. Fault-Injection (B): Lateral neck crash into overhanging lip ===
        // Pre-slot opens Y in [-3, +3]; shifting SlotMill (r_neck=2.5) to Y=+2.0 puts neck out to Y=+4.5 (> 3.0)!
        {
            let crash_chords = vec![SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(5.0, 2.0, -12.0),
                    p1: DVec3::new(35.0, 2.0, -12.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 5,
                is_rapid: false,
                profile_idx: 1,
                verify_tol: tol,
            }];
            let mut cpu =
                CpuExactBackend::new(SimulatedStock::TriDexel(Box::new(preslot_td.clone())), None);
            let mut gpu = WgpuComputeBackend::new(
                std::sync::Arc::clone(&ctx),
                SimulatedStock::TriDexel(Box::new(preslot_td.clone())),
                None,
            );
            let out_cpu = cpu.sweep_batch(&crash_chords, &tools, true);
            let out_gpu = gpu.sweep_batch(&crash_chords, &tools, true);
            assert!(
                out_cpu.violations.neck.is_some(),
                "SlotMill neck ramming overhanging lip must trigger neck violation on CPU"
            );
            assert!(
                out_gpu.violations.neck.is_some(),
                "SlotMill neck ramming overhanging lip must trigger neck violation on GPU"
            );
            let d_cpu = out_cpu.violations.neck.unwrap().0;
            let d_gpu = out_gpu.violations.neck.unwrap().0;
            assert!(
                (d_cpu - d_gpu).abs() < 0.2,
                "Overhang lip neck crash parity: cpu={d_cpu:.3}, gpu={d_gpu:.3}"
            );
        }
    }

    #[test]
    fn gpu_tridexel_ultra_emit_does_not_truncate_z_plane_above_old_2_4m_cap() {
        let ctx = GpuSimContext::shared_headless().expect("expected Vulkan/wgpu compute adapter");
        // Match the `undercut-deburr` 100 × 80 × 29 mm stock at Ultra resolution (`448 × 358 × 130`).
        let stock_spec = StockShapeSpec::Box {
            aabb_wcs: Aabb3::new(DVec3::new(-50.0, -40.0, -29.0), DVec3::new(50.0, 40.0, 0.0)),
        };
        let mut td =
            crate::tridexel::TriDexelStock::from_stock_spec(&stock_spec, SimResolution::Ultra)
                .unwrap();
        assert_eq!(td.dims[0], 448);

        // Carve two full-length through-slots along X (mirroring T-slot + dovetail cavities)
        // so Y-rays split into 3 spans and total emitted vertices exceed the old 2_400_000 cap!
        let slot_tools = vec![make_tool(
            "flat_8",
            25.0,
            CutterGeometry::FlatEndmill {
                diameter_mm: 8.5,
                flute_length_mm: 16.0,
                overall_length_mm: 55.0,
                shank_diameter_mm: 8.5,
                flutes: 3,
            },
        )];
        let tol = StockVerifyTolerances::default();
        let chords = vec![
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(-55.0, -20.0, -15.0),
                    p1: DVec3::new(55.0, -20.0, -15.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 0,
                is_rapid: false,
                profile_idx: 0,
                verify_tol: tol,
            },
            SweepChord {
                motion: SweepMotion::Linear {
                    p0: DVec3::new(-55.0, 12.0, -15.0),
                    p1: DVec3::new(55.0, 12.0, -15.0),
                    rotary_deg_0: 0.0,
                    rotary_deg_1: 0.0,
                },
                op_idx: 1,
                is_rapid: false,
                profile_idx: 0,
                verify_tol: tol,
            },
        ];

        let mut gpu = WgpuComputeBackend::new(
            std::sync::Arc::clone(&ctx),
            SimulatedStock::TriDexel(Box::new(td.clone())),
            None,
        );
        let _ = gpu.sweep_batch(&chords, &slot_tools, false);
        let handle = gpu.capture_render_handle(crate::metrics::SimStockColorMode::ByOperation);

        // Read back the GPU stock to compute the exact inline span count across all 3 planes.
        let (SimulatedStock::TriDexel(td_after), _) = gpu.into_parts() else {
            panic!("expected TriDexel stock");
        };
        td = *td_after;

        let mut expected_inline_spans: u64 = 0;
        for axis in Axis3::ALL {
            let p = td.plane(axis);
            for (r, &cnt) in p.counts.iter().enumerate() {
                let c = cnt as usize;
                if c > 0 && c <= crate::tridexel::INLINE_DEXEL_SLOTS {
                    let base = r * crate::tridexel::INLINE_DEXEL_SLOTS;
                    for k in 0..c {
                        let s = p.slots[base + k];
                        if s.max_local > s.min_local {
                            expected_inline_spans += 1;
                        }
                    }
                }
            }
        }
        let expected_verts = expected_inline_spans * 2 * 6;
        assert!(
            expected_verts > 2_400_000,
            "regression fixture must exceed old 2_400_000 vertex cap (got {expected_verts})"
        );

        let GpuStockRenderHandle::TriDexel(ref td_render) = *handle else {
            panic!("expected TriDexel GPU render handle");
        };
        let emitted_verts = u64::from(td_render.readback_emitted_vertex_count());
        assert_eq!(
            emitted_verts, expected_verts,
            "Ultra Tri-Dexel GPU emitter truncated surfaces: emitted {emitted_verts} vs expected {expected_verts} (max_vertices={})",
            td_render.max_vertices
        );

        // Verify worst-case 448³ cube fits within WebGPU's 128 MiB storage buffer binding limit!
        let cube_rays = 3usize * 448 * 448;
        let max_quads = render_handle::compute_tridexel_max_quads(ctx.device(), cube_rays);
        assert_eq!(
            max_quads as usize,
            cube_rays * crate::tridexel::INLINE_DEXEL_SLOTS * 2
        );
        assert!(
            (max_quads as usize) * TRIDEXEL_PACKED_QUAD_BYTES <= 128 * 1024 * 1024,
            "worst-case Ultra 448³ packed quad buffer must fit inside 128 MiB"
        );
    }
}
