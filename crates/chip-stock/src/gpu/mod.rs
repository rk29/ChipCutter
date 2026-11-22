//! Optional `wgpu` GPU compute volumetric stock simulation backend (`#[cfg(feature = "gpu-compute")]`).

pub mod backend;
pub mod context;
pub mod cylindrical;
pub mod heightfield;
pub mod types;

pub use backend::WgpuComputeBackend;
pub use context::GpuSimContext;
pub use types::{
    CylindricalUniforms, GPU_TILE_SIZE, GpuRotaryPose, GpuSweepChord, GpuToolProfile,
    GpuViolationAccum, GpuViolationSlot, HeightfieldUniforms,
};

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::f64::consts::PI;

    use chip_math::{Aabb3, DVec2, DVec3, DisplayUnit};
    use chip_tools::{CoolantMode, CutterGeometry, SpindleDirection, Tool, ToolId};

    use crate::{
        backend::{CpuExactBackend, SimStockBackend, SimulatedStock},
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
}
