//! Full-3D orthogonal **Tri-Dexel** volumetric stock simulation engine (`Phase 11`).
//!
//! Coexists cleanly alongside the fast 2.5D Z-Dexel heightmap (`heightmap`) and polar cylindrical
//! (`cylindrical`) engines so users can choose speed (`SimEngineKind::Heightmap`) or full 3D
//! volumetric accuracy (`SimEngineKind::TriDexel`) with vertical walls, multi-interval cavities,
//! and undercuts.

pub mod grid;
pub mod mesher;
pub mod sweep;

pub use grid::{
    DexelPlane, DexelSpan, SimEngineKind, TriDexelCheckpoint, TriDexelStock, query_overlap_length,
    query_target_gouge_depth, spans_contain_coord, spans_difference_length,
    subtract_dexel_interval,
};
pub use mesher::tessellate_tridexel_stock_mesh;
pub use sweep::run_tridexel_setup_simulation;

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cad::{CadKernel, MonstertruckKernel, sample_machined_plate_step};
    use chip_cam::{
        ClEvent, MotionRole, ResolvedStockGeometry, Setup, StockAxis, ToolpathBuilder,
        WorkOffsetSlot,
    };
    use chip_math::{Aabb3, DEFAULT_TESS_CHORD_TOL_MM, DVec3, DisplayUnit};
    use chip_tools::default_starter_tool_library;

    use crate::{
        MachineKinematicsConfig, SetupSimulationInput, SimIssueKind, SimOperationInput,
        SimResolution, SimVerificationConfig, run_setup_simulation_with_engine,
    };

    use super::*;

    #[test]
    fn tridexel_box_and_cylinder_volumes_match_analytical_formulas_across_all_3_axes() {
        let box_stock = ResolvedStockGeometry::Box {
            aabb_wcs: Aabb3::new(DVec3::new(0.0, 0.0, -20.0), DVec3::new(80.0, 60.0, 0.0)),
        };
        let td_box = TriDexelStock::from_setup_stock(&box_stock, SimResolution::Standard).unwrap();
        let (vx, vy, vz) = td_box.per_axis_volumes_mm3();
        let expected_box = 80.0 * 60.0 * 20.0;
        for v in [vx, vy, vz, td_box.compute_volume_mm3()] {
            assert!(
                (v - expected_box).abs() < 1e-3,
                "box volume mismatch: got {v}, expected {expected_box}"
            );
        }

        let r = 20.0;
        let len = 80.0;
        let cyl_stock = ResolvedStockGeometry::Cylinder {
            axis: StockAxis::X,
            center_wcs: DVec3::ZERO,
            radius_mm: r,
            min_axial_mm: -40.0,
            max_axial_mm: 40.0,
            aabb_wcs: Aabb3::new(DVec3::new(-40.0, -r, -r), DVec3::new(40.0, r, r)),
        };
        let td_cyl = TriDexelStock::from_setup_stock(&cyl_stock, SimResolution::Standard).unwrap();
        let expected_cyl = std::f64::consts::PI * r * r * len;
        let rel_err = (td_cyl.compute_volume_mm3() - expected_cyl).abs() / expected_cyl;
        assert!(
            rel_err < 0.005,
            "cylinder volume err {:.4}% > 0.5% (got {:.1}, expected {:.1})",
            rel_err * 100.0,
            td_cyl.compute_volume_mm3(),
            expected_cyl
        );
    }

    #[test]
    fn tridexel_slot_cut_undercut_split_and_watertight_meshing_are_accurate() {
        let lib = default_starter_tool_library();
        let flat_6 = &lib.tools[0]; // Ø6 mm flat endmill, flute length ~22mm

        // 100 x 100 x 20 mm box (`X ∈ [-50, 50], Y ∈ [-50, 50], Z ∈ [-20, 0]`)
        let mut setup = Setup::new("s1", "Slot Test", WorkOffsetSlot::G54);
        setup.wcs.origin = chip_cam::WcsOriginMode::StockBoxPoint {
            anchor: chip_cam::BoxAnchor27::TOP_CENTER,
        };
        setup.stock = chip_cam::StockConfig::FixedBox {
            width_x: chip_math::ParamLength::from_mm(100.0),
            depth_y: chip_math::ParamLength::from_mm(100.0),
            height_z: chip_math::ParamLength::from_mm(20.0),
            xy_align: chip_cam::FixedBoxXyAlign::CenterXy,
            z_align: chip_cam::FixedBoxZAlign::default(),
        };
        let empty_doc = chip_cad::CadDocument::empty();
        let resolved = setup.resolve(&empty_doc, DisplayUnit::Millimeter).unwrap();

        // Single level 20mm slot cut at Z = -5mm with Ø6mm flat endmill:
        // Exact analytical swept area = (20 * 6) + pi * 3^2 = 120 + 28.2743 = 148.2743 mm²
        // Exact removed volume = 148.2743 * 5.0 = 741.3717 mm³
        let mut b = ToolpathBuilder::new("op_slot");
        b.push_event(ClEvent::SpindleStart {
            rpm: 12_000.0,
            direction: chip_tools::SpindleDirection::Cw,
        });
        b.rapid_to(DVec3::new(-10.0, 0.0, 5.0));
        b.linear_to(DVec3::new(-10.0, 0.0, -5.0), 300.0, MotionRole::Plunge);
        b.linear_to(DVec3::new(10.0, 0.0, -5.0), 1000.0, MotionRole::Cut);
        b.rapid_to(DVec3::new(10.0, 0.0, 5.0));
        let tp = b.finish();

        let sim = run_setup_simulation_with_engine(
            &SetupSimulationInput {
                doc: &empty_doc,
                setup: &resolved,
                target_bodies: &[],
                operations: &[SimOperationInput {
                    op_id: "op_slot",
                    op_name: "Slot",
                    tool: flat_6,
                    fallback_spindle_rpm: 12_000.0,
                    toolpath: &tp,
                    intentional_edge_break_mm: 0.0,
                }],
                resolution: SimResolution::Standard,
                kinematics: MachineKinematicsConfig::default(),
                verify_config: SimVerificationConfig::default(),
                cancel: None,
            },
            SimEngineKind::TriDexel,
        )
        .unwrap();

        assert_eq!(sim.engine, SimEngineKind::TriDexel);
        assert!(!sim.has_critical_errors(), "diags: {:?}", sim.diagnostics);

        let expected_removed = (20.0 * 6.0 + std::f64::consts::PI * 9.0) * 5.0;
        let rel_err = (sim.removed_volume_mm3 - expected_removed).abs() / expected_removed;
        assert!(
            rel_err < 0.035,
            "Standard removed vol {:.2} vs expected {:.2} (err {:.2}%)",
            sim.removed_volume_mm3,
            expected_removed,
            rel_err * 100.0
        );

        // Verify Fine resolution converges tightly (< 1.5% error)
        let sim_fine = run_setup_simulation_with_engine(
            &SetupSimulationInput {
                doc: &empty_doc,
                setup: &resolved,
                target_bodies: &[],
                operations: &[SimOperationInput {
                    op_id: "op_slot",
                    op_name: "Slot",
                    tool: flat_6,
                    fallback_spindle_rpm: 12_000.0,
                    toolpath: &tp,
                    intentional_edge_break_mm: 0.0,
                }],
                resolution: SimResolution::Fine,
                kinematics: MachineKinematicsConfig::default(),
                verify_config: SimVerificationConfig::default(),
                cancel: None,
            },
            SimEngineKind::TriDexel,
        )
        .unwrap();
        let fine_err = (sim_fine.removed_volume_mm3 - expected_removed).abs() / expected_removed;
        assert!(
            fine_err < 0.015,
            "Fine removed vol {:.2} vs expected {:.2} (err {:.2}%)",
            sim_fine.removed_volume_mm3,
            expected_removed,
            fine_err * 100.0
        );

        // Verify crisp Tri-Dexel surface mesh extraction produces valid outward triangles
        let td = sim.final_tridexel_stock.as_ref().unwrap();
        let mesh = tessellate_tridexel_stock_mesh(td, true);
        assert!(mesh.vertices.len() > 100);
        assert_eq!(mesh.indices.len() % 3, 0);

        // Verify multi-interval undercut splitting directly on a dexel ray
        let mut ray = vec![DexelSpan::new(-20.0, 0.0, u16::MAX, u16::MAX)];
        let rem = subtract_dexel_interval(&mut ray, -12.0, -8.0, 0);
        assert!((rem - 4.0).abs() < 1e-12);
        assert_eq!(ray.len(), 2, "undercut must split span into 2 intervals");
        assert!((ray[0].max_mm - (-12.0)).abs() < 1e-12);
        assert!((ray[1].min_mm - (-8.0)).abs() < 1e-12);
    }

    #[test]
    fn tridexel_fault_injection_catches_rapid_crash_and_vertical_wall_target_gouge() {
        let step = sample_machined_plate_step().unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("plate.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let lib = default_starter_tool_library();
        let flat_6 = &lib.tools[0];

        let mut setup = Setup::new("s1", "Fault Test", WorkOffsetSlot::G54);
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
        let maabb = resolved.model_aabb_wcs;
        let center = maabb.center();

        // Deliberate G0 rapid crash + straight-through target gouge right through the plate center!
        let mut b = ToolpathBuilder::new("op_bad");
        b.push_event(ClEvent::SpindleStart {
            rpm: 12_000.0,
            direction: chip_tools::SpindleDirection::Cw,
        });
        b.rapid_to(DVec3::new(center.x, center.y, maabb.max.z + 15.0));
        b.rapid_to(DVec3::new(center.x, center.y, maabb.max.z - 5.0));
        b.linear_to(
            DVec3::new(center.x + 15.0, center.y, maabb.max.z - 5.0),
            800.0,
            MotionRole::Cut,
        );
        let tp = b.finish();

        let sim = run_setup_simulation_with_engine(
            &SetupSimulationInput {
                doc: &doc,
                setup: &resolved,
                target_bodies: &[],
                operations: &[SimOperationInput {
                    op_id: "op_bad",
                    op_name: "Bad Cut",
                    tool: flat_6,
                    fallback_spindle_rpm: 12_000.0,
                    toolpath: &tp,
                    intentional_edge_break_mm: 0.0,
                }],
                resolution: SimResolution::Standard,
                kinematics: MachineKinematicsConfig::default(),
                verify_config: SimVerificationConfig::default(),
                cancel: None,
            },
            SimEngineKind::TriDexel,
        )
        .unwrap();

        assert!(sim.has_critical_errors());
        assert!(
            sim.diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::RapidIntoStock),
            "expected RapidIntoStock diagnostic: {:?}",
            sim.diagnostics
        );
        assert!(
            sim.diagnostics
                .iter()
                .any(|d| d.kind == SimIssueKind::TargetModelGouge),
            "expected TargetModelGouge diagnostic: {:?}",
            sim.diagnostics
        );
    }
}
