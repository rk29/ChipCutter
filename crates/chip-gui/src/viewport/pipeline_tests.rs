#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::{collections::HashSet, sync::Arc};

use chip_cad::{CadKernel, FaceId, MonstertruckKernel, PickTarget};
use chip_cam::{Setup, WorkOffsetSlot};
use chip_fixtures::{sample_machined_plate_step, sample_multi_feature_bracket_step};
use chip_math::DEFAULT_TESS_CHORD_TOL_MM;

use super::*;
use crate::viewport::highlight::{ToolpathRenderEntry, ZPlaneVisual};

#[test]
fn viewport_wgsl_shader_has_fxc_safe_return_paths() {
    use wgpu::naga::{Block, Statement, front::wgsl, valid};

    fn block_contains_return(block: &Block) -> bool {
        block.iter().any(|stmt| match stmt {
            Statement::Return { .. } => true,
            Statement::Block(inner) => block_contains_return(inner),
            Statement::If { accept, reject, .. } => {
                block_contains_return(accept) || block_contains_return(reject)
            }
            Statement::Switch { cases, .. } => cases.iter().any(|c| block_contains_return(&c.body)),
            Statement::Loop {
                body, continuing, ..
            } => block_contains_return(body) || block_contains_return(continuing),
            _ => false,
        })
    }

    fn switch_contains_return(block: &Block) -> bool {
        block.iter().any(|stmt| match stmt {
            Statement::Switch { cases, .. } => cases.iter().any(|c| block_contains_return(&c.body)),
            Statement::Block(inner) => switch_contains_return(inner),
            Statement::If { accept, reject, .. } => {
                switch_contains_return(accept) || switch_contains_return(reject)
            }
            Statement::Loop {
                body, continuing, ..
            } => switch_contains_return(body) || switch_contains_return(continuing),
            _ => false,
        })
    }

    let module =
        wgsl::parse_str(include_str!("shaders.wgsl")).expect("parse viewport shaders.wgsl");
    let mut validator =
        valid::Validator::new(valid::ValidationFlags::all(), valid::Capabilities::empty());
    validator
        .validate(&module)
        .expect("validate viewport shaders.wgsl");

    let check_fn = |fn_name: &str, func: &wgpu::naga::Function| {
        if func.result.is_none() {
            return;
        }
        assert!(
            matches!(func.body.last(), Some(Statement::Return { value: Some(_) })),
            "viewport shaders.wgsl function `{fn_name}` must end with an unconditional \
             top-level `return` statement for Windows FXC compatibility"
        );
        assert!(
            !switch_contains_return(&func.body),
            "viewport shaders.wgsl function `{fn_name}` contains a `return` inside a \
             `switch` case, which can trigger FXC error X3507 on Windows"
        );
    };

    for (_, func) in module.functions.iter() {
        let fn_name = func.name.as_deref().unwrap_or("<unnamed>");
        check_fn(fn_name, func);
    }
    for ep in &module.entry_points {
        check_fn(&ep.name, &ep.function);
    }
}

#[test]
fn growable_gpu_buffer_reuses_allocation_and_grows_by_powers_of_two() {
    let instance =
        wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle_from_env());
    let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
        power_preference: wgpu::PowerPreference::default(),
        compatible_surface: None,
        force_fallback_adapter: false,
        apply_limit_buckets: false,
    }))
    .unwrap();
    let (device, queue) =
        pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default())).unwrap();

    let mut gbuf = GrowableGpuBuffer::new("test_growable", wgpu::BufferUsages::VERTEX);
    assert!(gbuf.active().is_none());
    assert_eq!(gbuf.capacity_bytes(), 0);

    gbuf.upload_slice::<OverlayVertexGpu>(&device, &queue, &[]);
    assert!(gbuf.active().is_none());
    assert_eq!(gbuf.capacity_bytes(), 0);

    // 3 * 28 bytes = 84 bytes -> clamped to minimum 256 bytes
    let small = vec![
        OverlayVertexGpu {
            pos_mm: [0.0, 0.0, 0.0],
            color: [1.0, 0.0, 0.0, 1.0],
        };
        3
    ];
    gbuf.upload_slice(&device, &queue, &small);
    assert_eq!(gbuf.active().map(|(_, c)| c), Some(3));
    assert_eq!(gbuf.capacity_bytes(), 256);

    // Empty frame keeps buffer allocated for reuse
    gbuf.upload_slice::<OverlayVertexGpu>(&device, &queue, &[]);
    assert!(gbuf.active().is_none());
    assert_eq!(gbuf.capacity_bytes(), 256);

    // Smaller/equal frame reuses existing 256-byte buffer without reallocating
    gbuf.upload_slice(&device, &queue, &small[..2]);
    assert_eq!(gbuf.active().map(|(_, c)| c), Some(2));
    assert_eq!(gbuf.capacity_bytes(), 256);

    // 20 * 28 bytes = 560 bytes -> grows to next power of two (1024 bytes)
    let larger = vec![
        OverlayVertexGpu {
            pos_mm: [1.0, 2.0, 3.0],
            color: [0.0, 1.0, 0.0, 0.5],
        };
        20
    ];
    gbuf.upload_slice(&device, &queue, &larger);
    assert_eq!(gbuf.active().map(|(_, c)| c), Some(20));
    assert_eq!(gbuf.capacity_bytes(), 1024);
}

#[test]
fn renders_offscreen_cad_viewport_frame_with_shaded_faces_and_edges() {
    let step = sample_machined_plate_step().unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes("sample.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
        .unwrap();

    let mut cam = CadCamera::default();
    cam.fit_aabb(&doc.aabb_mm);

    let highlight = HighlightState {
        selected: Some(PickTarget::Face(FaceId(0))),
        ..HighlightState::default()
    };

    let png = render_offscreen_png(&doc, &cam, &highlight, 640, 480).unwrap();
    assert!(png.starts_with(b"\x89PNG\r\n\x1a\n"));
    assert!(
        png.len() > 8_000,
        "expected rich shaded CAD PNG (>8KB), got {} bytes",
        png.len()
    );
}

#[test]
fn renders_stock_z_planes_wcs_triad_ghosting_and_multi_face_highlights() {
    let step = sample_multi_feature_bracket_step(false).unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes("multi.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
        .unwrap();

    let setup = Setup::new("s1", "Op 1", WorkOffsetSlot::G54);
    let resolved = setup
        .resolve_immutable(&doc, DisplayUnit::Millimeter)
        .unwrap();

    let mut cam = CadCamera::default();
    cam.align_to_setup_top(&resolved.frame, &resolved.stock.aabb_wcs());

    let mut sel_faces = HashSet::new();
    sel_faces.insert(FaceId(1));
    sel_faces.insert(FaceId(2));

    let highlight = HighlightState {
        extra_selected_faces: sel_faces,
        wcs_frame: resolved.frame,
        stock: Some(resolved.stock),
        show_stock: true,
        z_planes_wcs: vec![
            ZPlaneVisual {
                z_wcs: resolved.clearance_z_wcs,
                fill_rgba: [0.25, 0.72, 1.0, 0.12],
                border_rgba: [0.35, 0.80, 1.0, 0.85],
            },
            ZPlaneVisual {
                z_wcs: resolved.retract_z_wcs,
                fill_rgba: [1.0, 0.72, 0.22, 0.10],
                border_rgba: [1.0, 0.78, 0.28, 0.85],
            },
        ],
        ghost_part: true,
        show_grid: true,
        ..HighlightState::default()
    };

    let png = render_offscreen_png(&doc, &cam, &highlight, 640, 480).unwrap();
    assert!(png.starts_with(b"\x89PNG\r\n\x1a\n"));
    assert!(png.len() > 8_000);
}

#[test]
fn renders_instanced_toolpaths_and_holding_tab_markers() {
    use std::sync::atomic::AtomicBool;

    use chip_cam::{
        HeightReference, HeightSpec, Operation, OperationKind, StrategyParams, TabProfileKind,
        generate_operation_toolpath,
    };
    use chip_fixtures::sample_cam_torture_block_step;
    use chip_tools::default_starter_tool_library;

    let step = sample_cam_torture_block_step().unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes("torture.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
        .unwrap();
    let setup = Setup::new("s1", "Op 1", WorkOffsetSlot::G54);
    let resolved = setup
        .resolve_immutable(&doc, DisplayUnit::Millimeter)
        .unwrap();

    let starter = default_starter_tool_library();
    let flat_6mm = &starter.tools[0];
    let mut op = Operation::new("op_cnt", "Profile", OperationKind::Contour2D);
    op.tool_id = Some(flat_6mm.id.clone());
    let first_loop = doc.loops[0].id;
    op.selections
        .push(chip_cam::GeometrySelectionItem::ClosedLoop {
            binding: doc.bind_loop(first_loop).unwrap(),
            reverse: false,
            side_flip: false,
        });
    op.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
    op.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.0);
    if let StrategyParams::Contour2D(p) = op.strategy_mut() {
        p.tabs.enabled = true;
        p.tabs.profile = TabProfileKind::Triangular {
            ramp_angle_deg: 25.0,
        };
    }

    let tp = Arc::new(
        generate_operation_toolpath(
            &doc,
            &resolved,
            &op,
            flat_6mm,
            DisplayUnit::Millimeter,
            true,
            &AtomicBool::new(false),
        )
        .unwrap(),
    );

    let mut cam = CadCamera::default();
    cam.fit_aabb(&doc.aabb_mm);

    let highlight = HighlightState {
        wcs_frame: resolved.frame,
        stock: Some(resolved.stock),
        show_stock: true,
        toolpaths: vec![ToolpathRenderEntry {
            op_id: op.id.clone(),
            cache_key: [7_u8; 32],
            toolpath: tp,
            is_focused: true,
        }],
        show_toolpaths: true,
        show_rapids: true,
        ..HighlightState::default()
    };

    let png = render_offscreen_png(&doc, &cam, &highlight, 640, 480).unwrap();
    assert!(png.starts_with(b"\x89PNG\r\n\x1a\n"));
    assert!(png.len() > 10_000);
}

#[test]
fn renders_pass_6_machined_stock_mesh_and_3d_cutter_assembly() {
    use std::sync::atomic::AtomicBool;

    use crate::viewport::build_cutter_assembly_mesh;
    use chip_cad::SurfaceKind;
    use chip_cam::{GeometrySelectionItem, Operation, OperationKind, generate_operation_toolpath};
    use chip_fixtures::sample_cam_torture_block_step;
    use chip_sim::{
        MachineKinematicsConfig, SetupSimulationInput, SimOperationInput, SimResolution,
        SimVerificationConfig, run_setup_simulation,
    };
    use chip_stock::tessellate_stock_mesh;
    use chip_tools::default_starter_tool_library;

    let step = sample_cam_torture_block_step().unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes("torture.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
        .unwrap();
    let setup = Setup::new("s1", "Op 1", WorkOffsetSlot::G54);
    let resolved = setup
        .resolve_immutable(&doc, DisplayUnit::Millimeter)
        .unwrap();

    let starter = default_starter_tool_library();
    let flat_6mm = &starter.tools[0];
    let pkt_floor = doc
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z > 0.99 && (origin_mm.z - 12.0).abs() < 0.05
            )
        })
        .unwrap()
        .id;

    let mut op = Operation::new("op_pkt", "Island Pocket", OperationKind::Pocket2D);
    op.tool_id = Some(flat_6mm.id.clone());
    op.selections.push(GeometrySelectionItem::PlanarFace {
        binding: doc.bind_face(pkt_floor).unwrap(),
    });

    let tp = generate_operation_toolpath(
        &doc,
        &resolved,
        &op,
        flat_6mm,
        DisplayUnit::Millimeter,
        true,
        &AtomicBool::new(false),
    )
    .unwrap();

    let sim_ops = [SimOperationInput::from_operation(&op, flat_6mm, &tp)];
    let sim_res = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &resolved,
        target_bodies: &setup.target_bodies,
        fixtures: &[],
        operations: &sim_ops,
        resolution: SimResolution::Standard,
        kinematics: MachineKinematicsConfig::default(),
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
        ..SetupSimulationInput::new(&doc, &resolved, &[])
    })
    .unwrap();

    let stock_mesh = Arc::new(tessellate_stock_mesh(
        sim_res.as_heightfield().expect("expected HeightfieldStock"),
        true,
    ));
    let tool_mesh = Arc::new(build_cutter_assembly_mesh(flat_6mm, 24, false));
    let mid_snap = sim_res
        .timeline
        .sample_at_time(sim_res.timeline.total_time_sec * 0.5)
        .unwrap();

    let mut cam = CadCamera::default();
    cam.fit_aabb(&doc.aabb_mm);

    let highlight = HighlightState {
        wcs_frame: resolved.frame,
        sim_stock_source: Some(SimStockRenderSource::CpuMesh {
            revision: 1,
            mesh: stock_mesh,
            stock_model: chip_math::DMat4::IDENTITY,
        }),
        sim_tool_mesh: Some((1, tool_mesh, mid_snap.tip_pos_wcs)),
        sim_trail_limit: Some(mid_snap.global_seg_idx),
        ..HighlightState::default()
    };

    let png = render_offscreen_png(&doc, &cam, &highlight, 640, 480).unwrap();
    assert!(png.starts_with(b"\x89PNG\r\n\x1a\n"));
    assert!(png.len() > 10_000);
}

#[test]
fn renders_offscreen_viewport_in_both_dark_and_light_themes() {
    use crate::theme::AppTheme;

    let step_str = sample_machined_plate_step().unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes("plate.step", step_str.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
        .unwrap();
    let mut cam = CadCamera::default();
    cam.fit_aabb(&doc.aabb_mm);

    let dark_hl = HighlightState {
        theme: AppTheme::Dark,
        ..HighlightState::default()
    };
    let light_hl = HighlightState {
        theme: AppTheme::Light,
        ..HighlightState::default()
    };

    let dark_png = render_offscreen_png(&doc, &cam, &dark_hl, 320, 240).unwrap();
    let light_png = render_offscreen_png(&doc, &cam, &light_hl, 320, 240).unwrap();

    let dark_img = image::load_from_memory(&dark_png).unwrap().to_rgba8();
    let light_img = image::load_from_memory(&light_png).unwrap().to_rgba8();

    // Corner pixel `(4, 4)` samples the studio background gradient directly
    let dp = dark_img.get_pixel(4, 4);
    let lp = light_img.get_pixel(4, 4);
    assert!(
        dp[0] < 80 && lp[0] > 180,
        "expected dark corner {:?} < 80 and light corner {:?} > 180",
        dp,
        lp
    );
}

#[test]
fn cad_world_transform_matches_rotary_wcs_rotation_across_axes_directions_and_tilted_wcs() {
    use chip_cam::{
        ResolvedRotaryConfig, RotaryAxisKind, RotaryDirection, RotaryDriveMode, RotaryUnwindMode,
    };
    use chip_math::{DMat3, DVec3, Ray3, RigidFrame3};

    let basis = DMat3::from_axis_angle(DVec3::new(0.3, 0.5, 0.8).normalize(), 0.65);
    let wcs_frame = RigidFrame3::from_z_and_x_hint(
        DVec3::new(12.0, -7.5, 5.0),
        basis.z_axis,
        Some(basis.x_axis),
        false,
        false,
        0,
    )
    .unwrap();

    let sample_pts = [
        DVec3::new(0.0, 0.0, 0.0),
        DVec3::new(25.0, -14.0, 18.5),
        DVec3::new(-9.25, 31.0, -6.75),
    ];

    for axis in RotaryAxisKind::ALL {
        for direction in RotaryDirection::ALL {
            let rot = ResolvedRotaryConfig {
                mode: RotaryDriveMode::Simultaneous4Axis,
                axis,
                axis_letter: axis.default_letter(),
                direction,
                center_wcs: DVec3::new(3.5, -2.0, 4.25),
                stock_u_min: -40.0,
                stock_u_max: 40.0,
                stock_outer_radius_mm: 22.0,
                clearance_radius_mm: 30.0,
                chuck_end_positive: false,
                chuck_u_min: -60.0,
                chuck_u_max: -40.0,
                chuck_radius_mm: 32.0,
                unwind_mode: RotaryUnwindMode::ContinuousMultiTurn,
            };

            let hl = HighlightState {
                wcs_frame,
                rotary: Some(rot),
                sim_rotary_deg: 127.5,
                ..HighlightState::default()
            };
            assert!(hl.has_active_cad_rotary_transform());

            let mat = hl.cad_world_transform();
            for p_world in sample_pts {
                let p_wcs = wcs_frame.world_to_local_point(p_world);
                let expected_world =
                    wcs_frame.local_to_world_point(rot.rotate_point_wcs(p_wcs, 127.5));
                let actual_world = mat.transform_point3(p_world);
                assert!(
                    actual_world.distance(expected_world) < 1e-9,
                    "axis={axis:?} dir={direction:?}: actual={actual_world:?} expected={expected_world:?}"
                );
            }

            // Verify picking ray inverse round-trip
            let ray_model = Ray3::new(
                DVec3::new(40.0, -20.0, 60.0),
                DVec3::new(-0.4, 0.25, -0.88).normalize(),
            )
            .unwrap();
            let ray_world = Ray3::new(
                mat.transform_point3(ray_model.origin),
                mat.transform_vector3(ray_model.dir),
            )
            .unwrap();
            let ray_back = hl.world_ray_to_cad_model_space(&ray_world);
            assert!(ray_back.origin.distance(ray_model.origin) < 1e-9);
            assert!(ray_back.dir.distance(ray_model.dir) < 1e-9);
        }
    }
}

#[test]
fn rotary_simulation_rotates_ghost_model_solid_and_brep_edges_in_sync() {
    use chip_cam::{
        ResolvedRotaryConfig, RotaryAxisKind, RotaryDirection, RotaryDriveMode, RotaryUnwindMode,
    };
    use chip_math::{DVec3, RigidFrame3};

    let step = sample_machined_plate_step().unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes("plate.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
        .unwrap();

    let rot = ResolvedRotaryConfig {
        mode: RotaryDriveMode::Simultaneous4Axis,
        axis: RotaryAxisKind::XAxis,
        axis_letter: 'A',
        direction: RotaryDirection::RightHandCcw,
        center_wcs: DVec3::new(0.0, 20.0, 5.0),
        stock_u_min: -60.0,
        stock_u_max: 60.0,
        stock_outer_radius_mm: 40.0,
        clearance_radius_mm: 50.0,
        chuck_end_positive: false,
        chuck_u_min: -80.0,
        chuck_u_max: -60.0,
        chuck_radius_mm: 45.0,
        unwind_mode: RotaryUnwindMode::ContinuousMultiTurn,
    };

    // Pre-rotate a reference copy of the CAD render mesh + B-Rep edge polylines by +90°
    let angle_deg = 90.0;
    let mut doc_rot90 = doc.clone();
    for v in &mut doc_rot90.render_mesh.vertices {
        let p = rot.rotate_point_wcs(
            DVec3::new(
                f64::from(v.position_mm[0]),
                f64::from(v.position_mm[1]),
                f64::from(v.position_mm[2]),
            ),
            angle_deg,
        );
        let n_tip = rot.rotate_point_wcs(
            rot.center_wcs
                + DVec3::new(
                    f64::from(v.normal[0]),
                    f64::from(v.normal[1]),
                    f64::from(v.normal[2]),
                ),
            angle_deg,
        );
        let n = (n_tip - rot.center_wcs).normalize_or_zero();
        v.position_mm = [p.x as f32, p.y as f32, p.z as f32];
        v.normal = [n.x as f32, n.y as f32, n.z as f32];
    }
    for poly in &mut doc_rot90.render_mesh.edge_polylines {
        for pt in &mut poly.points_mm {
            *pt = rot.rotate_point_wcs(*pt, angle_deg);
        }
    }

    let mut cam = CadCamera::default();
    cam.fit_aabb(&chip_math::Aabb3::new(
        DVec3::new(-70.0, -70.0, -70.0),
        DVec3::new(70.0, 70.0, 70.0),
    ));

    let base_hl = HighlightState {
        wcs_frame: RigidFrame3::IDENTITY,
        rotary: Some(rot),
        ghost_part: true,
        show_stock: false,
        show_grid: false,
        ..HighlightState::default()
    };

    let mut hl_sim90 = base_hl.clone();
    hl_sim90.sim_rotary_deg = angle_deg;

    let png_sim90 = render_offscreen_png(&doc, &cam, &hl_sim90, 320, 240).unwrap();
    let png_ref90 = render_offscreen_png(&doc_rot90, &cam, &base_hl, 320, 240).unwrap();
    let png_unrot = render_offscreen_png(&doc, &cam, &base_hl, 320, 240).unwrap();

    let img_sim90 = image::load_from_memory(&png_sim90).unwrap().to_rgba8();
    let img_ref90 = image::load_from_memory(&png_ref90).unwrap().to_rgba8();
    let img_unrot = image::load_from_memory(&png_unrot).unwrap().to_rgba8();

    let mut diff_vs_ref = 0usize;
    let mut diff_vs_unrot = 0usize;
    for (p_sim, (p_ref, p_unrot)) in img_sim90
        .pixels()
        .zip(img_ref90.pixels().zip(img_unrot.pixels()))
    {
        let max_d_ref = (0..4)
            .map(|c| p_sim[c].abs_diff(p_ref[c]))
            .max()
            .unwrap_or(0);
        if max_d_ref > 2 {
            diff_vs_ref += 1;
        }
        let max_d_unrot = (0..4)
            .map(|c| p_sim[c].abs_diff(p_unrot[c]))
            .max()
            .unwrap_or(0);
        if max_d_unrot > 8 {
            diff_vs_unrot += 1;
        }
    }

    assert!(
        diff_vs_unrot > 1_000,
        "ghost model at 90° must move visibly relative to 0° (only {diff_vs_unrot} pixels changed)"
    );
    assert!(
        diff_vs_ref <= 16,
        "ghost model shaded solid + B-Rep edges at sim_rotary_deg=90° must match pre-rotated reference model (got {diff_vs_ref} differing pixels)"
    );
}

#[test]
fn growable_sim_stock_buffers_reuse_allocations_and_gpu_stock_model_rotates_cylindrical_stock() {
    use chip_cad::CadDocument;
    use chip_cam::{
        AxisAnchor, BoxAnchor27, MotionRole, RotaryAxisKind, RotaryDriveMode, StockAxis,
        StockConfig, ToolpathBuilder, WcsOriginMode,
    };
    use chip_math::{DMat4, DVec3, ParamLength};
    use chip_sim::{
        MachineKinematicsConfig, SetupSimulationInput, SimOperationInput, SimResolution,
        SimStockColorMode, SimVerificationConfig, run_setup_simulation,
    };
    use chip_stock::tessellate_cylindrical_stock_mesh_colored;
    use chip_tools::default_starter_tool_library;

    let lib = default_starter_tool_library();
    let flat_6mm = &lib.tools[0];

    let mut setup = Setup::new("s_rot_gpu_test", "Rotary A", WorkOffsetSlot::G54);
    setup.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27 {
            x: AxisAnchor::Center,
            y: AxisAnchor::Center,
            z: AxisAnchor::Center,
        },
    };
    setup.stock = StockConfig::FixedCylinder {
        axis: StockAxis::X,
        diameter: ParamLength::from_mm(40.0),
        length: ParamLength::from_mm(80.0),
        axial_top_offset: ParamLength::from_mm(0.0),
    };
    setup.rotary.mode = RotaryDriveMode::TwoLinearPlusRotary;
    setup.rotary.axis = RotaryAxisKind::XAxis;

    let doc = CadDocument::empty();
    let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

    // Carve a deep asymmetric flat/flute along +Z (A=0°) so 0° vs +90° is strongly distinct
    let mut b = ToolpathBuilder::new("op_rot_flat");
    b.rotary_rapid_to(DVec3::new(-30.0, 0.0, 28.0), 0.0);
    b.rotary_linear_to(
        DVec3::new(-30.0, 0.0, 12.0),
        0.0,
        400.0,
        12.0,
        MotionRole::Plunge,
    );
    b.rotary_linear_to(
        DVec3::new(30.0, 0.0, 12.0),
        0.0,
        600.0,
        12.0,
        MotionRole::Cut,
    );
    let tp = b.finish();

    let sim = run_setup_simulation(&SetupSimulationInput {
        doc: &doc,
        setup: &resolved,
        target_bodies: &[],
        fixtures: &[],
        operations: &[SimOperationInput {
            op_id: "op_rot_flat",
            op_name: "Asymmetric Axial Flute",
            tool: flat_6mm,
            fallback_spindle_rpm: 12_000.0,
            intentional_edge_break_mm: 0.0,
            toolpath: &tp,
        }],
        resolution: SimResolution::Standard,
        kinematics: MachineKinematicsConfig::default(),
        verify_config: SimVerificationConfig::default(),
        initial_stock: None,
        initial_raw_volume_mm3: None,
        carried_over_from_setup: None,
        cancel: None,
        ..SetupSimulationInput::new(&doc, &resolved, &[])
    })
    .unwrap();

    let cyl = sim.as_cylindrical().expect("expected CylindricalStock");
    let angle_deg = 90.0;
    let mesh_local_0 = Arc::new(tessellate_cylindrical_stock_mesh_colored(
        cyl,
        SimStockColorMode::ByOperation,
        0.0,
    ));
    let mesh_cpu_rot90 = Arc::new(tessellate_cylindrical_stock_mesh_colored(
        cyl,
        SimStockColorMode::ByOperation,
        angle_deg,
    ));

    // 1. Verify `stock_model_wcs_transform` is identity for 3-axis (rotary=None) and 0°,
    // and rotates around the rotary centerline for 4th-axis Cylindrical stock.
    let hl_3axis = HighlightState {
        wcs_frame: resolved.frame,
        rotary: None,
        sim_rotary_deg: 90.0,
        ..HighlightState::default()
    };
    assert_eq!(hl_3axis.stock_model_wcs_transform(), DMat4::IDENTITY);

    let base_hl = HighlightState {
        wcs_frame: resolved.frame,
        rotary: resolved.rotary,
        show_stock: false,
        show_grid: false,
        ..HighlightState::default()
    };
    assert_eq!(base_hl.stock_model_wcs_transform(), DMat4::IDENTITY);

    // 2. Verify reusable `GrowableGpuBuffer` stock uploads on a live Vulkan `ViewportRenderer`
    // across multiple frames & revisions without reallocating GPU buffers, and verify that
    // GPU `stock_model` rotation matches CPU pre-rotated reference pixels!
    let instance =
        wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle_from_env());
    let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
        power_preference: wgpu::PowerPreference::default(),
        compatible_surface: None,
        force_fallback_adapter: false,
        apply_limit_buckets: false,
    }))
    .unwrap();
    let (device, queue) =
        pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default())).unwrap();

    let mut vp = ViewportRenderer::new(&device);
    vp.targets = Some(FrameTargets::create(&device, 320, 240));

    let mut cam = CadCamera::default();
    cam.fit_aabb(&chip_math::Aabb3::new(
        DVec3::new(-50.0, -40.0, -40.0),
        DVec3::new(50.0, 40.0, 40.0),
    ));

    let mut hl_unrot = base_hl.clone();
    hl_unrot.sim_rotary_deg = 0.0;
    hl_unrot.sim_stock_source = Some(SimStockRenderSource::CpuMesh {
        revision: 1,
        mesh: Arc::clone(&mesh_local_0),
        stock_model: hl_unrot.stock_model_wcs_transform(),
    });

    let mut hl_gpu90 = base_hl.clone();
    hl_gpu90.sim_rotary_deg = angle_deg;
    hl_gpu90.sim_stock_source = Some(SimStockRenderSource::CpuMesh {
        revision: 1,
        mesh: Arc::clone(&mesh_local_0),
        stock_model: hl_gpu90.stock_model_wcs_transform(),
    });

    let mut hl_rev2 = hl_gpu90.clone();
    hl_rev2.sim_stock_source = Some(SimStockRenderSource::CpuMesh {
        revision: 2,
        mesh: Arc::clone(&mesh_local_0),
        stock_model: hl_rev2.stock_model_wcs_transform(),
    });

    // Frame 1: initial upload of rev 1
    vp.render_internal(
        &device,
        &queue,
        &cam,
        Some((&doc, 1)),
        &hl_unrot,
        DisplayUnit::Millimeter,
    );
    let v_cap_1 = vp.uploaded_sim_stock.vertex_capacity_bytes();
    let i_cap_1 = vp.uploaded_sim_stock.index_capacity_bytes();
    assert!(v_cap_1 > 0 && i_cap_1 > 0);
    assert_eq!(vp.uploaded_sim_stock.uploaded_revision(), Some(1));

    // Frame 2: same rev 1 with `sim_rotary_deg = 90°` -> zero re-upload, zero reallocation!
    vp.render_internal(
        &device,
        &queue,
        &cam,
        Some((&doc, 1)),
        &hl_gpu90,
        DisplayUnit::Millimeter,
    );
    assert_eq!(vp.uploaded_sim_stock.vertex_capacity_bytes(), v_cap_1);
    assert_eq!(vp.uploaded_sim_stock.index_capacity_bytes(), i_cap_1);
    assert_eq!(vp.uploaded_sim_stock.uploaded_revision(), Some(1));

    // Frame 3: bumped rev 2 of equal size -> reuses existing GrowableGpuBuffer allocations!
    vp.render_internal(
        &device,
        &queue,
        &cam,
        Some((&doc, 1)),
        &hl_rev2,
        DisplayUnit::Millimeter,
    );
    assert_eq!(vp.uploaded_sim_stock.vertex_capacity_bytes(), v_cap_1);
    assert_eq!(vp.uploaded_sim_stock.index_capacity_bytes(), i_cap_1);
    assert_eq!(vp.uploaded_sim_stock.uploaded_revision(), Some(2));

    // Frame 4: pixel-accurate verification of GPU `stock_model` 90° vs CPU pre-rotated 90° & unrotated 0°
    let mut hl_cpu90 = base_hl;
    hl_cpu90.sim_rotary_deg = 0.0;
    hl_cpu90.sim_stock_source = Some(SimStockRenderSource::CpuMesh {
        revision: 1,
        mesh: mesh_cpu_rot90,
        stock_model: DMat4::IDENTITY,
    });

    let png_gpu90 = render_offscreen_png(&doc, &cam, &hl_gpu90, 320, 240).unwrap();
    let png_cpu90 = render_offscreen_png(&doc, &cam, &hl_cpu90, 320, 240).unwrap();
    let png_unrot = render_offscreen_png(&doc, &cam, &hl_unrot, 320, 240).unwrap();

    let img_gpu90 = image::load_from_memory(&png_gpu90).unwrap().to_rgba8();
    let img_cpu90 = image::load_from_memory(&png_cpu90).unwrap().to_rgba8();
    let img_unrot = image::load_from_memory(&png_unrot).unwrap().to_rgba8();

    let mut diff_vs_cpu90 = 0usize;
    let mut diff_vs_unrot = 0usize;
    for (p_gpu, (p_cpu, p_unrot)) in img_gpu90
        .pixels()
        .zip(img_cpu90.pixels().zip(img_unrot.pixels()))
    {
        let max_d_cpu = (0..4)
            .map(|c| p_gpu[c].abs_diff(p_cpu[c]))
            .max()
            .unwrap_or(0);
        if max_d_cpu > 2 {
            diff_vs_cpu90 += 1;
        }
        let max_d_unrot = (0..4)
            .map(|c| p_gpu[c].abs_diff(p_unrot[c]))
            .max()
            .unwrap_or(0);
        if max_d_unrot > 8 {
            diff_vs_unrot += 1;
        }
    }

    assert!(
        diff_vs_unrot > 500,
        "GPU stock_model rotation at 90° must visibly rotate machined flute relative to 0° (only {diff_vs_unrot} pixels changed)"
    );
    assert!(
        diff_vs_cpu90 <= 16,
        "GPU stock_model rotation at 90° must match CPU pre-rotated cylindrical stock reference (got {diff_vs_cpu90} differing pixels)"
    );
}

#[cfg(feature = "gpu-compute")]
#[test]
fn gpu_grid_direct_render_matches_cpu_mesh_across_heightfield_cylindrical_and_tridexel() {
    use crate::viewport::GpuStockRenderHandle;
    use chip_cad::CadDocument;
    use chip_cam::{
        AxisAnchor, BoxAnchor27, MotionRole, RotaryAxisKind, RotaryDriveMode, Setup, StockAxis,
        StockConfig, ToolpathBuilder, WcsOriginMode, WorkOffsetSlot,
    };
    use chip_math::{Aabb3, DMat4, DVec3, ParamLength};
    use chip_sim::{
        GpuSimContext, MachineKinematicsConfig, SetupSimulationInput, SimComputeBackendChoice,
        SimEngineKind, SimMeshOptions, SimOperationInput, SimResolution, SimStockColorMode,
        SimVerificationConfig, WgpuComputeBackend, run_setup_simulation,
    };
    use chip_tools::default_starter_tool_library;

    let ctx = GpuSimContext::shared_headless().expect("expected Vulkan/wgpu compute adapter");
    let device = ctx.device();
    let queue = ctx.queue();
    let lib = default_starter_tool_library();
    let flat_6mm = &lib.tools[0];
    let (w, h) = (320u32, 240u32);

    let compare_rgba =
        |label: &str, bg: &[u8], cpu: &[u8], gpu: &[u8], min_iou: f64, max_rgb_mae: f64| {
            let mut bg_diff_cpu = 0usize;
            let mut bg_diff_gpu = 0usize;
            let mut inter = 0usize;
            let mut union = 0usize;
            let mut rgb_err_sum = 0.0_f64;
            for px in 0..(w * h) as usize {
                let o = px * 4;
                let dc = (0..3)
                    .map(|c| cpu[o + c].abs_diff(bg[o + c]))
                    .max()
                    .unwrap_or(0);
                let dg = (0..3)
                    .map(|c| gpu[o + c].abs_diff(bg[o + c]))
                    .max()
                    .unwrap_or(0);
                let c_fg = dc > 8;
                let g_fg = dg > 8;
                if c_fg {
                    bg_diff_cpu += 1;
                }
                if g_fg {
                    bg_diff_gpu += 1;
                }
                if c_fg || g_fg {
                    union += 1;
                }
                if c_fg && g_fg {
                    inter += 1;
                    let err = (cpu[o].abs_diff(gpu[o]) as f64
                        + cpu[o + 1].abs_diff(gpu[o + 1]) as f64
                        + cpu[o + 2].abs_diff(gpu[o + 2]) as f64)
                        / 3.0;
                    rgb_err_sum += err;
                }
            }
            assert!(
                bg_diff_cpu > 1_500 && bg_diff_gpu > 1_500,
                "{label}: expected rich stock coverage (cpu_fg={bg_diff_cpu}, gpu_fg={bg_diff_gpu})"
            );
            let iou = inter as f64 / union.max(1) as f64;
            let mae = rgb_err_sum / inter.max(1) as f64;
            assert!(
                iou >= min_iou,
                "{label}: silhouette IoU {iou:.3} below {min_iou:.3} (inter={inter}, union={union})"
            );
            assert!(
                mae <= max_rgb_mae,
                "{label}: overlapping RGB MAE {mae:.2} exceeded {max_rgb_mae:.2}"
            );
        };

    let count_rgb_diff = |a: &[u8], b: &[u8]| -> usize {
        (0..(w * h) as usize)
            .filter(|&px| {
                let o = px * 4;
                (0..3)
                    .map(|c| a[o + c].abs_diff(b[o + c]))
                    .max()
                    .unwrap_or(0)
                    > 15
            })
            .count()
    };

    let doc_empty = CadDocument::empty();

    // === 1. Heightfield (ByOperation + ErrorHeatmap) ===
    {
        let mut setup = Setup::new("s_hf_par", "HF Parity", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        };
        setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(100.0),
            depth_y: ParamLength::from_mm(100.0),
            height_z: ParamLength::from_mm(20.0),
            xy_align: chip_cam::FixedBoxXyAlign::CenterXy,
            z_align: chip_cam::FixedBoxZAlign::CenterZ,
        };
        let resolved = setup.resolve(&doc_empty, DisplayUnit::Millimeter).unwrap();

        let mut b = ToolpathBuilder::new("op_hf");
        b.rapid_to(DVec3::new(-25.0, 0.0, 10.0));
        b.linear_to(DVec3::new(-25.0, 0.0, -6.0), 300.0, MotionRole::Plunge);
        b.linear_to(DVec3::new(25.0, 0.0, -6.0), 600.0, MotionRole::Cut);
        // Through-hole to exercise inactive cell discarding & vertical hole walls
        b.rapid_to(DVec3::new(0.0, 15.0, 10.0));
        b.linear_to(DVec3::new(0.0, 15.0, -25.0), 300.0, MotionRole::Plunge);
        let tp = b.finish();

        let mut sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc_empty,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput {
                op_id: "op_hf",
                op_name: "Slot + Hole",
                tool: flat_6mm,
                fallback_spindle_rpm: 12_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp,
            }],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            engine: SimEngineKind::Heightmap,
            backend_choice: SimComputeBackendChoice::Gpu,
            gpu_ctx: Some(&ctx),
            ..SetupSimulationInput::new(&doc_empty, &resolved, &[])
        })
        .unwrap();
        // Inject synthetic flat target surface at z_wcs = -4.0 (gouge in slot, excess on top)
        if let Some(hf) = sim.final_stock.as_heightfield_mut() {
            let zt = hf.z_wcs_to_local(-4.0);
            hf.z_target.fill(zt);
        }

        let mut backend = WgpuComputeBackend::new(Arc::clone(&ctx), sim.final_stock.clone(), None);
        let gpu_handle = GpuStockRenderHandle::from_buffer_snapshot(
            &ctx,
            &backend.capture_buffer_snapshot(),
            SimMeshOptions::from(SimStockColorMode::ByOperation),
        );

        let mut cam = CadCamera::default();
        cam.fit_aabb(&Aabb3::new(
            DVec3::new(-45.0, -35.0, -25.0),
            DVec3::new(45.0, 35.0, 5.0),
        ));
        let base_hl = HighlightState {
            wcs_frame: resolved.frame,
            show_stock: false,
            show_grid: false,
            ..HighlightState::default()
        };
        let bg =
            render_offscreen_rgba_on_device(device, queue, None, &cam, &base_hl, w, h).unwrap();

        let mut gpu_by_op_px = Vec::new();
        let mut gpu_hm_px = Vec::new();
        for (name, mode) in [
            ("Heightfield ByOperation", SimStockColorMode::ByOperation),
            (
                "Heightfield ErrorHeatmap",
                SimStockColorMode::ErrorHeatmap {
                    tolerance_mm: 0.05,
                    max_scale_mm: 2.0,
                },
            ),
        ] {
            let cpu_mesh = Arc::new(sim.extract_stock_mesh(&sim.final_stock, mode));
            let mut hl_cpu = base_hl.clone();
            hl_cpu.sim_stock_source = Some(SimStockRenderSource::CpuMesh {
                revision: 1,
                mesh: cpu_mesh,
                stock_model: DMat4::IDENTITY,
            });
            let mut hl_gpu = base_hl.clone();
            hl_gpu.sim_stock_source = Some(SimStockRenderSource::GpuGrid {
                revision: 1,
                handle: Arc::clone(&gpu_handle),
                stock_model: DMat4::IDENTITY,
                color_mode: mode,
                mesh_skinning: false,
            });
            let px_cpu =
                render_offscreen_rgba_on_device(device, queue, None, &cam, &hl_cpu, w, h).unwrap();
            let px_gpu =
                render_offscreen_rgba_on_device(device, queue, None, &cam, &hl_gpu, w, h).unwrap();
            compare_rgba(name, &bg, &px_cpu, &px_gpu, 0.96, 6.0);
            if matches!(mode, SimStockColorMode::ByOperation) {
                gpu_by_op_px = px_gpu;
            } else {
                gpu_hm_px = px_gpu;
            }
        }
        assert!(
            count_rgb_diff(&gpu_by_op_px, &gpu_hm_px) > 1_000,
            "Heightfield GPU ErrorHeatmap must visually differ from ByOperation"
        );
    }

    // === 2. 4th-Axis Cylindrical (ByOperation + ErrorHeatmap + GPU stock_model rotation) ===
    {
        let mut setup = Setup::new("s_cyl_par", "Cyl Parity", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27 {
                x: AxisAnchor::Center,
                y: AxisAnchor::Center,
                z: AxisAnchor::Center,
            },
        };
        setup.stock = StockConfig::FixedCylinder {
            axis: StockAxis::X,
            diameter: ParamLength::from_mm(40.0),
            length: ParamLength::from_mm(80.0),
            axial_top_offset: ParamLength::from_mm(0.0),
        };
        setup.rotary.mode = RotaryDriveMode::TwoLinearPlusRotary;
        setup.rotary.axis = RotaryAxisKind::XAxis;
        let resolved = setup.resolve(&doc_empty, DisplayUnit::Millimeter).unwrap();

        let mut b = ToolpathBuilder::new("op_cyl");
        b.rotary_rapid_to(DVec3::new(-28.0, 0.0, 28.0), 0.0);
        b.rotary_linear_to(
            DVec3::new(-28.0, 0.0, 13.0),
            0.0,
            400.0,
            13.0,
            MotionRole::Plunge,
        );
        b.rotary_linear_to(
            DVec3::new(28.0, 0.0, 13.0),
            0.0,
            600.0,
            13.0,
            MotionRole::Cut,
        );
        let tp = b.finish();

        let mut sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc_empty,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput {
                op_id: "op_cyl",
                op_name: "Rotary Flute",
                tool: flat_6mm,
                fallback_spindle_rpm: 12_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp,
            }],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            engine: SimEngineKind::Heightmap,
            backend_choice: SimComputeBackendChoice::Gpu,
            gpu_ctx: Some(&ctx),
            ..SetupSimulationInput::new(&doc_empty, &resolved, &[])
        })
        .unwrap();
        if let Some(cyl) = sim.final_stock.as_cylindrical_mut() {
            cyl.r_target.fill(16.0_f32);
        }

        let mut backend = WgpuComputeBackend::new(Arc::clone(&ctx), sim.final_stock.clone(), None);
        let gpu_handle = GpuStockRenderHandle::from_buffer_snapshot(
            &ctx,
            &backend.capture_buffer_snapshot(),
            SimMeshOptions::from(SimStockColorMode::ByOperation),
        );

        let mut cam = CadCamera::default();
        cam.fit_aabb(&Aabb3::new(
            DVec3::new(-45.0, -30.0, -30.0),
            DVec3::new(45.0, 30.0, 30.0),
        ));
        let base_hl = HighlightState {
            wcs_frame: resolved.frame,
            rotary: resolved.rotary,
            sim_rotary_deg: 60.0,
            show_stock: false,
            show_grid: false,
            ..HighlightState::default()
        };
        let stock_model = base_hl.stock_model_wcs_transform();
        let bg =
            render_offscreen_rgba_on_device(device, queue, None, &cam, &base_hl, w, h).unwrap();

        let mut gpu_by_op_px = Vec::new();
        let mut gpu_hm_px = Vec::new();
        for (name, mode) in [
            ("Cylindrical ByOperation", SimStockColorMode::ByOperation),
            (
                "Cylindrical ErrorHeatmap",
                SimStockColorMode::ErrorHeatmap {
                    tolerance_mm: 0.05,
                    max_scale_mm: 3.0,
                },
            ),
        ] {
            let cpu_mesh = Arc::new(sim.extract_stock_mesh(&sim.final_stock, mode));
            let mut hl_cpu = base_hl.clone();
            hl_cpu.sim_stock_source = Some(SimStockRenderSource::CpuMesh {
                revision: 1,
                mesh: cpu_mesh,
                stock_model,
            });
            let mut hl_gpu = base_hl.clone();
            hl_gpu.sim_stock_source = Some(SimStockRenderSource::GpuGrid {
                revision: 1,
                handle: Arc::clone(&gpu_handle),
                stock_model,
                color_mode: mode,
                mesh_skinning: false,
            });
            let px_cpu =
                render_offscreen_rgba_on_device(device, queue, None, &cam, &hl_cpu, w, h).unwrap();
            let px_gpu =
                render_offscreen_rgba_on_device(device, queue, None, &cam, &hl_gpu, w, h).unwrap();
            compare_rgba(name, &bg, &px_cpu, &px_gpu, 0.96, 8.0);
            if matches!(mode, SimStockColorMode::ByOperation) {
                gpu_by_op_px = px_gpu;
            } else {
                gpu_hm_px = px_gpu;
            }
        }
        assert!(
            count_rgb_diff(&gpu_by_op_px, &gpu_hm_px) > 1_000,
            "Cylindrical GPU ErrorHeatmap must visually differ from ByOperation"
        );
    }

    // === 3. Tri-Dexel (tridexel_emit.wgsl + draw_indirect: ByOperation + instant GPU recolor to ErrorHeatmap) ===
    {
        let step = sample_machined_plate_step().unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("plate.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();
        let mut setup = Setup::new("s_td_par", "TD Parity", WorkOffsetSlot::G54);
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
        let aabb = resolved.stock.aabb_wcs();
        let center = aabb.center();

        let mut b = ToolpathBuilder::new("op_td");
        b.rapid_to(DVec3::new(center.x - 15.0, center.y, aabb.max.z + 10.0));
        b.linear_to(
            DVec3::new(center.x - 15.0, center.y, aabb.max.z - 5.0),
            300.0,
            MotionRole::Plunge,
        );
        b.linear_to(
            DVec3::new(center.x + 15.0, center.y, aabb.max.z - 5.0),
            600.0,
            MotionRole::Cut,
        );
        let tp = b.finish();

        let sim = run_setup_simulation(&SetupSimulationInput {
            doc: &doc,
            setup: &resolved,
            target_bodies: &[],
            fixtures: &[],
            operations: &[SimOperationInput {
                op_id: "op_td",
                op_name: "TD Slot",
                tool: flat_6mm,
                fallback_spindle_rpm: 12_000.0,
                intentional_edge_break_mm: 0.0,
                toolpath: &tp,
            }],
            resolution: SimResolution::Standard,
            kinematics: MachineKinematicsConfig::default(),
            verify_config: SimVerificationConfig::default(),
            initial_stock: None,
            initial_raw_volume_mm3: None,
            carried_over_from_setup: None,
            cancel: None,
            engine: SimEngineKind::TriDexel,
            backend_choice: SimComputeBackendChoice::Gpu,
            gpu_ctx: Some(&ctx),
            ..SetupSimulationInput::new(&doc, &resolved, &[])
        })
        .unwrap();

        let mut backend = WgpuComputeBackend::new(
            Arc::clone(&ctx),
            sim.final_stock.clone(),
            sim.target_tridexel_stock.clone(),
        );
        let gpu_handle = GpuStockRenderHandle::from_buffer_snapshot(
            &ctx,
            &backend.capture_buffer_snapshot(),
            SimMeshOptions::from(SimStockColorMode::ByOperation),
        );
        let GpuStockRenderHandle::TriDexel(ref td_data) = *gpu_handle else {
            panic!("expected TriDexel GPU render handle");
        };
        assert_eq!(td_data.emit_dispatch_count(), 1);

        let mut cam = CadCamera::default();
        cam.fit_aabb(&doc.aabb_mm);
        let base_hl = HighlightState {
            wcs_frame: resolved.frame,
            show_stock: false,
            show_grid: false,
            ..HighlightState::default()
        };
        let bg =
            render_offscreen_rgba_on_device(device, queue, None, &cam, &base_hl, w, h).unwrap();

        let mut gpu_by_op_px = Vec::new();
        let mut gpu_hm_px = Vec::new();
        for (name, mode) in [
            ("TriDexel ByOperation", SimStockColorMode::ByOperation),
            (
                "TriDexel ErrorHeatmap",
                SimStockColorMode::ErrorHeatmap {
                    tolerance_mm: 0.05,
                    max_scale_mm: 2.0,
                },
            ),
        ] {
            let cpu_mesh = Arc::new(sim.extract_stock_mesh(&sim.final_stock, mode));
            let mut hl_cpu = base_hl.clone();
            hl_cpu.sim_stock_source = Some(SimStockRenderSource::CpuMesh {
                revision: 1,
                mesh: cpu_mesh,
                stock_model: DMat4::IDENTITY,
            });
            let mut hl_gpu = base_hl.clone();
            hl_gpu.sim_stock_source = Some(SimStockRenderSource::GpuGrid {
                revision: 1,
                handle: Arc::clone(&gpu_handle),
                stock_model: DMat4::IDENTITY,
                color_mode: mode,
                mesh_skinning: false,
            });
            let px_cpu =
                render_offscreen_rgba_on_device(device, queue, None, &cam, &hl_cpu, w, h).unwrap();
            let px_gpu =
                render_offscreen_rgba_on_device(device, queue, None, &cam, &hl_gpu, w, h).unwrap();
            compare_rgba(name, &bg, &px_cpu, &px_gpu, 0.90, 22.0);
            if matches!(mode, SimStockColorMode::ByOperation) {
                gpu_by_op_px = px_gpu;
            } else {
                gpu_hm_px = px_gpu;
            }
        }
        // Same handle re-dispatched tridexel_emit exactly once when switching to ErrorHeatmap!
        assert_eq!(td_data.emit_dispatch_count(), 2);
        assert!(
            count_rgb_diff(&gpu_by_op_px, &gpu_hm_px) > 1_000,
            "TriDexel GPU ErrorHeatmap must visually differ from ByOperation"
        );
    }
}

#[test]
fn renders_translucent_workholding_fixtures_and_toggles_cleanly() {
    use chip_cam::{FixtureBody, resolve_setup_fixtures_parametric};
    use chip_math::DVec2;

    use crate::viewport::FixtureVisualMesh;

    let step = sample_multi_feature_bracket_step(false).unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes("bracket.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
        .unwrap();

    let mut setup = Setup::new("s_fix", "Vise + Clamps Setup", WorkOffsetSlot::G54);
    setup.fixtures = vec![
        FixtureBody::default_vise("v1", "4in Vise"),
        FixtureBody::default_toe_clamp("c1", "Right Clamp", DVec2::new(47.0, 0.0), 0.0),
        FixtureBody::default_spoilboard("sb1", "MDF Spoilboard"),
    ];
    let resolved = setup
        .resolve_immutable(&doc, DisplayUnit::Millimeter)
        .unwrap();
    let solids =
        resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter).unwrap();
    assert_eq!(solids.len(), 3);

    let fixture_meshes: Vec<FixtureVisualMesh> = solids
        .into_iter()
        .enumerate()
        .map(|(idx, sol)| FixtureVisualMesh {
            revision: 1,
            fixture_id: sol.fixture_id,
            mesh: Arc::new(sol.mesh_wcs),
            base_rgba: sol.color_rgba,
            colliding: false,
            selected: idx == 1,
        })
        .collect();

    let mut cam = CadCamera::default();
    cam.fit_aabb(&chip_math::Aabb3::new(
        chip_math::DVec3::new(-90.0, -90.0, -60.0),
        chip_math::DVec3::new(110.0, 90.0, 40.0),
    ));

    let hl_on = HighlightState {
        wcs_frame: resolved.frame,
        stock: Some(resolved.stock),
        show_stock: true,
        show_fixtures: true,
        fixture_meshes: fixture_meshes.clone(),
        ..HighlightState::default()
    };
    let hl_off = HighlightState {
        show_fixtures: false,
        ..hl_on.clone()
    };

    let png_on = render_offscreen_png(&doc, &cam, &hl_on, 480, 360).unwrap();
    let png_off = render_offscreen_png(&doc, &cam, &hl_off, 480, 360).unwrap();
    let img_on = image::load_from_memory(&png_on).unwrap().to_rgba8();
    let img_off = image::load_from_memory(&png_off).unwrap().to_rgba8();

    let diff_px = img_on
        .pixels()
        .zip(img_off.pixels())
        .filter(|(a, b)| (0..3).any(|c| a[c].abs_diff(b[c]) > 10))
        .count();
    assert!(
        diff_px > 4_000,
        "expected translucent workholding fixtures to visibly change >4000 pixels when toggled on vs off (got {diff_px})"
    );
}

#[cfg(feature = "gpu-compute")]
#[test]
fn tridexel_ultra_gpu_renders_flat_top_z_surfaces_without_truncation() {
    use crate::viewport::GpuStockRenderHandle;
    use chip_math::{Aabb3, DMat4, DVec3, RigidFrame3};
    use chip_sim::{
        GpuSimContext, RadialCutterProfile, SimMeshOptions, SimResolution, SimStockBackend,
        SimStockColorMode, SimulatedStock, StockShapeSpec, StockVerifyTolerances, SweepChord,
        SweepMotion, SweepToolSpec, TriDexelStock, WgpuComputeBackend,
    };
    use chip_stock::tessellate_tridexel_stock_mesh_colored;
    use chip_tools::default_starter_tool_library;

    use crate::camera::StandardView;

    let ctx = GpuSimContext::shared_headless().expect("expected Vulkan/wgpu compute adapter");
    let device = ctx.device();
    let queue = ctx.queue();
    let lib = default_starter_tool_library();
    let flat_6mm = &lib.tools[0];
    let tool_spec = SweepToolSpec {
        profile: RadialCutterProfile::from_tool(flat_6mm),
        is_flat_endmill: true,
    };

    // 100 × 80 × 29 mm stock at Ultra (`448 × 358 × 130`), matching `undercut-deburr`.
    let aabb = Aabb3::new(DVec3::new(-50.0, -40.0, -29.0), DVec3::new(50.0, 40.0, 0.0));
    let td = TriDexelStock::from_stock_spec(
        &StockShapeSpec::Box { aabb_wcs: aabb },
        SimResolution::Ultra,
    )
    .unwrap();
    let tol = StockVerifyTolerances::default();
    let chords = vec![
        SweepChord {
            motion: SweepMotion::Linear {
                p0: DVec3::new(-55.0, -20.0, -14.0),
                p1: DVec3::new(55.0, -20.0, -14.0),
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
                p0: DVec3::new(-55.0, 12.0, -14.0),
                p1: DVec3::new(55.0, 12.0, -14.0),
                rotary_deg_0: 0.0,
                rotary_deg_1: 0.0,
            },
            op_idx: 1,
            is_rapid: false,
            profile_idx: 0,
            verify_tol: tol,
        },
    ];

    let mut backend = WgpuComputeBackend::new(
        Arc::clone(&ctx),
        SimulatedStock::TriDexel(Box::new(td)),
        None,
    );
    let _ = backend.sweep_batch(&chords, &[tool_spec], false);
    let gpu_handle = GpuStockRenderHandle::from_buffer_snapshot(
        &ctx,
        &backend.capture_buffer_snapshot(),
        SimMeshOptions::from(SimStockColorMode::ByOperation),
    );
    let GpuStockRenderHandle::TriDexel(ref td_render) = *gpu_handle else {
        panic!("expected TriDexel GPU render handle");
    };
    let emitted_verts = td_render.readback_emitted_vertex_count();
    assert!(
        emitted_verts > 2_400_000,
        "Ultra Tri-Dexel render fixture must emit >2.4M vertices (got {emitted_verts})"
    );

    let (SimulatedStock::TriDexel(td_cpu), _) = backend.into_parts() else {
        panic!("expected TriDexel stock");
    };
    let cpu_mesh = Arc::new(tessellate_tridexel_stock_mesh_colored(
        &td_cpu,
        None,
        SimStockColorMode::ByOperation,
    ));

    // Top-down orthographic-like view straight onto the flat +Z top surfaces (Z-plane rays).
    let (w, h) = (320u32, 240u32);
    let mut cam = CadCamera::default();
    cam.fit_aabb(&aabb);
    cam.set_standard_view(StandardView::Top);
    let base_hl = HighlightState {
        wcs_frame: RigidFrame3::IDENTITY,
        show_stock: false,
        show_grid: false,
        ..HighlightState::default()
    };
    let bg = render_offscreen_rgba_on_device(device, queue, None, &cam, &base_hl, w, h).unwrap();

    let mut hl_cpu = base_hl.clone();
    hl_cpu.sim_stock_source = Some(SimStockRenderSource::CpuMesh {
        revision: 1,
        mesh: cpu_mesh,
        stock_model: DMat4::IDENTITY,
    });
    let mut hl_gpu = base_hl;
    hl_gpu.sim_stock_source = Some(SimStockRenderSource::GpuGrid {
        revision: 1,
        handle: Arc::clone(&gpu_handle),
        stock_model: DMat4::IDENTITY,
        color_mode: SimStockColorMode::ByOperation,
        mesh_skinning: false,
    });

    let px_cpu = render_offscreen_rgba_on_device(device, queue, None, &cam, &hl_cpu, w, h).unwrap();
    let px_gpu = render_offscreen_rgba_on_device(device, queue, None, &cam, &hl_gpu, w, h).unwrap();

    let mut inter = 0usize;
    let mut union = 0usize;
    let mut cpu_fg_cnt = 0usize;
    let mut gpu_fg_cnt = 0usize;
    for px in 0..(w * h) as usize {
        let o = px * 4;
        let dc = (0..3)
            .map(|c| px_cpu[o + c].abs_diff(bg[o + c]))
            .max()
            .unwrap_or(0);
        let dg = (0..3)
            .map(|c| px_gpu[o + c].abs_diff(bg[o + c]))
            .max()
            .unwrap_or(0);
        let c_fg = dc > 8;
        let g_fg = dg > 8;
        if c_fg {
            cpu_fg_cnt += 1;
        }
        if g_fg {
            gpu_fg_cnt += 1;
        }
        if c_fg || g_fg {
            union += 1;
        }
        if c_fg && g_fg {
            inter += 1;
        }
    }
    let iou = inter as f64 / union.max(1) as f64;
    assert!(
        cpu_fg_cnt > 10_000 && gpu_fg_cnt > 10_000 && iou >= 0.95,
        "Ultra Tri-Dexel flat +Z surfaces missing on GPU: cpu_fg={cpu_fg_cnt}, gpu_fg={gpu_fg_cnt}, iou={iou:.4}"
    );
}

#[cfg(feature = "gpu-compute")]
#[test]
fn tridexel_curved_cylinder_gpu_matches_cpu_surface_nets_without_ribbon_stripes() {
    use crate::viewport::GpuStockRenderHandle;
    use chip_math::{Aabb3, DMat4, DVec3, RigidFrame3};
    use chip_sim::{
        Axis3, GpuSimContext, RotaryStockSpec, SimMeshOptions, SimResolution, SimStockBackend,
        SimStockColorMode, SimulatedStock, StockShapeSpec, StockVerifyTolerances, SweepChord,
        SweepMotion, SweepToolSpec, TriDexelStock, WgpuComputeBackend,
    };
    use chip_stock::tessellate_tridexel_stock_mesh_colored;
    use chip_tools::default_starter_tool_library;

    use crate::camera::StandardView;

    let Some(ctx) = GpuSimContext::shared_headless() else {
        return;
    };
    let device = ctx.device();
    let queue = ctx.queue();

    let r_stock = 21.0;
    let len = 80.0;
    let aabb = Aabb3::new(
        DVec3::new(-0.5 * len, -r_stock, -r_stock),
        DVec3::new(0.5 * len, r_stock, r_stock),
    );
    let stock_spec = StockShapeSpec::Cylinder {
        axis: Axis3::X,
        center_wcs: DVec3::ZERO,
        radius_mm: r_stock,
        min_axial_mm: -0.5 * len,
        max_axial_mm: 0.5 * len,
        aabb_wcs: aabb,
    };
    let mut td = TriDexelStock::from_stock_spec(&stock_spec, SimResolution::Standard).unwrap();
    td.rotary = Some(RotaryStockSpec {
        axis: Axis3::X,
        direction_sign: 1.0,
        axis_letter: 'A',
        center_wcs: DVec3::ZERO,
        stock_u_min: -0.5 * len,
        stock_u_max: 0.5 * len,
        stock_outer_radius_mm: r_stock,
        chuck_u_min: -60.0,
        chuck_u_max: -40.0,
        chuck_radius_mm: 32.5,
    });

    let lib = default_starter_tool_library();
    let tool_spec = SweepToolSpec {
        profile: chip_sim::RadialCutterProfile::from_tool(&lib.tools[0]),
        is_flat_endmill: true,
    };
    let chords = vec![SweepChord {
        motion: SweepMotion::Linear {
            p0: DVec3::new(-22.0, 0.0, 14.0),
            p1: DVec3::new(22.0, 0.0, 14.0),
            rotary_deg_0: 0.0,
            rotary_deg_1: 120.0,
        },
        op_idx: 0,
        is_rapid: false,
        profile_idx: 0,
        verify_tol: StockVerifyTolerances::default(),
    }];

    let mut backend = WgpuComputeBackend::new(
        Arc::clone(&ctx),
        SimulatedStock::TriDexel(Box::new(td)),
        None,
    );
    let _ = backend.sweep_batch(&chords, &[tool_spec], false);
    let gpu_handle = GpuStockRenderHandle::from_buffer_snapshot(
        &ctx,
        &backend.capture_buffer_snapshot(),
        SimMeshOptions::from(SimStockColorMode::ByOperation),
    );
    let (SimulatedStock::TriDexel(td_cpu), _) = backend.into_parts() else {
        panic!("expected TriDexel stock");
    };
    let cpu_mesh = Arc::new(tessellate_tridexel_stock_mesh_colored(
        &td_cpu,
        None,
        SimStockColorMode::ByOperation,
    ));

    let (w, h) = (320u32, 240u32);
    let mut cam = CadCamera::default();
    cam.fit_aabb(&aabb);
    cam.set_standard_view(StandardView::Isometric);
    let base_hl = HighlightState {
        wcs_frame: RigidFrame3::IDENTITY,
        show_stock: false,
        show_grid: false,
        ..HighlightState::default()
    };
    let bg = render_offscreen_rgba_on_device(device, queue, None, &cam, &base_hl, w, h).unwrap();

    let mut hl_cpu = base_hl.clone();
    hl_cpu.sim_stock_source = Some(SimStockRenderSource::CpuMesh {
        revision: 1,
        mesh: cpu_mesh,
        stock_model: DMat4::IDENTITY,
    });
    let mut hl_gpu = base_hl;
    hl_gpu.sim_stock_source = Some(SimStockRenderSource::GpuGrid {
        revision: 1,
        handle: Arc::clone(&gpu_handle),
        stock_model: DMat4::IDENTITY,
        color_mode: SimStockColorMode::ByOperation,
        mesh_skinning: false,
    });

    let px_cpu = render_offscreen_rgba_on_device(device, queue, None, &cam, &hl_cpu, w, h).unwrap();
    let px_gpu = render_offscreen_rgba_on_device(device, queue, None, &cam, &hl_gpu, w, h).unwrap();

    let mut fg_overlap = 0usize;
    let mut union = 0usize;
    let mut rgb_diff_sum = 0u64;
    let mut large_stripe_pixels = 0usize;
    for px in 0..(w * h) as usize {
        let o = px * 4;
        let c_fg = (0..3)
            .map(|c| px_cpu[o + c].abs_diff(bg[o + c]))
            .max()
            .unwrap_or(0)
            > 8;
        let g_fg = (0..3)
            .map(|c| px_gpu[o + c].abs_diff(bg[o + c]))
            .max()
            .unwrap_or(0)
            > 8;
        if c_fg || g_fg {
            union += 1;
        }
        if c_fg && g_fg {
            fg_overlap += 1;
            let max_ch_diff = (0..3)
                .map(|c| px_cpu[o + c].abs_diff(px_gpu[o + c]))
                .max()
                .unwrap_or(0);
            rgb_diff_sum += u64::from(max_ch_diff);
            if max_ch_diff > 28 {
                large_stripe_pixels += 1;
            }
        }
    }
    let iou = fg_overlap as f64 / union.max(1) as f64;
    let mean_diff = rgb_diff_sum as f64 / fg_overlap.max(1) as f64;
    let stripe_frac = large_stripe_pixels as f64 / fg_overlap.max(1) as f64;
    assert!(
        fg_overlap > 10_000 && iou >= 0.96,
        "GPU curved cylinder silhouette must match CPU Surface Nets (overlap={fg_overlap}, iou={iou:.4})"
    );
    assert!(
        mean_diff < 6.0 && stripe_frac < 0.03,
        "GPU curved cylinder shading must match CPU Surface Nets without dark ribbon stripes (mean_diff={mean_diff:.2}, stripe_frac={stripe_frac:.4})"
    );

    // Also verify Smooth Skinned mode (`mesh_skinning = true`) renders cleanly on both CPU & GPU
    // and matches between CPU and GPU pipelines!
    let cpu_mesh_skin = Arc::new(chip_stock::tessellate_tridexel_stock_mesh_with_options(
        &td_cpu,
        None,
        chip_sim::SimMeshOptions::new(SimStockColorMode::ByOperation, true),
    ));
    hl_cpu.sim_stock_source = Some(SimStockRenderSource::CpuMesh {
        revision: 2,
        mesh: cpu_mesh_skin,
        stock_model: DMat4::IDENTITY,
    });
    hl_gpu.sim_stock_source = Some(SimStockRenderSource::GpuGrid {
        revision: 2,
        handle: Arc::clone(&gpu_handle),
        stock_model: DMat4::IDENTITY,
        color_mode: SimStockColorMode::ByOperation,
        mesh_skinning: true,
    });
    let px_cpu_skin =
        render_offscreen_rgba_on_device(device, queue, None, &cam, &hl_cpu, w, h).unwrap();
    let px_gpu_skin =
        render_offscreen_rgba_on_device(device, queue, None, &cam, &hl_gpu, w, h).unwrap();
    let mut skin_overlap = 0usize;
    let mut skin_union = 0usize;
    let mut skin_diff_sum = 0u64;
    for px in 0..(w * h) as usize {
        let o = px * 4;
        let c_fg = (0..3)
            .map(|c| px_cpu_skin[o + c].abs_diff(bg[o + c]))
            .max()
            .unwrap_or(0)
            > 8;
        let g_fg = (0..3)
            .map(|c| px_gpu_skin[o + c].abs_diff(bg[o + c]))
            .max()
            .unwrap_or(0)
            > 8;
        if c_fg || g_fg {
            skin_union += 1;
        }
        if c_fg && g_fg {
            skin_overlap += 1;
            let max_ch_diff = (0..3)
                .map(|c| px_cpu_skin[o + c].abs_diff(px_gpu_skin[o + c]))
                .max()
                .unwrap_or(0);
            skin_diff_sum += u64::from(max_ch_diff);
        }
    }
    let skin_iou = skin_overlap as f64 / skin_union.max(1) as f64;
    let skin_mean_diff = skin_diff_sum as f64 / skin_overlap.max(1) as f64;
    assert!(
        skin_overlap > 10_000 && skin_iou >= 0.96,
        "GPU skinned cylinder silhouette must match CPU skinned mesh (overlap={skin_overlap}, iou={skin_iou:.4})"
    );
    assert!(
        skin_mean_diff < 8.0,
        "GPU skinned cylinder shading must match CPU skinned mesh (mean_diff={skin_mean_diff:.2})"
    );

    // Also verify close-up 45° helical-cut curved surface visual smoothness (Crisp vs Skinned)
    // on BOTH GPU and CPU (and optionally dump side-by-side PNGs when CHIPCUTTER_DUMP_SKIN_PNG_DIR is set).
    let (cw, ch) = (512u32, 384u32);
    let mut close_cam = CadCamera {
        target_mm: DVec3::ZERO,
        distance_mm: 26.0,
        ..CadCamera::default()
    };
    close_cam.set_standard_view(StandardView::Isometric);
    close_cam.distance_mm = 26.0;

    let render_mode = |is_gpu: bool, skin: bool| -> Vec<u8> {
        let mut hl = HighlightState {
            wcs_frame: RigidFrame3::IDENTITY,
            show_stock: false,
            show_grid: false,
            ..HighlightState::default()
        };
        hl.sim_stock_source = if is_gpu {
            Some(SimStockRenderSource::GpuGrid {
                revision: if skin { 12 } else { 11 },
                handle: Arc::clone(&gpu_handle),
                stock_model: DMat4::IDENTITY,
                color_mode: SimStockColorMode::ByOperation,
                mesh_skinning: skin,
            })
        } else {
            let m = Arc::new(chip_stock::tessellate_tridexel_stock_mesh_with_options(
                &td_cpu,
                None,
                chip_sim::SimMeshOptions::new(SimStockColorMode::ByOperation, skin),
            ));
            Some(SimStockRenderSource::CpuMesh {
                revision: if skin { 12 } else { 11 },
                mesh: m,
                stock_model: DMat4::IDENTITY,
            })
        };
        render_offscreen_rgba_on_device(device, queue, None, &close_cam, &hl, cw, ch).unwrap()
    };

    let close_gpu_crisp = render_mode(true, false);
    let close_gpu_skin = render_mode(true, true);
    let close_cpu_crisp = render_mode(false, false);
    let close_cpu_skin = render_mode(false, true);

    // Measure pixel-to-pixel vertical luminance roughness on the cylinder body:
    // true 3D Surface Nets skinning must strictly eliminate stair-step banding on both GPU & CPU!
    let close_bg = render_offscreen_rgba_on_device(
        device,
        queue,
        None,
        &close_cam,
        &HighlightState {
            wcs_frame: RigidFrame3::IDENTITY,
            show_stock: false,
            show_grid: false,
            ..HighlightState::default()
        },
        cw,
        ch,
    )
    .unwrap();
    let vertical_roughness = |px: &[u8]| -> f64 {
        let mut sum = 0.0_f64;
        let mut cnt = 0usize;
        for y in 1..(ch - 1) as usize {
            for x in 0..cw as usize {
                let o0 = (y * cw as usize + x) * 4;
                let o1 = ((y + 1) * cw as usize + x) * 4;
                let fg0 = (0..3)
                    .map(|c| px[o0 + c].abs_diff(close_bg[o0 + c]))
                    .max()
                    .unwrap_or(0)
                    > 8;
                let fg1 = (0..3)
                    .map(|c| px[o1 + c].abs_diff(close_bg[o1 + c]))
                    .max()
                    .unwrap_or(0)
                    > 8;
                if fg0 && fg1 {
                    let d = (0..3)
                        .map(|c| f64::from(px[o0 + c].abs_diff(px[o1 + c])))
                        .sum::<f64>()
                        / 3.0;
                    sum += d;
                    cnt += 1;
                }
            }
        }
        sum / cnt.max(1) as f64
    };
    let gpu_crisp_r = vertical_roughness(&close_gpu_crisp);
    let gpu_skin_r = vertical_roughness(&close_gpu_skin);
    let cpu_crisp_r = vertical_roughness(&close_cpu_crisp);
    let cpu_skin_r = vertical_roughness(&close_cpu_skin);
    assert!(
        gpu_skin_r < gpu_crisp_r * 0.55 && cpu_skin_r < cpu_crisp_r * 0.55,
        "3D Surface Nets skinning must sharply reduce stair-step luminance roughness (GPU {gpu_skin_r:.2} vs {gpu_crisp_r:.2}, CPU {cpu_skin_r:.2} vs {cpu_crisp_r:.2})"
    );
}

#[test]
fn m95_offscreen_wgpu_renders_measure_leaders_and_reach_heatmap_in_dark_and_light() {
    use std::sync::Arc;

    use chip_cad::MeasureSnapTarget;
    use chip_cam::{
        ReachAnalysisResult, ReachHighlightEdge, ReachHighlightFace, ReachIssueKind, ReachSeverity,
    };
    use chip_math::{DVec3, RigidFrame3};

    use crate::{
        camera::{CadCamera, StandardView},
        theme::AppTheme,
        viewport::{HighlightState, MeasureToolState, render_offscreen_rgba_on_device},
    };

    let instance =
        wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle_from_env());
    let Ok(adapter) = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
        power_preference: wgpu::PowerPreference::LowPower,
        compatible_surface: None,
        force_fallback_adapter: false,
        ..Default::default()
    })) else {
        return;
    };
    let Ok((device, queue)) =
        pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default()))
    else {
        return;
    };

    let (w, h) = (320u32, 240u32);
    let mut cam = CadCamera {
        target_mm: DVec3::new(0.0, 0.0, 0.0),
        distance_mm: 95.0,
        ..CadCamera::default()
    };
    cam.set_standard_view(StandardView::Top);
    cam.distance_mm = 95.0;

    // Build synthetic Amber (left quad + edge) and Crimson (right quad + edge) reach regions
    let reach = Arc::new(ReachAnalysisResult {
        tool_name: "6mm Flat".to_string(),
        tool_diameter_mm: 6.0,
        tool_flute_length_mm: 12.0,
        tool_effective_reach_mm: 12.0,
        tool_stickout_length_mm: 15.0,
        min_internal_radius_mm: Some(2.0),
        has_sharp_internal_corners: false,
        max_feature_depth_mm: 18.0,
        amber_count: 2,
        crimson_count: 2,
        tight_corner_count: 1,
        narrow_slot_count: 1,
        flute_exceeded_count: 0,
        stickout_exceeded_count: 0,
        highlight_faces: vec![
            ReachHighlightFace {
                face_id: None,
                kind: ReachIssueKind::TightCorner,
                severity: ReachSeverity::Warning,
                triangles_world: vec![
                    (
                        [
                            DVec3::new(-28.0, -14.0, 0.0),
                            DVec3::new(-6.0, -14.0, 0.0),
                            DVec3::new(-6.0, 14.0, 0.0),
                        ],
                        DVec3::Z,
                    ),
                    (
                        [
                            DVec3::new(-28.0, -14.0, 0.0),
                            DVec3::new(-6.0, 14.0, 0.0),
                            DVec3::new(-28.0, 14.0, 0.0),
                        ],
                        DVec3::Z,
                    ),
                ],
                measured_value_mm: 2.0,
                limit_value_mm: 3.0,
                detail: "Amber tight fillet".to_string(),
            },
            ReachHighlightFace {
                face_id: None,
                kind: ReachIssueKind::ExceedsStickout,
                severity: ReachSeverity::Critical,
                triangles_world: vec![
                    (
                        [
                            DVec3::new(6.0, -14.0, 0.0),
                            DVec3::new(28.0, -14.0, 0.0),
                            DVec3::new(28.0, 14.0, 0.0),
                        ],
                        DVec3::Z,
                    ),
                    (
                        [
                            DVec3::new(6.0, -14.0, 0.0),
                            DVec3::new(28.0, 14.0, 0.0),
                            DVec3::new(6.0, 14.0, 0.0),
                        ],
                        DVec3::Z,
                    ),
                ],
                measured_value_mm: 18.0,
                limit_value_mm: 15.0,
                detail: "Crimson stickout collision".to_string(),
            },
        ],
        highlight_edges: vec![
            ReachHighlightEdge {
                edge_id: None,
                kind: ReachIssueKind::TightCorner,
                severity: ReachSeverity::Warning,
                segments_world: vec![(DVec3::new(-28.0, -16.0, 0.2), DVec3::new(-6.0, -16.0, 0.2))],
                measured_value_mm: 2.0,
                limit_value_mm: 3.0,
                detail: "Amber edge".to_string(),
            },
            ReachHighlightEdge {
                edge_id: None,
                kind: ReachIssueKind::NarrowSlotBlocked,
                severity: ReachSeverity::Critical,
                segments_world: vec![(DVec3::new(6.0, -16.0, 0.2), DVec3::new(28.0, -16.0, 0.2))],
                measured_value_mm: 4.0,
                limit_value_mm: 6.0,
                detail: "Crimson edge".to_string(),
            },
        ],
    });

    let mut measure = MeasureToolState::default();
    measure.on_click_target(MeasureSnapTarget::BRepVertex {
        pos_wcs: DVec3::new(-20.0, 20.0, 0.0),
    });
    measure.on_click_target(MeasureSnapTarget::BRepVertex {
        pos_wcs: DVec3::new(20.0, 24.0, 5.0),
    });

    for theme in [AppTheme::Dark, AppTheme::Light] {
        let base_hl = HighlightState {
            theme,
            wcs_frame: RigidFrame3::IDENTITY,
            show_stock: false,
            show_grid: false,
            ..HighlightState::default()
        };
        let base_px =
            render_offscreen_rgba_on_device(&device, &queue, None, &cam, &base_hl, w, h).unwrap();

        let active_hl = HighlightState {
            theme,
            wcs_frame: RigidFrame3::IDENTITY,
            show_stock: false,
            show_grid: false,
            show_reach_heatmap: true,
            reach_analysis: Some(Arc::clone(&reach)),
            measure_state: Some(measure.clone()),
            ..HighlightState::default()
        };
        let active_px =
            render_offscreen_rgba_on_device(&device, &queue, None, &cam, &active_hl, w, h).unwrap();

        let mut amber_px = 0usize;
        let mut crimson_px = 0usize;
        let mut changed_px = 0usize;
        for i in 0..(w as usize * h as usize) {
            let o = i * 4;
            let (r, g, b) = (active_px[o], active_px[o + 1], active_px[o + 2]);
            let diff = r
                .abs_diff(base_px[o])
                .max(g.abs_diff(base_px[o + 1]))
                .max(b.abs_diff(base_px[o + 2]));
            if diff > 14 {
                changed_px += 1;
                // Amber: strongly warmer than blue with elevated green
                if r > b.saturating_add(30) && g > b.saturating_add(15) && r >= g {
                    amber_px += 1;
                }
                // Crimson: red strongly dominates both green and blue
                if r > g.saturating_add(35) && r > b.saturating_add(35) {
                    crimson_px += 1;
                }
            }
        }
        assert!(
            changed_px > 500,
            "{theme:?}: expected measurement + reach overlays to visibly paint viewport (changed={changed_px})"
        );
        assert!(
            amber_px > 200,
            "{theme:?}: expected distinct Amber reach pixels (got {amber_px})"
        );
        assert!(
            crimson_px > 200,
            "{theme:?}: expected distinct Crimson reach pixels (got {crimson_px})"
        );
    }
}
