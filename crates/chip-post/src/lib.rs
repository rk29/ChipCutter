//! User-editable TOML post-processor profiles, ATC vs. split-by-tool partitioning,
//! modal state tracking, arc splitting, and G-code emission (`chip-post`).

mod cycles_arcs;
pub mod emitter;
mod modal;
pub mod partition;
pub mod profile;

pub use emitter::post_process_setup;
pub use partition::{
    OperationPostInput, PostOutputFile, SetupPostInput, describe_wcs_origin,
    sanitize_file_extension, validate_safe_output_filename,
};
pub use profile::{
    ArcCenterMode, BUILTIN_GRBL_ROTARY_WRAP_TOML, BUILTIN_GRBL_TOML, BUILTIN_GRBLHAL_4AXIS_TOML,
    BUILTIN_GRBLHAL_ATC_TOML, BUILTIN_LINUXCNC_4AXIS_TOML, BUILTIN_LINUXCNC_TOML, CommentStyle,
    LineNumberConfig, PostError, PostProfile, PostProfileSource, PostUnits, RotaryPostConfig,
    SwapAxisTarget, ToolChangeMode, builtin_post_profiles, is_builtin_post_id,
    load_available_post_profiles, resolve_post_profile, resolve_post_profile_cli_override,
    user_post_profiles_dir, validate_post_id,
};

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cam::{
        ClEvent, CutterCompMode, DrillCycleKind, DrillCycleSpec, MotionRole, Operation,
        OperationKind, ResolvedStockGeometry, ToolpathBuilder, WorkOffsetSlot, XyArcMove,
    };
    use chip_math::{Aabb3, DVec2, DVec3};
    use chip_tools::{CoolantMode, SpindleDirection, default_starter_tool_library};

    use super::*;

    fn build_synthetic_multi_tool_job() -> (
        chip_tools::ToolLibrary,
        Operation,
        chip_cam::GeneratedToolpath,
        Operation,
        chip_cam::GeneratedToolpath,
        Operation,
        chip_cam::GeneratedToolpath,
    ) {
        let lib = default_starter_tool_library();
        let t_face = &lib.tools[6]; // T7
        let t_flat = &lib.tools[0]; // T1
        let t_drill = &lib.tools[5]; // T6

        // Op 1: Facing with T7 (includes nested parens in comment to test sanitization + <=80 char clamp!)
        let mut op1 = Operation::new(
            "op_1",
            "Face Top (Rough & Finish Decking Pass With Extra Long Descriptive Title For 80-Char GRBL Limit Check)",
            OperationKind::Facing,
        );
        op1.tool_id = Some(t_face.id.clone());
        let mut b1 = ToolpathBuilder::new("op_1");
        b1.push_event(ClEvent::OperationStart {
            op_id: op1.id.clone(),
            op_name: op1.name.clone(),
            strategy_name: "Facing".to_string(),
            est_time_sec: 12.0,
        });
        b1.push_event(ClEvent::ToolChange {
            tool_number: t_face.tool_number,
            length_offset: t_face.length_offset_number,
            diameter_offset: t_face.diameter_offset_number,
            tool_desc: t_face.name.clone(),
            is_first_tool: true,
        });
        b1.push_event(ClEvent::SpindleStart {
            rpm: 4_500.0,
            direction: SpindleDirection::Cw,
        });
        b1.push_event(ClEvent::Coolant {
            mode: CoolantMode::AirBlast,
        });
        b1.rapid_to(DVec3::new(-20.0, 10.0, 25.0));
        b1.rapid_to(DVec3::new(-20.0, 10.0, 2.0));
        b1.linear_to(DVec3::new(-20.0, 10.0, 0.0), 400.0, MotionRole::Plunge);
        b1.linear_to(DVec3::new(120.0, 10.0, 0.0), 1_500.0, MotionRole::Cut);
        b1.rapid_to(DVec3::new(120.0, 10.0, 25.0));
        b1.push_event(ClEvent::OperationEnd);
        let tp1 = b1.finish();

        // Op 2: Helical 360° Bore + Wear Comp with T1
        let mut op2 = Operation::new("op_2", "Ø18mm Helical Bore", OperationKind::DrillBore);
        op2.tool_id = Some(t_flat.id.clone());
        let mut b2 = ToolpathBuilder::new("op_2");
        b2.push_event(ClEvent::OperationStart {
            op_id: op2.id.clone(),
            op_name: op2.name.clone(),
            strategy_name: "Helical Bore".to_string(),
            est_time_sec: 28.0,
        });
        b2.push_event(ClEvent::ToolChange {
            tool_number: t_flat.tool_number,
            length_offset: t_flat.length_offset_number,
            diameter_offset: t_flat.diameter_offset_number,
            tool_desc: t_flat.name.clone(),
            is_first_tool: false,
        });
        b2.push_event(ClEvent::SpindleStart {
            rpm: 12_000.0,
            direction: SpindleDirection::Cw,
        });
        b2.rapid_to(DVec3::new(56.0, 40.0, 25.0));
        b2.rapid_to(DVec3::new(56.0, 40.0, 2.0));
        b2.linear_to(DVec3::new(56.0, 40.0, 0.0), 500.0, MotionRole::Plunge);
        b2.push_event(ClEvent::CutterComp {
            mode: CutterCompMode::WearLeft,
        });
        // Full 2-turn (720°) helical CCW bore from Z=0 down to Z=-10 around center (50, 40), R = 6.0!
        b2.arc_xy_to(XyArcMove {
            end_wcs: DVec3::new(56.0, 40.0, -10.0),
            center_xy: DVec2::new(50.0, 40.0),
            radius_mm: 6.0,
            ccw: true,
            turns: 1, // 1 extra turn beyond base 360° => 720° total!
            feed_mm_min: 800.0,
            role: MotionRole::Ramp,
        });
        b2.push_event(ClEvent::CutterComp {
            mode: CutterCompMode::Off,
        });
        b2.rapid_to(DVec3::new(50.0, 40.0, 25.0));
        b2.push_event(ClEvent::OperationEnd);
        let tp2 = b2.finish();

        // Op 3: Deep Peck Drill Cycle with T6
        let mut op3 = Operation::new("op_3", "4x Peck Holes", OperationKind::DrillBore);
        op3.tool_id = Some(t_drill.id.clone());
        let mut b3 = ToolpathBuilder::new("op_3");
        b3.push_event(ClEvent::OperationStart {
            op_id: op3.id.clone(),
            op_name: op3.name.clone(),
            strategy_name: "Deep Peck Drill".to_string(),
            est_time_sec: 18.0,
        });
        b3.push_event(ClEvent::ToolChange {
            tool_number: t_drill.tool_number,
            length_offset: t_drill.length_offset_number,
            diameter_offset: t_drill.diameter_offset_number,
            tool_desc: t_drill.name.clone(),
            is_first_tool: false,
        });
        b3.push_event(ClEvent::SpindleStart {
            rpm: 4_800.0,
            direction: SpindleDirection::Cw,
        });
        b3.push_event(ClEvent::Coolant {
            mode: CoolantMode::Flood,
        });
        b3.emit_drill_cycle(DrillCycleSpec {
            kind: DrillCycleKind::DeepPeck,
            bottom_wcs: DVec3::new(20.0, 20.0, -16.0),
            r_plane_z_wcs: 2.0,
            clear_z_wcs: 15.0,
            peck_q_mm: 4.0,
            decreasing_factor: None,
            min_peck_mm: 1.5,
            chip_break_retract_mm: 0.5,
            dwell_sec: 0.0,
            feed_mm_min: 320.0,
        });
        b3.push_event(ClEvent::OperationEnd);
        let tp3 = b3.finish();

        (lib, op1, tp1, op2, tp2, op3, tp3)
    }

    #[test]
    fn grbl_post_splits_files_expands_peck_drills_splits_quadrant_arcs_and_enforces_80_chars() {
        let (lib, op1, tp1, op2, tp2, op3, tp3) = build_synthetic_multi_tool_job();
        let grbl = resolve_post_profile("grbl", None).unwrap();

        let input = SetupPostInput {
            project_name: "motor_mount.chip.toml",
            setup_name: "Setup 1 - Top Op",
            wcs_slot: WorkOffsetSlot::G54,
            wcs_origin_description: "Stock Top Front-Left".to_string(),
            stock: ResolvedStockGeometry::Box {
                aabb_wcs: Aabb3::new(DVec3::new(0.0, 0.0, -15.0), DVec3::new(104.0, 84.0, 0.0)),
            },
            rotary: None,
            clearance_z_wcs: 25.0,
            retract_z_wcs: 15.0,
            operations: vec![
                OperationPostInput {
                    op: &op1,
                    tool: &lib.tools[6],
                    toolpath: &tp1,
                },
                OperationPostInput {
                    op: &op2,
                    tool: &lib.tools[0],
                    toolpath: &tp2,
                },
                OperationPostInput {
                    op: &op3,
                    tool: &lib.tools[5],
                    toolpath: &tp3,
                },
            ],
        };

        let files = post_process_setup(&input, &grbl, "motor_mount").unwrap();
        assert_eq!(
            files.len(),
            3,
            "GRBL split_files_by_tool must emit 3 separate numbered .nc files for T7, T1, T6"
        );
        assert!(files[0].filename.starts_with("motor_mount_01_T7_"));
        assert!(files[1].filename.starts_with("motor_mount_02_T1_"));
        assert!(files[2].filename.starts_with("motor_mount_03_T6_"));

        for f in &files {
            assert!(
                f.max_line_chars <= 80,
                "GRBL file '{}' exceeded 80 chars ({}):\n{}",
                f.filename,
                f.max_line_chars,
                f.content
            );
            for line in f.content.lines() {
                assert!(line.len() <= 80, "line > 80 chars: {line}");
                // Verify zero nested parentheses inside `( ... )` comments
                if line.starts_with('(') {
                    let inner = &line[1..line.len() - 1];
                    assert!(
                        !inner.contains('(') && !inner.contains(')'),
                        "nested parens in comment: {line}"
                    );
                }
            }
            // GRBL must never emit unsupported G40/G41/G42/G43/G49/G81/G82/G83/G73/M6
            for forbidden in [
                "G40", "G41", "G42", "G43", "G49", "G81", "G82", "G83", "G73", "M6",
            ] {
                let has_code = f
                    .content
                    .lines()
                    .filter(|l| !l.starts_with('(') && !l.starts_with(';'))
                    .any(|l| l.split_whitespace().any(|w| w == forbidden));
                assert!(
                    !has_code,
                    "forbidden code '{forbidden}' found in GRBL output:\n{}",
                    f.content
                );
            }
        }

        // File 2 (`720°` helical CCW bore) with `max_arc_sweep_deg = 90.0` MUST split into `8` quadrant G3 arcs!
        let g3_lines: Vec<&str> = files[1]
            .content
            .lines()
            .filter(|l| !l.starts_with('(') && (l.contains("I") && l.contains("J")))
            .collect();
        assert_eq!(
            g3_lines.len(),
            8,
            "720° helix must split into 8 x 90° quadrant arcs in GRBL, got {}:\n{}",
            g3_lines.len(),
            files[1].content
        );
        // First quadrant starts at (56, 40) around (50, 40) -> I-6 J0, ends at (50, 46, Z=-1.25)!
        assert!(
            g3_lines[0].contains("G3")
                && g3_lines[0].contains("X50")
                && g3_lines[0].contains("Y46")
                && g3_lines[0].contains("Z-1.25")
                && g3_lines[0].contains("I-6")
                && g3_lines[0].contains("J0")
        );

        // File 3 (DeepPeck drill from R=2.0 by Q=4.0 to Z=-16.0) must contain explicit G1 Z-2, Z-6, Z-10, Z-14, Z-16 pecks!
        assert!(files[2].content.contains("Z-2"));
        assert!(files[2].content.contains("Z-6"));
        assert!(files[2].content.contains("Z-10"));
        assert!(files[2].content.contains("Z-14"));
        assert!(files[2].content.contains("Z-16"));
    }

    #[test]
    fn linuxcnc_and_grblhal_atc_emit_m6_g43_g64_native_cycles_and_support_inch_g20_scaling() {
        let (lib, op1, tp1, op2, tp2, op3, tp3) = build_synthetic_multi_tool_job();

        // 1. grblHAL ATC: single .nc file with M6 tool changes and <= 80 chars/line
        let grbl_atc = resolve_post_profile("grblhal_atc", None).unwrap();
        let mut input = SetupPostInput {
            project_name: "motor_mount.chip.toml",
            setup_name: "Setup 1",
            wcs_slot: WorkOffsetSlot::G59_1,
            wcs_origin_description: "Model Top Center".to_string(),
            stock: ResolvedStockGeometry::Box {
                aabb_wcs: Aabb3::new(DVec3::new(0.0, 0.0, -25.4), DVec3::new(50.8, 25.4, 0.0)),
            },
            rotary: None,
            clearance_z_wcs: 25.4,
            retract_z_wcs: 12.7,
            operations: vec![
                OperationPostInput {
                    op: &op1,
                    tool: &lib.tools[6],
                    toolpath: &tp1,
                },
                OperationPostInput {
                    op: &op2,
                    tool: &lib.tools[0],
                    toolpath: &tp2,
                },
                OperationPostInput {
                    op: &op3,
                    tool: &lib.tools[5],
                    toolpath: &tp3,
                },
            ],
        };
        let atc_files = post_process_setup(&input, &grbl_atc, "motor_mount").unwrap();
        assert_eq!(atc_files.len(), 1);
        assert!(atc_files[0].max_line_chars <= 80);
        assert!(atc_files[0].content.contains("T7 M6"));
        assert!(atc_files[0].content.contains("T1 M6"));
        assert!(atc_files[0].content.contains("T6 M6"));

        // 2. LinuxCNC RS274NGC (metric G21)
        let linuxcnc = resolve_post_profile("linuxcnc", None).unwrap();
        let lc_files = post_process_setup(&input, &linuxcnc, "motor_mount").unwrap();
        assert_eq!(lc_files.len(), 1);
        assert_eq!(lc_files[0].filename, "motor_mount.ngc");
        let ngc = &lc_files[0].content;
        assert!(ngc.starts_with("( ===="));
        assert!(ngc.contains("G17 G21 G40 G49 G59.1 G80 G90 G94 G64 P0.0250"));
        assert!(ngc.contains("T1 M6\nG43 H1"));
        assert!(ngc.contains("G41 D1"));
        assert!(ngc.contains("G98 G83 X20.0000 Y20.0000 Z-16.0000 R2.0000 Q4.0000 F320.0"));
        assert!(ngc.contains("G80"));
        assert!(ngc.ends_with("M2\n%\n"));

        // 3. LinuxCNC in G20 Inch mode (`25.4 mm -> 1.0000 in`, `1500 mm/min -> 59.1 ipm`, `G64 P0.0010`)
        let mut lc_inch = linuxcnc.clone();
        lc_inch.units = PostUnits::Inch;
        input.wcs_slot = WorkOffsetSlot::G54;
        let inch_files = post_process_setup(&input, &lc_inch, "imperial_mount").unwrap();
        let inch_ngc = &inch_files[0].content;
        assert!(inch_ngc.contains("G17 G20 G40 G49 G54 G80 G90 G94 G64 P0.0010"));
        assert!(inch_ngc.contains("Stock:        X=2.0000  Y=1.0000  Z=1.0000 in"));
        assert!(inch_ngc.contains("F59.1")); // 1500 / 25.4 = 59.055 ipm
    }

    #[test]
    fn four_axis_posts_emit_g93_inverse_time_g94_restore_and_swapped_axis_wrapping() {
        use chip_cam::{
            ResolvedRotaryConfig, RotaryAxisKind, RotaryDirection, RotaryDriveMode,
            RotaryUnwindMode, StockAxis,
        };

        let lib = default_starter_tool_library();
        let t1 = &lib.tools[0];
        let mut op = Operation::new("op_rot_1", "Rotary Spiral Test", OperationKind::Facing);
        op.tool_id = Some(t1.id.clone());

        let mut b = ToolpathBuilder::new("op_rot_1");
        b.push_event(ClEvent::OperationStart {
            op_id: op.id.clone(),
            op_name: op.name.clone(),
            strategy_name: "Rotary Test".to_string(),
            est_time_sec: 10.0,
        });
        b.push_event(ClEvent::ToolChange {
            tool_number: t1.tool_number,
            length_offset: t1.length_offset_number,
            diameter_offset: t1.diameter_offset_number,
            tool_desc: t1.name.clone(),
            is_first_tool: true,
        });
        b.push_event(ClEvent::SpindleStart {
            rpm: 12_000.0,
            direction: SpindleDirection::Cw,
        });
        b.rapid_to(DVec3::new(-30.0, 0.0, 32.0));
        b.rotary_index_to(90.0, true, None);
        b.linear_to(DVec3::new(-30.0, 0.0, 20.0), 300.0, MotionRole::Plunge);
        // Simultaneous X + A helical cut at R = 20 mm, surface feed 600 mm/min
        b.rotary_linear_to(
            DVec3::new(30.0, 0.0, 20.0),
            450.0,
            600.0,
            20.0,
            MotionRole::Cut,
        );
        // Return to pure linear move -> post MUST restore G94!
        b.linear_to(DVec3::new(30.0, 0.0, 32.0), 1_200.0, MotionRole::LeadOut);
        b.push_event(ClEvent::OperationEnd);
        let tp = b.finish();

        let rotary_x = ResolvedRotaryConfig {
            mode: RotaryDriveMode::Simultaneous4Axis,
            axis: RotaryAxisKind::XAxis,
            direction: RotaryDirection::RightHandCcw,
            axis_letter: 'A',
            center_wcs: DVec3::ZERO,
            stock_u_min: -40.0,
            stock_u_max: 40.0,
            stock_outer_radius_mm: 22.0,
            clearance_radius_mm: 32.0,
            chuck_end_positive: false,
            chuck_u_min: -60.0,
            chuck_u_max: -40.0,
            chuck_radius_mm: 32.5,
            unwind_mode: RotaryUnwindMode::ContinuousMultiTurn,
        };

        let input = SetupPostInput {
            project_name: "rotary_shaft.chip.toml",
            setup_name: "Setup 1 - A-Axis",
            wcs_slot: WorkOffsetSlot::G54,
            wcs_origin_description: "Stock Center".to_string(),
            stock: ResolvedStockGeometry::Cylinder {
                axis: StockAxis::X,
                center_wcs: DVec3::ZERO,
                radius_mm: 22.0,
                min_axial_mm: -40.0,
                max_axial_mm: 40.0,
                aabb_wcs: Aabb3::new(
                    DVec3::new(-40.0, -22.0, -22.0),
                    DVec3::new(40.0, 22.0, 22.0),
                ),
            },
            rotary: Some(rotary_x),
            clearance_z_wcs: 35.0,
            retract_z_wcs: 32.0,
            operations: vec![OperationPostInput {
                op: &op,
                tool: t1,
                toolpath: &tp,
            }],
        };

        // 1. LinuxCNC 4-Axis: emits A90.000 index, G93 + A450.000 + F<inv>, then restores G94!
        let lc4 = resolve_post_profile("linuxcnc_4axis", None).unwrap();
        let lc4_out = post_process_setup(&input, &lc4, "rotary_shaft").unwrap();
        assert_eq!(lc4_out.len(), 1);
        let ngc = &lc4_out[0].content;
        assert!(ngc.contains("Rotary 4th:   A-Axis"));
        assert!(ngc.contains("A90.000"), "missing A90 index:\n{ngc}");
        assert!(
            ngc.contains("G93 G1 X30.0000 A450.000 F"),
            "missing G93 simultaneous cut:\n{ngc}"
        );
        let g93_pos = ngc.find("G93 G1").unwrap();
        let g94_after = ngc[g93_pos..].find("\nG94\n");
        assert!(
            g94_after.is_some(),
            "G94 must be restored immediately after G93 block:\n{ngc}"
        );

        // 2. grblHAL 4-Axis: stays <= 80 chars/line with G93 + A450
        let gh4 = resolve_post_profile("grblhal_4axis", None).unwrap();
        let gh4_out = post_process_setup(&input, &gh4, "rotary_shaft").unwrap();
        assert_eq!(gh4_out.len(), 1);
        assert!(gh4_out[0].max_line_chars <= 80);
        assert!(gh4_out[0].content.contains("G93 G1 X30 A450 F"));

        // 3. GRBL 3-Axis Swapped-Axis Rotary Wrap: maps A450 deg @ R=20mm -> Y = 2.5*pi*20 = 157.080 mm, zero A words!
        let wrap = resolve_post_profile("grbl_rotary_wrap", None).unwrap();
        let wrap_out = post_process_setup(&input, &wrap, "rotary_shaft").unwrap();
        assert_eq!(wrap_out.len(), 1);
        assert!(wrap_out[0].max_line_chars <= 80);
        let nc = &wrap_out[0].content;
        assert!(
            !nc.lines()
                .filter(|l| !l.starts_with('('))
                .any(|l| l.contains('A')),
            "swapped-axis wrap must never emit A word:\n{nc}"
        );
        assert!(
            nc.contains("Y157.08"),
            "450 deg at R=20mm must map to Y157.080 mm:\n{nc}"
        );
    }

    #[test]
    fn builtin_post_profiles_resolve_without_repo_posts_directory() {
        let empty_dir = std::env::temp_dir().join(format!(
            "chipcutter_empty_posts_test_{}",
            std::process::id()
        ));
        let _ = std::fs::create_dir_all(&empty_dir);
        let profiles = load_available_post_profiles(Some(&empty_dir));
        let _ = std::fs::remove_dir_all(&empty_dir);

        let ids: Vec<&str> = profiles.iter().map(|p| p.id.as_str()).collect();
        for required in [
            "grbl",
            "grblhal_atc",
            "linuxcnc",
            "linuxcnc_4axis",
            "grblhal_4axis",
            "grbl_rotary_wrap",
        ] {
            assert!(
                ids.contains(&required),
                "missing built-in post profile '{required}' in {ids:?}"
            );
            assert!(
                resolve_post_profile(required, None).is_ok(),
                "resolve_post_profile failed for built-in '{required}'"
            );
        }
    }

    #[test]
    fn quadrant_arc_with_one_ulp_overshoot_does_not_spuriously_split() {
        let lib = default_starter_tool_library();
        let t1 = &lib.tools[0];
        let mut op = Operation::new("op_arc", "Arc Boundary Test", OperationKind::Contour2D);
        op.tool_id = Some(t1.id.clone());
        let grbl = resolve_post_profile("grbl", None).unwrap();

        let count_arc_lines = |angle_deg: f64| -> usize {
            let r = 10.0_f64;
            let theta = angle_deg.to_radians();
            let start = DVec3::new(r, 0.0, -2.0);
            let end = DVec3::new(r * theta.cos(), r * theta.sin(), -2.0);
            let mut b = ToolpathBuilder::new("op_arc");
            b.push_event(ClEvent::OperationStart {
                op_id: op.id.clone(),
                op_name: op.name.clone(),
                strategy_name: "2D Contour".to_string(),
                est_time_sec: 1.0,
            });
            b.push_event(ClEvent::ToolChange {
                tool_number: t1.tool_number,
                length_offset: t1.length_offset_number,
                diameter_offset: t1.diameter_offset_number,
                tool_desc: t1.name.clone(),
                is_first_tool: true,
            });
            b.push_event(ClEvent::SpindleStart {
                rpm: 12_000.0,
                direction: SpindleDirection::Cw,
            });
            b.rapid_to(DVec3::new(r, 0.0, 5.0));
            b.rapid_to(DVec3::new(r, 0.0, 2.0));
            b.linear_to(start, 300.0, MotionRole::Plunge);
            b.arc_xy_to(XyArcMove {
                end_wcs: end,
                center_xy: DVec2::ZERO,
                radius_mm: r,
                ccw: true,
                turns: 0,
                feed_mm_min: 1000.0,
                role: MotionRole::Cut,
            });
            b.rapid_to(DVec3::new(end.x, end.y, 5.0));
            b.push_event(ClEvent::OperationEnd);
            let tp = b.finish();

            let input = SetupPostInput {
                project_name: "arc_boundary.chip.toml",
                setup_name: "Setup 1",
                wcs_slot: WorkOffsetSlot::G54,
                wcs_origin_description: "Stock Top Center".to_string(),
                stock: ResolvedStockGeometry::Box {
                    aabb_wcs: Aabb3::new(
                        DVec3::new(-20.0, -20.0, -10.0),
                        DVec3::new(20.0, 20.0, 0.0),
                    ),
                },
                rotary: None,
                clearance_z_wcs: 10.0,
                retract_z_wcs: 5.0,
                operations: vec![OperationPostInput {
                    op: &op,
                    tool: t1,
                    toolpath: &tp,
                }],
            };
            let files = post_process_setup(&input, &grbl, "arc_boundary").unwrap();
            files[0]
                .content
                .lines()
                .filter(|l| !l.starts_with('(') && l.contains('I') && l.contains('J'))
                .count()
        };

        assert_eq!(count_arc_lines(90.0), 1, "exact 90° arc must stay 1 block");
        assert_eq!(
            count_arc_lines(90.0 + 1e-11),
            1,
            "1-ULP overshoot past 90° must not split into two 45° halves"
        );
        assert_eq!(
            count_arc_lines(90.05),
            2,
            "genuinely >90° arc must split into 2 blocks for GRBL"
        );
    }

    #[test]
    fn milestone_32_security_sanitizes_extensions_protects_builtins_and_blocks_gcode_injection() {
        // 1. Path traversal in PostProfile::extension is stripped to safe alphanumerics
        assert_eq!(sanitize_file_extension("../../evil.sh"), "evilsh");
        assert_eq!(sanitize_file_extension("/etc/passwd"), "etcpasswd");
        assert_eq!(sanitize_file_extension("..."), "nc");
        assert_eq!(sanitize_file_extension(".ngc"), "ngc");
        assert!(validate_safe_output_filename("job_01.nc").is_ok());
        assert!(validate_safe_output_filename("../escape.nc").is_err());
        assert!(validate_safe_output_filename("sub/file.nc").is_err());
        assert!(validate_safe_output_filename("sub\\file.nc").is_err());
        assert!(validate_safe_output_filename("..").is_err());

        // 2. Untrusted project/bundle directory cannot shadow built-in post IDs
        let tmp_dir =
            std::env::temp_dir().join(format!("chip_post_m32_shadow_{}", std::process::id()));
        let posts_dir = tmp_dir.join("posts");
        let _ = std::fs::remove_dir_all(&tmp_dir);
        std::fs::create_dir_all(&posts_dir).unwrap();
        let mut fake_grbl = resolve_post_profile("grbl", None).unwrap();
        fake_grbl.program_header = vec!["G0 Z-999".to_string()];
        std::fs::write(
            posts_dir.join("grbl.post.toml"),
            fake_grbl.to_toml_string().unwrap(),
        )
        .unwrap();

        let resolved_grbl = resolve_post_profile("grbl", Some(&tmp_dir)).unwrap();
        assert!(resolved_grbl.is_builtin);
        assert_eq!(resolved_grbl.source, PostProfileSource::BuiltIn);
        assert!(
            !resolved_grbl
                .program_header
                .iter()
                .any(|l| l.contains("-999")),
            "untrusted project posts/grbl.post.toml must never shadow built-in grbl"
        );

        // 3. ID-only resolve_post_profile refuses arbitrary filesystem paths
        let custom_file = posts_dir.join("grbl.post.toml");
        assert!(
            resolve_post_profile(&custom_file.display().to_string(), Some(&tmp_dir)).is_err(),
            "untrusted project ID lookup must not read arbitrary file paths"
        );
        // Explicit CLI override still allows operator-chosen file path
        let cli_prof =
            resolve_post_profile_cli_override(&custom_file.display().to_string(), None).unwrap();
        assert_eq!(cli_prof.source, PostProfileSource::ExplicitFile);
        let _ = std::fs::remove_dir_all(&tmp_dir);

        // 4. Operation/tool names with newlines and closing parens cannot inject raw G-code
        let lib = default_starter_tool_library();
        let mut t1 = lib.tools[0].clone();
        t1.name = "Flat)\nG0 Z-50\n(Endmill".to_string();
        let mut op = Operation::new(
            "op_inj",
            "Pocket)\nM3 S99999\n(Pass",
            OperationKind::Pocket2D,
        );
        op.tool_id = Some(t1.id.clone());
        let mut b = ToolpathBuilder::new("op_inj");
        b.push_event(ClEvent::OperationStart {
            op_id: op.id.clone(),
            op_name: op.name.clone(),
            strategy_name: "Pocket)\nG0 X-999".to_string(),
            est_time_sec: 1.0,
        });
        b.push_event(ClEvent::ToolChange {
            tool_number: t1.tool_number,
            length_offset: t1.length_offset_number,
            diameter_offset: t1.diameter_offset_number,
            tool_desc: t1.name.clone(),
            is_first_tool: true,
        });
        b.rapid_to(DVec3::new(0.0, 0.0, 5.0));
        b.push_event(ClEvent::OperationEnd);
        let tp = b.finish();

        let mut custom_post = resolve_post_profile("linuxcnc", None).unwrap();
        custom_post.extension = "../../evil.sh".to_string();
        custom_post.operation_header = vec!["M0 ({op_name} - {strategy_name})".to_string()];
        custom_post.tool_change_block = vec!["T{tool_number} M6 ({tool_desc})".to_string()];

        let input = SetupPostInput {
            project_name: "inj_test",
            setup_name: "Setup 1",
            wcs_slot: WorkOffsetSlot::G54,
            wcs_origin_description: "Top".to_string(),
            stock: ResolvedStockGeometry::Box {
                aabb_wcs: Aabb3::new(DVec3::new(0.0, 0.0, -10.0), DVec3::new(50.0, 50.0, 0.0)),
            },
            rotary: None,
            clearance_z_wcs: 10.0,
            retract_z_wcs: 5.0,
            operations: vec![OperationPostInput {
                op: &op,
                tool: &t1,
                toolpath: &tp,
            }],
        };
        let files = post_process_setup(&input, &custom_post, "inj_job").unwrap();
        assert_eq!(files[0].filename, "inj_job.evilsh");
        assert!(validate_safe_output_filename(&files[0].filename).is_ok());
        for line in files[0].content.lines() {
            assert!(
                !line.trim_start().starts_with("G0 Z-50")
                    && !line.trim_start().starts_with("M3 S99999")
                    && !line.trim_start().starts_with("G0 X-999"),
                "injected G-code block detected: {line}"
            );
        }

        // 5. Non-finite coordinates fail closed instead of emitting NaN/inf
        let mut b_nan = ToolpathBuilder::new("op_nan");
        b_nan.push_event(ClEvent::RapidMove {
            end_wcs: DVec3::new(f64::NAN, 0.0, 5.0),
        });
        let tp_nan = b_nan.finish();
        let nan_input = SetupPostInput {
            operations: vec![OperationPostInput {
                op: &op,
                tool: &t1,
                toolpath: &tp_nan,
            }],
            ..input
        };
        let err = post_process_setup(&nan_input, &custom_post, "nan_job").unwrap_err();
        assert!(
            matches!(err, PostError::NonFiniteValue { .. }),
            "expected NonFiniteValue error, got {err:?}"
        );
    }
}
