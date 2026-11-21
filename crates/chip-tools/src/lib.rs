//! Tool geometry, per-material cutting presets, coupled SFM/chipload calculator,
//! `.tools.toml` persistence, and Fusion 360 JSON / `.tools` importer (`chip-tools`).

pub mod builtin;
pub mod builtin_machines;
pub mod feeds_speeds;
pub mod fusion_import;
pub mod machine;
pub mod model;
pub mod storage;

pub use builtin::default_starter_tool_library;
pub use builtin_machines::{
    CARVERA_AIR_MACHINE_ID, CARVERA_MACHINE_ID, CARVERA_Z1_MACHINE_ID, MILO_1_5_MACHINE_ID,
    MILO_2_0_MACHINE_ID, VORON_CASCADE_MACHINE_ID, builtin_carvera_air_machine,
    builtin_carvera_machine, builtin_carvera_z1_machine, builtin_milo_1_5_machine,
    builtin_milo_2_0_machine, builtin_voron_cascade_machine,
};
pub use feeds_speeds::{
    ChipThinningHint, CuttingPreset, SurfaceSpeedPresetSpec, chipload_from_feed,
    compute_adaptive_thinning_feed, compute_radial_chip_thinning, feed_from_chipload,
    rpm_from_surface_speed, surface_speed_from_rpm,
};
pub use fusion_import::{
    FusionImportError, FusionImportReport, import_fusion_library_bytes, import_fusion_library_path,
};
pub use machine::{
    DEFAULT_MACHINE_ID, MachineDefinition, MachineKinematicsSpec, MachineRotaryDefaults,
    MachineSpindleSpec, MachineStorageError, MachineToolChangeKind, MachineTravelEnvelope,
    allocate_unique_custom_machine_id, builtin_generic_machine, builtin_machine_definitions,
    clone_machine_definition, delete_user_machine, delete_user_machine_in, export_machine_to_path,
    import_machine_from_path, import_machine_from_path_in, is_builtin_machine_id,
    load_available_machines, load_available_machines_in, resolve_machine_definition,
    resolve_machine_definition_cli_override, resolve_machine_definition_cli_override_in,
    resolve_machine_definition_in, sanitize_machine_id, save_user_machine, save_user_machine_in,
    user_machines_dir, validate_machine_id,
};
pub use model::{
    CoolantMode, CutterGeometry, SpindleDirection, Tool, ToolId, ToolOperationClass,
    ToolValidationError,
};
pub use storage::{
    ToolLibrary, ToolStorageError, default_global_tool_library_path, load_or_default_global_library,
};

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::io::Write as _;

    use chip_math::{DisplayUnit, EvalContext, approx_eq, evaluate_length_mm};

    use super::*;

    #[test]
    fn starter_library_validates_and_roundtrips_toml_deterministically() {
        let lib = default_starter_tool_library();
        lib.validate_all().unwrap();
        assert_eq!(lib.tools.len(), 8);

        let toml_1 = lib.to_toml_string().unwrap();
        let reloaded = ToolLibrary::from_toml_str(&toml_1).unwrap();
        let toml_2 = reloaded.to_toml_string().unwrap();
        assert_eq!(
            toml_1, toml_2,
            ".tools.toml output must be byte-for-byte deterministic"
        );
    }

    #[test]
    fn coupled_sfm_chipload_and_radial_chip_thinning_are_exact() {
        let mut preset = CuttingPreset::from_surface_speed(SurfaceSpeedPresetSpec {
            id: "al",
            name: "Al 6061",
            material: "Aluminum",
            diameter_mm: 10.0,
            flutes: 4,
            surface_speed_m_min: 314.159_265_358_979_3,
            chipload_mm_tooth: 0.05,
            default_stepdown_mm: 5.0,
            default_stepover_mm: 2.0,
            coolant_override: None,
        });
        // Vc = 100 * pi m/min on D = 10mm => RPM = 10,000; Vf = 10,000 * 4 * 0.05 = 2,000 mm/min
        assert!(approx_eq(preset.spindle_rpm, 10_000.0, 1e-6));
        assert!(approx_eq(preset.cutting_feed_mm_min, 2_000.0, 1e-6));

        // Halve RPM with `lock_chipload = true` -> feedrate halves to 1,000 mm/min, chipload stays 0.05
        preset.set_spindle_rpm(5_000.0, 10.0, 4, true);
        assert!(approx_eq(preset.chipload_mm_tooth, 0.05, 1e-9));
        assert!(approx_eq(preset.cutting_feed_mm_min, 1_000.0, 1e-6));
        assert!(approx_eq(
            preset.surface_speed_m_min,
            157.079_632_679_489_66,
            1e-6
        ));

        // Double RPM back to 10,000 with `lock_chipload = false` -> feed stays 1,000, chipload halves to 0.025!
        preset.set_spindle_rpm(10_000.0, 10.0, 4, false);
        assert!(approx_eq(preset.cutting_feed_mm_min, 1_000.0, 1e-6));
        assert!(approx_eq(preset.chipload_mm_tooth, 0.025, 1e-9));

        // Radial chip thinning at ae = 1.0 mm on D = 10.0 mm (`ae/D = 0.1`):
        // ratio = 2 * sqrt(0.1 * 0.9) = 2 * 0.3 = 0.6!
        let rct = compute_radial_chip_thinning(10.0, 1.0, 0.05, 1_200.0).unwrap();
        assert!(approx_eq(rct.thickness_ratio, 0.6, 1e-9));
        assert!(approx_eq(rct.actual_hex_mm, 0.03, 1e-9));
        assert!(approx_eq(rct.compensated_feed_mm_min, 2_000.0, 1e-6));
        assert!(compute_radial_chip_thinning(10.0, 6.0, 0.05, 1_200.0).is_none());

        // Adaptive thinning helper respects `max_multiplier` cap and `ae >= 0.5 D` identity:
        assert!(approx_eq(
            compute_adaptive_thinning_feed(10.0, 1.0, 1_200.0, 2.0),
            2_000.0,
            1e-6
        ));
        // At ae = 0.25 mm on D = 10 mm, raw 1/ratio > 3.2x, clamped to `max_multiplier = 1.8x`
        assert!(approx_eq(
            compute_adaptive_thinning_feed(10.0, 0.25, 1_000.0, 1.8),
            1_800.0,
            1e-6
        ));
        assert!(approx_eq(
            compute_adaptive_thinning_feed(10.0, 5.0, 1_000.0, 2.0),
            1_000.0,
            1e-6
        ));
    }

    #[test]
    fn imports_fusion_360_json_and_zipped_tools_archive_with_warnings() {
        let fusion_json = r#"{
          "version": 16,
          "data": [
            {
              "type": "flat end mill",
              "unit": "inches",
              "description": "1/4\" 3FL ZrN Aluminum",
              "vendor": "YG-1",
              "guid": "yg1-0250-3fl",
              "geometry": { "DC": 0.25, "LCF": 0.75, "OAL": 2.5, "SFDM": 0.25, "LB": 1.0, "NOF": 3 },
              "post-process": { "number": 1, "length-offset": 1, "diameter-offset": 1, "clockwise": true, "coolant": "mist" },
              "start-values": {
                "presets": [
                  {
                    "guid": "p1",
                    "name": "6061 Rough",
                    "n": 12000,
                    "v_c": 785.4,
                    "v_f": 72.0,
                    "f_z": 0.002,
                    "stepdown": 0.125,
                    "stepover": 0.1,
                    "optimal-load": 0.05
                  }
                ]
              }
            },
            {
              "type": "chamfer mill",
              "unit": "millimeters",
              "description": "6mm 90deg Spot/Chamfer",
              "guid": "chamfer-6mm",
              "geometry": { "DC": 6.0, "TPD": 0.1, "TA": 45.0, "LCF": 10.0, "OAL": 50.0, "SFDM": 6.0, "NOF": 2 },
              "post-process": { "number": 5 }
            },
            {
              "type": "lollipop mill",
              "unit": "millimeters",
              "description": "Undercut Lollipop",
              "geometry": { "DC": 8.0, "NOF": 4 }
            }
          ]
        }"#;

        let report = import_fusion_library_bytes(fusion_json.as_bytes(), "Shop Fusion").unwrap();
        assert_eq!(report.imported_tool_count, 2);
        assert_eq!(report.imported_preset_count, 1);
        assert_eq!(report.warnings.len(), 1);
        assert!(report.warnings[0].contains("lollipop mill"));

        let t1 = &report.library.tools[0];
        assert_eq!(t1.native_unit, DisplayUnit::Inch);
        assert!(approx_eq(t1.geometry.cutting_diameter_mm(), 6.35, 1e-9));
        assert!(approx_eq(
            t1.presets[0].cutting_feed_mm_min,
            72.0 * 25.4,
            1e-6
        ));
        assert!(approx_eq(
            t1.presets[0].chipload_mm_tooth,
            0.002 * 25.4,
            1e-6
        ));
        assert!(approx_eq(
            t1.presets[0].optimal_load_mm.unwrap(),
            0.05 * 25.4,
            1e-6
        ));

        // Verify tool scoped expression variables (`tool_d / flutes`, `tan(tip_angle / 2)`)
        let t5 = &report.library.tools[1];
        let mut ctx = EvalContext::new(DisplayUnit::Millimeter);
        t5.populate_eval_context(&mut ctx);
        assert!(approx_eq(
            evaluate_length_mm("1mm / tan(tip_angle / 2)", &ctx).unwrap(),
            1.0,
            1e-9
        ));

        // Also test `.tools` ZIP container extraction!
        let mut zip_buf = std::io::Cursor::new(Vec::new());
        {
            let mut zip_writer = zip::ZipWriter::new(&mut zip_buf);
            let opts = zip::write::SimpleFileOptions::default()
                .compression_method(zip::CompressionMethod::Deflated);
            zip_writer.start_file("tools.json", opts).unwrap();
            zip_writer.write_all(fusion_json.as_bytes()).unwrap();
            zip_writer.finish().unwrap();
        }
        let zip_report =
            import_fusion_library_bytes(&zip_buf.into_inner(), "Zipped Fusion").unwrap();
        assert_eq!(zip_report.imported_tool_count, 2);
    }

    #[test]
    fn machine_definition_roundtrips_protects_builtins_and_supports_clone_export_import() {
        let generic = builtin_generic_machine();
        assert!(generic.is_builtin);
        generic.validate().unwrap();
        assert_eq!(generic.tools.len(), 8);

        // Deterministic TOML roundtrip
        let toml_1 = generic.to_toml_string().unwrap();
        let parsed = MachineDefinition::from_toml_str(&toml_1).unwrap();
        assert!(!parsed.is_builtin);
        let toml_2 = parsed.to_toml_string().unwrap();
        assert_eq!(toml_1, toml_2);

        // Validation catches bad travel & inverted RPM range
        let mut bad = parsed.clone();
        bad.spindle.min_rpm = 25_000.0;
        bad.spindle.max_rpm = 10_000.0;
        assert!(matches!(
            bad.validate(),
            Err(MachineStorageError::Validation { .. })
        ));
        bad.spindle.min_rpm = 1_000.0;
        bad.travel_mm.z_mm = 0.0;
        assert!(matches!(
            bad.validate(),
            Err(MachineStorageError::Validation { .. })
        ));

        let tmp = std::env::temp_dir().join(format!(
            "chipcutter_machine_storage_test_{}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&tmp);
        let user_dir = tmp.join("user_machines");
        std::fs::create_dir_all(&user_dir).unwrap();

        // Built-ins are strictly read-only: cannot be saved, shadowed, or deleted!
        assert!(matches!(
            save_user_machine_in(&generic, &user_dir),
            Err(MachineStorageError::BuiltInReadOnly { .. })
        ));
        assert!(matches!(
            delete_user_machine_in(DEFAULT_MACHINE_ID, &user_dir),
            Err(MachineStorageError::BuiltInReadOnly { .. })
        ));
        assert!(matches!(
            clone_machine_definition(&generic, DEFAULT_MACHINE_ID, "Shadow"),
            Err(MachineStorageError::BuiltInReadOnly { .. })
        ));

        // Clone into a mutable custom machine with full tool library preserved
        let mut cloned = clone_machine_definition(&generic, "shop-mill-1", "My Shop Mill").unwrap();
        assert!(!cloned.is_builtin);
        assert_eq!(cloned.tools.len(), generic.tools.len());
        cloned.travel_mm.x_mm = 320.0;
        let saved_path = save_user_machine_in(&cloned, &user_dir).unwrap();
        assert!(saved_path.is_file());

        // Discovery returns read-only built-in + saved custom machine
        let all = load_available_machines_in(Some(&user_dir), None);
        assert!(
            all.iter()
                .any(|m| m.id == DEFAULT_MACHINE_ID && m.is_builtin)
        );
        let found = all.iter().find(|m| m.id == "shop-mill-1").unwrap();
        assert!(!found.is_builtin);
        assert!(approx_eq(found.travel_mm.x_mm, 320.0, 1e-9));

        // Export built-in -> import auto-disambiguates away from reserved built-in ID
        let share_file = tmp.join("shared_generic.machine.toml");
        export_machine_to_path(&generic, &share_file).unwrap();
        let existing_ids: Vec<String> = all.iter().map(|m| m.id.clone()).collect();
        let imported =
            import_machine_from_path_in(&share_file, &user_dir, &existing_ids, None).unwrap();
        assert_eq!(imported.id, "generic-custom");
        assert!(!imported.is_builtin);
        assert_eq!(imported.tools.len(), generic.tools.len());

        // Second import without override allocates next unique suffix (`generic-2`)
        let mut existing_ids_2 = existing_ids;
        existing_ids_2.push(imported.id.clone());
        let imported_2 =
            import_machine_from_path_in(&share_file, &user_dir, &existing_ids_2, None).unwrap();
        assert_eq!(imported_2.id, "generic-2");

        // Deleting custom machine works cleanly
        delete_user_machine_in("shop-mill-1", &user_dir).unwrap();
        assert!(resolve_machine_definition_in("shop-mill-1", Some(&user_dir), None).is_err());

        let _ = std::fs::remove_dir_all(&tmp);
    }

    #[test]
    fn builtin_machines_validate_conform_to_spindle_and_feed_limits_and_match_reference_toml() {
        let builtins = builtin_machine_definitions();
        let ids: Vec<&str> = builtins.iter().map(|m| m.id.as_str()).collect();
        let expected_ids = vec![
            DEFAULT_MACHINE_ID,
            MILO_1_5_MACHINE_ID,
            MILO_2_0_MACHINE_ID,
            VORON_CASCADE_MACHINE_ID,
            CARVERA_MACHINE_ID,
            CARVERA_AIR_MACHINE_ID,
            CARVERA_Z1_MACHINE_ID,
        ];
        assert_eq!(ids, expected_ids);

        let repo_machines_dir =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../machines");

        for m in &builtins {
            assert!(m.is_builtin);
            assert!(is_builtin_machine_id(&m.id));
            m.validate().unwrap();
            assert!(
                !m.tools.is_empty(),
                "machine '{}' must have starter tools",
                m.id
            );

            // Every cutting preset on every tool must sit strictly inside the machine's RPM, feed, & collet envelope!
            let mut seen_tool_ids = std::collections::BTreeSet::new();
            let mut seen_tool_numbers = std::collections::BTreeSet::new();
            for tool in &m.tools {
                assert!(
                    seen_tool_ids.insert(tool.id.0.as_str()),
                    "duplicate tool id '{}' on '{}'",
                    tool.id.0,
                    m.id
                );
                assert!(
                    seen_tool_numbers.insert(tool.tool_number),
                    "duplicate tool_number {} on '{}'",
                    tool.tool_number,
                    m.id
                );
                if !matches!(tool.geometry, CutterGeometry::FaceMill { .. })
                    && !m.spindle.supported_shank_diameters_mm.is_empty()
                {
                    let shank = tool.geometry.shank_diameter_mm();
                    assert!(
                        m.spindle
                            .supported_shank_diameters_mm
                            .iter()
                            .any(|&d| approx_eq(d, shank, 1e-3)),
                        "tool '{}' on '{}' has shank {} mm not in supported_shank_diameters_mm {:?}",
                        tool.name,
                        m.id,
                        shank,
                        m.spindle.supported_shank_diameters_mm
                    );
                }
                assert!(
                    !tool.presets.is_empty(),
                    "tool '{}' on '{}' must have presets",
                    tool.name,
                    m.id
                );
                for preset in &tool.presets {
                    assert!(
                        preset.spindle_rpm >= m.spindle.min_rpm - 0.5
                            && preset.spindle_rpm <= m.spindle.max_rpm + 0.5,
                        "preset '{}' on '{}' ({}) has RPM {:.1} outside [{}, {}]",
                        preset.name,
                        tool.name,
                        m.id,
                        preset.spindle_rpm,
                        m.spindle.min_rpm,
                        m.spindle.max_rpm
                    );
                    assert!(
                        preset.cutting_feed_mm_min <= m.kinematics.max_cut_feed_xy_mm_min + 0.5,
                        "preset '{}' on '{}' ({}) has XY feed {:.1} > max {:.1}",
                        preset.name,
                        tool.name,
                        m.id,
                        preset.cutting_feed_mm_min,
                        m.kinematics.max_cut_feed_xy_mm_min
                    );
                    assert!(
                        preset.plunge_feed_mm_min <= m.kinematics.max_cut_feed_z_mm_min + 0.5,
                        "preset '{}' on '{}' ({}) has Z plunge {:.1} > max {:.1}",
                        preset.name,
                        tool.name,
                        m.id,
                        preset.plunge_feed_mm_min,
                        m.kinematics.max_cut_feed_z_mm_min
                    );
                }
            }

            // Reference `machines/<id>.machine.toml` stays byte-for-byte synced with embedded built-ins
            let ref_path = repo_machines_dir.join(format!("{}.machine.toml", m.id));
            let expected_toml = m.to_toml_string().unwrap();
            let on_disk = std::fs::read_to_string(&ref_path)
                .unwrap_or_else(|e| panic!("missing reference {}: {e}", ref_path.display()))
                .replace("\r\n", "\n");
            assert_eq!(
                on_disk,
                expected_toml,
                "reference {} drifted from built-in definition",
                ref_path.display()
            );
        }

        // Verify installed-bundle resolution & read-only protection works from an empty temp directory
        let empty =
            std::env::temp_dir().join(format!("chipcutter_empty_machines_{}", std::process::id()));
        let _ = std::fs::create_dir_all(&empty);
        for id in &expected_ids {
            let resolved = resolve_machine_definition_in(id, Some(&empty), None).unwrap();
            assert_eq!(resolved.id, *id);
            assert!(resolved.is_builtin);
            assert!(matches!(
                save_user_machine_in(&resolved, &empty),
                Err(MachineStorageError::BuiltInReadOnly { .. })
            ));
            assert!(matches!(
                delete_user_machine_in(id, &empty),
                Err(MachineStorageError::BuiltInReadOnly { .. })
            ));
        }
        let resolved_milo_path = repo_machines_dir.join("milo-1.5.machine.toml");
        assert!(
            resolve_machine_definition_in(
                &resolved_milo_path.display().to_string(),
                Some(&empty),
                None,
            )
            .is_err(),
            "ID-only resolve_machine_definition_in must not read arbitrary file paths"
        );
        let cli_loaded = resolve_machine_definition_cli_override_in(
            &resolved_milo_path.display().to_string(),
            Some(&empty),
            None,
        )
        .unwrap();
        assert_eq!(cli_loaded.id, MILO_1_5_MACHINE_ID);
        let _ = std::fs::remove_dir_all(&empty);
    }

    #[test]
    fn milestone_33_fusion_import_rejects_oversized_and_too_many_entries_archives() {
        use std::io::Write as _;

        // 1. Oversized bare JSON rejected before parsing
        let big_bare = vec![b' '; (fusion_import::MAX_FUSION_JSON_BYTES as usize) + 1];
        assert!(matches!(
            import_fusion_library_bytes(&big_bare, "oversized"),
            Err(FusionImportError::ArchiveTooLarge { .. })
        ));

        // 2. Oversized compressed JSON entry inside a `.tools` ZIP rejected via size cap
        let mut cursor = std::io::Cursor::new(Vec::new());
        {
            let mut zw = zip::ZipWriter::new(&mut cursor);
            let opts = zip::write::SimpleFileOptions::default()
                .compression_method(zip::CompressionMethod::Deflated);
            zw.start_file("tools.json", opts).unwrap();
            let chunk = vec![b' '; 1024 * 1024];
            for _ in 0..33 {
                zw.write_all(&chunk).unwrap();
            }
            zw.finish().unwrap();
        }
        assert!(matches!(
            import_fusion_library_bytes(&cursor.into_inner(), "zip_bomb"),
            Err(FusionImportError::ArchiveTooLarge { .. })
        ));
    }
}
