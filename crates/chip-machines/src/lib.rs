//! CNC machine definitions, built-in machine catalogs (`generic`, `milo-1.5`, `milo-2.0`,
//! `voron-cascade`, `carvera`, `carvera-air`, `carvera-z1`), and upgrade-safe `.machine.toml`
//! user configuration storage (`chip-machines`).

pub mod builtin_machines;
pub mod machine;

pub use builtin_machines::{
    CARVERA_AIR_MACHINE_ID, CARVERA_MACHINE_ID, CARVERA_Z1_MACHINE_ID, MILO_1_5_MACHINE_ID,
    MILO_2_0_MACHINE_ID, VORON_CASCADE_MACHINE_ID, builtin_carvera_air_machine,
    builtin_carvera_machine, builtin_carvera_z1_machine, builtin_milo_1_5_machine,
    builtin_milo_2_0_machine, builtin_voron_cascade_machine,
};
pub use machine::{
    BUILTIN_MACHINE_IDS, DEFAULT_MACHINE_ID, MACHINE_PRESET_STORE_SPEC, MachineDefinition,
    MachineKinematicsSpec, MachineRotaryDefaults, MachineSpindleSpec, MachineStorageError,
    MachineToolChangeKind, MachineTravelEnvelope, allocate_unique_custom_machine_id,
    builtin_generic_machine, builtin_machine_definitions, clone_machine_definition,
    delete_user_machine, delete_user_machine_in, export_machine_to_path, import_machine_from_path,
    import_machine_from_path_in, is_builtin_machine_id, load_available_machines,
    load_available_machines_in, resolve_machine_definition,
    resolve_machine_definition_cli_override, resolve_machine_definition_cli_override_in,
    resolve_machine_definition_in, sanitize_machine_id, save_user_machine, save_user_machine_in,
    user_machines_dir, validate_machine_id,
};

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::approx_eq;
    use chip_tools::CutterGeometry;

    use super::*;

    #[test]
    fn machine_definition_roundtrips_protects_builtins_and_supports_clone_export_import() {
        let generic = builtin_generic_machine();
        assert!(generic.is_builtin);
        generic.validate().unwrap();
        assert_eq!(generic.tools.len(), 10);

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
        assert_eq!(BUILTIN_MACHINE_IDS, expected_ids.as_slice());

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
    fn makera_machines_include_complete_essential_extended_bit_set() {
        let makera_machines = [
            builtin_carvera_machine(),
            builtin_carvera_air_machine(),
            builtin_carvera_z1_machine(),
        ];

        for machine in &makera_machines {
            machine.validate().unwrap();
            assert_eq!(
                machine.tools.len(),
                26,
                "Makera machine '{}' must include all 26 unique cutters from the Essential Milling Bit Set (Extended)",
                machine.id
            );

            for tool in &machine.tools {
                assert!(
                    approx_eq(tool.geometry.shank_diameter_mm(), 3.175, 1e-6),
                    "tool '{}' on '{}' must use a 3.175mm (1/8\") shank",
                    tool.name,
                    machine.id
                );
            }

            let flats: Vec<_> = machine
                .tools
                .iter()
                .filter(|t| matches!(t.geometry, CutterGeometry::FlatEndmill { .. }))
                .collect();
            assert_eq!(flats.len(), 9, "'{}' flat/corn count", machine.id);
            let single_flute_flats: Vec<(f64, f64)> = flats
                .iter()
                .filter_map(|t| match t.geometry {
                    CutterGeometry::FlatEndmill {
                        diameter_mm,
                        flute_length_mm,
                        flutes: 1,
                        ..
                    } => Some((diameter_mm, flute_length_mm)),
                    _ => None,
                })
                .collect();
            assert_eq!(
                single_flute_flats,
                vec![
                    (3.175, 12.0),
                    (3.175, 25.0),
                    (2.0, 8.0),
                    (1.0, 3.0),
                    (3.175, 42.0),
                ]
            );
            let corn_diameters: Vec<f64> = flats
                .iter()
                .filter_map(|t| match t.geometry {
                    CutterGeometry::FlatEndmill {
                        diameter_mm,
                        flutes: 2,
                        ..
                    } => Some(diameter_mm),
                    _ => None,
                })
                .collect();
            assert_eq!(corn_diameters, vec![0.6, 1.0, 2.0, 3.0]);

            let balls: Vec<(f64, f64)> = machine
                .tools
                .iter()
                .filter_map(|t| match t.geometry {
                    CutterGeometry::BallEndmill {
                        diameter_mm,
                        flute_length_mm,
                        flutes: 2,
                        ..
                    } => Some((diameter_mm, flute_length_mm)),
                    _ => None,
                })
                .collect();
            assert_eq!(
                balls,
                vec![
                    (3.175, 10.0),
                    (2.0, 6.0),
                    (1.0, 3.0),
                    (3.175, 22.0),
                    (2.0, 12.0),
                    (1.0, 4.0),
                ]
            );

            let chamfers: Vec<(f64, f64, u32)> = machine
                .tools
                .iter()
                .filter_map(|t| match t.geometry {
                    CutterGeometry::ChamferMill {
                        included_angle_deg,
                        tip_diameter_mm,
                        flutes,
                        ..
                    } => Some((included_angle_deg, tip_diameter_mm, flutes)),
                    _ => None,
                })
                .collect();
            assert_eq!(
                chamfers,
                vec![
                    (30.0, 0.2, 1),
                    (60.0, 0.1, 1),
                    (90.0, 0.1, 2),
                    (30.0, 0.3, 1),
                ]
            );

            let drills: Vec<f64> = machine
                .tools
                .iter()
                .filter_map(|t| match t.geometry {
                    CutterGeometry::TwistDrill { diameter_mm, .. } => Some(diameter_mm),
                    _ => None,
                })
                .collect();
            assert_eq!(drills, vec![1.0, 2.0, 2.5, 3.0]);

            let thread_mills: Vec<(f64, f64, f64)> = machine
                .tools
                .iter()
                .filter_map(|t| match t.geometry {
                    CutterGeometry::ThreadMill {
                        diameter_mm,
                        pitch_mm,
                        neck_length_mm,
                        ..
                    } => Some((diameter_mm, pitch_mm, neck_length_mm)),
                    _ => None,
                })
                .collect();
            assert_eq!(
                thread_mills,
                vec![(2.42, 0.5, 9.0), (3.15, 0.7, 12.0), (4.0, 0.8, 15.0)]
            );
        }
    }

    #[test]
    fn match_best_preset_prefers_6063_on_carvera_1fl_oflutes() {
        use chip_tools::{
            ALUMINUM_6061_T6_ID, ALUMINUM_6063_T5_ID, CoolantMode, HARDWOOD_PLYWOOD_ID,
            ProjectMaterialSelection, match_best_preset_for_material, resolve_workpiece_material,
        };

        let al6063 = resolve_workpiece_material(Some(&ProjectMaterialSelection::builtin(
            ALUMINUM_6063_T5_ID,
        )));
        let al6061 = resolve_workpiece_material(Some(&ProjectMaterialSelection::builtin(
            ALUMINUM_6061_T6_ID,
        )));
        let wood = resolve_workpiece_material(Some(&ProjectMaterialSelection::builtin(
            HARDWOOD_PLYWOOD_ID,
        )));

        for machine in [
            builtin_carvera_machine(),
            builtin_carvera_air_machine(),
            builtin_carvera_z1_machine(),
        ] {
            let t1 = machine.tools.iter().find(|t| t.tool_number == 1).unwrap();
            let t2 = machine.tools.iter().find(|t| t.tool_number == 2).unwrap();
            let t3 = machine.tools.iter().find(|t| t.tool_number == 3).unwrap();
            let t4 = machine.tools.iter().find(|t| t.tool_number == 4).unwrap();

            for t in [t1, t2, t3, t4] {
                let p6063 = match_best_preset_for_material(t, &al6063).unwrap();
                assert!(
                    p6063.material.contains("6063"),
                    "machine '{}' T{} must pick 6063 preset for aluminum-6063-t5, got '{}'",
                    machine.id,
                    t.tool_number,
                    p6063.name
                );
                assert_eq!(p6063.coolant_override, Some(CoolantMode::AirBlast));

                let p6061 = match_best_preset_for_material(t, &al6061).unwrap();
                assert!(
                    p6061.material.contains("6061"),
                    "machine '{}' T{} must keep picking 6061 preset for aluminum-6061-t6, got '{}'",
                    machine.id,
                    t.tool_number,
                    p6061.name
                );
            }

            let t2_6061 = match_best_preset_for_material(t2, &al6061).unwrap();
            let t2_6063 = match_best_preset_for_material(t2, &al6063).unwrap();
            assert!(t2_6063.default_stepdown_mm < t2_6061.default_stepdown_mm);
            assert!(t2_6063.default_stepover_mm < t2_6061.default_stepover_mm);

            let t1_wood = match_best_preset_for_material(t1, &wood).unwrap();
            assert!(t1_wood.id.ends_with("_wood"));
        }
    }
}
