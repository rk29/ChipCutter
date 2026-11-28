//! Tool geometry, per-material cutting presets, coupled SFM/chipload calculator,
//! `.tools.toml` persistence, and Fusion 360 JSON / `.tools` importer (`chip-tools`).

pub mod builtin;
pub mod envelope;
pub mod feeds_speeds;
#[cfg(feature = "fusion-import")]
pub mod fusion_import;
pub mod materials;
pub mod model;
#[cfg(feature = "user-config")]
pub mod preset_store;
pub mod storage;
pub mod thread_tables;

pub use builtin::{
    default_starter_tool_library, starter_dovetail_60deg_12mm, starter_lollipop_6mm,
    starter_tslot_12mm, starter_undercut_tools,
};
pub use envelope::{
    AnalyticCutterEnvelope, CutterEnvelopeShape, ToolAssemblyEnvelope, ToolAssemblyPart,
};
pub use feeds_speeds::{
    ChipThinningHint, CuttingPreset, SurfaceSpeedPresetSpec, chipload_from_feed,
    compute_adaptive_thinning_feed, compute_radial_chip_thinning, feed_from_chipload,
    rpm_from_surface_speed, surface_speed_from_rpm,
};
#[cfg(feature = "fusion-import")]
pub use fusion_import::{
    FusionImportError, FusionImportReport, import_fusion_library_bytes, import_fusion_library_path,
};
pub use materials::{
    ACETAL_DELRIN_ID, ACRYLIC_PMMA_ID, ALUMINUM_6061_T6_ID, ALUMINUM_6063_T5_ID,
    ALUMINUM_7075_T6_ID, BRASS_C360_ID, CUSTOM_WORKPIECE_MATERIAL_ID,
    DEFAULT_WORKPIECE_MATERIAL_ID, G10_CARBON_FIBER_ID, HARDWOOD_PLYWOOD_ID, HDPE_PLASTIC_ID,
    MDF_ID, MILD_STEEL_1018_ID, MaterialCategory, ProjectMaterialSelection,
    WorkpieceMaterialProfile, builtin_workpiece_materials, match_best_preset_for_material,
    resolve_workpiece_material,
};
pub use model::{
    CoolantMode, CutterGeometry, SpindleDirection, Tool, ToolId, ToolValidationError,
    is_exact_tool_match, rank_replacement_tools,
};
pub use storage::{ToolLibrary, ToolStorageError};
#[cfg(feature = "user-config")]
pub use storage::{default_global_tool_library_path, load_or_default_global_library};
pub use thread_tables::{
    ISO_60_H_OVER_P, ThreadSpecEntry, ThreadStandardKind, all_thread_specs, find_thread_spec,
    iso60_minor_diameter_internal_mm, iso60_pitch_diameter_mm, suggest_thread_for_hole_diameter,
    thread_specs_for_standard,
};

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    #[cfg(feature = "fusion-import")]
    use std::io::Write as _;

    use chip_math::{DisplayUnit, approx_eq};
    #[cfg(feature = "fusion-import")]
    use chip_math::{EvalContext, evaluate_length_mm};

    use super::*;

    #[test]
    fn starter_library_validates_and_roundtrips_toml_deterministically() {
        let lib = default_starter_tool_library();
        lib.validate_all().unwrap();
        assert_eq!(lib.tools.len(), 10);
        assert!(
            lib.tools
                .iter()
                .filter(|t| matches!(t.geometry, CutterGeometry::ThreadMill { .. }))
                .count()
                == 2
        );

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

    #[cfg(feature = "fusion-import")]
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
              "type": "form mill",
              "unit": "millimeters",
              "description": "Custom Exotic Form Cutter",
              "geometry": { "DC": 8.0, "NOF": 4 }
            }
          ]
        }"#;

        let report = import_fusion_library_bytes(fusion_json.as_bytes(), "Shop Fusion").unwrap();
        assert_eq!(report.imported_tool_count, 2);
        assert_eq!(report.imported_preset_count, 1);
        assert_eq!(report.warnings.len(), 1);
        assert!(report.warnings[0].contains("form mill"));

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

    #[cfg(feature = "fusion-import")]
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

    #[cfg(feature = "fusion-import")]
    #[test]
    fn thread_mill_geometry_validates_invariants_roundtrips_toml_and_imports_from_fusion() {
        let valid_tm = Tool {
            id: ToolId("tm_m8_multi".to_string()),
            tool_number: 11,
            length_offset_number: 11,
            diameter_offset_number: 11,
            name: "6.0mm 4-Tooth M8x1.25 Thread Mill".to_string(),
            vendor: Some("Harvey Tool".to_string()),
            product_id: Some("HT-M8-125".to_string()),
            native_unit: DisplayUnit::Millimeter,
            spindle_direction: SpindleDirection::Cw,
            default_coolant: CoolantMode::Mist,
            stickout_length_mm: 24.0,
            geometry: CutterGeometry::ThreadMill {
                diameter_mm: 6.0,
                neck_diameter_mm: 3.6,
                neck_length_mm: 18.0,
                pitch_mm: 1.25,
                teeth_count: 4,
                thread_angle_deg: 60.0,
                flute_length_mm: 5.0,
                overall_length_mm: 57.0,
                shank_diameter_mm: 6.0,
                flutes: 3,
            },
            presets: Vec::new(),
        };
        valid_tm.validate().unwrap();

        // Deterministic TOML roundtrip with `type = "thread_mill"`
        let lib = ToolLibrary {
            schema_version: ToolLibrary::CURRENT_SCHEMA_VERSION,
            name: "Thread Mill Test".to_string(),
            tools: vec![valid_tm.clone()],
        };
        let toml_text = lib.to_toml_string().unwrap();
        assert!(toml_text.contains("type = \"thread_mill\""));
        let reloaded = ToolLibrary::from_toml_str(&toml_text).unwrap();
        assert_eq!(reloaded.tools[0], valid_tm);

        // Validation rejects bad neck diameter, zero pitch, zero teeth, bad angle, bad reach
        let mut bad_neck = valid_tm.clone();
        if let CutterGeometry::ThreadMill {
            neck_diameter_mm, ..
        } = &mut bad_neck.geometry
        {
            *neck_diameter_mm = 6.0;
        }
        assert!(matches!(
            bad_neck.validate(),
            Err(ToolValidationError::InvalidThreadMillNeck { .. })
        ));

        let mut bad_pitch = valid_tm.clone();
        if let CutterGeometry::ThreadMill { pitch_mm, .. } = &mut bad_pitch.geometry {
            *pitch_mm = 0.0;
        }
        assert!(matches!(
            bad_pitch.validate(),
            Err(ToolValidationError::InvalidThreadMillPitch { .. })
        ));

        let mut bad_teeth = valid_tm.clone();
        if let CutterGeometry::ThreadMill { teeth_count, .. } = &mut bad_teeth.geometry {
            *teeth_count = 0;
        }
        assert!(matches!(
            bad_teeth.validate(),
            Err(ToolValidationError::InvalidThreadMillTeethCount { .. })
        ));

        let mut bad_angle = valid_tm.clone();
        if let CutterGeometry::ThreadMill {
            thread_angle_deg, ..
        } = &mut bad_angle.geometry
        {
            *thread_angle_deg = 20.0;
        }
        assert!(matches!(
            bad_angle.validate(),
            Err(ToolValidationError::InvalidThreadMillAngle { .. })
        ));

        let mut bad_reach = valid_tm.clone();
        if let CutterGeometry::ThreadMill { neck_length_mm, .. } = &mut bad_reach.geometry {
            *neck_length_mm = 3.0; // < flute_length_mm (5.0)
        }
        assert!(matches!(
            bad_reach.validate(),
            Err(ToolValidationError::InvalidThreadMillReach { .. })
        ));

        // Fusion 360 `"thread mill"` import (metric single-point + imperial multi-form)
        let fusion_tm_json = r#"{
          "version": 16,
          "data": [
            {
              "type": "thread mill",
              "unit": "millimeters",
              "description": "4.75mm Single-Point 60deg Thread Mill",
              "vendor": "Harvey Tool",
              "guid": "tm-sp-475",
              "geometry": {
                "DC": 4.75,
                "ND": 2.85,
                "LN": 16.0,
                "TP": 1.0,
                "NOT": 1,
                "thread-profile-angle": 60.0,
                "LCF": 1.2,
                "LB": 22.0,
                "OAL": 50.0,
                "SFDM": 6.0,
                "NOF": 3
              },
              "post-process": { "number": 9, "clockwise": true, "coolant": "mist" }
            },
            {
              "type": "thread mill",
              "unit": "inches",
              "description": "1/4-20 Multi-Form Thread Mill",
              "guid": "tm-mf-qtr20",
              "geometry": {
                "DC": 0.185,
                "TP": 0.05,
                "LCF": 0.25,
                "LB": 0.625,
                "OAL": 2.5,
                "SFDM": 0.25,
                "NOF": 4
              },
              "post-process": { "number": 10 }
            }
          ]
        }"#;

        let report =
            import_fusion_library_bytes(fusion_tm_json.as_bytes(), "Fusion Thread Mills").unwrap();
        assert_eq!(report.imported_tool_count, 2);
        assert!(
            report.warnings.is_empty(),
            "unexpected warnings: {:?}",
            report.warnings
        );

        let sp = &report.library.tools[0];
        match sp.geometry {
            CutterGeometry::ThreadMill {
                diameter_mm,
                neck_diameter_mm,
                neck_length_mm,
                pitch_mm,
                teeth_count,
                thread_angle_deg,
                flutes,
                ..
            } => {
                assert!(approx_eq(diameter_mm, 4.75, 1e-6));
                assert!(approx_eq(neck_diameter_mm, 2.85, 1e-6));
                assert!(approx_eq(neck_length_mm, 16.0, 1e-6));
                assert!(approx_eq(pitch_mm, 1.0, 1e-6));
                assert_eq!(teeth_count, 1);
                assert!(approx_eq(thread_angle_deg, 60.0, 1e-6));
                assert_eq!(flutes, 3);
            }
            ref other => panic!("expected ThreadMill, got {other:?}"),
        }

        let mf = &report.library.tools[1];
        assert_eq!(mf.native_unit, DisplayUnit::Inch);
        match mf.geometry {
            CutterGeometry::ThreadMill {
                diameter_mm,
                pitch_mm,
                teeth_count,
                flutes,
                ..
            } => {
                assert!(approx_eq(diameter_mm, 0.185 * 25.4, 1e-6));
                assert!(approx_eq(pitch_mm, 0.05 * 25.4, 1e-6));
                assert_eq!(teeth_count, 5); // 0.25 / 0.05 = 5 teeth inferred cleanly
                assert_eq!(flutes, 4);
            }
            ref other => panic!("expected ThreadMill, got {other:?}"),
        }
    }

    #[cfg(feature = "fusion-import")]
    #[test]
    fn undercut_cutters_validate_roundtrip_toml_and_import_from_fusion() {
        // Starter library T1..T10 indices remain untouched, while `starter_undercut_tools()`
        // provides validated starter Lollipop, Slot/T-Slot, and Dovetail cutters.
        let starter_default = default_starter_tool_library();
        assert_eq!(starter_default.tools.len(), 10);

        let undercuts = starter_undercut_tools();
        assert_eq!(undercuts.len(), 3);
        for t in &undercuts {
            t.validate().unwrap();
        }
        let lolli = &undercuts[0];
        let slot = &undercuts[1];
        let dove = &undercuts[2];

        // Deterministic TOML roundtrip with exact serde tag names
        let lib = ToolLibrary {
            schema_version: ToolLibrary::CURRENT_SCHEMA_VERSION,
            name: "Undercut Catalog".to_string(),
            tools: undercuts.clone(),
        };
        let toml_text = lib.to_toml_string().unwrap();
        assert!(toml_text.contains("type = \"lollipop_endmill\""));
        assert!(toml_text.contains("type = \"slot_mill\""));
        assert!(toml_text.contains("type = \"dovetail_cutter\""));
        let reloaded = ToolLibrary::from_toml_str(&toml_text).unwrap();
        assert_eq!(reloaded.tools, undercuts);

        // Validation rejections: neck >= diameter, bad dovetail angle, neck > D_top, bad corner radius, bad reach
        let mut bad_lolli = lolli.clone();
        if let CutterGeometry::LollipopEndmill {
            neck_diameter_mm, ..
        } = &mut bad_lolli.geometry
        {
            *neck_diameter_mm = 6.0;
        }
        assert!(matches!(
            bad_lolli.validate(),
            Err(ToolValidationError::InvalidRelievedNeck { .. })
        ));

        let mut bad_slot_rc = slot.clone();
        if let CutterGeometry::SlotMill {
            corner_radius_mm, ..
        } = &mut bad_slot_rc.geometry
        {
            *corner_radius_mm = 2.5; // > 0.5 * 4.0
        }
        assert!(matches!(
            bad_slot_rc.validate(),
            Err(ToolValidationError::InvalidUndercutCornerRadius { .. })
        ));

        let mut bad_slot_reach = slot.clone();
        if let CutterGeometry::SlotMill { neck_length_mm, .. } = &mut bad_slot_reach.geometry {
            *neck_length_mm = 2.0; // < cutting_thickness_mm (4.0)
        }
        assert!(matches!(
            bad_slot_reach.validate(),
            Err(ToolValidationError::InvalidRelievedReach { .. })
        ));

        let mut bad_dove_ang = dove.clone();
        if let CutterGeometry::DovetailCutter { side_angle_deg, .. } = &mut bad_dove_ang.geometry {
            *side_angle_deg = 15.0; // outside open interval (15°, 85°)
        }
        assert!(matches!(
            bad_dove_ang.validate(),
            Err(ToolValidationError::InvalidDovetailAngle { .. })
        ));

        let mut bad_dove_top = dove.clone();
        if let CutterGeometry::DovetailCutter {
            neck_diameter_mm, ..
        } = &mut bad_dove_top.geometry
        {
            *neck_diameter_mm = 9.0; // > derived D_top (~6.5 mm)
        }
        assert!(matches!(
            bad_dove_top.validate(),
            Err(ToolValidationError::InvalidDovetailTopNeck { .. })
        ));

        // Fusion 360 JSON import for "lollipop mill", "slot mill", and "dovetail mill"
        let fusion_undercuts = r#"{
          "version": 16,
          "data": [
            {
              "type": "lollipop mill",
              "unit": "millimeters",
              "description": "6mm Undercut Lollipop",
              "guid": "fusion-lollipop-6",
              "geometry": { "DC": 6.0, "ND": 3.0, "LN": 18.0, "LB": 24.0, "OAL": 55.0, "SFDM": 6.0, "NOF": 4 },
              "post-process": { "number": 13 }
            },
            {
              "type": "slot mill",
              "unit": "millimeters",
              "description": "12x4 T-Slot Cutter",
              "guid": "fusion-slot-12",
              "geometry": { "DC": 12.0, "LCF": 4.0, "RE": 0.2, "ND": 5.0, "LN": 18.0, "LB": 26.0, "OAL": 58.0, "SFDM": 6.0, "NOF": 4 },
              "post-process": { "number": 14 }
            },
            {
              "type": "dovetail mill",
              "unit": "millimeters",
              "description": "12mm 60deg Dovetail",
              "guid": "fusion-dove-12",
              "geometry": { "DC": 12.0, "TA": 60.0, "LCF": 5.0, "RE": 0.0, "ND": 5.5, "LN": 18.0, "LB": 26.0, "OAL": 58.0, "SFDM": 6.0, "NOF": 4 },
              "post-process": { "number": 15 }
            }
          ]
        }"#;

        let report =
            import_fusion_library_bytes(fusion_undercuts.as_bytes(), "Fusion Undercuts").unwrap();
        assert_eq!(report.imported_tool_count, 3);
        assert!(
            report.warnings.is_empty(),
            "warnings: {:?}",
            report.warnings
        );
        assert!(matches!(
            report.library.tools[0].geometry,
            CutterGeometry::LollipopEndmill { diameter_mm, neck_diameter_mm, .. }
                if approx_eq(diameter_mm, 6.0, 1e-6) && approx_eq(neck_diameter_mm, 3.0, 1e-6)
        ));
        assert!(matches!(
            report.library.tools[1].geometry,
            CutterGeometry::SlotMill { diameter_mm, cutting_thickness_mm, corner_radius_mm, .. }
                if approx_eq(diameter_mm, 12.0, 1e-6)
                    && approx_eq(cutting_thickness_mm, 4.0, 1e-6)
                    && approx_eq(corner_radius_mm, 0.2, 1e-6)
        ));
        assert!(matches!(
            report.library.tools[2].geometry,
            CutterGeometry::DovetailCutter { diameter_mm, side_angle_deg, cutting_height_mm, .. }
                if approx_eq(diameter_mm, 12.0, 1e-6)
                    && approx_eq(side_angle_deg, 60.0, 1e-6)
                    && approx_eq(cutting_height_mm, 5.0, 1e-6)
        ));
    }
}
