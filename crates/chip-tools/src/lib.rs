//! Tool geometry, per-material cutting presets, coupled SFM/chipload calculator,
//! `.tools.toml` persistence, and Fusion 360 JSON / `.tools` importer (`chip-tools`).

pub mod builtin;
pub mod feeds_speeds;
pub mod fusion_import;
pub mod model;
pub mod storage;

pub use builtin::default_starter_tool_library;
pub use feeds_speeds::{
    ChipThinningHint, CuttingPreset, SurfaceSpeedPresetSpec, chipload_from_feed,
    compute_radial_chip_thinning, feed_from_chipload, rpm_from_surface_speed,
    surface_speed_from_rpm,
};
pub use fusion_import::{
    FusionImportError, FusionImportReport, import_fusion_library_bytes, import_fusion_library_path,
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
        assert_eq!(lib.tools.len(), 7);

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
                    "stepover": 0.1
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
}
