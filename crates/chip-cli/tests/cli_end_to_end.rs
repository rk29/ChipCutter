#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::{
    path::{Path, PathBuf},
    process::Command,
};

use chip_cad::{CadKernel, MonstertruckKernel, SurfaceKind};
use chip_cam::{GeometrySelectionItem, HeightReference, HeightSpec, Operation, OperationKind};
use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, approx_eq};
use chip_project::{ModelEntry, ProjectDocument, compute_sha256_hex};
use chip_tools::default_starter_tool_library;

fn cli_bin() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_chipcutter"))
}

fn golden_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/golden")
}

struct TempTestDir(PathBuf);

impl TempTestDir {
    fn new(tag: &str) -> Self {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_nanos());
        let dir = std::env::temp_dir().join(format!(
            "chipcutter_m5_e2e_{tag}_{}_{nanos}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).expect("create temp test dir");
        Self(dir)
    }

    fn path(&self) -> &Path {
        &self.0
    }
}

impl Drop for TempTestDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

#[test]
fn golden_gcode_snapshots_and_chipz_portable_build_match_byte_for_byte() {
    let tmp = TempTestDir::new("golden_and_chipz");
    let step_path = tmp.path().join("torture.step");
    let proj_path = tmp.path().join("torture.chip.toml");

    // 1. Author the seeded CAM Torture Block STEP + .chip.toml via headless CLI
    let gen_out = Command::new(cli_bin())
        .args([
            "generate-sample-step",
            step_path.to_str().unwrap(),
            "--cam-torture",
            "--project-out",
            proj_path.to_str().unwrap(),
        ])
        .output()
        .expect("run generate-sample-step");
    assert!(
        gen_out.status.success(),
        "generate-sample-step failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&gen_out.stdout),
        String::from_utf8_lossy(&gen_out.stderr)
    );

    // 2. Build Grbl (manual tool-change split) and compare byte-for-byte with golden snapshots
    let grbl_out_dir = tmp.path().join("out_grbl");
    let build_grbl = Command::new(cli_bin())
        .args([
            "build",
            proj_path.to_str().unwrap(),
            "-o",
            grbl_out_dir.to_str().unwrap(),
            "--post",
            "grbl",
        ])
        .output()
        .expect("run build --post grbl");
    assert!(
        build_grbl.status.success(),
        "build --post grbl failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&build_grbl.stdout),
        String::from_utf8_lossy(&build_grbl.stderr)
    );

    let expected_grbl_files = [
        "torture_01_T7_38mm_3_insert_apkt_face.nc",
        "torture_02_T1_6mm_3fl_carbide_flat_end.nc",
        "torture_03_T6_6_0mm_118_carbide_stub_d.nc",
        "torture_04_T5_6mm_90_carbide_chamfer_s.nc",
    ];
    for fname in expected_grbl_files {
        let actual = std::fs::read_to_string(grbl_out_dir.join(fname))
            .unwrap_or_else(|e| panic!("missing emitted Grbl file {fname}: {e}"));
        let golden = std::fs::read_to_string(golden_dir().join("grbl").join(fname))
            .unwrap_or_else(|e| panic!("missing golden Grbl file {fname}: {e}"));
        assert_eq!(
            actual, golden,
            "emitted Grbl file '{fname}' drifted from golden snapshot"
        );
    }
    assert!(grbl_out_dir.join("build_summary.json").is_file());

    // 3. Build LinuxCNC RS274NGC (single ATC file) and compare byte-for-byte with golden snapshot
    let lcnc_out_dir = tmp.path().join("out_linuxcnc");
    let build_lcnc = Command::new(cli_bin())
        .args([
            "build",
            proj_path.to_str().unwrap(),
            "-o",
            lcnc_out_dir.to_str().unwrap(),
            "--post",
            "linuxcnc",
        ])
        .output()
        .expect("run build --post linuxcnc");
    assert!(
        build_lcnc.status.success(),
        "build --post linuxcnc failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&build_lcnc.stdout),
        String::from_utf8_lossy(&build_lcnc.stderr)
    );

    let golden_lcnc = std::fs::read_to_string(golden_dir().join("linuxcnc/torture.ngc"))
        .expect("read golden torture.ngc");
    let actual_lcnc =
        std::fs::read_to_string(lcnc_out_dir.join("torture.ngc")).expect("read actual torture.ngc");
    assert_eq!(
        actual_lcnc, golden_lcnc,
        "emitted LinuxCNC 'torture.ngc' drifted from golden snapshot"
    );

    // 4. Pack `.chipz` into an isolated directory with NO external `.step` file, then validate & build directly from `.chipz`!
    let isolated_dir = tmp.path().join("isolated_bundle");
    std::fs::create_dir_all(&isolated_dir).unwrap();
    let chipz_path = isolated_dir.join("torture.chipz");

    let pack_out = Command::new(cli_bin())
        .args([
            "bundle",
            "pack",
            proj_path.to_str().unwrap(),
            "-o",
            chipz_path.to_str().unwrap(),
        ])
        .output()
        .expect("run bundle pack");
    assert!(
        pack_out.status.success(),
        "bundle pack failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&pack_out.stdout),
        String::from_utf8_lossy(&pack_out.stderr)
    );
    assert!(chipz_path.is_file());

    // Validate `--generate` directly against the isolated `.chipz`
    let val_chipz = Command::new(cli_bin())
        .args(["validate", "--generate", chipz_path.to_str().unwrap()])
        .output()
        .expect("run validate --generate on .chipz");
    assert!(
        val_chipz.status.success(),
        "validate --generate on .chipz failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&val_chipz.stdout),
        String::from_utf8_lossy(&val_chipz.stderr)
    );

    // Build directly from the isolated `.chipz` and prove byte-for-byte identical G-code!
    let chipz_build_dir = tmp.path().join("out_from_chipz");
    let build_chipz = Command::new(cli_bin())
        .args([
            "build",
            chipz_path.to_str().unwrap(),
            "-o",
            chipz_build_dir.to_str().unwrap(),
            "--post",
            "linuxcnc",
        ])
        .output()
        .expect("run build on .chipz");
    assert!(
        build_chipz.status.success(),
        "build on .chipz failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&build_chipz.stdout),
        String::from_utf8_lossy(&build_chipz.stderr)
    );
    let chipz_lcnc = std::fs::read_to_string(chipz_build_dir.join("torture.ngc"))
        .expect("read torture.ngc built from .chipz");
    assert_eq!(
        chipz_lcnc, golden_lcnc,
        "G-code built from isolated .chipz must match golden LinuxCNC G-code byte-for-byte"
    );

    // Also verify `bundle unpack` extracts `project.chip.toml` and `models/` cleanly
    let unpacked_dir = tmp.path().join("unpacked_bundle");
    let unpack_out = Command::new(cli_bin())
        .args([
            "bundle",
            "unpack",
            chipz_path.to_str().unwrap(),
            "-o",
            unpacked_dir.to_str().unwrap(),
        ])
        .output()
        .expect("run bundle unpack");
    assert!(unpack_out.status.success());
    assert!(unpacked_dir.join("project.chip.toml").is_file());
    assert!(unpacked_dir.join("models").is_dir());
}

#[test]
fn cli_build_blocks_gcode_emission_when_target_gouge_is_injected() {
    let tmp = TempTestDir::new("gouge_gate");
    let step_path = tmp.path().join("torture.step");
    let proj_path = tmp.path().join("gouging_job.chip.toml");

    let gen_out = Command::new(cli_bin())
        .args([
            "generate-sample-step",
            step_path.to_str().unwrap(),
            "--cam-torture",
            "--project-out",
            proj_path.to_str().unwrap(),
        ])
        .output()
        .expect("generate sample job");
    assert!(gen_out.status.success());

    // Deliberately drop the island pocket bottom height -5.0 mm below the selected Z=12mm floor!
    let mut proj = ProjectDocument::load_from_path(&proj_path).expect("load seeded project");
    let pkt_op = proj.setups[0]
        .operations
        .iter_mut()
        .find(|op| op.id == "op_4_island_pocket")
        .expect("find island pocket op");
    pkt_op.heights.bottom = HeightSpec::new(HeightReference::FromSelection, -5.0);
    proj.save_to_path(&proj_path).expect("save gouging project");

    let bad_out_dir = tmp.path().join("bad_out");
    let build_res = Command::new(cli_bin())
        .args([
            "build",
            proj_path.to_str().unwrap(),
            "-o",
            bad_out_dir.to_str().unwrap(),
            "--post",
            "linuxcnc",
        ])
        .output()
        .expect("run build on gouging project");

    assert!(
        !build_res.status.success(),
        "chipcutter build MUST exit non-zero when physical heightmap verifier detects a target gouge!"
    );
    let stderr = String::from_utf8_lossy(&build_res.stderr);
    assert!(
        stderr.contains("Build ABORTED") || stderr.contains("Target Part Gouge"),
        "expected target gouge abort message on stderr, got:\n{stderr}"
    );
    assert!(
        !bad_out_dir.join("gouging_job.ngc").exists(),
        "G-code must NOT be written when build fails safety verification!"
    );
}

#[test]
fn cli_reload_step_handles_v1_to_v2_rebind_and_deleted_hole_flagging() {
    let tmp = TempTestDir::new("reload_v1_v2");
    let v1_path = tmp.path().join("part_v1.step");
    let v2_path = tmp.path().join("part_v2.step");
    let proj_v1_path = tmp.path().join("rev_job.chip.toml");
    let proj_v2_path = tmp.path().join("rev_job_v2.chip.toml");

    assert!(
        Command::new(cli_bin())
            .args([
                "generate-sample-step",
                v1_path.to_str().unwrap(),
                "--revision-v1"
            ])
            .status()
            .unwrap()
            .success()
    );
    assert!(
        Command::new(cli_bin())
            .args([
                "generate-sample-step",
                v2_path.to_str().unwrap(),
                "--revision-v2"
            ])
            .status()
            .unwrap()
            .success()
    );

    let v1_bytes = std::fs::read(&v1_path).unwrap();
    let doc_v1 = MonstertruckKernel
        .load_step_bytes("part_v1.step", &v1_bytes, DEFAULT_TESS_CHORD_TOL_MM)
        .unwrap();

    let starter = default_starter_tool_library();
    let flat_6mm = &starter.tools[0];
    let drill_6mm = &starter.tools[5];

    let mut proj = ProjectDocument::new("CLI Reload Revision Test");
    proj.models.push(ModelEntry {
        id: "model_1".to_string(),
        path: "part_v1.step".to_string(),
        sha256: compute_sha256_hex(&v1_bytes),
        unit_override: None,
    });
    proj.ensure_tool_embedded(flat_6mm);
    proj.ensure_tool_embedded(drill_6mm);

    // Resized pocket (`Z = 12.0` -> `Z = 11.0`)
    let v1_pkt = doc_v1
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z > 0.99 && approx_eq(origin_mm.z, 12.0, 1e-2)
            )
        })
        .unwrap()
        .id;
    let mut op_pkt = Operation::new("op_pkt", "Resized Blind Pocket", OperationKind::Pocket2D);
    op_pkt.tool_id = Some(flat_6mm.id.clone());
    op_pkt.selections.push(GeometrySelectionItem::PlanarFace {
        binding: doc_v1.bind_face(v1_pkt).unwrap(),
    });

    // Surviving Ø6mm hole (`Y = 48.0` -> `Y = 47.0`)
    let v1_keep_hole = doc_v1
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Cylinder {
                    axis_origin_mm,
                    radius_mm,
                    is_internal: true,
                    ..
                } if approx_eq(radius_mm, 3.0, 1e-2) && approx_eq(axis_origin_mm.y, 48.0, 0.2)
            )
        })
        .unwrap()
        .id;
    let mut op_keep = Operation::new("op_keep", "Surviving Hole", OperationKind::DrillBore);
    op_keep.tool_id = Some(drill_6mm.id.clone());
    op_keep
        .selections
        .push(GeometrySelectionItem::CylindricalHole {
            binding: doc_v1.bind_face(v1_keep_hole).unwrap(),
        });

    // Deleted Ø6mm hole (`Y = 60.0` -> deleted in v2!)
    let v1_del_hole = doc_v1
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Cylinder {
                    axis_origin_mm,
                    radius_mm,
                    is_internal: true,
                    ..
                } if approx_eq(radius_mm, 3.0, 1e-2) && approx_eq(axis_origin_mm.y, 60.0, 0.2)
            )
        })
        .unwrap()
        .id;
    let mut op_del = Operation::new("op_del", "Deleted Mounting Hole", OperationKind::DrillBore);
    op_del.tool_id = Some(drill_6mm.id.clone());
    op_del.feed_overrides.spindle_rpm = Some(4_800.0);
    op_del
        .selections
        .push(GeometrySelectionItem::CylindricalHole {
            binding: doc_v1.bind_face(v1_del_hole).unwrap(),
        });

    proj.setups[0].operations = vec![op_pkt, op_keep, op_del];
    proj.save_to_path(&proj_v1_path).unwrap();

    // Without `--allow-needs-repick`, reload-step must exit non-zero so CI catches orphaned features!
    let strict_reload = Command::new(cli_bin())
        .args([
            "reload-step",
            proj_v1_path.to_str().unwrap(),
            "--step",
            v2_path.to_str().unwrap(),
            "-o",
            proj_v2_path.to_str().unwrap(),
        ])
        .output()
        .expect("run strict reload-step");
    assert!(
        !strict_reload.status.success(),
        "reload-step without --allow-needs-repick must exit non-zero when a feature was deleted"
    );

    // With `--allow-needs-repick`, reload-step exits 0 and writes the rebound v2 project!
    let allow_reload = Command::new(cli_bin())
        .args([
            "reload-step",
            proj_v1_path.to_str().unwrap(),
            "--step",
            v2_path.to_str().unwrap(),
            "-o",
            proj_v2_path.to_str().unwrap(),
            "--allow-needs-repick",
        ])
        .output()
        .expect("run reload-step --allow-needs-repick");
    assert!(allow_reload.status.success());
    let stdout = String::from_utf8_lossy(&allow_reload.stdout);
    assert!(stdout.contains("[REBOUND ]"));
    assert!(stdout.contains("[REPICK! ]"));

    // Verify tool + spindle RPM override on `op_del` survived, and suppressing `op_del` lets v2 build cleanly!
    let mut reloaded = ProjectDocument::load_from_path(&proj_v2_path).unwrap();
    assert_eq!(reloaded.models[0].path, "part_v2.step");
    assert_eq!(
        reloaded.setups[0].operations[2].feed_overrides.spindle_rpm,
        Some(4_800.0)
    );
    reloaded.setups[0].operations[2].suppressed = true;
    reloaded.save_to_path(&proj_v2_path).unwrap();

    let v2_build_out = tmp.path().join("out_v2");
    let build_v2 = Command::new(cli_bin())
        .args([
            "build",
            proj_v2_path.to_str().unwrap(),
            "-o",
            v2_build_out.to_str().unwrap(),
            "--post",
            "linuxcnc",
        ])
        .output()
        .expect("build reloaded v2 project");
    assert!(
        build_v2.status.success(),
        "reloaded v2 build failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&build_v2.stdout),
        String::from_utf8_lossy(&build_v2.stderr)
    );
    assert!(v2_build_out.join("rev_job_v2.ngc").is_file());
}

#[test]
fn cli_pure2d_inspect_import_bundle_simulate_and_build_work_end_to_end() {
    use chip_project::build_pure2d_router_panel_demo_project;
    use chip_sketch::{sample_badge_logo_svg, sample_router_panel_dxf};

    let tmp = TempTestDir::new("pure2d");
    let dxf_path = tmp.path().join("router_panel.dxf");
    let svg_path = tmp.path().join("badge_logo.svg");
    std::fs::write(&dxf_path, sample_router_panel_dxf()).unwrap();
    std::fs::write(&svg_path, sample_badge_logo_svg()).unwrap();

    // 1. `chipcutter inspect` on both `.dxf` and `.svg`
    let insp_dxf = Command::new(cli_bin())
        .args(["inspect", dxf_path.to_str().unwrap()])
        .output()
        .expect("inspect dxf");
    assert!(insp_dxf.status.success());
    let dxf_out = String::from_utf8_lossy(&insp_dxf.stdout);
    assert!(dxf_out.contains("AutoCAD DXF"));
    assert!(dxf_out.contains("4 Circles"));

    let insp_svg = Command::new(cli_bin())
        .args(["inspect", svg_path.to_str().unwrap()])
        .output()
        .expect("inspect svg");
    assert!(insp_svg.status.success());
    let svg_out = String::from_utf8_lossy(&insp_svg.stdout);
    assert!(svg_out.contains("W3C SVG"));

    // 2. `chipcutter import-2d` initializes a fresh Pure-2D `.chip.toml`
    let imported_toml = tmp.path().join("imported_svg.chip.toml");
    let imp_status = Command::new(cli_bin())
        .args([
            "import-2d",
            svg_path.to_str().unwrap(),
            "-o",
            imported_toml.to_str().unwrap(),
            "--thickness",
            "8.0",
        ])
        .status()
        .expect("import-2d svg");
    assert!(imp_status.success());
    let loaded_imp = ProjectDocument::load_from_path(&imported_toml).unwrap();
    assert!(loaded_imp.models.is_empty());
    assert_eq!(loaded_imp.sketches.len(), 1);

    // 3. Full Pure-2D Router Panel `.chip.toml` -> `bundle pack` -> `simulate` -> `build`!
    let starter = default_starter_tool_library();
    let pure2d_proj = build_pure2d_router_panel_demo_project(
        Some("router_panel.dxf".to_string()),
        &starter.tools,
    );
    let pure2d_toml = tmp.path().join("panel_job.chip.toml");
    pure2d_proj.save_to_path(&pure2d_toml).unwrap();

    let chipz_path = tmp.path().join("panel_job.chipz");
    assert!(
        Command::new(cli_bin())
            .args([
                "bundle",
                "pack",
                pure2d_toml.to_str().unwrap(),
                "-o",
                chipz_path.to_str().unwrap(),
            ])
            .status()
            .unwrap()
            .success()
    );

    let out_dir = tmp.path().join("nc_pure2d");
    let build_res = Command::new(cli_bin())
        .args([
            "build",
            chipz_path.to_str().unwrap(),
            "-o",
            out_dir.to_str().unwrap(),
            "--post",
            "grbl",
        ])
        .output()
        .expect("build pure-2d chipz");
    assert!(
        build_res.status.success(),
        "pure-2d build failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&build_res.stdout),
        String::from_utf8_lossy(&build_res.stderr)
    );
    assert!(
        out_dir
            .join("panel_job_01_T6_6_0mm_118_carbide_stub_d.nc")
            .is_file()
    );
    assert!(
        out_dir
            .join("panel_job_02_T1_6mm_3fl_carbide_flat_end.nc")
            .is_file()
    );
    assert!(out_dir.join("build_summary.json").is_file());
}

#[test]
fn v2_golden_adaptive_rest_and_two_tool_vcarve_snapshots_match_byte_for_byte() {
    use chip_cad::sample_cam_torture_block_step;
    use chip_project::{
        build_adaptive_rest_demo_project, build_vcarve_sign_and_inlay_demo_project,
    };

    let tmp = TempTestDir::new("v2_goldens");
    let starter = default_starter_tool_library();
    let update_goldens = std::env::var_os("UPDATE_V2_GOLDENS").is_some();

    // 1. v2 Golden A: 2D Adaptive Roughing + 2D Rest Cleanup (`linuxcnc/v2_adaptive_rest.ngc`)
    let step_str = sample_cam_torture_block_step().unwrap();
    let step_path = tmp.path().join("torture.step");
    std::fs::write(&step_path, step_str.as_bytes()).unwrap();
    let cad_doc = MonstertruckKernel
        .load_step_bytes(
            "torture.step",
            step_str.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    let mut adap_proj = build_adaptive_rest_demo_project(
        &cad_doc,
        "torture.step".to_string(),
        &compute_sha256_hex(step_str.as_bytes()),
        &starter.tools,
    );
    for op in &mut adap_proj.setups[0].operations {
        if op.id == "op_3_open_shoulder_adaptive" {
            op.suppressed = true;
        }
    }
    let adap_toml = tmp.path().join("v2_adaptive_rest.chip.toml");
    adap_proj.save_to_path(&adap_toml).unwrap();

    let adap_out = tmp.path().join("out_adap_lcnc");
    let adap_res = Command::new(cli_bin())
        .args([
            "build",
            adap_toml.to_str().unwrap(),
            "-o",
            adap_out.to_str().unwrap(),
            "--post",
            "linuxcnc",
        ])
        .output()
        .expect("build v2_adaptive_rest");
    assert!(
        adap_res.status.success(),
        "v2_adaptive_rest build failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&adap_res.stdout),
        String::from_utf8_lossy(&adap_res.stderr)
    );
    let actual_adap = std::fs::read_to_string(adap_out.join("v2_adaptive_rest.ngc")).unwrap();
    let golden_adap_path = golden_dir().join("linuxcnc/v2_adaptive_rest.ngc");
    if update_goldens {
        std::fs::write(&golden_adap_path, &actual_adap).unwrap();
    }
    let expected_adap = std::fs::read_to_string(&golden_adap_path).unwrap();
    assert_eq!(
        actual_adap, expected_adap,
        "v2_adaptive_rest.ngc drifted from golden snapshot"
    );

    // 2. v2 Golden B: Two-Tool V-Inlay Male Plug (`grbl` split files `T1` Stage 1 + `T8` Stage 2)
    let mut vcarve_proj = build_vcarve_sign_and_inlay_demo_project(&starter.tools);
    vcarve_proj.setups.remove(0);
    let vcarve_toml = tmp.path().join("v2_inlay.chip.toml");
    vcarve_proj.save_to_path(&vcarve_toml).unwrap();

    let vc_out = tmp.path().join("out_vc_grbl");
    let vc_res = Command::new(cli_bin())
        .args([
            "build",
            vcarve_toml.to_str().unwrap(),
            "-o",
            vc_out.to_str().unwrap(),
            "--post",
            "grbl",
        ])
        .output()
        .expect("build v2_inlay");
    assert!(
        vc_res.status.success(),
        "v2_inlay build failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&vc_res.stdout),
        String::from_utf8_lossy(&vc_res.stderr)
    );

    let v2_grbl_files = [
        "v2_inlay_01_T1_6mm_3fl_carbide_flat_end.nc",
        "v2_inlay_02_T8_6mm_60_carbide_engraving.nc",
    ];
    for fname in v2_grbl_files {
        let actual = std::fs::read_to_string(vc_out.join(fname)).unwrap_or_else(|e| {
            panic!(
                "missing emitted v2 Grbl file {fname} ({e}); stdout:\n{}",
                String::from_utf8_lossy(&vc_res.stdout)
            )
        });
        let gpath = golden_dir().join("grbl").join(fname);
        if update_goldens {
            std::fs::write(&gpath, &actual).unwrap();
        }
        let expected = std::fs::read_to_string(&gpath)
            .unwrap_or_else(|e| panic!("missing golden v2 Grbl file {fname}: {e}"));
        assert_eq!(actual, expected, "{fname} drifted from golden snapshot");
    }
}

#[test]
fn cli_inspect_import_mesh_validate_and_bundle_roundtrip_for_stl_obj_3mf() {
    use chip_cad::{
        MeshUnitKind, sample_box_3mf_bytes, sample_relief_wave_obj_text,
        sample_sculpted_dome_stl_bytes,
    };

    let tmp = TempTestDir::new("mesh_m16");
    let stl_path = tmp.path().join("dome.stl");
    let obj_path = tmp.path().join("wave.obj");
    let mf_path = tmp.path().join("box.3mf");

    std::fs::write(&stl_path, sample_sculpted_dome_stl_bytes()).unwrap();
    std::fs::write(&obj_path, sample_relief_wave_obj_text()).unwrap();
    std::fs::write(&mf_path, sample_box_3mf_bytes(MeshUnitKind::Inch, None)).unwrap();

    // 1. Inspect STL, OBJ, and 3MF
    for (p, expected_fmt) in [
        (&stl_path, "Binary STL"),
        (&obj_path, "Wavefront OBJ"),
        (&mf_path, "3MF (OPC Archive)"),
    ] {
        let out = Command::new(cli_bin())
            .args(["inspect", p.to_str().unwrap()])
            .output()
            .expect("inspect mesh");
        assert!(
            out.status.success(),
            "inspect {} failed:\n{}",
            p.display(),
            String::from_utf8_lossy(&out.stderr)
        );
        let stdout = String::from_utf8_lossy(&out.stdout);
        assert!(
            stdout.contains(expected_fmt),
            "missing {expected_fmt} in:\n{stdout}"
        );
        assert!(
            stdout.contains("Watertight"),
            "expected Watertight in:\n{stdout}"
        );
    }

    // 2. `chipcutter import-mesh` -> `schema_version = 3` project
    let proj_path = tmp.path().join("dome.chip.toml");
    let imp = Command::new(cli_bin())
        .args([
            "import-mesh",
            stl_path.to_str().unwrap(),
            "-o",
            proj_path.to_str().unwrap(),
            "--mesh-units",
            "mm",
        ])
        .output()
        .expect("import-mesh");
    assert!(
        imp.status.success(),
        "import-mesh failed:\n{}",
        String::from_utf8_lossy(&imp.stderr)
    );
    let proj = ProjectDocument::load_from_path(&proj_path).unwrap();
    assert_eq!(proj.schema_version, 3);
    assert_eq!(proj.models.len(), 1);
    assert_eq!(proj.models[0].unit_override, Some(MeshUnitKind::Millimeter));

    // 3. `chipcutter validate` on imported mesh project succeeds with 0 errors
    let val = Command::new(cli_bin())
        .args(["validate", proj_path.to_str().unwrap()])
        .output()
        .expect("validate mesh project");
    assert!(
        val.status.success(),
        "validate mesh project failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&val.stdout),
        String::from_utf8_lossy(&val.stderr)
    );

    // 4. `chipcutter bundle pack` & `unpack` round-trip preserves STL mesh + SHA-256
    let chipz_path = tmp.path().join("dome.chipz");
    let pack = Command::new(cli_bin())
        .args([
            "bundle",
            "pack",
            proj_path.to_str().unwrap(),
            "-o",
            chipz_path.to_str().unwrap(),
        ])
        .output()
        .expect("bundle pack mesh");
    assert!(
        pack.status.success(),
        "bundle pack mesh failed:\n{}",
        String::from_utf8_lossy(&pack.stderr)
    );

    let unpack_dir = tmp.path().join("unpacked_dome");
    let unpack = Command::new(cli_bin())
        .args([
            "bundle",
            "unpack",
            chipz_path.to_str().unwrap(),
            "-o",
            unpack_dir.to_str().unwrap(),
        ])
        .output()
        .expect("bundle unpack mesh");
    assert!(
        unpack.status.success(),
        "bundle unpack mesh failed:\n{}",
        String::from_utf8_lossy(&unpack.stderr)
    );
    assert!(unpack_dir.join("models/dome.stl").is_file());
    let val_chipz = Command::new(cli_bin())
        .args(["validate", chipz_path.to_str().unwrap()])
        .output()
        .expect("validate mesh chipz");
    assert!(
        val_chipz.status.success(),
        "validate mesh .chipz failed:\n{}",
        String::from_utf8_lossy(&val_chipz.stderr)
    );
}

#[test]
fn v3_golden_3axis_mesh_surfacing_snapshots_match_byte_for_byte() {
    use chip_cad::sample_sculpted_dome_stl_bytes;
    use chip_cam::{
        HeightReference, HeightSpec, MachiningBoundaryMode, Operation, OperationKind,
        ParallelCutDirection, StrategyParams, ToolContainmentMode,
    };
    use chip_math::ParamLength;
    use chip_project::build_sculpted_mesh_dome_project;

    let tmp = TempTestDir::new("v3_goldens");
    let starter = default_starter_tool_library();
    let update_goldens = std::env::var_os("UPDATE_V3_GOLDENS").is_some();

    let stl_bytes = sample_sculpted_dome_stl_bytes();
    let stl_path = tmp.path().join("sculpted_dome.stl");
    std::fs::write(&stl_path, &stl_bytes).unwrap();
    let sha = compute_sha256_hex(&stl_bytes);

    let mut proj =
        build_sculpted_mesh_dome_project("sculpted_dome.stl".to_string(), &sha, &starter.tools);
    // Replace Op 2 (Waterline) with a crisp 3D Parallel Raster finish pass so this golden exercises
    // Adaptive3D + Parallel3D + Pencil3D together while keeping golden files concise and deterministic.
    let ball_6_id = proj.setups[0].operations[1].tool_id.clone();
    let ball_6_preset = proj.setups[0].operations[1].preset_id.clone();
    let mut op_par = Operation::new(
        "op_2_parallel_3d",
        "3D Parallel Finish",
        OperationKind::Parallel3D,
    );
    op_par.tool_id = ball_6_id;
    op_par.preset_id = ball_6_preset;
    op_par.heights.bottom = HeightSpec::new(HeightReference::ModelTop, -25.0);
    if let StrategyParams::Parallel3D(p) = op_par.strategy_mut() {
        p.boundary_mode = MachiningBoundaryMode::ModelSilhouette;
        p.tool_containment = ToolContainmentMode::ToolInsideBoundary;
        p.use_cusp_height = false;
        p.stepover = ParamLength::from_mm(3.0);
        p.pass_angle_deg = 0.0;
        p.cut_direction = ParallelCutDirection::ZigZag;
    }
    proj.setups[0].operations[1] = op_par;
    // Keep Adaptive3D coarse enough for compact deterministic golden snapshots
    if let StrategyParams::Adaptive3D(p) = proj.setups[0].operations[0].strategy_mut() {
        p.max_roughing_stepdown = ParamLength::from_mm(8.0);
        p.fine_stepdown = ParamLength::from_mm(4.0);
        p.adaptive.optimal_load = ParamLength::from_mm(2.0);
    }

    let proj_toml = tmp.path().join("v3_mesh_dome_3axis.chip.toml");
    proj.save_to_path(&proj_toml).unwrap();

    // 1. LinuxCNC ATC single-file 3-axis golden (`linuxcnc/v3_mesh_dome_3axis.ngc`)
    let lcnc_out = tmp.path().join("out_v3_lcnc");
    let lcnc_res = Command::new(cli_bin())
        .args([
            "build",
            proj_toml.to_str().unwrap(),
            "-o",
            lcnc_out.to_str().unwrap(),
            "--post",
            "linuxcnc",
        ])
        .output()
        .expect("build v3_mesh_dome_3axis linuxcnc");
    assert!(
        lcnc_res.status.success(),
        "v3 LinuxCNC build failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&lcnc_res.stdout),
        String::from_utf8_lossy(&lcnc_res.stderr)
    );
    let actual_lcnc = std::fs::read_to_string(lcnc_out.join("v3_mesh_dome_3axis.ngc")).unwrap();
    let golden_lcnc = golden_dir().join("linuxcnc/v3_mesh_dome_3axis.ngc");
    if update_goldens {
        std::fs::write(&golden_lcnc, &actual_lcnc).unwrap();
    }
    let expected_lcnc = std::fs::read_to_string(&golden_lcnc).unwrap();
    assert_eq!(
        actual_lcnc, expected_lcnc,
        "v3_mesh_dome_3axis.ngc drifted from golden snapshot"
    );

    // 2. GRBL 3-tool split golden (`grbl/v3_mesh_dome_3axis_01..03`)
    let grbl_out = tmp.path().join("out_v3_grbl");
    let grbl_res = Command::new(cli_bin())
        .args([
            "build",
            proj_toml.to_str().unwrap(),
            "-o",
            grbl_out.to_str().unwrap(),
            "--post",
            "grbl",
        ])
        .output()
        .expect("build v3_mesh_dome_3axis grbl");
    assert!(
        grbl_res.status.success(),
        "v3 GRBL build failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&grbl_res.stdout),
        String::from_utf8_lossy(&grbl_res.stderr)
    );

    let mut nc_files: Vec<String> = std::fs::read_dir(&grbl_out)
        .unwrap()
        .filter_map(|e| {
            let name = e.ok()?.file_name().to_string_lossy().into_owned();
            name.ends_with(".nc").then_some(name)
        })
        .collect();
    nc_files.sort();
    assert_eq!(
        nc_files.len(),
        3,
        "Expected 3 split GRBL files (T1 Adaptive3D, T4 Parallel3D, T11 Pencil3D), got: {nc_files:?}"
    );
    for fname in nc_files {
        let actual = std::fs::read_to_string(grbl_out.join(&fname)).unwrap();
        let golden = golden_dir().join("grbl").join(&fname);
        if update_goldens {
            std::fs::write(&golden, &actual).unwrap();
        }
        let expected = std::fs::read_to_string(&golden).unwrap();
        assert_eq!(actual, expected, "{fname} drifted from golden snapshot");
    }
}
