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

/// Reads a checked-in golden snapshot normalizing CRLF -> LF so byte-for-byte
/// comparisons stay platform-independent even on Windows `core.autocrlf=true` checkouts.
fn read_golden_normalized(path: &Path) -> String {
    std::fs::read_to_string(path)
        .unwrap_or_else(|e| panic!("missing golden file {}: {e}", path.display()))
        .replace("\r\n", "\n")
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
        let golden = read_golden_normalized(&golden_dir().join("grbl").join(fname));
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

    let golden_lcnc = read_golden_normalized(&golden_dir().join("linuxcnc/torture.ngc"));
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
        role: Default::default(),
        unit_override: None,
        transform: chip_cad::ModelTransform::IDENTITY,
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
    let expected_adap = read_golden_normalized(&golden_adap_path);
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
        let expected = read_golden_normalized(&gpath);
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
        p.boundary.boundary_mode = MachiningBoundaryMode::ModelSilhouette;
        p.boundary.tool_containment = ToolContainmentMode::ToolInsideBoundary;
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
    let expected_lcnc = read_golden_normalized(&golden_lcnc);
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
        let expected = read_golden_normalized(&golden);
        assert_eq!(actual, expected, "{fname} drifted from golden snapshot");
    }
}

#[test]
fn phase_5_v3_rotary_golden_gcode_snapshots_match_exact_expected_output() {
    use chip_cad::{sample_helical_flute_shaft_stl_bytes, sample_rotary_shaft_stl_bytes};
    use chip_cam::StrategyParams;
    use chip_math::ParamLength;
    use chip_project::{
        build_rotary_helical_flute_project, build_rotary_shaft_preview_project,
        build_rotary_wrapped_engraving_project, compute_sha256_hex,
    };
    use chip_tools::default_starter_tool_library;

    let tmp = TempTestDir::new("phase5_v3_rotary_goldens");
    let update_goldens = std::env::var("CHIPCUTTER_UPDATE_GOLDENS").is_ok();
    let starter = default_starter_tool_library();

    // 1. Rotary Indexed Shaft (`linuxcnc_4axis`: continuous Z+X+A spiral + 4× Indexed 3+1 flats)
    let shaft_stl = sample_rotary_shaft_stl_bytes();
    std::fs::write(tmp.path().join("rotary_shaft.stl"), &shaft_stl).unwrap();
    let shaft_sha = compute_sha256_hex(&shaft_stl);
    let mut shaft_proj = build_rotary_shaft_preview_project(
        "rotary_shaft.stl".to_string(),
        &shaft_sha,
        &starter.tools,
    );
    // Keep golden snapshot compact while preserving full continuous spiral + 4 indexed faces
    shaft_proj.setups[0].operations[0].passes.multiple_depths = false;
    if let StrategyParams::Rotary2Linear(p) = shaft_proj.setups[0].operations[0].strategy_mut() {
        p.use_cusp_height = false;
        p.stepover = ParamLength::from_mm(8.0);
    }
    for op in &mut shaft_proj.setups[0].operations[1..] {
        op.passes.multiple_depths = false;
    }
    let shaft_toml = tmp.path().join("v3_rotary_indexed_shaft.chip.toml");
    shaft_proj.save_to_path(&shaft_toml).unwrap();

    let shaft_out = tmp.path().join("out_rotary_shaft");
    let shaft_res = Command::new(cli_bin())
        .args([
            "build",
            shaft_toml.to_str().unwrap(),
            "-o",
            shaft_out.to_str().unwrap(),
            "--post",
            "linuxcnc_4axis",
        ])
        .output()
        .expect("build v3_rotary_indexed_shaft");
    assert!(
        shaft_res.status.success(),
        "rotary shaft build failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&shaft_res.stdout),
        String::from_utf8_lossy(&shaft_res.stderr)
    );
    let actual_shaft =
        std::fs::read_to_string(shaft_out.join("v3_rotary_indexed_shaft.ngc")).unwrap();
    assert!(actual_shaft.contains("G93") && actual_shaft.contains("A90.000"));
    let golden_shaft = golden_dir().join("linuxcnc/v3_rotary_indexed_shaft.ngc");
    if update_goldens {
        std::fs::write(&golden_shaft, &actual_shaft).unwrap();
    }
    let expected_shaft = read_golden_normalized(&golden_shaft);
    assert_eq!(
        actual_shaft, expected_shaft,
        "v3_rotary_indexed_shaft.ngc drifted from golden snapshot"
    );

    // 2. Wrapped Rotary Engraving (`grbl_rotary_wrap`: Y-swapped, zero A motion words)
    let mut wrap_proj = build_rotary_wrapped_engraving_project(&starter.tools);
    // Keep the flute pocket op for a crisp single-file swapped-axis golden
    wrap_proj.setups[0].operations.remove(0);
    let wrap_toml = tmp.path().join("v3_rotary_wrapped.chip.toml");
    wrap_proj.save_to_path(&wrap_toml).unwrap();

    let wrap_out = tmp.path().join("out_rotary_wrap");
    let wrap_res = Command::new(cli_bin())
        .args([
            "build",
            wrap_toml.to_str().unwrap(),
            "-o",
            wrap_out.to_str().unwrap(),
            "--post",
            "grbl_rotary_wrap",
        ])
        .output()
        .expect("build v3_rotary_wrapped");
    assert!(
        wrap_res.status.success(),
        "rotary wrap build failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&wrap_res.stdout),
        String::from_utf8_lossy(&wrap_res.stderr)
    );
    let mut wrap_files: Vec<String> = std::fs::read_dir(&wrap_out)
        .unwrap()
        .filter_map(|e| {
            let name = e.ok()?.file_name().to_string_lossy().into_owned();
            name.ends_with(".nc").then_some(name)
        })
        .collect();
    wrap_files.sort();
    assert_eq!(wrap_files.len(), 1);
    let actual_wrap = std::fs::read_to_string(wrap_out.join(&wrap_files[0])).unwrap();
    assert!(
        actual_wrap.lines().all(|line| {
            let t = line.trim();
            t.starts_with('(')
                || t.starts_with(';')
                || !t.split_whitespace().any(|tok| tok.starts_with('A'))
        }),
        "grbl_rotary_wrap golden must contain zero A motion words"
    );
    let golden_wrap = golden_dir().join("grbl/v3_rotary_wrapped.nc");
    if update_goldens {
        std::fs::write(&golden_wrap, &actual_wrap).unwrap();
    }
    let expected_wrap = read_golden_normalized(&golden_wrap);
    assert_eq!(
        actual_wrap, expected_wrap,
        "v3_rotary_wrapped.nc drifted from golden snapshot"
    );

    // 3. Simultaneous 4-Axis Helical Flute (`grblhal_4axis`: simultaneous G93 G1 X Y Z A)
    let flute_stl = sample_helical_flute_shaft_stl_bytes();
    std::fs::write(tmp.path().join("helical_flute_shaft.stl"), &flute_stl).unwrap();
    let flute_sha = compute_sha256_hex(&flute_stl);
    let mut flute_proj = build_rotary_helical_flute_project(
        "helical_flute_shaft.stl".to_string(),
        &flute_sha,
        &starter.tools,
    );
    // Keep finishing pass with coarse stepover for a compact simultaneous 4-axis golden
    flute_proj.setups[0].operations.remove(0);
    if let StrategyParams::Rotary4AxisSim(p) = flute_proj.setups[0].operations[0].strategy_mut() {
        p.use_cusp_height = false;
        p.stepover = ParamLength::from_mm(12.0);
    }
    let flute_toml = tmp.path().join("v3_rotary_4axis_sim.chip.toml");
    flute_proj.save_to_path(&flute_toml).unwrap();

    let flute_out = tmp.path().join("out_rotary_4ax");
    let flute_res = Command::new(cli_bin())
        .args([
            "build",
            flute_toml.to_str().unwrap(),
            "-o",
            flute_out.to_str().unwrap(),
            "--post",
            "grblhal_4axis",
        ])
        .output()
        .expect("build v3_rotary_4axis_sim");
    assert!(
        flute_res.status.success(),
        "rotary 4-axis sim build failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&flute_res.stdout),
        String::from_utf8_lossy(&flute_res.stderr)
    );
    let actual_flute = std::fs::read_to_string(flute_out.join("v3_rotary_4axis_sim.nc")).unwrap();
    assert!(
        actual_flute.contains("G93")
            && actual_flute.lines().any(|l| {
                let t = l.trim();
                !t.starts_with('(')
                    && !t.starts_with(';')
                    && t.contains('X')
                    && t.contains('Y')
                    && t.contains('Z')
                    && t.contains('A')
                    && t.contains('F')
            }),
        "grblhal_4axis golden must emit simultaneous G93 X Y Z A F blocks"
    );
    let golden_flute = golden_dir().join("grbl/v3_rotary_4axis_sim.nc");
    if update_goldens {
        std::fs::write(&golden_flute, &actual_flute).unwrap();
    }
    let expected_flute = read_golden_normalized(&golden_flute);
    assert_eq!(
        actual_flute, expected_flute,
        "v3_rotary_4axis_sim.nc drifted from golden snapshot"
    );
}

#[test]
fn cli_transform_model_and_import_mesh_transform_flags_work_end_to_end() {
    use chip_cad::{ModelTransform, sample_sculpted_dome_stl_bytes};

    let tmp = TempTestDir::new("transform_model_m35");
    let stl_path = tmp.path().join("dome.stl");
    std::fs::write(&stl_path, sample_sculpted_dome_stl_bytes()).unwrap();

    // 1. `import-mesh` with orientation, scale, and placement flags
    let proj_path = tmp.path().join("dome_oriented.chip.toml");
    let imp = Command::new(cli_bin())
        .args([
            "import-mesh",
            stl_path.to_str().unwrap(),
            "-o",
            proj_path.to_str().unwrap(),
            "--mesh-units",
            "mm",
            "--y-up-to-z-up",
            "--scale",
            "1.5",
            "--center-xy",
            "--drop-z0",
        ])
        .output()
        .expect("import-mesh with transform flags");
    assert!(
        imp.status.success(),
        "import-mesh failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&imp.stdout),
        String::from_utf8_lossy(&imp.stderr)
    );
    let proj = ProjectDocument::load_from_path(&proj_path).unwrap();
    let xf = proj.models[0].transform;
    assert!((xf.rotate_deg[0] - 90.0).abs() < 1e-4, "xf={xf:?}");
    assert!((xf.scale[0] - 1.5).abs() < 1e-6, "xf={xf:?}");

    // 2. `transform-model` --fit-size-x 120 --center-xy --drop-z0
    let tr = Command::new(cli_bin())
        .args([
            "transform-model",
            proj_path.to_str().unwrap(),
            "--fit-size-x",
            "120",
            "--center-xy",
            "--drop-z0",
        ])
        .output()
        .expect("transform-model fit-size-x");
    assert!(
        tr.status.success(),
        "transform-model failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&tr.stdout),
        String::from_utf8_lossy(&tr.stderr)
    );
    let tr_stdout = String::from_utf8_lossy(&tr.stdout);
    assert!(
        tr_stdout.contains("120.00"),
        "expected fitted X size 120.00 in stdout:\n{tr_stdout}"
    );

    // 3. `validate` succeeds on the transformed project
    let val = Command::new(cli_bin())
        .args(["validate", proj_path.to_str().unwrap()])
        .output()
        .expect("validate transformed project");
    assert!(
        val.status.success(),
        "validate failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&val.stdout),
        String::from_utf8_lossy(&val.stderr)
    );

    // 4. `transform-model --reset` returns transform to identity and omits it from TOML
    let rst = Command::new(cli_bin())
        .args(["transform-model", proj_path.to_str().unwrap(), "--reset"])
        .output()
        .expect("transform-model reset");
    assert!(rst.status.success());
    let proj_reset = ProjectDocument::load_from_path(&proj_path).unwrap();
    assert_eq!(proj_reset.models[0].transform, ModelTransform::IDENTITY);
    let raw_toml = std::fs::read_to_string(&proj_path).unwrap();
    assert!(
        !raw_toml.contains("transform"),
        "identity transform should be omitted from TOML:\n{raw_toml}"
    );
}

#[test]
fn cli_simulate_heightmap_and_tridexel_emit_error_volume_and_rms_reports() {
    let tmp = TempTestDir::new("sim_tridexel_metrics_m39");
    let step_path = tmp.path().join("multi_feature.step");
    let proj_path = tmp.path().join("multi_feature.chip.toml");

    let gen_out = Command::new(cli_bin())
        .args([
            "generate-sample-step",
            step_path.to_str().unwrap(),
            "--multi-feature",
            "--project-out",
            proj_path.to_str().unwrap(),
        ])
        .output()
        .expect("generate multi-feature project");
    assert!(
        gen_out.status.success(),
        "generate-sample-step failed:\n{}",
        String::from_utf8_lossy(&gen_out.stderr)
    );

    // 1. Heightmap simulation JSON report with quantitative error metrics
    let hm_report_path = tmp.path().join("sim_heightmap.json");
    let hm_out = Command::new(cli_bin())
        .args([
            "simulate",
            proj_path.to_str().unwrap(),
            "--engine",
            "heightmap",
            "--resolution",
            "standard",
            "--tolerance",
            "0.05",
            "--report",
            hm_report_path.to_str().unwrap(),
        ])
        .output()
        .expect("run simulate --engine heightmap");
    assert!(
        hm_out.status.success(),
        "heightmap simulate failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&hm_out.stdout),
        String::from_utf8_lossy(&hm_out.stderr)
    );
    let hm_stdout = String::from_utf8_lossy(&hm_out.stdout);
    assert!(hm_stdout.contains("Error Volume:"));
    assert!(hm_stdout.contains("RMS="));

    let hm_json: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&hm_report_path).unwrap()).unwrap();
    assert_eq!(hm_json["passed"], true);
    let hm_setup = &hm_json["setups"][0];
    assert_eq!(hm_setup["engine"], "heightmap");
    let hm_err = &hm_setup["error_report"];
    assert!(
        hm_err.is_object(),
        "heightmap error_report missing: {hm_setup}"
    );
    assert!(hm_err["total_error_volume_mm3"].as_f64().unwrap() >= 0.0);
    assert!(hm_err["rms_error_mm"].as_f64().unwrap() >= 0.0);
    assert!(
        hm_err["per_operation"].as_array().unwrap().len() >= 2,
        "expected per_operation progression"
    );

    // 2. Tri-Dexel 3D volumetric simulation JSON report with custom tolerance
    let td_report_path = tmp.path().join("sim_tridexel.json");
    let td_out = Command::new(cli_bin())
        .args([
            "simulate",
            proj_path.to_str().unwrap(),
            "--engine",
            "tri-dexel",
            "--resolution",
            "standard",
            "--tolerance",
            "0.10",
            "--report",
            td_report_path.to_str().unwrap(),
        ])
        .output()
        .expect("run simulate --engine tri-dexel");
    assert!(
        td_out.status.success(),
        "tri-dexel simulate failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&td_out.stdout),
        String::from_utf8_lossy(&td_out.stderr)
    );
    let td_stdout = String::from_utf8_lossy(&td_out.stdout);
    assert!(td_stdout.contains("Tri-Dexel"));
    assert!(td_stdout.contains("Error Volume:"));
    assert!(td_stdout.contains("RMS="));

    let td_json: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&td_report_path).unwrap()).unwrap();
    assert_eq!(td_json["passed"], true);
    let td_setup = &td_json["setups"][0];
    assert_eq!(td_setup["engine"], "tri_dexel");
    let td_err = &td_setup["error_report"];
    assert!(
        td_err.is_object(),
        "tri-dexel error_report missing: {td_setup}"
    );
    assert!((td_err["tolerance_mm"].as_f64().unwrap() - 0.10).abs() < 1e-6);
    assert!(td_err["target_volume_mm3"].as_f64().unwrap() > 1000.0);
    assert!(td_err["total_error_volume_mm3"].as_f64().unwrap() >= 0.0);
    assert!(td_err["rms_error_mm"].as_f64().unwrap() >= 0.0);
    assert!(td_err["in_tolerance_surface_pct"].as_f64().unwrap() >= 0.0);

    let per_op = td_err["per_operation"].as_array().unwrap();
    assert!(
        per_op.len() >= 2,
        "expected multi-op progression: {per_op:?}"
    );
    let first_err = per_op.first().unwrap()["total_error_volume_mm3"]
        .as_f64()
        .unwrap();
    let last_err = per_op.last().unwrap()["total_error_volume_mm3"]
        .as_f64()
        .unwrap();
    assert!(
        last_err < first_err,
        "subsequent operations should reduce total error volume: first={first_err:.2}, last={last_err:.2}"
    );
}

#[test]
fn v4_golden_thread_milling_snapshots_and_tridexel_sim_match_byte_for_byte() {
    let tmp = TempTestDir::new("v4_thread_mill_goldens");
    let update_goldens = std::env::var("CHIPCUTTER_UPDATE_GOLDENS").is_ok();
    let step_path = tmp.path().join("18-threaded-manifold.step");
    let proj_path = tmp.path().join("v4_thread_mill.chip.toml");

    // 1. Generate starter STEP + seeded Thread Milling project via `--tutorial 18-thread-milling`
    let gen_out = Command::new(cli_bin())
        .args([
            "generate-sample-step",
            step_path.to_str().unwrap(),
            "--tutorial",
            "18-thread-milling",
            "--project-out",
            proj_path.to_str().unwrap(),
        ])
        .output()
        .expect("run generate-sample-step --tutorial 18-thread-milling");
    assert!(
        gen_out.status.success(),
        "generate-sample-step failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&gen_out.stdout),
        String::from_utf8_lossy(&gen_out.stderr)
    );

    // 2. Validate & generate all 6 operations
    let val_out = Command::new(cli_bin())
        .args(["validate", "--generate", proj_path.to_str().unwrap()])
        .output()
        .expect("run validate --generate on thread-mill project");
    assert!(
        val_out.status.success(),
        "validate --generate failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&val_out.stdout),
        String::from_utf8_lossy(&val_out.stderr)
    );

    // 3. Simulate cleanly (0 crashes) in BOTH Heightmap AND Tri-Dexel engines
    for engine in ["heightmap", "tri-dexel"] {
        let rep_path = tmp.path().join(format!("sim_{engine}.json"));
        let sim_out = Command::new(cli_bin())
            .args([
                "simulate",
                proj_path.to_str().unwrap(),
                "--engine",
                engine,
                "--resolution",
                "standard",
                "--tolerance",
                "0.05",
                "--report",
                rep_path.to_str().unwrap(),
            ])
            .output()
            .expect("run simulate on thread-mill project");
        assert!(
            sim_out.status.success(),
            "simulate --engine {engine} failed:\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&sim_out.stdout),
            String::from_utf8_lossy(&sim_out.stderr)
        );
        let rep_json: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(&rep_path).unwrap()).unwrap();
        assert_eq!(
            rep_json["passed"], true,
            "thread-mill simulation ({engine}) must pass with 0 critical errors: {rep_json}"
        );
    }

    // 4. Build LinuxCNC ATC single-file golden (`linuxcnc/v4_thread_mill.ngc`)
    let lcnc_out = tmp.path().join("out_v4_lcnc");
    let lcnc_res = Command::new(cli_bin())
        .args([
            "build",
            proj_path.to_str().unwrap(),
            "-o",
            lcnc_out.to_str().unwrap(),
            "--post",
            "linuxcnc",
        ])
        .output()
        .expect("build v4_thread_mill linuxcnc");
    assert!(
        lcnc_res.status.success(),
        "v4 LinuxCNC build failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&lcnc_res.stdout),
        String::from_utf8_lossy(&lcnc_res.stderr)
    );
    let actual_lcnc = std::fs::read_to_string(lcnc_out.join("v4_thread_mill.ngc")).unwrap();
    assert!(
        actual_lcnc.contains("T9 M6\nG43 H9") && actual_lcnc.contains("T11 M6\nG43 H11"),
        "expected T9 and T11 thread mill tool changes in LinuxCNC output"
    );
    assert!(
        actual_lcnc
            .lines()
            .any(|l| l.starts_with("G3 ") && l.contains("Z") && l.contains("I")),
        "expected internal climb helical G3 arc with Z/I in LinuxCNC output"
    );
    assert!(
        actual_lcnc
            .lines()
            .any(|l| l.starts_with("G2 ") && l.contains("Z") && l.contains("I")),
        "expected external climb helical G2 arc with Z/I in LinuxCNC output"
    );
    let golden_lcnc = golden_dir().join("linuxcnc/v4_thread_mill.ngc");
    if update_goldens {
        std::fs::write(&golden_lcnc, &actual_lcnc).unwrap();
    }
    let expected_lcnc = read_golden_normalized(&golden_lcnc);
    assert_eq!(
        actual_lcnc, expected_lcnc,
        "v4_thread_mill.ngc drifted from golden snapshot"
    );

    // 5. Build GRBL split-by-tool goldens (`grbl/v4_thread_mill_*.nc`)
    let grbl_out = tmp.path().join("out_v4_grbl");
    let grbl_res = Command::new(cli_bin())
        .args([
            "build",
            proj_path.to_str().unwrap(),
            "-o",
            grbl_out.to_str().unwrap(),
            "--post",
            "grbl",
        ])
        .output()
        .expect("build v4_thread_mill grbl");
    assert!(
        grbl_res.status.success(),
        "v4 GRBL build failed:\nstdout: {}\nstderr: {}",
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
        5,
        "Expected 5 split GRBL files (T7 Face, T1 Deck, T3 Bore, T9 Internal Thread, T11 External Thread), got: {nc_files:?}"
    );
    for fname in nc_files {
        let actual = std::fs::read_to_string(grbl_out.join(&fname)).unwrap();
        for (idx, line) in actual.lines().enumerate() {
            assert!(
                line.len() <= 80,
                "GRBL file {fname} line {} exceeds 80 chars ({}): {line}",
                idx + 1,
                line.len()
            );
        }
        let golden = golden_dir().join("grbl").join(&fname);
        if update_goldens {
            std::fs::write(&golden, &actual).unwrap();
        }
        let expected = read_golden_normalized(&golden);
        assert_eq!(actual, expected, "{fname} drifted from golden snapshot");
    }
}

#[test]
fn cli_simulate_backend_cpu_vs_gpu_json_and_benchmark_parity() {
    let tmp = TempTestDir::new("sim_backend_parity_m55");
    let step_path = tmp.path().join("cam_torture.step");
    let proj_path = tmp.path().join("cam_torture.chip.toml");

    let gen_out = Command::new(cli_bin())
        .args([
            "generate-sample-step",
            step_path.to_str().unwrap(),
            "--cam-torture",
            "--project-out",
            proj_path.to_str().unwrap(),
        ])
        .output()
        .expect("generate cam-torture project");
    assert!(
        gen_out.status.success(),
        "generate-sample-step failed:\n{}",
        String::from_utf8_lossy(&gen_out.stderr)
    );

    // 1. CPU vs GPU JSON parity on Heightmap & Tri-Dexel
    for engine in ["heightmap", "tri-dexel"] {
        let cpu_out = Command::new(cli_bin())
            .args([
                "simulate",
                proj_path.to_str().unwrap(),
                "--engine",
                engine,
                "--backend",
                "cpu",
                "--resolution",
                "standard",
                "--json",
            ])
            .output()
            .expect("run simulate --backend cpu --json");
        assert!(
            cpu_out.status.success(),
            "simulate --engine {engine} --backend cpu --json failed:\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&cpu_out.stdout),
            String::from_utf8_lossy(&cpu_out.stderr)
        );
        let gpu_out = Command::new(cli_bin())
            .args([
                "simulate",
                proj_path.to_str().unwrap(),
                "--engine",
                engine,
                "--backend",
                "gpu",
                "--resolution",
                "standard",
                "--json",
            ])
            .output()
            .expect("run simulate --backend gpu --json");
        assert!(
            gpu_out.status.success(),
            "simulate --engine {engine} --backend gpu --json failed:\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&gpu_out.stdout),
            String::from_utf8_lossy(&gpu_out.stderr)
        );

        let cpu_json: serde_json::Value = serde_json::from_slice(&cpu_out.stdout)
            .expect("parse stdout JSON for --backend cpu --json");
        let gpu_json: serde_json::Value = serde_json::from_slice(&gpu_out.stdout)
            .expect("parse stdout JSON for --backend gpu --json");
        assert_eq!(cpu_json["passed"], gpu_json["passed"]);

        let cpu_setup = &cpu_json["setups"][0];
        let gpu_setup = &gpu_json["setups"][0];
        assert_eq!(cpu_setup["backend"]["is_gpu"], false);
        assert_eq!(cpu_setup["backend"]["backend_api"], "CPU");
        assert!(gpu_setup["backend"]["adapter_name"].is_string());
        assert!(gpu_setup["backend"]["backend_api"].is_string());

        // Timing breakdown metadata must be present and positive
        for (tag, s) in [("cpu", cpu_setup), ("gpu", gpu_setup)] {
            let timing = &s["timing"];
            assert!(
                timing["total_ms"].as_f64().unwrap() > 0.0,
                "{engine}/{tag} total_ms must be > 0: {timing}"
            );
            assert!(
                timing["total_segments"].as_u64().unwrap() > 0,
                "{engine}/{tag} total_segments must be > 0: {timing}"
            );
            assert!(
                timing["segments_per_sec"].as_f64().unwrap() > 0.0,
                "{engine}/{tag} segments_per_sec must be > 0: {timing}"
            );
        }

        // Identical Rule 1-5 crash/gouge diagnostic counts
        assert_eq!(
            cpu_setup["critical_error_count"], gpu_setup["critical_error_count"],
            "{engine}: CPU vs GPU critical_error_count mismatch"
        );
        assert_eq!(
            cpu_setup["warning_count"], gpu_setup["warning_count"],
            "{engine}: CPU vs GPU warning_count mismatch"
        );

        // Matching StockErrorReport metrics within f32-vs-f64 tolerance
        let cpu_err = &cpu_setup["error_report"];
        let gpu_err = &gpu_setup["error_report"];
        assert!(cpu_err.is_object() && gpu_err.is_object());
        let cpu_rms = cpu_err["rms_error_mm"].as_f64().unwrap();
        let gpu_rms = gpu_err["rms_error_mm"].as_f64().unwrap();
        assert!(
            (cpu_rms - gpu_rms).abs() < 0.05,
            "{engine}: CPU vs GPU RMS mismatch: cpu={cpu_rms:.4}, gpu={gpu_rms:.4}"
        );
        let cpu_vol = cpu_err["total_error_volume_mm3"].as_f64().unwrap();
        let gpu_vol = gpu_err["total_error_volume_mm3"].as_f64().unwrap();
        let rel_vol_diff = (cpu_vol - gpu_vol).abs() / cpu_vol.max(100.0);
        assert!(
            rel_vol_diff < 0.03,
            "{engine}: CPU vs GPU total_error_volume_mm3 mismatch: cpu={cpu_vol:.2}, gpu={gpu_vol:.2}"
        );
        let cpu_intol = cpu_err["in_tolerance_surface_pct"].as_f64().unwrap();
        let gpu_intol = gpu_err["in_tolerance_surface_pct"].as_f64().unwrap();
        assert!(
            (cpu_intol - gpu_intol).abs() < 2.0,
            "{engine}: CPU vs GPU in_tolerance_surface_pct mismatch: cpu={cpu_intol:.2}, gpu={gpu_intol:.2}"
        );
    }

    // 2. Verify `--benchmark` human-readable output format
    let bench_out = Command::new(cli_bin())
        .args([
            "simulate",
            proj_path.to_str().unwrap(),
            "--backend",
            "auto",
            "--benchmark",
        ])
        .output()
        .expect("run simulate --benchmark");
    assert!(
        bench_out.status.success(),
        "simulate --benchmark failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&bench_out.stdout),
        String::from_utf8_lossy(&bench_out.stderr)
    );
    let bench_stdout = String::from_utf8_lossy(&bench_out.stdout);
    assert!(bench_stdout.contains("Benchmark: backend="));
    assert!(bench_stdout.contains("Timing: total="));
    assert!(bench_stdout.contains("target_setup="));
    assert!(bench_stdout.contains("sweep="));
    assert!(bench_stdout.contains("metrics="));
    assert!(bench_stdout.contains("throughput="));
    assert!(bench_stdout.contains("Error Volume:"));
}

#[test]
fn cli_workholding_add_vise_clamp_spoilboard_and_list_roundtrip() {
    let tmp = TempTestDir::new("workholding_cli_m60");
    let step_path = tmp.path().join("bracket.step");
    let proj_path = tmp.path().join("bracket.chip.toml");

    // 1. Seed a sample bracket project
    let gen_cmd = Command::new(cli_bin())
        .args([
            "generate-sample-step",
            step_path.to_str().unwrap(),
            "--multi-feature",
            "--project-out",
            proj_path.to_str().unwrap(),
        ])
        .output()
        .expect("generate sample bracket");
    assert!(gen_cmd.status.success());

    // 2. Add a vise with parallels, a toe clamp, and a spoilboard
    let vise = Command::new(cli_bin())
        .args([
            "workholding",
            "add-vise",
            proj_path.to_str().unwrap(),
            "--name",
            "Kurt D688 Vise",
            "--axis",
            "y",
            "--jaw-width",
            "120",
            "--grip-depth",
            "5",
            "--parallels",
        ])
        .output()
        .expect("workholding add-vise");
    assert!(
        vise.status.success(),
        "add-vise failed:\n{}",
        String::from_utf8_lossy(&vise.stderr)
    );

    let clamp = Command::new(cli_bin())
        .args([
            "workholding",
            "add-clamp",
            proj_path.to_str().unwrap(),
            "--name",
            "Right Edge Clamp",
            "--x",
            "47",
            "--y",
            "0",
            "--yaw",
            "0",
        ])
        .output()
        .expect("workholding add-clamp");
    assert!(
        clamp.status.success(),
        "add-clamp failed:\n{}",
        String::from_utf8_lossy(&clamp.stderr)
    );

    let sb = Command::new(cli_bin())
        .args([
            "workholding",
            "add-spoilboard",
            proj_path.to_str().unwrap(),
            "--thickness",
            "18",
            "--margin-xy",
            "30",
            "--allowed-overcut",
            "0.5",
        ])
        .output()
        .expect("workholding add-spoilboard");
    assert!(
        sb.status.success(),
        "add-spoilboard failed:\n{}",
        String::from_utf8_lossy(&sb.stderr)
    );

    // 3. `workholding list` prints all 3 resolved fixtures with bounding boxes and triangle counts
    let list = Command::new(cli_bin())
        .args(["workholding", "list", proj_path.to_str().unwrap()])
        .output()
        .expect("workholding list");
    assert!(
        list.status.success(),
        "workholding list failed:\n{}",
        String::from_utf8_lossy(&list.stderr)
    );
    let list_out = String::from_utf8_lossy(&list.stdout);
    assert!(list_out.contains("3 fixture(s)"), "stdout:\n{list_out}");
    assert!(list_out.contains("Kurt D688 Vise"), "stdout:\n{list_out}");
    assert!(list_out.contains("Right Edge Clamp"), "stdout:\n{list_out}");
    assert!(list_out.contains("MDF Spoilboard"), "stdout:\n{list_out}");
    assert!(list_out.contains("WCS AABB"), "stdout:\n{list_out}");

    // 4. `validate` passes cleanly on the project with workholding configured
    let val = Command::new(cli_bin())
        .args(["validate", proj_path.to_str().unwrap()])
        .output()
        .expect("validate with workholding");
    assert!(
        val.status.success(),
        "validate failed:\nstdout:\n{}\nstderr:\n{}",
        String::from_utf8_lossy(&val.stdout),
        String::from_utf8_lossy(&val.stderr)
    );
}

#[test]
fn cli_m61_simulate_json_reports_fixture_collision_and_build_blocks_export() {
    use chip_cam::FixtureBody;
    use chip_math::DVec2;
    use chip_project::build_pure2d_router_panel_demo_project;

    let tmp = TempTestDir::new("m61_cli_fixture_gate");
    let tools = chip_tools::default_starter_tool_library().tools;
    let mut proj = build_pure2d_router_panel_demo_project(None, &tools);
    // Place a ToeClamp directly across the center of the router panel cutting paths!
    proj.setups[0].fixtures.push(FixtureBody::default_toe_clamp(
        "blocking_clamp",
        "Blocking Center Clamp",
        DVec2::new(40.0, 30.0),
        0.0,
    ));
    let proj_path = tmp.path().join("blocked_by_clamp.chip.toml");
    proj.save_to_path(&proj_path).unwrap();

    // 1. `chipcutter simulate --json` fails with exit code != 0 and emits `fixture_collision` + `fixture_id`!
    let sim = Command::new(cli_bin())
        .args(["simulate", proj_path.to_str().unwrap(), "--json"])
        .output()
        .expect("simulate blocked project");
    assert!(
        !sim.status.success(),
        "simulate must fail when cutting passes collide with a fixture"
    );
    let sim_stdout = String::from_utf8_lossy(&sim.stdout);
    let report: serde_json::Value = serde_json::from_str(&sim_stdout)
        .unwrap_or_else(|e| panic!("invalid JSON from simulate: {e}\n{sim_stdout}"));
    assert_eq!(report["passed"], serde_json::Value::Bool(false));
    let diags = report["setups"][0]["diagnostics"]
        .as_array()
        .expect("diagnostics array");
    assert!(
        diags.iter().any(|d| {
            d["kind"] == "fixture_collision"
                && d["fixture_id"] == "blocking_clamp"
                && d["fixture_name"] == "Blocking Center Clamp"
        }),
        "expected fixture_collision diagnostic in JSON:\n{sim_stdout}"
    );

    // 2. `chipcutter build` blocks G-code export and produces NO .nc files!
    let out_dir = tmp.path().join("nc_blocked");
    let build = Command::new(cli_bin())
        .args([
            "build",
            proj_path.to_str().unwrap(),
            "-o",
            out_dir.to_str().unwrap(),
        ])
        .output()
        .expect("build blocked project");
    assert!(
        !build.status.success(),
        "build must abort on FixtureCollision critical error"
    );
    let build_stderr = String::from_utf8_lossy(&build.stderr);
    assert!(
        build_stderr.contains("Fixture / Clamp Collision")
            && build_stderr.contains("Blocking Center Clamp"),
        "expected fixture collision message on stderr:\n{build_stderr}"
    );
    let nc_count = std::fs::read_dir(&out_dir)
        .map(|rd| {
            rd.filter_map(|e| e.ok())
                .filter(|e| e.path().extension().and_then(|s| s.to_str()) == Some("nc"))
                .count()
        })
        .unwrap_or(0);
    assert_eq!(
        nc_count, 0,
        "blocked build must NOT emit any .nc G-code files"
    );
}

#[test]
fn cli_simulate_all_setups_carryover_emits_cumulative_json_and_text_reports() {
    use chip_cam::{
        BoxAnchor27, HeightReference, HeightSpec, MachiningBoundaryMode, Operation, OperationKind,
        Setup, StockConfig, StrategyParams, WcsOrientationMode, WcsOriginMode, WorkOffsetSlot,
    };
    use chip_math::ParamLength;
    use chip_project::ModelEntry;

    // Build a watertight two-sided stepped block STL (`60 x 50 x 16 mm`, boss `32 x 26 x 6 mm`)
    let mut tris: Vec<([f32; 3], [f32; 3], [f32; 3])> = Vec::new();
    let mut push_quad = |a: [f32; 3], b: [f32; 3], c: [f32; 3], d: [f32; 3]| {
        tris.push((a, b, c));
        tris.push((a, c, d));
    };
    let (x0, x1, y0, y1, z0, z1) = (0.0_f32, 60.0_f32, 0.0_f32, 50.0_f32, 0.0_f32, 10.0_f32);
    let (bx0, bx1, by0, by1, bz1) = (14.0_f32, 46.0_f32, 12.0_f32, 38.0_f32, 16.0_f32);
    push_quad([x0, y0, z0], [x0, y1, z0], [x1, y1, z0], [x1, y0, z0]);
    push_quad([x0, y0, z0], [x1, y0, z0], [x1, y0, z1], [x0, y0, z1]);
    push_quad([x1, y1, z0], [x0, y1, z0], [x0, y1, z1], [x1, y1, z1]);
    push_quad([x0, y1, z0], [x0, y0, z0], [x0, y0, z1], [x0, y1, z1]);
    push_quad([x1, y0, z0], [x1, y1, z0], [x1, y1, z1], [x1, y0, z1]);
    push_quad([x0, y0, z1], [x1, y0, z1], [x1, by0, z1], [x0, by0, z1]);
    push_quad([x0, by1, z1], [x1, by1, z1], [x1, y1, z1], [x0, y1, z1]);
    push_quad([x0, by0, z1], [bx0, by0, z1], [bx0, by1, z1], [x0, by1, z1]);
    push_quad([bx1, by0, z1], [x1, by0, z1], [x1, by1, z1], [bx1, by1, z1]);
    push_quad(
        [bx0, by0, z1],
        [bx1, by0, z1],
        [bx1, by0, bz1],
        [bx0, by0, bz1],
    );
    push_quad(
        [bx1, by1, z1],
        [bx0, by1, z1],
        [bx0, by1, bz1],
        [bx1, by1, bz1],
    );
    push_quad(
        [bx0, by1, z1],
        [bx0, by0, z1],
        [bx0, by0, bz1],
        [bx0, by1, bz1],
    );
    push_quad(
        [bx1, by0, z1],
        [bx1, by1, z1],
        [bx1, by1, bz1],
        [bx1, by0, bz1],
    );
    push_quad(
        [bx0, by0, bz1],
        [bx1, by0, bz1],
        [bx1, by1, bz1],
        [bx0, by1, bz1],
    );

    let mut stl_bytes = Vec::with_capacity(84 + tris.len() * 50);
    stl_bytes.extend_from_slice(&[0u8; 80]);
    stl_bytes.extend_from_slice(&(tris.len() as u32).to_le_bytes());
    for (v0, v1, v2) in tris {
        let e1 = [v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]];
        let e2 = [v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]];
        let nx = e1[1] * e2[2] - e1[2] * e2[1];
        let ny = e1[2] * e2[0] - e1[0] * e2[2];
        let nz = e1[0] * e2[1] - e1[1] * e2[0];
        let len = (nx * nx + ny * ny + nz * nz).sqrt().max(1e-12);
        for c in [nx / len, ny / len, nz / len] {
            stl_bytes.extend_from_slice(&c.to_le_bytes());
        }
        for v in [v0, v1, v2] {
            for c in v {
                stl_bytes.extend_from_slice(&c.to_le_bytes());
            }
        }
        stl_bytes.extend_from_slice(&0u16.to_le_bytes());
    }

    let tmp = TempTestDir::new("m62_cli_all_setups");
    let stl_path = tmp.path().join("stepped_block.stl");
    std::fs::write(&stl_path, &stl_bytes).unwrap();
    let sha = compute_sha256_hex(&stl_bytes);

    let starter = default_starter_tool_library();
    let flat_6 = starter.tools[0].clone();

    let mut proj = ProjectDocument::new("M62 Two-Sided CLI Test");
    proj.models
        .push(ModelEntry::new("m_part", "stepped_block.stl", sha));
    proj.tools.push(flat_6.clone());

    let mut s1 = Setup::new("setup_1", "Setup 1 - Top Op", WorkOffsetSlot::G54);
    s1.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    s1.stock = StockConfig::RelativeBox {
        side_pos_x: ParamLength::from_mm(3.0),
        side_neg_x: ParamLength::from_mm(3.0),
        side_pos_y: ParamLength::from_mm(3.0),
        side_neg_y: ParamLength::from_mm(3.0),
        symmetric_xy: true,
        top_offset_z: ParamLength::from_mm(1.0),
        bottom_offset_z: ParamLength::from_mm(5.0),
        sheet_thickness_z: ParamLength::from_mm(12.0),
        round_increment_mm: None,
    };
    let mut op1_face = Operation::new("op1_face", "Op 1.1 Face Top", OperationKind::Facing);
    op1_face.tool_id = Some(flat_6.id.clone());
    op1_face.preset_id = flat_6.presets.first().map(|p| p.id.clone());
    op1_face.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op1_face.heights.bottom = HeightSpec::new(HeightReference::ModelTop, 0.0);
    op1_face.passes.multiple_depths = false;
    op1_face.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
    s1.operations.push(op1_face);

    let mut op1 = Operation::new("op1", "Op 1.2 Top Adaptive", OperationKind::Adaptive3D);
    op1.tool_id = Some(flat_6.id.clone());
    op1.preset_id = flat_6.presets.first().map(|p| p.id.clone());
    op1.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
    op1.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 2.0);
    op1.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
    op1.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
    if let StrategyParams::Adaptive3D(p) = op1.strategy_mut() {
        p.boundary.boundary_mode = MachiningBoundaryMode::StockBox;
        p.max_roughing_stepdown = ParamLength::from_mm(5.0);
        p.fine_stepdown = ParamLength::from_mm(2.0);
        p.adaptive.optimal_load = ParamLength::from_mm(2.2);
    }
    s1.operations.push(op1);

    let mut s2 = Setup::new("setup_2", "Setup 2 - Flip Bottom Op", WorkOffsetSlot::G55);
    s2.wcs.orientation = WcsOrientationMode::ModelOrientation {
        flip_z: true,
        flip_x: false,
        clock_90: 0,
        extra_z_rot_deg: 0.0,
    };
    s2.wcs.origin = WcsOriginMode::StockBoxPoint {
        anchor: BoxAnchor27::TOP_CENTER,
    };
    s2.stock = StockConfig::RelativeBox {
        side_pos_x: ParamLength::from_mm(3.0),
        side_neg_x: ParamLength::from_mm(3.0),
        side_pos_y: ParamLength::from_mm(3.0),
        side_neg_y: ParamLength::from_mm(3.0),
        symmetric_xy: true,
        top_offset_z: ParamLength::from_mm(5.0),
        bottom_offset_z: ParamLength::from_mm(1.0),
        sheet_thickness_z: ParamLength::from_mm(12.0),
        round_increment_mm: None,
    };
    s2.carryover_from_setup_id = Some("setup_1".to_string());

    let mut op2_face = Operation::new("op2_face", "Op 2.1 Face Hat", OperationKind::Facing);
    op2_face.tool_id = Some(flat_6.id.clone());
    op2_face.preset_id = flat_6.presets.first().map(|p| p.id.clone());
    op2_face.heights.top = HeightSpec::new(HeightReference::StockTop, 0.0);
    op2_face.heights.bottom = HeightSpec::new(HeightReference::ModelTop, 0.0);
    op2_face.passes.multiple_depths = true;
    op2_face.passes.max_stepdown = ParamLength::from_mm(2.5);
    op2_face.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
    s2.operations.push(op2_face);

    let mut op2 = Operation::new("op2", "Op 2.2 Bottom Rest", OperationKind::Adaptive3D);
    op2.tool_id = Some(flat_6.id.clone());
    op2.preset_id = flat_6.presets.first().map(|p| p.id.clone());
    op2.heights.top = HeightSpec::new(HeightReference::ModelTop, 0.0);
    op2.heights.bottom = HeightSpec::new(HeightReference::ModelBottom, 0.0);
    op2.passes.stock_to_leave_radial = ParamLength::from_mm(0.0);
    op2.passes.stock_to_leave_axial = ParamLength::from_mm(0.0);
    if let StrategyParams::Adaptive3D(p) = op2.strategy_mut() {
        p.boundary.boundary_mode = MachiningBoundaryMode::StockBox;
        p.max_roughing_stepdown = ParamLength::from_mm(4.0);
        p.fine_stepdown = ParamLength::from_mm(2.0);
        p.adaptive.optimal_load = ParamLength::from_mm(2.2);
    }
    s2.operations.push(op2);

    proj.setups = vec![s1, s2];
    let proj_path = tmp.path().join("two_sided.chip.toml");
    proj.save_to_path(&proj_path).unwrap();

    // 1. JSON mode with `--all-setups`
    let json_out = Command::new(cli_bin())
        .args([
            "simulate",
            proj_path.to_str().unwrap(),
            "--engine",
            "tri-dexel",
            "--all-setups",
            "--json",
        ])
        .output()
        .expect("simulate --all-setups --json");
    assert!(
        json_out.status.success(),
        "simulate --all-setups --json failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&json_out.stdout),
        String::from_utf8_lossy(&json_out.stderr)
    );
    let json_str = String::from_utf8_lossy(&json_out.stdout);
    let report: serde_json::Value =
        serde_json::from_str(&json_str).unwrap_or_else(|e| panic!("invalid JSON: {e}\n{json_str}"));
    assert_eq!(report["passed"], serde_json::Value::Bool(true));
    assert_eq!(report["setups"].as_array().unwrap().len(), 2);
    assert_eq!(
        report["setups"][1]["carried_over_from_setup"].as_str(),
        Some("Setup 1 - Top Op")
    );
    let cum = &report["cumulative_error_report"];
    assert!(
        cum.is_object(),
        "expected cumulative_error_report object in JSON:\n{json_str}"
    );
    let in_tol = cum["in_tolerance_surface_pct"].as_f64().unwrap();
    let max_gouge = cum["max_gouge_mm"].as_f64().unwrap();
    assert!(
        in_tol >= 95.0 && max_gouge <= 0.02,
        "expected cumulative In-Tol >= 95% and max_gouge <= 0.02 mm, got In-Tol={in_tol:.2}%, max_gouge={max_gouge:.4} mm"
    );

    // 2. Human-readable text mode with `--all-setups`
    let txt_out = Command::new(cli_bin())
        .args([
            "simulate",
            proj_path.to_str().unwrap(),
            "--engine",
            "tri-dexel",
            "--all-setups",
        ])
        .output()
        .expect("simulate --all-setups");
    assert!(txt_out.status.success());
    let txt_str = String::from_utf8_lossy(&txt_out.stdout);
    assert!(
        txt_str.contains("carried over from 'Setup 1 - Top Op'")
            && txt_str.contains("Cumulative Multi-Setup Metrology"),
        "expected carryover & cumulative summary in text output:\n{txt_str}"
    );
}

#[test]
fn cli_workholding_flip_jig_and_soft_jaws_wizards_end_to_end() {
    use chip_cad::{MeshImportOptions, load_mesh_bytes};
    use chip_project::{
        build_two_sided_vise_workholding_project, compute_sha256_hex,
        sample_two_sided_vise_part_stl_bytes,
    };
    use chip_tools::default_starter_tool_library;

    let tmp = TempTestDir::new("workholding_wizards_m63");
    let stl_bytes = sample_two_sided_vise_part_stl_bytes();
    let stl_path = tmp.path().join("bracket.stl");
    std::fs::write(&stl_path, &stl_bytes).unwrap();
    let sha = compute_sha256_hex(&stl_bytes);

    // 1. Seed a single-setup project for CLI wizard testing
    let starter = default_starter_tool_library();
    let mut base_proj =
        build_two_sided_vise_workholding_project("bracket.stl".to_string(), &sha, &starter.tools);
    base_proj.setups.truncate(1);
    base_proj.setups[0].fixtures.clear();
    let base_path = tmp.path().join("base.chip.toml");
    base_proj.save_to_path(&base_path).unwrap();

    // 2. `chipcutter workholding flip-jig` -> creates Flip-Jig sketch, Op-1 pin op, Setup 2 Flip, Spoilboard setup
    let flip_out_path = tmp.path().join("with_flip_jig.chip.toml");
    let flip_out = Command::new(cli_bin())
        .args([
            "workholding",
            "flip-jig",
            base_path.to_str().unwrap(),
            "--axis",
            "x",
            "--pin-dia",
            "6",
            "--count",
            "2",
            "--poka-yoke",
            "4",
            "-o",
            flip_out_path.to_str().unwrap(),
        ])
        .output()
        .expect("workholding flip-jig");
    assert!(
        flip_out.status.success(),
        "workholding flip-jig failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&flip_out.stdout),
        String::from_utf8_lossy(&flip_out.stderr)
    );
    let flip_proj = ProjectDocument::load_from_path(&flip_out_path).unwrap();
    assert_eq!(flip_proj.setups.len(), 3);
    assert_eq!(
        flip_proj.setups[1].carryover_from_setup_id.as_deref(),
        Some("setup_1")
    );
    assert_eq!(flip_proj.setups[1].fixtures.len(), 2);

    // 3. `chipcutter workholding soft-jaws --add-setup --export-stl-dir` -> CNC setup + watertight STLs
    let sj_proj_path = tmp.path().join("with_soft_jaws.chip.toml");
    let sj_stl_dir = tmp.path().join("soft_jaw_stls");
    let sj_out = Command::new(cli_bin())
        .args([
            "workholding",
            "soft-jaws",
            base_path.to_str().unwrap(),
            "--grip-depth",
            "4.0",
            "--clearance",
            "0.10",
            "--jaw-width",
            "100",
            "--jaw-depth",
            "35",
            "--jaw-height",
            "38",
            "--bolt-pattern",
            "milo4",
            "--add-setup",
            "-o",
            sj_proj_path.to_str().unwrap(),
            "--export-stl-dir",
            sj_stl_dir.to_str().unwrap(),
        ])
        .output()
        .expect("workholding soft-jaws");
    assert!(
        sj_out.status.success(),
        "workholding soft-jaws failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&sj_out.stdout),
        String::from_utf8_lossy(&sj_out.stderr)
    );
    let sj_proj = ProjectDocument::load_from_path(&sj_proj_path).unwrap();
    assert_eq!(sj_proj.setups.len(), 2);
    assert_eq!(sj_proj.setups[1].operations.len(), 3);

    for fname in ["fixed_jaw.stl", "movable_jaw.stl"] {
        let p = sj_stl_dir.join(fname);
        let bytes = std::fs::read(&p).expect("exported soft-jaw STL must exist");
        let mesh = load_mesh_bytes(fname, &bytes, &MeshImportOptions::default()).unwrap();
        assert!(
            mesh.diagnostics.is_watertight,
            "{fname} not watertight: {:?}",
            mesh.diagnostics
        );
        assert_eq!(mesh.diagnostics.open_boundary_edges, 0);
        assert_eq!(mesh.diagnostics.non_manifold_edges, 0);
        assert!(mesh.diagnostics.signed_volume_mm3 > 0.0);
    }

    // 4. Built-in `two-sided-vise` project passes `chipcutter simulate --all-setups --json`!
    let two_sided_proj =
        build_two_sided_vise_workholding_project("bracket.stl".to_string(), &sha, &starter.tools);
    let two_sided_path = tmp.path().join("two_sided_vise.chip.toml");
    two_sided_proj.save_to_path(&two_sided_path).unwrap();
    let sim_out = Command::new(cli_bin())
        .args([
            "simulate",
            two_sided_path.to_str().unwrap(),
            "--engine",
            "tri-dexel",
            "--all-setups",
            "--json",
        ])
        .output()
        .expect("simulate two-sided-vise --all-setups --json");
    assert!(
        sim_out.status.success(),
        "simulate two-sided-vise failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&sim_out.stdout),
        String::from_utf8_lossy(&sim_out.stderr)
    );
    let sim_json: serde_json::Value =
        serde_json::from_slice(&sim_out.stdout).expect("valid simulation JSON");
    assert_eq!(sim_json["passed"], serde_json::Value::Bool(true));
    assert!(
        sim_json["cumulative_error_report"]["in_tolerance_surface_pct"]
            .as_f64()
            .unwrap()
            >= 95.0
    );
}

#[test]
fn cli_undercut_block_e2e_generate_validate_simulate_build_and_chipz_roundtrip() {
    let tmp = TempTestDir::new("undercut_block_e2e");
    let step_path = tmp.path().join("undercut_block.step");
    let proj_path = tmp.path().join("undercut_block.chip.toml");

    // 1. Generate programmatic Undercut & 3D Deburr STEP + seeded 6-operation `.chip.toml`
    let gen_out = Command::new(cli_bin())
        .args([
            "generate-sample-step",
            step_path.to_str().unwrap(),
            "--undercut-block",
            "--project-out",
            proj_path.to_str().unwrap(),
        ])
        .output()
        .expect("run generate-sample-step --undercut-block");
    assert!(
        gen_out.status.success(),
        "generate-sample-step --undercut-block failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&gen_out.stdout),
        String::from_utf8_lossy(&gen_out.stderr)
    );
    assert!(step_path.is_file());
    assert!(proj_path.is_file());

    // 2. Validate + generate all 6 operations cleanly
    let val_out = Command::new(cli_bin())
        .args(["validate", "--generate", proj_path.to_str().unwrap()])
        .output()
        .expect("run validate --generate");
    assert!(
        val_out.status.success(),
        "validate --generate failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&val_out.stdout),
        String::from_utf8_lossy(&val_out.stderr)
    );

    // 3. Simulate with `--engine tri-dexel --backend auto --json` and verify 0 critical errors + In-Tol >= 90%
    let sim_out = Command::new(cli_bin())
        .args([
            "simulate",
            proj_path.to_str().unwrap(),
            "--engine",
            "tri-dexel",
            "--backend",
            "auto",
            "--json",
        ])
        .output()
        .expect("run simulate --engine tri-dexel --backend auto --json");
    assert!(
        sim_out.status.success(),
        "simulate tri-dexel failed:\nstdout: {}\nstderr: {}",
        String::from_utf8_lossy(&sim_out.stdout),
        String::from_utf8_lossy(&sim_out.stderr)
    );
    let sim_json: serde_json::Value =
        serde_json::from_slice(&sim_out.stdout).expect("parse simulate JSON");
    assert_eq!(sim_json["passed"], serde_json::Value::Bool(true));
    assert_eq!(
        sim_json["setups"][0]["critical_error_count"]
            .as_u64()
            .unwrap(),
        0
    );
    let in_tol = sim_json["setups"][0]["error_report"]["in_tolerance_surface_pct"]
        .as_f64()
        .unwrap();
    assert!(
        in_tol >= 90.0,
        "CLI undercut-block Tri-Dexel In-Tol ({in_tol:.2}%) < 90%"
    );

    // 4. Build LinuxCNC and GRBL G-code (automatically verified via volumetric Tri-Dexel!)
    let lcnc_dir = tmp.path().join("out_linuxcnc");
    let build_lcnc = Command::new(cli_bin())
        .args([
            "build",
            proj_path.to_str().unwrap(),
            "-o",
            lcnc_dir.to_str().unwrap(),
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
    let lcnc_nc = std::fs::read_to_string(lcnc_dir.join("undercut_block.ngc"))
        .expect("read undercut_block.ngc");
    assert!(lcnc_nc.contains("G54") && lcnc_nc.contains("M6"));
    assert!(!lcnc_nc.contains("NaN") && !lcnc_nc.contains("inf"));

    let grbl_dir = tmp.path().join("out_grbl");
    let build_grbl = Command::new(cli_bin())
        .args([
            "build",
            proj_path.to_str().unwrap(),
            "-o",
            grbl_dir.to_str().unwrap(),
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
    assert!(grbl_dir.join("build_summary.json").is_file());

    // 5. Pack portable `.chipz` into isolated directory & verify validate + build match byte-for-byte!
    let isolated_dir = tmp.path().join("isolated_chipz");
    std::fs::create_dir_all(&isolated_dir).unwrap();
    let chipz_path = isolated_dir.join("undercut_block.chipz");
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
        "bundle pack failed:\nstderr: {}",
        String::from_utf8_lossy(&pack_out.stderr)
    );

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
        "build on .chipz failed:\nstderr: {}",
        String::from_utf8_lossy(&build_chipz.stderr)
    );
    let chipz_nc = std::fs::read_to_string(chipz_build_dir.join("undercut_block.ngc"))
        .expect("read undercut_block.ngc from .chipz");
    assert_eq!(
        chipz_nc, lcnc_nc,
        "G-code built from isolated .chipz must match direct .chip.toml build byte-for-byte"
    );
}

#[test]
fn cli_m68_tutorial_starter_generators_match_checked_in_assets_byte_for_byte() {
    let tmp = TempTestDir::new("m68_tutorial_assets");
    let assets_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
        .join("docs")
        .join("tutorials")
        .join("assets");

    let cases = [
        ("20-feed-opt", "20-feed-opt-pocket.step", true),
        ("21-deburr-boss", "21-sloped-boss-deburr.step", true),
        ("21-deburr-flange", "21-overhang-flange.step", true),
        ("22-undercut-block", "22-undercut-deburr-block.step", true),
        ("13-pure2d-svg", "13-badge-logo.svg", true),
        ("08-bracket-3mf", "08-bracket.3mf", true),
    ];

    for (slug, filename, test_project_out) in cases {
        let out_file = tmp.path().join(filename);
        let proj_file = tmp.path().join(format!("{slug}.chip.toml"));
        let mut cmd = Command::new(cli_bin());
        cmd.args([
            "generate-sample-step",
            "--tutorial",
            slug,
            out_file.to_str().unwrap(),
        ]);
        if test_project_out {
            cmd.args(["--project-out", proj_file.to_str().unwrap()]);
        }
        let res = cmd.output().expect("run generate-sample-step --tutorial");
        assert!(
            res.status.success(),
            "generate-sample-step --tutorial {slug} failed:\nstdout: {}\nstderr: {}",
            String::from_utf8_lossy(&res.stdout),
            String::from_utf8_lossy(&res.stderr)
        );
        let generated = std::fs::read(&out_file).expect("read generated starter asset");
        let checked_in = std::fs::read(assets_dir.join(filename))
            .unwrap_or_else(|e| panic!("read checked-in {}: {e}", filename));
        assert_eq!(
            generated, checked_in,
            "CLI --tutorial {slug} output differs from docs/tutorials/assets/{filename}"
        );
        if test_project_out {
            assert!(
                proj_file.is_file(),
                "expected --project-out to create {}",
                proj_file.display()
            );
        }
    }
}
