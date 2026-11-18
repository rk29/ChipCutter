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
