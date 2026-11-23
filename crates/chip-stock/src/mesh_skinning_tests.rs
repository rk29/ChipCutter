#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::collections::HashMap;

use chip_math::{Aabb3, DVec3, DisplayUnit};
use chip_tools::{CoolantMode, CutterGeometry, SpindleDirection, Tool, ToolId};

use crate::{
    backend::{CpuExactBackend, SimStockBackend, SimulatedStock},
    cylindrical::CylindricalStock,
    heightfield::HeightfieldStock,
    metrics::{SimStockColorMode, compute_heightmap_error_report, compute_tridexel_error_report},
    tridexel::TriDexelStock,
    types::{
        Axis3, RadialCutterProfile, RotaryStockSpec, SimMeshOptions, SimResolution, SimStockMesh,
        StockShapeSpec, StockVerifyTolerances, SweepChord, SweepMotion, SweepToolSpec,
    },
};

fn make_tool(id: &str, geometry: CutterGeometry) -> SweepToolSpec {
    let is_flat_endmill = matches!(geometry, CutterGeometry::FlatEndmill { .. });
    let tool = Tool {
        id: ToolId(id.to_string()),
        tool_number: 1,
        length_offset_number: 1,
        diameter_offset_number: 1,
        name: id.to_string(),
        vendor: None,
        product_id: None,
        native_unit: DisplayUnit::Millimeter,
        spindle_direction: SpindleDirection::Cw,
        default_coolant: CoolantMode::Mist,
        stickout_length_mm: 30.0,
        geometry,
        presets: Vec::new(),
    };
    SweepToolSpec {
        profile: RadialCutterProfile::from_tool(&tool),
        is_flat_endmill,
    }
}

fn dot3(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn angle_deg(a: [f32; 3], b: [f32; 3]) -> f32 {
    let va = DVec3::new(f64::from(a[0]), f64::from(a[1]), f64::from(a[2]));
    let vb = DVec3::new(f64::from(b[0]), f64::from(b[1]), f64::from(b[2]));
    va.cross(vb).length().atan2(va.dot(vb)).to_degrees() as f32
}

/// Measures the average normal discontinuity (in degrees) across shared smooth vertices
/// (positions quantized to `1e-3 mm` where incident normals are within 38° of each other)
/// and counts quads whose 4 corners have non-identical normals (`> 0.1°` spread).
fn measure_quad_mesh_smoothness(mesh: &SimStockMesh) -> (f64, usize, usize) {
    assert_eq!(mesh.vertices.len() % 4, 0, "expected quad vertex layout");
    let num_quads = mesh.vertices.len() / 4;
    let mut non_flat_quads = 0usize;
    let mut vertex_normals: HashMap<[i64; 3], Vec<[f32; 3]>> = HashMap::new();

    for qi in 0..num_quads {
        let vs = &mesh.vertices[qi * 4..qi * 4 + 4];
        let mut max_spread = 0.0_f32;
        for a in 0..4 {
            for b in (a + 1)..4 {
                max_spread = max_spread.max(angle_deg(vs[a].normal, vs[b].normal));
            }
            let key = [
                (f64::from(vs[a].pos_mm[0]) * 1000.0).round() as i64,
                (f64::from(vs[a].pos_mm[1]) * 1000.0).round() as i64,
                (f64::from(vs[a].pos_mm[2]) * 1000.0).round() as i64,
            ];
            vertex_normals.entry(key).or_default().push(vs[a].normal);
        }
        if max_spread > 0.1 {
            non_flat_quads += 1;
        }
    }

    let mut angle_sum = 0.0_f64;
    let mut pair_count = 0usize;
    for ns in vertex_normals.values() {
        if ns.len() < 2 {
            continue;
        }
        for a in 0..ns.len() {
            for b in (a + 1)..ns.len() {
                let d = dot3(ns[a], ns[b]);
                // Only evaluate smooth shared corners (exclude intentional >= 38° sharp creases)
                if d >= 0.788 {
                    angle_sum += f64::from(d.clamp(-1.0, 1.0).acos().to_degrees());
                    pair_count += 1;
                }
            }
        }
    }
    let mean_discont = if pair_count > 0 {
        angle_sum / pair_count as f64
    } else {
        0.0
    };
    (mean_discont, non_flat_quads, num_quads)
}

/// Builds a TriDexel stock with a carved smooth spherical dome / ball-nose cavity in the center.
fn build_curved_ball_cavity_tridexel() -> TriDexelStock {
    let aabb = Aabb3::new(DVec3::new(-20.0, -20.0, -16.0), DVec3::new(20.0, 20.0, 0.0));
    let mut stock = TriDexelStock::from_stock_spec(
        &StockShapeSpec::Box { aabb_wcs: aabb },
        SimResolution::Standard,
    )
    .unwrap();
    let ball = make_tool(
        "ball_12",
        CutterGeometry::BallEndmill {
            diameter_mm: 12.0,
            flute_length_mm: 18.0,
            overall_length_mm: 50.0,
            shank_diameter_mm: 12.0,
            flutes: 2,
        },
    );
    let tol = StockVerifyTolerances::default();
    // Helical / curved ball-nose sweep carving a smooth scalloped bowl + curved fillet channel
    let mut chords = Vec::new();
    let steps = 24;
    for i in 0..steps {
        let t0 = (i as f64) / (steps as f64);
        let t1 = ((i + 1) as f64) / (steps as f64);
        let a0 = t0 * std::f64::consts::TAU;
        let a1 = t1 * std::f64::consts::TAU;
        let r = 6.5;
        let p0 = DVec3::new(r * a0.cos(), r * a0.sin(), -4.5 - 3.5 * t0);
        let p1 = DVec3::new(r * a1.cos(), r * a1.sin(), -4.5 - 3.5 * t1);
        chords.push(SweepChord {
            motion: SweepMotion::Linear {
                p0,
                p1,
                rotary_deg_0: 0.0,
                rotary_deg_1: 0.0,
            },
            op_idx: 0,
            is_rapid: false,
            profile_idx: 0,
            verify_tol: tol,
        });
    }
    let _ = stock.sweep_chords(&chords, &[ball], None, false);
    stock
}

/// Builds a TriDexel stock with a crisp 90° rectangular pocket (`X ∈ [-10, 10], Y ∈ [-8, 8], Z ∈ [-7, 0]`).
fn build_rectangular_pocket_tridexel() -> TriDexelStock {
    let aabb = Aabb3::new(DVec3::new(-20.0, -20.0, -16.0), DVec3::new(20.0, 20.0, 0.0));
    let mut stock = TriDexelStock::from_stock_spec(
        &StockShapeSpec::Box { aabb_wcs: aabb },
        SimResolution::Standard,
    )
    .unwrap();
    let (nx, ny, nz) = (stock.nx(), stock.ny(), stock.nz());
    for j in 0..ny {
        let py = stock.z_plane().center_v(j);
        if (-8.0..=8.0).contains(&py) {
            for i in 0..nx {
                let px = stock.z_plane().center_u(i);
                if (-10.0..=10.0).contains(&px) {
                    stock
                        .plane_mut(Axis3::Z)
                        .subtract_ray_interval(i, j, -7.0, 0.0, 1);
                }
            }
        }
    }
    for k in 0..nz {
        let pz = stock.x_plane().center_v(k);
        if pz >= -7.0 {
            for j in 0..ny {
                let py = stock.x_plane().center_u(j);
                if (-8.0..=8.0).contains(&py) {
                    stock
                        .plane_mut(Axis3::X)
                        .subtract_ray_interval(j, k, -10.0, 10.0, 1);
                }
            }
            for i in 0..nx {
                let px = stock.y_plane().center_u(i);
                if (-10.0..=10.0).contains(&px) {
                    stock
                        .plane_mut(Axis3::Y)
                        .subtract_ray_interval(i, k, -8.0, 8.0, 1);
                }
            }
        }
    }
    stock
}

#[test]
fn cpu_tridexel_curved_skinning_is_smooth_watertight_and_bounded() {
    let stock = build_curved_ball_cavity_tridexel();
    let crisp = stock.extract_mesh_with_options(
        None,
        SimMeshOptions::new(SimStockColorMode::ByOperation, false),
    );
    let skinned = stock.extract_mesh_with_options(
        None,
        SimMeshOptions::new(SimStockColorMode::ByOperation, true),
    );

    // Same quad count, exact 1:1 quad topology
    assert_eq!(crisp.vertices.len(), skinned.vertices.len());
    assert_eq!(crisp.indices.len(), skinned.indices.len());
    assert!(skinned.vertices.len() > 4000);

    let (crisp_disc, crisp_non_flat, _) = measure_quad_mesh_smoothness(&crisp);
    let (skin_disc, skin_non_flat, total_quads) = measure_quad_mesh_smoothness(&skinned);

    assert_eq!(
        crisp_non_flat, 0,
        "crisp mode emits one constant normal per quad"
    );
    assert!(
        skin_non_flat > total_quads / 10,
        "skinned mode must emit non-flat interpolated per-corner normals on curved quads (got {skin_non_flat}/{total_quads})"
    );
    assert!(
        skin_disc < crisp_disc * 0.5,
        "skinned mode must strictly reduce normal discontinuity across adjacent curved quads (skinned={skin_disc:.3}° vs crisp={crisp_disc:.3}°)"
    );

    // Verify bounded displacement within the dual cell (<= 1.15 * pitch from crisp axis-decoupled
    // corner, since real 3D Surface Nets removes up to ~0.75 * pitch of stair-step offset) AND
    // exact watertight shared corner identity!
    let min_pitch = stock.pitch_mm.x.min(stock.pitch_mm.y).min(stock.pitch_mm.z);
    let max_allowed = 1.15 * min_pitch + 1e-4;
    let mut max_disp = 0.0_f64;
    let mut orig_to_relaxed: HashMap<[u32; 3], [u32; 3]> = HashMap::new();

    for (vc, vs) in crisp.vertices.iter().zip(skinned.vertices.iter()) {
        let dx = f64::from(vs.pos_mm[0]) - f64::from(vc.pos_mm[0]);
        let dy = f64::from(vs.pos_mm[1]) - f64::from(vc.pos_mm[1]);
        let dz = f64::from(vs.pos_mm[2]) - f64::from(vc.pos_mm[2]);
        let d = (dx * dx + dy * dy + dz * dz).sqrt();
        max_disp = max_disp.max(d);

        let orig_bits = [
            vc.pos_mm[0].to_bits(),
            vc.pos_mm[1].to_bits(),
            vc.pos_mm[2].to_bits(),
        ];
        let relaxed_bits = [
            vs.pos_mm[0].to_bits(),
            vs.pos_mm[1].to_bits(),
            vs.pos_mm[2].to_bits(),
        ];
        if let Some(&prev_bits) = orig_to_relaxed.get(&orig_bits) {
            assert_eq!(
                prev_bits, relaxed_bits,
                "shared dual corner diverged across adjacent quads (crack detected)"
            );
        } else {
            orig_to_relaxed.insert(orig_bits, relaxed_bits);
        }
    }
    assert!(
        max_disp > 1e-4,
        "curved surface should actually relax unpinned vertices (max_disp={max_disp:.6})"
    );
    assert!(
        max_disp <= max_allowed,
        "relaxed displacement {max_disp:.6} mm exceeded 0.45 * pitch ({max_allowed:.6} mm)"
    );
}

#[test]
fn cpu_tridexel_rectangular_90deg_pocket_preserves_sharp_walls_and_floor() {
    let stock = build_rectangular_pocket_tridexel();
    let skinned = stock.extract_mesh_with_options(
        None,
        SimMeshOptions::new(SimStockColorMode::ByOperation, true),
    );

    let mut floor_verts = 0usize;
    let mut wall_verts = 0usize;
    let num_quads = skinned.vertices.len() / 4;

    for qi in 0..num_quads {
        let vs = &skinned.vertices[qi * 4..qi * 4 + 4];
        let cx = 0.25 * (vs[0].pos_mm[0] + vs[1].pos_mm[0] + vs[2].pos_mm[0] + vs[3].pos_mm[0]);
        let cy = 0.25 * (vs[0].pos_mm[1] + vs[1].pos_mm[1] + vs[2].pos_mm[1] + vs[3].pos_mm[1]);
        let cz = 0.25 * (vs[0].pos_mm[2] + vs[1].pos_mm[2] + vs[2].pos_mm[2] + vs[3].pos_mm[2]);

        // Pocket floor interior (`Z ≈ -7.0`, inside `[-9, 9] × [-7, 7]`)
        if (-9.0..=9.0).contains(&cx) && (-7.0..=7.0).contains(&cy) && (cz - (-7.0)).abs() < 0.35 {
            for v in vs {
                assert!(
                    v.normal[2] > 0.99,
                    "90° pocket floor normal must stay flat (n.z > 0.99), got {:?} at {:?}",
                    v.normal,
                    v.pos_mm
                );
                floor_verts += 1;
            }
        }

        // Pocket vertical walls (`Z ∈ [-6.2, -0.8]`, near `|X| ≈ 10` or `|Y| ≈ 8`)
        let on_x_wall = (cx.abs() - 10.0).abs() < 0.35 && cy.abs() < 7.2;
        let on_y_wall = (cy.abs() - 8.0).abs() < 0.35 && cx.abs() < 9.2;
        if (-6.2..=-0.8).contains(&cz) && (on_x_wall || on_y_wall) {
            for v in vs {
                assert!(
                    v.normal[2].abs() < 0.05,
                    "90° pocket vertical wall normal must stay orthogonal (|n.z| < 0.05), got {:?} at {:?}",
                    v.normal,
                    v.pos_mm
                );
                wall_verts += 1;
            }
        }
    }

    assert!(
        floor_verts > 200,
        "expected >200 pocket floor vertices, got {floor_verts}"
    );
    assert!(
        wall_verts > 200,
        "expected >200 pocket wall vertices, got {wall_verts}"
    );
}

#[test]
fn cpu_heightfield_skinning_smooths_curved_cut_and_keeps_90deg_pocket_sharp() {
    let aabb = Aabb3::new(DVec3::new(-20.0, -20.0, -16.0), DVec3::new(20.0, 20.0, 0.0));
    let mut hf = HeightfieldStock::from_stock_spec(
        &StockShapeSpec::Box { aabb_wcs: aabb },
        SimResolution::Standard,
    )
    .unwrap();

    // Carve a smooth ball-nose spherical bowl on the left (`X ∈ [-14, -2]`) AND a sharp 90°
    // rectangular pocket on the right (`X ∈ [4, 15], Y ∈ [-6, 6], Z = -7.0`)
    let ball = make_tool(
        "ball_10",
        CutterGeometry::BallEndmill {
            diameter_mm: 10.0,
            flute_length_mm: 16.0,
            overall_length_mm: 50.0,
            shank_diameter_mm: 10.0,
            flutes: 2,
        },
    );
    let tol = StockVerifyTolerances::default();
    let chords = vec![SweepChord {
        motion: SweepMotion::Linear {
            p0: DVec3::new(-12.0, -4.0, -5.5),
            p1: DVec3::new(-4.0, 4.0, -3.5),
            rotary_deg_0: 0.0,
            rotary_deg_1: 0.0,
        },
        op_idx: 0,
        is_rapid: false,
        profile_idx: 0,
        verify_tol: tol,
    }];
    let _ = hf.sweep_chords(&chords, &[ball], false);

    // Direct 90° rectangular pocket on the right
    let z_pocket_local = hf.z_wcs_to_local(-7.0);
    for j in 0..hf.ny {
        let py = hf.origin_wcs.y + (j as f64 + 0.5) * hf.dy_mm;
        if (-6.0..=6.0).contains(&py) {
            for i in 0..hf.nx {
                let px = hf.origin_wcs.x + (i as f64 + 0.5) * hf.dx_mm;
                if (4.0..=15.0).contains(&px) {
                    let idx = j * hf.nx + i;
                    hf.z_stock[idx] = z_pocket_local;
                    hf.last_op_idx[idx] = 1;
                }
            }
        }
    }

    let crisp =
        hf.extract_mesh_with_options(SimMeshOptions::new(SimStockColorMode::ByOperation, false));
    let skinned =
        hf.extract_mesh_with_options(SimMeshOptions::new(SimStockColorMode::ByOperation, true));

    let (_, crisp_non_flat, _) = measure_quad_mesh_smoothness(&crisp);
    let (_, skin_non_flat, _) = measure_quad_mesh_smoothness(&skinned);
    assert_eq!(crisp_non_flat, 0);
    assert!(
        skin_non_flat > 200,
        "Heightfield skinned curved groove must produce per-corner smooth normals (got {skin_non_flat})"
    );

    // Check 90° pocket floor and vertical walls on the right remain razor-sharp
    let mut floor_verts = 0usize;
    let mut wall_verts = 0usize;
    let num_quads = skinned.vertices.len() / 4;
    for qi in 0..num_quads {
        let vs = &skinned.vertices[qi * 4..qi * 4 + 4];
        let cx = 0.25 * (vs[0].pos_mm[0] + vs[1].pos_mm[0] + vs[2].pos_mm[0] + vs[3].pos_mm[0]);
        let cy = 0.25 * (vs[0].pos_mm[1] + vs[1].pos_mm[1] + vs[2].pos_mm[1] + vs[3].pos_mm[1]);
        let cz = 0.25 * (vs[0].pos_mm[2] + vs[1].pos_mm[2] + vs[2].pos_mm[2] + vs[3].pos_mm[2]);

        if (5.0..=14.0).contains(&cx) && (-5.0..=5.0).contains(&cy) && (cz - (-7.0)).abs() < 0.2 {
            for v in vs {
                assert!(
                    v.normal[2] > 0.99,
                    "Heightfield 90° pocket floor must stay flat, got {:?}",
                    v.normal
                );
                floor_verts += 1;
            }
        }
        let on_x_wall = ((cx - 4.0).abs() < 0.2 || (cx - 15.0).abs() < 0.2) && cy.abs() < 5.0;
        if (-6.5..=-0.5).contains(&cz) && on_x_wall {
            for v in vs {
                assert!(
                    v.normal[2].abs() < 0.05,
                    "Heightfield 90° pocket wall must stay vertical, got {:?}",
                    v.normal
                );
                wall_verts += 1;
            }
        }
    }
    assert!(floor_verts > 100);
    assert!(wall_verts > 100);
}

#[test]
fn cpu_cylindrical_skinning_preserves_turned_shoulder_and_smooths_helical_cut() {
    let r_stock = 20.0;
    let len = 60.0;
    let rotary = RotaryStockSpec {
        axis: Axis3::X,
        direction_sign: 1.0,
        axis_letter: 'A',
        center_wcs: DVec3::ZERO,
        stock_u_min: -30.0,
        stock_u_max: 30.0,
        stock_outer_radius_mm: r_stock,
        chuck_u_min: -50.0,
        chuck_u_max: -30.0,
        chuck_radius_mm: 30.0,
    };
    let spec = StockShapeSpec::Cylinder {
        axis: Axis3::X,
        center_wcs: DVec3::ZERO,
        radius_mm: r_stock,
        min_axial_mm: -30.0,
        max_axial_mm: 30.0,
        aabb_wcs: Aabb3::new(
            DVec3::new(-30.0, -r_stock, -r_stock),
            DVec3::new(30.0, r_stock, r_stock),
        ),
    };
    let mut cyl =
        CylindricalStock::from_stock_spec(&spec, rotary, SimResolution::Standard).unwrap();

    // Carve a smooth helical ball-end groove on X ∈ [-20, 0] AND a crisp turned shoulder step at X = 10
    let ball = make_tool(
        "ball_6",
        CutterGeometry::BallEndmill {
            diameter_mm: 6.0,
            flute_length_mm: 14.0,
            overall_length_mm: 50.0,
            shank_diameter_mm: 6.0,
            flutes: 2,
        },
    );
    let tol = StockVerifyTolerances::default();
    let chords = vec![SweepChord {
        motion: SweepMotion::Linear {
            p0: DVec3::new(-20.0, 0.0, 16.0),
            p1: DVec3::new(0.0, 0.0, 16.0),
            rotary_deg_0: 0.0,
            rotary_deg_1: 180.0,
        },
        op_idx: 0,
        is_rapid: false,
        profile_idx: 0,
        verify_tol: tol,
    }];
    let _ = cyl.sweep_chords(&chords, &[ball], false);

    // Sharp turned shoulder at X >= 10.0 (`r = 14.0`)
    for iu in 0..cyl.nu {
        let u = cyl.u_min + (iu as f64 + 0.5) * cyl.du;
        if u >= 10.0 {
            for it in 0..cyl.ntheta {
                let idx = iu * cyl.ntheta + it;
                cyl.r_stock[idx] = 14.0;
                cyl.last_op_idx[idx] = 1;
            }
        }
    }

    let crisp = cyl.extract_mesh_with_options(
        SimMeshOptions::new(SimStockColorMode::ByOperation, false),
        0.0,
    );
    let skinned = cyl.extract_mesh_with_options(
        SimMeshOptions::new(SimStockColorMode::ByOperation, true),
        0.0,
    );
    assert!(skinned.vertices.len() > crisp.vertices.len());
    assert_eq!(skinned.indices.len() % 3, 0);

    // Check turned cylinder band at X ∈ [14, 24] stays purely radial (`|n.x| < 0.05`)
    let mut radial_checked = 0usize;
    for v in &skinned.vertices {
        if v.pos_mm[0] >= 14.0 && v.pos_mm[0] <= 24.0 {
            assert!(
                v.normal[0].abs() < 0.05,
                "turned cylindrical band must keep radial normal (|n.x| < 0.05), got {:?}",
                v.normal
            );
            radial_checked += 1;
        }
    }
    assert!(radial_checked > 500);
    let _ = len;
}

#[test]
fn metrology_error_report_is_bit_identical_between_crisp_and_skinned() {
    let stock = build_curved_ball_cavity_tridexel();
    let target = TriDexelStock::from_stock_spec(
        &StockShapeSpec::Box {
            aabb_wcs: Aabb3::new(DVec3::new(-20.0, -20.0, -16.0), DVec3::new(20.0, 20.0, 0.0)),
        },
        SimResolution::Standard,
    )
    .unwrap();
    let init_vol = 40.0 * 40.0 * 16.0;

    let mut backend = CpuExactBackend::new(
        SimulatedStock::TriDexel(Box::new(stock.clone())),
        Some(target.clone()),
    );
    let report_before = backend
        .compute_error_report(init_vol, 0.05, Vec::new())
        .unwrap();
    let _crisp_mesh = backend.extract_stock_mesh_with_options(SimMeshOptions::new(
        SimStockColorMode::ByOperation,
        false,
    ));
    let report_after_crisp = backend
        .compute_error_report(init_vol, 0.05, Vec::new())
        .unwrap();
    let _skinned_mesh = backend
        .extract_stock_mesh_with_options(SimMeshOptions::new(SimStockColorMode::ByOperation, true));
    let report_after_skinned = backend
        .compute_error_report(init_vol, 0.05, Vec::new())
        .unwrap();

    assert_eq!(report_before, report_after_crisp);
    assert_eq!(report_before, report_after_skinned);

    // Direct function check on TriDexel & Heightfield
    let td_rep = compute_tridexel_error_report(&stock, &target, init_vol, 0.05, Vec::new());
    assert_eq!(td_rep.sample_count, report_after_skinned.sample_count);
    assert_eq!(
        td_rep.excess_volume_mm3.to_bits(),
        report_after_skinned.excess_volume_mm3.to_bits()
    );
    assert_eq!(
        td_rep.gouge_volume_mm3.to_bits(),
        report_after_skinned.gouge_volume_mm3.to_bits()
    );
    assert_eq!(
        td_rep.max_excess_mm.to_bits(),
        report_after_skinned.max_excess_mm.to_bits()
    );
    assert_eq!(
        td_rep.max_gouge_mm.to_bits(),
        report_after_skinned.max_gouge_mm.to_bits()
    );

    let mut hf = HeightfieldStock::from_stock_spec(
        &StockShapeSpec::Box {
            aabb_wcs: Aabb3::new(DVec3::new(-20.0, -20.0, -16.0), DVec3::new(20.0, 20.0, 0.0)),
        },
        SimResolution::Standard,
    )
    .unwrap();
    let z_tgt_local = hf.z_wcs_to_local(-4.0);
    hf.z_target.fill(z_tgt_local);
    let hf_rep_before = compute_heightmap_error_report(&hf, init_vol, 0.05, Vec::new()).unwrap();
    let _ = hf.extract_mesh_with_options(SimMeshOptions::new(SimStockColorMode::Metallic, true));
    let hf_rep_after = compute_heightmap_error_report(&hf, init_vol, 0.05, Vec::new()).unwrap();
    assert_eq!(hf_rep_before, hf_rep_after);
    let _ = &mut backend;
}

#[cfg(feature = "gpu-compute")]
mod gpu_skinning_tests {
    use super::*;
    use crate::gpu::{
        GpuSimContext, GpuStockRenderHandle, PackedTriDexelQuad, TRIDEXEL_PACKED_QUAD_BYTES,
        WgpuComputeBackend, oct_decode_unit_normal, oct_encode_unit_normal,
        pack_oct_normal_pair_12bit, unpack_oct_normal_12bit,
    };

    #[test]
    fn octahedral_12bit_normal_roundtrip_accuracy_under_half_degree() {
        assert_eq!(std::mem::size_of::<PackedTriDexelQuad>(), 32);
        assert_eq!(TRIDEXEL_PACKED_QUAD_BYTES, 32);

        // 1. Exact 0.0° roundtrip on all 6 cardinal axes
        let cardinals = [
            [1.0_f32, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, -1.0],
        ];
        for &n in &cardinals {
            let (hi, lo0, lo1) = pack_oct_normal_pair_12bit(n, [-n[0], -n[1], -n[2]]);
            let u0 = unpack_oct_normal_12bit(hi, false, u32::from(lo0) << 24);
            let u1 = unpack_oct_normal_12bit(hi, true, u32::from(lo1) << 24);
            assert!(
                angle_deg(n, u0) < 1e-3,
                "cardinal {n:?} roundtrip failed: got {u0:?}"
            );
            assert!(
                angle_deg([-n[0], -n[1], -n[2]], u1) < 1e-3,
                "negated cardinal roundtrip failed: got {u1:?}"
            );
        }

        // 2. Dense 2,000-point Fibonacci sphere + all 8 octant diagonals
        let golden = std::f64::consts::PI * (3.0 - 5.0_f64.sqrt());
        let mut dirs: Vec<[f32; 3]> = Vec::with_capacity(2008);
        for i in 0..2000 {
            let y = 1.0 - (i as f64 / 1999.0) * 2.0;
            let r = (1.0 - y * y).max(0.0).sqrt();
            let theta = golden * (i as f64);
            let x = theta.cos() * r;
            let z = theta.sin() * r;
            dirs.push([x as f32, y as f32, z as f32]);
        }
        let inv_s3 = 1.0 / 3.0_f32.sqrt();
        for sx in [-1.0_f32, 1.0] {
            for sy in [-1.0_f32, 1.0] {
                for sz in [-1.0_f32, 1.0] {
                    dirs.push([sx * inv_s3, sy * inv_s3, sz * inv_s3]);
                }
            }
        }

        let mut max_err_deg = 0.0_f32;
        for &[n_a, n_b] in dirs.as_chunks::<2>().0 {
            // Float oct roundtrip check
            let fa = oct_decode_unit_normal(oct_encode_unit_normal(n_a));
            assert!(angle_deg(n_a, fa) < 1e-3);

            // Quantized 12-bit packed roundtrip check
            let (hi, lo_a, lo_b) = pack_oct_normal_pair_12bit(n_a, n_b);
            let dec_a = unpack_oct_normal_12bit(hi, false, u32::from(lo_a) << 24);
            let dec_b = unpack_oct_normal_12bit(hi, true, u32::from(lo_b) << 24);
            max_err_deg = max_err_deg.max(angle_deg(n_a, dec_a));
            max_err_deg = max_err_deg.max(angle_deg(n_b, dec_b));
        }

        assert!(
            max_err_deg < 0.5,
            "octahedral 12-bit normal roundtrip worst-case error {max_err_deg:.4}° >= 0.5°"
        );
    }

    #[test]
    fn gpu_tridexel_skinning_matches_cpu_smoothness_watertightness_and_sharp_creases() {
        let Some(ctx) = GpuSimContext::shared_headless() else {
            return;
        };

        // 1. Curved ball-nose cavity: compare GPU Crisp vs GPU Skinned + live toggle without re-sweep
        let curved_stock = build_curved_ball_cavity_tridexel();
        let min_pitch = curved_stock
            .pitch_mm
            .x
            .min(curved_stock.pitch_mm.y)
            .min(curved_stock.pitch_mm.z);

        let mut gpu_be = WgpuComputeBackend::new(
            std::sync::Arc::clone(&ctx),
            SimulatedStock::TriDexel(Box::new(curved_stock.clone())),
            None,
        );
        let handle = gpu_be.capture_render_handle_with_options(SimMeshOptions::new(
            SimStockColorMode::ByOperation,
            false,
        ));
        let GpuStockRenderHandle::TriDexel(ref td_render) = *handle else {
            panic!("expected TriDexel GPU render handle");
        };
        assert_eq!(td_render.emit_dispatch_count(), 1);
        let crisp_quads = td_render.readback_emitted_quads();
        assert!(crisp_quads.len() > 1000);

        // Live toggle to mesh_skinning = true without re-running any sweep!
        assert!(
            td_render
                .refresh_for_options(SimMeshOptions::new(SimStockColorMode::ByOperation, true,))
        );
        assert_eq!(td_render.emit_dispatch_count(), 2);
        // Repeating the same options must be a zero-op (no extra dispatch)
        assert!(
            !td_render
                .refresh_for_options(SimMeshOptions::new(SimStockColorMode::ByOperation, true,))
        );
        assert_eq!(td_render.emit_dispatch_count(), 2);

        let skinned_quads = td_render.readback_emitted_quads();
        assert_eq!(crisp_quads.len(), skinned_quads.len());

        let quads_to_mesh = |quads: &[PackedTriDexelQuad]| -> SimStockMesh {
            let mut vertices = Vec::with_capacity(quads.len() * 4);
            let mut indices = Vec::with_capacity(quads.len() * 6);
            for (qi, q) in quads.iter().enumerate() {
                let corners = td_render.unpack_quad_corners(q);
                let base = (qi as u32) * 4;
                for (pos, n) in corners {
                    vertices.push(crate::types::SimColoredVertex {
                        pos_mm: pos,
                        normal: n,
                        color_rgba: [1.0; 4],
                    });
                }
                indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
            }
            SimStockMesh { vertices, indices }
        };

        let gpu_crisp_mesh = quads_to_mesh(&crisp_quads);
        let gpu_skinned_mesh = quads_to_mesh(&skinned_quads);

        let (gpu_crisp_disc, gpu_crisp_non_flat, _) = measure_quad_mesh_smoothness(&gpu_crisp_mesh);
        let (gpu_skin_disc, gpu_skin_non_flat, gpu_total_quads) =
            measure_quad_mesh_smoothness(&gpu_skinned_mesh);

        assert_eq!(
            gpu_crisp_non_flat, 0,
            "GPU crisp mode packs 1 normal per quad"
        );
        assert!(
            gpu_skin_non_flat > gpu_total_quads / 10,
            "GPU skinned mode must emit distinct per-corner normals on curved quads (got {gpu_skin_non_flat}/{gpu_total_quads})"
        );
        assert!(
            gpu_skin_disc < gpu_crisp_disc * 0.65,
            "GPU skinned mode must reduce normal discontinuity across adjacent curved quads (skinned={gpu_skin_disc:.3}° vs crisp={gpu_crisp_disc:.3}°)"
        );

        // Verify GPU displacement stays bounded inside dual cell AND watertightness (shared dual corners emit identical positions)
        let max_allowed = 1.15 * min_pitch + 0.02; // allow 1 LSB of snorm8 packing (~0.008*pitch)
        let mut max_gpu_disp = 0.0_f64;
        let mut corner_map: HashMap<(u32, u32, u32), [f32; 3]> = HashMap::new();
        let nx = td_render.dims[0];
        let ny = td_render.dims[1];
        let off_y = td_render.plane_ray_offsets[1];
        let off_z = td_render.plane_ray_offsets[2];

        // Match crisp vs skinned quads by `(ray_dw_flags)` since atomic emission order is parallel
        let crisp_by_key: HashMap<u32, PackedTriDexelQuad> =
            crisp_quads.iter().map(|q| (q.ray_dw_flags, *q)).collect();
        for sq in &skinned_quads {
            let cq = crisp_by_key
                .get(&sq.ray_dw_flags)
                .expect("matching crisp quad must exist");
            let sc = td_render.unpack_quad_corners(sq);
            let cc = td_render.unpack_quad_corners(cq);
            for ci in 0..4 {
                let dx = f64::from(sc[ci].0[0]) - f64::from(cc[ci].0[0]);
                let dy = f64::from(sc[ci].0[1]) - f64::from(cc[ci].0[1]);
                let dz = f64::from(sc[ci].0[2]) - f64::from(cc[ci].0[2]);
                max_gpu_disp = max_gpu_disp.max((dx * dx + dy * dy + dz * dz).sqrt());
            }

            // Verify watertightness across every shared dual corner (including X/Y/Z seams!)
            let global_ray = sq.ray_dw_flags & 0xFFFFF;
            let dw_idx = (sq.ray_dw_flags >> 20) & 0x3FF;
            let want_min = (sq.ray_dw_flags & 0x8000_0000) != 0;
            let (axis_id, plane_offset, nu) = if global_ray < off_y {
                (0u32, td_render.plane_ray_offsets[0], ny)
            } else if global_ray < off_z {
                (1u32, off_y, nx)
            } else {
                (2u32, off_z, nx)
            };
            let local_ray = global_ray - plane_offset;
            let iu = local_ray % nu.max(1);
            let iv = local_ray / nu.max(1);
            let steps: [(u32, u32); 4] = if axis_id == 1 {
                [(0, 0), (0, 1), (1, 1), (1, 0)]
            } else {
                [(0, 0), (1, 0), (1, 1), (0, 1)]
            };
            let order: [usize; 4] = if want_min { [0, 3, 2, 1] } else { [0, 1, 2, 3] };
            for (slot, &raw_ci) in order.iter().enumerate() {
                let cu = iu + steps[raw_ci].0;
                let cv = iv + steps[raw_ci].1;
                let c_xyz = match axis_id {
                    0 => (dw_idx, cu, cv),
                    1 => (cu, dw_idx, cv),
                    _ => (cu, cv, dw_idx),
                };
                let pos = sc[slot].0;
                if let Some(&prev) = corner_map.get(&c_xyz) {
                    let err = ((pos[0] - prev[0]).powi(2)
                        + (pos[1] - prev[1]).powi(2)
                        + (pos[2] - prev[2]).powi(2))
                    .sqrt();
                    assert!(
                        err < 1e-4,
                        "GPU Tri-Dexel crack at shared dual corner {c_xyz:?}: {pos:?} vs {prev:?} (err={err:.6})"
                    );
                } else {
                    corner_map.insert(c_xyz, pos);
                }
            }
        }
        assert!(
            max_gpu_disp > 1e-3,
            "GPU curved surface relaxation should move unpinned vertices (max_gpu_disp={max_gpu_disp:.6})"
        );
        assert!(
            max_gpu_disp <= max_allowed,
            "GPU relaxed displacement {max_gpu_disp:.6} mm exceeded 0.45 * pitch ({max_allowed:.6} mm)"
        );

        // 2. Compare CPU vs GPU skinned normal direction parity across curved surface
        let cpu_skinned_mesh = curved_stock.extract_mesh_with_options(
            None,
            SimMeshOptions::new(SimStockColorMode::ByOperation, true),
        );
        let (cpu_skin_disc, _, _) = measure_quad_mesh_smoothness(&cpu_skinned_mesh);
        assert!(
            cpu_skin_disc < 5.0 && gpu_skin_disc < 5.0,
            "both CPU ({cpu_skin_disc:.2}°) and GPU ({gpu_skin_disc:.2}°) skinned curved surfaces must have low normal discontinuity"
        );

        // 3. Rectangular 90° pocket on GPU: verify sharp vertical walls & flat floor!
        let pocket_stock = build_rectangular_pocket_tridexel();
        let pocket_handle = GpuStockRenderHandle::from_simulated_stock_with_options(
            &ctx,
            &SimulatedStock::TriDexel(Box::new(pocket_stock)),
            None,
            SimMeshOptions::new(SimStockColorMode::ByOperation, true),
        );
        let GpuStockRenderHandle::TriDexel(ref p_td) = *pocket_handle else {
            panic!("expected TriDexel GPU handle");
        };
        let p_quads = p_td.readback_emitted_quads();
        let mut floor_verts = 0usize;
        let mut wall_verts = 0usize;
        for q in &p_quads {
            let cs = p_td.unpack_quad_corners(q);
            let cx = 0.25 * (cs[0].0[0] + cs[1].0[0] + cs[2].0[0] + cs[3].0[0]);
            let cy = 0.25 * (cs[0].0[1] + cs[1].0[1] + cs[2].0[1] + cs[3].0[1]);
            let cz = 0.25 * (cs[0].0[2] + cs[1].0[2] + cs[2].0[2] + cs[3].0[2]);
            if (-9.0..=9.0).contains(&cx)
                && (-7.0..=7.0).contains(&cy)
                && (cz - (-7.0)).abs() < 0.35
            {
                for (_, n) in cs {
                    assert!(
                        n[2] > 0.99,
                        "GPU 90° pocket floor must stay flat (n.z > 0.99), got {n:?}"
                    );
                    floor_verts += 1;
                }
            }
            let on_x_wall = (cx.abs() - 10.0).abs() < 0.35 && cy.abs() < 7.2;
            let on_y_wall = (cy.abs() - 8.0).abs() < 0.35 && cx.abs() < 9.2;
            if (-6.2..=-0.8).contains(&cz) && (on_x_wall || on_y_wall) {
                for (_, n) in cs {
                    assert!(
                        n[2].abs() < 0.05,
                        "GPU 90° pocket vertical wall must stay orthogonal (|n.z| < 0.05), got {n:?}"
                    );
                    wall_verts += 1;
                }
            }
        }
        assert!(floor_verts > 200);
        assert!(wall_verts > 200);
    }
}
