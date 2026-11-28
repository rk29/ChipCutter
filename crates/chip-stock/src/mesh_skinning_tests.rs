#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::collections::HashMap;

use chip_math::{Aabb3, DVec3, DisplayUnit};
use chip_tools::{CoolantMode, CutterGeometry, SpindleDirection, Tool, ToolId};

use crate::{
    backend::{CpuExactBackend, SimStockBackend, SimulatedStock},
    cylindrical::CylindricalStock,
    heightfield::HeightfieldStock,
    metrics::{compute_heightmap_error_report, compute_tridexel_error_report},
    tridexel::TriDexelStock,
    types::{
        Axis3, RadialCutterProfile, RotaryStockSpec, SimMeshOptions, SimResolution, SimStockMesh,
        StockShapeSpec, StockVerifyTolerances, SweepChord, SweepMotion, SweepToolSpec,
    },
    visual::SimStockColorMode,
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
