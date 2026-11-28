#![allow(clippy::unwrap_used, clippy::expect_used)]

use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DVec3, Ray3, approx_eq};

use chip_cad::{
    CadKernel, CadPicker, MonstertruckKernel, PickFilter, PickTarget, StepUnitKind, SurfaceKind,
};
use chip_fixtures::*;

#[test]
fn roundtrips_sample_step_and_recognizes_planes_holes_and_arcs() {
    let step_mm = sample_machined_plate_step().unwrap();
    let kernel = MonstertruckKernel;
    let doc = kernel
        .load_step_bytes("sample.step", step_mm.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
        .unwrap();

    assert_eq!(doc.detected_unit, StepUnitKind::Millimeter);
    assert_eq!(doc.bodies.len(), 1);

    let size = doc.aabb_mm.size();
    assert!(approx_eq(size.x, 80.0, 0.05));
    assert!(approx_eq(size.y, 60.0, 0.05));
    assert!(approx_eq(size.z, 16.0, 0.05));

    let planar_count = doc
        .faces
        .iter()
        .filter(|f| matches!(f.surface, SurfaceKind::Plane { .. }))
        .count();
    assert!(
        planar_count >= 7,
        "expected >= 7 planar faces, got {planar_count}"
    );

    let internal_holes: Vec<f64> = doc
        .faces
        .iter()
        .filter_map(|f| match f.surface {
            SurfaceKind::Cylinder {
                radius_mm,
                is_internal: true,
                ..
            } => Some(radius_mm),
            _ => None,
        })
        .collect();
    assert!(
        internal_holes.iter().any(|&r| approx_eq(r, 8.0, 1e-3)),
        "expected R=8mm (Ø16mm) internal bore, got {internal_holes:?}"
    );
    assert!(
        internal_holes.iter().any(|&r| approx_eq(r, 4.0, 1e-3)),
        "expected R=4mm (Ø8mm) internal bore, got {internal_holes:?}"
    );

    let external_fillets: Vec<f64> = doc
        .faces
        .iter()
        .filter_map(|f| match f.surface {
            SurfaceKind::Cylinder {
                radius_mm,
                is_internal: false,
                ..
            } => Some(radius_mm),
            _ => None,
        })
        .collect();
    assert!(
        external_fillets.iter().any(|&r| approx_eq(r, 10.0, 1e-3)),
        "expected R=10mm external corner fillet, got {external_fillets:?}"
    );
}

#[test]
fn normalizes_imperial_inch_step_to_canonical_millimeters() {
    let step_in = sample_machined_plate_step_inches().unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes(
            "inch_part.step",
            step_in.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();

    assert_eq!(doc.detected_unit, StepUnitKind::Inch);
    let size = doc.aabb_mm.size();
    assert!(approx_eq(size.x, 80.0, 0.05));
    assert!(approx_eq(size.y, 60.0, 0.05));
    assert!(approx_eq(size.z, 16.0, 0.05));
}

#[test]
fn spatial_picker_prioritizes_top_edges_and_picks_planar_and_hole_faces() {
    let step_mm = sample_machined_plate_step().unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes("sample.step", step_mm.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
        .unwrap();
    let picker = CadPicker::build(&doc);

    // 1. Ray straight down onto top planar face at (50, 20)
    let face_ray = Ray3::new(DVec3::new(50.0, 20.0, 50.0), DVec3::NEG_Z).unwrap();
    let hit_face = picker.pick(&doc, &face_ray, 0.2, PickFilter::Any).unwrap();
    match hit_face.target {
        PickTarget::Face(fid) => {
            let face = doc.face(fid).unwrap();
            assert!(matches!(face.surface, SurfaceKind::Plane { normal, .. } if normal.z > 0.99));
            assert!(approx_eq(hit_face.point_mm.z, 16.0, 1e-3));
        }
        other => panic!("expected Face hit, got {other:?}"),
    }

    // 2. Ray within 0.4 mm of front top edge `(35, 0.4, 16)` should prioritize the Edge over the top Face!
    let edge_ray = Ray3::new(DVec3::new(35.0, 0.4, 50.0), DVec3::NEG_Z).unwrap();
    let hit_edge = picker.pick(&doc, &edge_ray, 0.2, PickFilter::Any).unwrap();
    assert!(matches!(hit_edge.target, PickTarget::Edge(_)));

    // 3. EdgeLoops filter expands the picked edge to its full closed wire!
    let hit_loop = picker
        .pick(&doc, &edge_ray, 0.2, PickFilter::EdgeLoops)
        .unwrap();
    match hit_loop.target {
        PickTarget::Loop { edges, .. } => assert!(edges.len() >= 4),
        other => panic!("expected Loop hit, got {other:?}"),
    }
}

#[test]
fn multi_feature_fixture_supports_same_diameter_coplanar_blind_hole_and_open_chains() {
    use chip_cad::topology::{
        TopologyGraph, analyze_cylindrical_hole, find_coplanar_faces, find_same_diameter_holes,
        group_coaxial_holes,
    };

    let step = sample_multi_feature_bracket_step(false).unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes(
            "multi_bracket.step",
            step.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();

    // 1. Verify 4x Ø6mm (R=3mm) through-holes (8 half-cylinder faces -> 4 logical holes!)
    let r3_seed = doc
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Cylinder {
                    radius_mm,
                    is_internal: true,
                    ..
                } if approx_eq(radius_mm, 3.0, 1e-3)
            )
        })
        .unwrap()
        .id;

    let same_r3_faces = find_same_diameter_holes(&doc, r3_seed, 1e-3, 1e-3);
    let logical_r3 = group_coaxial_holes(&doc, &same_r3_faces, DVec3::Z);
    assert_eq!(
        logical_r3.len(),
        4,
        "expected 4 distinct logical Ø6mm mounting holes"
    );
    for hole in &logical_r3 {
        assert_eq!(
            hole.faces.len(),
            2,
            "each revolved STEP hole has 2 half-cylinder faces"
        );
        assert!(approx_eq(hole.span.top_proj_mm, 20.0, 0.05));
        assert!(approx_eq(hole.span.bottom_proj_mm, 0.0, 0.05));
        assert!(
            hole.span.is_through_hole,
            "mounting hole {:?} should be through",
            hole.primary_face
        );
    }

    // 2. Verify Ø10mm (R=5mm) blind hole is classified as blind with Z = 5..12 mm!
    let r5_blind = doc
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Cylinder {
                    radius_mm,
                    is_internal: true,
                    ..
                } if approx_eq(radius_mm, 5.0, 1e-3)
            )
        })
        .unwrap()
        .id;
    let blind_span = analyze_cylindrical_hole(&doc, r5_blind, DVec3::Z).unwrap();
    assert!(approx_eq(blind_span.top_proj_mm, 12.0, 0.05));
    assert!(approx_eq(blind_span.bottom_proj_mm, 5.0, 0.05));
    assert!(
        !blind_span.is_through_hole,
        "recessed bore should be classified as a blind hole"
    );

    // 3. Verify coplanar pocket floors at Z = 12.0 mm and adjacent wall top Z = 20.0 mm
    let z12_floor = doc
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
    let coplanar_z12 = find_coplanar_faces(&doc, z12_floor, 1e-3, 1e-2);
    assert_eq!(
        coplanar_z12.len(),
        2,
        "expected Pocket A and Pocket B floors at Z=12mm to match"
    );

    // 4. Verify TopologyGraph open-chain pathfinding along top outer edges across the R10 fillet
    let topo = TopologyGraph::build(&doc);
    let top_back_edge = doc
        .edges
        .iter()
        .find(|e| {
            approx_eq(e.aabb_mm.min.z, 20.0, 1e-2)
                && approx_eq(e.aabb_mm.max.y, 70.0, 1e-2)
                && approx_eq(e.aabb_mm.min.y, 70.0, 1e-2)
        })
        .unwrap()
        .id;
    let top_left_edge = doc
        .edges
        .iter()
        .find(|e| {
            approx_eq(e.aabb_mm.min.z, 20.0, 1e-2)
                && approx_eq(e.aabb_mm.max.x, 0.0, 1e-2)
                && approx_eq(e.aabb_mm.min.x, 0.0, 1e-2)
        })
        .unwrap()
        .id;
    let chain = topo
        .find_open_edge_chain(&doc, top_back_edge, top_left_edge)
        .unwrap();
    assert_eq!(
        chain.len(),
        3,
        "expected [top_back_edge -> R10 fillet arc -> top_left_edge], got {chain:?}"
    );

    // 5. Verify tilted STEP fixture exports & loads cleanly as 1 manifold solid
    let tilted_step = sample_multi_feature_bracket_step(true).unwrap();
    let tilted_doc = MonstertruckKernel
        .load_step_bytes(
            "tilted.step",
            tilted_step.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    assert_eq!(tilted_doc.bodies.len(), 1);
}

#[test]
fn tutorial_minimal_step_fixtures_roundtrip_with_exact_minimal_features() {
    let kernel = MonstertruckKernel;

    // 01 Facing Billet: 100 x 80 x 20 mm plain box (6 planar faces, zero holes)
    let facing_step = tutorial_facing_billet_step().unwrap();
    let facing_doc = kernel
        .load_step_bytes(
            "01-facing-billet.step",
            facing_step.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    assert!(approx_eq(facing_doc.aabb_mm.size().x, 100.0, 1e-2));
    assert!(approx_eq(facing_doc.aabb_mm.size().y, 80.0, 1e-2));
    assert!(approx_eq(facing_doc.aabb_mm.size().z, 20.0, 1e-2));
    assert_eq!(facing_doc.faces.len(), 6);

    // 02 Contour Plate: 100 x 70 x 12 mm with 1 external R10 fillet & 1 45° chamfer (zero holes)
    let contour_step = tutorial_contour_plate_step().unwrap();
    let contour_doc = kernel
        .load_step_bytes(
            "02-contour-plate.step",
            contour_step.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    assert!(approx_eq(contour_doc.aabb_mm.size().x, 100.0, 1e-2));
    assert!(approx_eq(contour_doc.aabb_mm.size().y, 70.0, 1e-2));
    assert!(approx_eq(contour_doc.aabb_mm.size().z, 12.0, 1e-2));
    assert_eq!(
        contour_doc
            .faces
            .iter()
            .filter(|f| matches!(
                f.surface,
                SurfaceKind::Cylinder {
                    is_internal: true,
                    ..
                }
            ))
            .count(),
        0
    );

    // 03 Pocket Block: 100 x 80 x 20 mm with 1 closed island pocket (Z=12) + 1 open step (Z=14)
    let pocket_step = tutorial_pocket_block_step().unwrap();
    let pocket_doc = kernel
        .load_step_bytes(
            "03-pocket-block.step",
            pocket_step.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    assert!(approx_eq(pocket_doc.aabb_mm.size().x, 100.0, 1e-2));
    assert!(approx_eq(pocket_doc.aabb_mm.size().y, 80.0, 1e-2));
    assert!(approx_eq(pocket_doc.aabb_mm.size().z, 20.0, 1e-2));
    let pkt_floor = pocket_doc
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z > 0.99 && approx_eq(origin_mm.z, 12.0, 1e-2)
            )
        })
        .unwrap();
    assert_eq!(pkt_floor.inner_loops.len(), 1);
    assert!(pocket_doc.faces.iter().any(|f| matches!(
        f.surface,
        SurfaceKind::Plane { origin_mm, normal }
        if normal.z > 0.99 && approx_eq(origin_mm.z, 14.0, 1e-2)
    )));

    // 04 Drill & Bore Plate: 100 x 70 x 16 mm with 4x Ø6mm through-holes + 1x Ø18mm blind bore
    let drill_step = tutorial_drill_bore_plate_step().unwrap();
    let drill_doc = kernel
        .load_step_bytes(
            "04-drill-bore-plate.step",
            drill_step.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    assert!(approx_eq(drill_doc.aabb_mm.size().x, 100.0, 1e-2));
    assert!(approx_eq(drill_doc.aabb_mm.size().y, 70.0, 1e-2));
    assert!(approx_eq(drill_doc.aabb_mm.size().z, 16.0, 1e-2));
    let r3_faces: Vec<_> = drill_doc
        .faces
        .iter()
        .filter(|f| {
            matches!(
                f.surface,
                SurfaceKind::Cylinder {
                    radius_mm,
                    is_internal: true,
                    ..
                } if approx_eq(radius_mm, 3.0, 1e-2)
            )
        })
        .collect();
    assert!(!r3_faces.is_empty());
    let r9_faces: Vec<_> = drill_doc
        .faces
        .iter()
        .filter(|f| {
            matches!(
                f.surface,
                SurfaceKind::Cylinder {
                    radius_mm,
                    is_internal: true,
                    ..
                } if approx_eq(radius_mm, 9.0, 1e-2)
            )
        })
        .collect();
    assert!(!r9_faces.is_empty());

    // 05 Adaptive Pocket Block: 100 x 80 x 20 mm with 1 deep Z=10mm island pocket
    let adp_step = tutorial_adaptive_pocket_step().unwrap();
    let adp_doc = kernel
        .load_step_bytes(
            "05-adaptive-pocket.step",
            adp_step.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    assert!(approx_eq(adp_doc.aabb_mm.size().x, 100.0, 1e-2));
    assert!(approx_eq(adp_doc.aabb_mm.size().y, 80.0, 1e-2));
    assert!(approx_eq(adp_doc.aabb_mm.size().z, 20.0, 1e-2));
    let adp_floor = adp_doc
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z > 0.99 && approx_eq(origin_mm.z, 10.0, 1e-2)
            )
        })
        .unwrap();
    assert_eq!(adp_floor.inner_loops.len(), 1);

    // 06 Chamfer & Engrave Block: 100 x 70 x 18 mm with 1 modeled 45° x 1.5 mm front bevel
    let ch_step = tutorial_chamfer_engrave_block_step().unwrap();
    let ch_doc = kernel
        .load_step_bytes(
            "06-chamfer-engrave-block.step",
            ch_step.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    assert!(approx_eq(ch_doc.aabb_mm.size().x, 100.0, 1e-2));
    assert!(approx_eq(ch_doc.aabb_mm.size().y, 70.0, 1e-2));
    assert!(approx_eq(ch_doc.aabb_mm.size().z, 18.0, 1e-2));
    assert!(ch_doc.faces.iter().any(|f| matches!(
        f.surface,
        SurfaceKind::Plane { normal, .. }
        if normal.z > 0.65 && normal.z < 0.76
    )));
}

#[test]
fn tutorial_indexed_4side_block_step_roundtrips_with_exact_face_normals_and_angles() {
    let step = tutorial_indexed_4side_block_step().unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes(
            "11-indexed-multiface-block.step",
            step.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();

    assert_eq!(doc.bodies.len(), 1);
    let size = doc.aabb_mm.size();
    assert!(approx_eq(size.x, 70.0, 1e-2));
    assert!(approx_eq(size.y, 32.0, 1e-2));
    assert!(approx_eq(size.z, 32.0, 1e-2));
    assert!(approx_eq(doc.aabb_mm.min.x, -35.0, 1e-2));
    assert!(approx_eq(doc.aabb_mm.max.x, 35.0, 1e-2));
    assert!(approx_eq(doc.aabb_mm.min.y, -16.0, 1e-2));
    assert!(approx_eq(doc.aabb_mm.max.y, 16.0, 1e-2));
    assert!(approx_eq(doc.aabb_mm.min.z, -16.0, 1e-2));
    assert!(approx_eq(doc.aabb_mm.max.z, 16.0, 1e-2));

    // Verify exact right-hand X/A index angles for all 5 radial planar faces: 0°, 45°, 90°, 180°, 270°
    // (For XAxis RightHandCcw: e1 = +Z, e2 = -Y -> angle = (-atan2(-n.y, n.z)).rem_euclid(360))
    let mut radial_angles: Vec<f64> = doc
        .faces
        .iter()
        .filter_map(|f| match f.surface {
            SurfaceKind::Plane { normal, .. } if normal.x.abs() < 1e-3 => {
                let c1 = normal.z;
                let c2 = -normal.y;
                if c1.hypot(c2) > 0.5 {
                    let a = (-c2.atan2(c1).to_degrees()).rem_euclid(360.0);
                    Some(if (a - 360.0).abs() < 1e-6 { 0.0 } else { a })
                } else {
                    None
                }
            }
            _ => None,
        })
        .collect();
    radial_angles.sort_by(f64::total_cmp);

    for expected_deg in [0.0_f64, 45.0, 90.0, 180.0, 270.0] {
        assert!(
            radial_angles
                .iter()
                .any(|&a| approx_eq(a, expected_deg, 1e-2)),
            "expected planar face at index angle {expected_deg}°, found angles: {radial_angles:?}"
        );
    }
    assert_eq!(doc.faces.len(), 7, "expected clean 7-face chamfered prism");
}

#[test]
fn sample_threaded_manifold_step_roundtrips_with_pilot_holes_and_proud_boss() {
    use chip_cad::topology::{find_same_diameter_holes, group_coaxial_holes};

    let step = sample_threaded_manifold_step().unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes(
            "18-threaded-manifold.step",
            step.as_bytes(),
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();

    assert_eq!(doc.bodies.len(), 1);
    let size = doc.aabb_mm.size();
    assert!(approx_eq(size.x, 80.0, 1e-2));
    assert!(approx_eq(size.y, 60.0, 1e-2));
    assert!(approx_eq(size.z, 20.0, 1e-2));

    // 2× Ø6.80 mm (R = 3.40 mm) M8 pilot holes
    let m8_seed = doc
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Cylinder {
                    radius_mm,
                    is_internal: true,
                    ..
                } if approx_eq(radius_mm, 3.40, 1e-2)
            )
        })
        .unwrap()
        .id;
    let m8_holes = group_coaxial_holes(
        &doc,
        &find_same_diameter_holes(&doc, m8_seed, 1e-3, 1e-3),
        DVec3::Z,
    );
    assert_eq!(m8_holes.len(), 2);
    for h in &m8_holes {
        assert!(h.span.is_through_hole);
        assert!(approx_eq(h.span.top_proj_mm, 12.0, 0.05));
        assert!(approx_eq(h.span.bottom_proj_mm, 0.0, 0.05));
    }

    // 2× Ø5.11 mm (R = 2.555 mm) 1/4-20 UNC pilot holes
    let unc_seed = doc
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Cylinder {
                    radius_mm,
                    is_internal: true,
                    ..
                } if approx_eq(radius_mm, 2.555, 1e-2)
            )
        })
        .unwrap()
        .id;
    let unc_holes = group_coaxial_holes(
        &doc,
        &find_same_diameter_holes(&doc, unc_seed, 1e-3, 1e-3),
        DVec3::Z,
    );
    assert_eq!(unc_holes.len(), 2);

    // 1× proud Ø12.0 mm (R = 6.0 mm) external circular boss (`Z = 12..20 mm`)
    let boss_cyl_count = doc
        .faces
        .iter()
        .filter(|f| {
            matches!(
                f.surface,
                SurfaceKind::Cylinder {
                    radius_mm,
                    is_internal: false,
                    ..
                } if approx_eq(radius_mm, 6.0, 1e-2)
            )
        })
        .count();
    assert_eq!(
        boss_cyl_count, 2,
        "revolved Ø12 mm boss has 2 half-cylinders"
    );

    let boss_top = doc
        .faces
        .iter()
        .find(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z > 0.99 && approx_eq(origin_mm.z, 20.0, 1e-2)
            )
        })
        .expect("expected circular boss top face at Z = 20 mm");
    assert!(approx_eq(
        boss_top.area_mm2,
        std::f64::consts::PI * 36.0,
        0.5
    ));
}

#[test]
fn sample_undercut_deburr_block_step_bytes_roundtrips_with_tslot_dovetail_and_overhang() {
    let bytes = sample_undercut_deburr_block_step_bytes().unwrap();
    let doc = MonstertruckKernel
        .load_step_bytes(
            "sample_undercut_deburr_block.step",
            &bytes,
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();

    assert_eq!(doc.bodies.len(), 1);
    let size = doc.aabb_mm.size();
    assert!(approx_eq(size.x, 100.0, 1e-2));
    assert!(approx_eq(size.y, 80.0, 1e-2));
    assert!(approx_eq(size.z, 28.0, 1e-2));

    // Verify both M8 T-slot floor (Y=20, Z=14) and 60° dovetail floor (Y=52, Z=14) have +Z outward normals
    let slot_floors: Vec<_> = doc
        .faces
        .iter()
        .filter(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z > 0.99 && approx_eq(origin_mm.z, 14.0, 0.05)
            )
        })
        .collect();
    assert_eq!(
        slot_floors.len(),
        2,
        "expected exactly 2 slot floor faces at Z=14 (T-slot & 60° dovetail), got {}",
        slot_floors.len()
    );

    // Verify T-slot overhanging ceiling faces (`normal.z < -0.99` at `Z = 20.0`)
    let tslot_ceilings = doc
        .faces
        .iter()
        .filter(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { origin_mm, normal }
                if normal.z < -0.99 && approx_eq(origin_mm.z, 20.0, 0.05)
            )
        })
        .count();
    assert_eq!(tslot_ceilings, 2, "expected 2 T-slot ceiling lips at Z=20");

    // Verify 2x 60° dovetail overhanging flank faces (`normal.z ≈ -0.5, |normal.y| ≈ 0.866`)
    let dovetail_flanks = doc
        .faces
        .iter()
        .filter(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { normal, .. }
                if approx_eq(normal.z, -0.5, 1e-2) && approx_eq(normal.y.abs(), 0.8660254, 1e-2)
            )
        })
        .count();
    assert_eq!(dovetail_flanks, 2, "expected 2x 60° dovetail flanks");

    // Verify +Y sloped overhang wall (`normal.y > 0.9, normal.z ≈ -0.298`) & tangent R=3 concave cylinder
    let overhang_wall = doc.faces.iter().any(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { normal, .. }
            if normal.y > 0.9 && normal.z < -0.25 && normal.z > -0.35
        )
    });
    assert!(overhang_wall, "expected +Y sloped overhang wall");
    let groove_cyl = doc.faces.iter().any(|f| {
        matches!(
            f.surface,
            SurfaceKind::Cylinder { radius_mm, .. } if approx_eq(radius_mm, 3.0, 0.05)
        )
    });
    assert!(groove_cyl, "expected tangent R=3.0 mm concave groove arc");
}

#[test]
fn sample_dovetail_rail_keyway_and_curved_deburr_and_3mf_and_svg_fixtures_roundtrip() {
    use chip_cad::{MeshImportOptions, MeshUnitKind, load_mesh_bytes};
    use chip_fixtures::sample_sculpted_mount_3mf_bytes;

    // 1. Dovetail rail + blind keyway + Ø8 mm entry bore STEP
    let dove_bytes = sample_dovetail_rail_keyway_step_bytes().unwrap();
    let dove_doc = MonstertruckKernel
        .load_step_bytes(
            "sample_dovetail_rail_keyway.step",
            &dove_bytes,
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    assert_eq!(dove_doc.bodies.len(), 1);
    let dsz = dove_doc.aabb_mm.size();
    assert!(approx_eq(dsz.x, 90.0, 1e-2));
    assert!(approx_eq(dsz.y, 50.0, 1e-2));
    assert!(approx_eq(dsz.z, 22.0, 1e-2));

    // 2x external 60° overhanging dovetail flanks (`normal.z ≈ -0.5, |normal.y| ≈ 0.866`)
    let ext_flanks = dove_doc
        .faces
        .iter()
        .filter(|f| {
            matches!(
                f.surface,
                SurfaceKind::Plane { normal, .. }
                if approx_eq(normal.z, -0.5, 1e-2) && approx_eq(normal.y.abs(), 0.8660254, 1e-2)
            )
        })
        .count();
    assert_eq!(ext_flanks, 2, "expected 2 external 60° dovetail flanks");

    // Blind top keyway floor at Z = 16.0 and Ø8.0 mm (R = 4.0) entry bore
    assert!(dove_doc.faces.iter().any(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { origin_mm, normal }
            if normal.z > 0.99 && approx_eq(origin_mm.z, 16.0, 0.05)
        )
    }));
    assert!(dove_doc.faces.iter().any(|f| {
        matches!(
            f.surface,
            SurfaceKind::Cylinder {
                radius_mm,
                is_internal: true,
                ..
            } if approx_eq(radius_mm, 4.0, 0.05)
        )
    }));

    // 2. Curved sloped boss + underside overhang combo STEP
    let curv_bytes = sample_curved_deburr_overhang_step_bytes().unwrap();
    let curv_doc = MonstertruckKernel
        .load_step_bytes(
            "sample_curved_deburr_overhang.step",
            &curv_bytes,
            DEFAULT_TESS_CHORD_TOL_MM,
        )
        .unwrap();
    assert_eq!(curv_doc.bodies.len(), 1);
    let csz = curv_doc.aabb_mm.size();
    assert!(approx_eq(csz.x, 80.0, 1e-2));
    assert!(approx_eq(csz.y, 50.0, 1e-2));
    assert!(
        (23.0..=23.5).contains(&csz.z),
        "expected curved arch Z height in [23.0, 23.5], got {}",
        csz.z
    );
    assert!(curv_doc.faces.iter().any(|f| {
        matches!(
            f.surface,
            SurfaceKind::Plane { origin_mm, normal }
            if normal.z < -0.99 && approx_eq(origin_mm.z, 9.5, 0.05)
        )
    }));

    // 3. Watertight unit-aware millimeter 3MF sculpted mount bracket
    let mf_bytes = sample_sculpted_mount_3mf_bytes();
    let mesh = load_mesh_bytes("08-bracket.3mf", &mf_bytes, &MeshImportOptions::default()).unwrap();
    assert_eq!(mesh.applied_unit, MeshUnitKind::Millimeter);
    assert!(mesh.diagnostics.is_watertight);
    assert_eq!(mesh.diagnostics.open_boundary_edges, 0);
    assert_eq!(mesh.diagnostics.non_manifold_edges, 0);
    assert!(mesh.diagnostics.signed_volume_mm3 > 0.0);
    let msz = mesh.aabb_mm.size();
    assert!(approx_eq(msz.x, 80.0, 0.05));
    assert!(approx_eq(msz.y, 56.0, 0.05));
    assert!(approx_eq(msz.z, 20.0, 0.05));

    // 4. Clean 96 DPI SVG badge bytes containing cubic Béziers & layers
    let svg_bytes = sample_svg_badge_bytes();
    let svg_str = std::str::from_utf8(&svg_bytes).unwrap();
    assert!(svg_str.contains("id=\"OUTLINE\""));
    assert!(svg_str.contains("id=\"EMBLEM_POCKET\""));
    assert!(svg_str.contains("id=\"VCARVE_ACCENT\""));
    assert!(svg_str.contains(" C "));
}
