use monstertruck_io::step::save::{
    CompleteStepDisplay, SiPrefix, StepHeaderDescriptor, StepMeasurementContext, StepModels,
};
use monstertruck_modeling::{builder::SweepAngle, *};
use thiserror::Error;

/// Error building or serializing the programmatic sample STEP fixture.
#[derive(Debug, Error)]
pub enum SampleStepError {
    #[error("modeling error building sample bracket: {0}")]
    Modeling(String),
}

/// Builds a deterministic CNC-style prismatic mounting plate (`80 × 60 × 16 mm`) with:
/// - A `45°` corner chamfer (`10 mm × 10 mm`),
/// - An outer corner fillet radius (`R = 10 mm`),
/// - Two through cylindrical mounting & bearing bores (`Ø 16 mm` and `Ø 8 mm`),
///
/// and serializes it as a standards-compliant ISO 10303-21 STEP file!
pub fn sample_machined_plate_step() -> std::result::Result<String, SampleStepError> {
    build_plate_step_with_scale(1.0, SiPrefix::Milli, false)
}

/// Builds the exact same part authored in Imperial inches (`CONVERSION_BASED_UNIT('INCH'...)`)
/// for unit-normalization regression testing.
pub fn sample_machined_plate_step_inches() -> std::result::Result<String, SampleStepError> {
    build_plate_step_with_scale(1.0 / 25.4, SiPrefix::Milli, true)
}

/// Builds a rich multi-feature prismatic bracket (`90 × 70 × 20 mm`) for Milestone 2+ testing:
/// - Outer `90 × 70 × 20 mm` body with a `45°` chamfer (`10 × 10 mm`) and `R10` corner fillet,
/// - **4x `Ø6.0 mm` (`R = 3.0 mm`) mounting through-holes** (`Z = 0..20 mm`),
/// - **2x coplanar recessed pockets** whose floors sit at `Z = 12.0 mm` (`8 mm` deep from `Z = 20 mm`),
/// - **1x `Ø10.0 mm` (`R = 5.0 mm`) blind cylindrical bore** inside Pocket 1 (`Z = 5.0..12.0 mm`),
/// - Optional `tilted = true` rigid `30°` X-tilt + `25°` Z-rotation so WCS alignment and tight
///   oriented bounding-box math are exercised on non-axis-aligned STEP geometry.
pub fn sample_multi_feature_bracket_step(
    tilted: bool,
) -> std::result::Result<String, SampleStepError> {
    // 1. Outer base profile (CCW on Z = 0): 90 x 70 mm with chamfer + R10 fillet
    let v0 = builder::vertex(Point3::new(0.0, 0.0, 0.0));
    let v1 = builder::vertex(Point3::new(80.0, 0.0, 0.0));
    let v2 = builder::vertex(Point3::new(90.0, 10.0, 0.0));
    let v3 = builder::vertex(Point3::new(90.0, 70.0, 0.0));
    let v4 = builder::vertex(Point3::new(10.0, 70.0, 0.0));
    let v5 = builder::vertex(Point3::new(0.0, 60.0, 0.0));

    let inv_sqrt2 = std::f64::consts::FRAC_1_SQRT_2;
    let fillet_mid = Point3::new(10.0 - 10.0 * inv_sqrt2, 60.0 + 10.0 * inv_sqrt2, 0.0);
    let fillet_edge = builder::try_circle_arc(&v4, &v5, fillet_mid)
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;

    let outer_wire: Wire = vec![
        builder::line(&v0, &v1),
        builder::line(&v1, &v2),
        builder::line(&v2, &v3),
        builder::line(&v3, &v4),
        fillet_edge,
        builder::line(&v5, &v0),
    ]
    .into();

    // 4x Ø6.0 mm (R = 3.0 mm) through-holes
    let hole_centers = [(12.0, 12.0), (76.0, 16.0), (78.0, 58.0), (14.0, 56.0)];
    let mut bottom_wires = vec![outer_wire];
    for (cx, cy) in hole_centers {
        let seed = builder::vertex(Point3::new(cx + 3.0, cy, 0.0));
        let wire_ccw: Wire = builder::revolve(
            &seed,
            Point3::new(cx, cy, 0.0),
            Vector3::unit_z(),
            SweepAngle::Closed,
            2,
        );
        bottom_wires.push(wire_ccw.inverse());
    }

    let base_face_z0 = builder::try_attach_plane(bottom_wires)
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let base_solid = builder::extrude(&base_face_z0, Vector3::new(0.0, 0.0, 20.0));

    // Extract faces of the extruded 20mm solid and replace the top Z=20 face with a face
    // containing two recessed cavities: Pocket A ([24..52] x [18..52]) and Pocket B ([58..70] x [26..46]).
    let base_shell = base_solid
        .into_boundaries()
        .into_iter()
        .next()
        .ok_or_else(|| SampleStepError::Modeling("missing extruded shell".to_string()))?;

    let mut shell_faces: Vec<Face> = base_shell.into_iter().collect();
    // In `builder::extrude`, the last face is the translated top cap at Z = +20 (outward normal +Z).
    let old_top_face = shell_faces
        .pop()
        .ok_or_else(|| SampleStepError::Modeling("empty shell".to_string()))?;
    let mut top_wires: Vec<Wire> = old_top_face.boundaries();

    // Helper: build a recessed rectangular pocket from Z_floor up to Z_top = 20.0, optionally containing
    // a blind cylindrical bore from Z_bore_bottom up to Z_floor!
    // Creating the plug via `extrude(floor_face, +Z)` guarantees exact half-edge sharing:
    // - plug bottom face (normal -Z) inverted -> floor face at Z_floor (normal +Z)!
    // - plug side faces (normal outward from plug) inverted -> pocket side walls (normal inward into cavity)!
    // - plug top face boundary wire[0] inverted -> inner hole wire on `top_wires` at Z = 20!
    let rect_wire_ccw = |x0: f64, y0: f64, x1: f64, y1: f64, z: f64| -> Wire {
        let p0 = builder::vertex(Point3::new(x0, y0, z));
        let p1 = builder::vertex(Point3::new(x1, y0, z));
        let p2 = builder::vertex(Point3::new(x1, y1, z));
        let p3 = builder::vertex(Point3::new(x0, y1, z));
        vec![
            builder::line(&p0, &p1),
            builder::line(&p1, &p2),
            builder::line(&p2, &p3),
            builder::line(&p3, &p0),
        ]
        .into()
    };

    // Blind bore inside Pocket A: Ø10.0 mm (R = 5.0 mm) centered at (38, 35) from Z = 5.0 up to Z = 12.0
    let blind_seed = builder::vertex(Point3::new(38.0 + 5.0, 35.0, 5.0));
    let blind_wire_z5: Wire = builder::revolve(
        &blind_seed,
        Point3::new(38.0, 35.0, 5.0),
        Vector3::unit_z(),
        SweepAngle::Closed,
        2,
    );
    let blind_floor_z5 = builder::try_attach_plane(vec![blind_wire_z5])
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let blind_plug = builder::extrude(&blind_floor_z5, Vector3::new(0.0, 0.0, 7.0)); // Z = 5 -> 12
    let mut blind_faces: Vec<Face> = blind_plug
        .into_boundaries()
        .into_iter()
        .next()
        .ok_or_else(|| SampleStepError::Modeling("missing blind plug shell".to_string()))?
        .into_iter()
        .collect();
    let blind_top_cap_z12 = blind_faces
        .pop()
        .ok_or_else(|| SampleStepError::Modeling("empty blind plug".to_string()))?;
    let blind_mouth_wire_z12 = blind_top_cap_z12
        .boundaries()
        .into_iter()
        .next()
        .ok_or_else(|| SampleStepError::Modeling("missing blind mouth wire".to_string()))?;
    for f in blind_faces {
        // Bottom cap (-Z) inverted -> +Z blind floor at Z=5; side walls inverted -> internal cylinder!
        shell_faces.push(f.inverse());
    }

    // Pocket A at Z = 12.0..20.0 (with inner hole `blind_mouth_wire_z12.inverse()` on its Z=12 floor!)
    let pkt_a_outer_z12 = rect_wire_ccw(24.0, 18.0, 52.0, 52.0, 12.0);
    let pkt_a_floor_proto =
        builder::try_attach_plane(vec![pkt_a_outer_z12, blind_mouth_wire_z12.inverse()])
            .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    // Extrude only the outer boundary of Pocket A from Z=12 to Z=20 so the blind bore stays at Z=5..12:
    // To share the exact `pkt_a_outer_z12` edges with `pkt_a_floor_proto`, grab `pkt_a_floor_proto.boundaries()[0]`!
    let shared_pkt_a_wire_z12 = pkt_a_floor_proto.boundaries()[0].clone();
    shell_faces.push(pkt_a_floor_proto); // +Z floor at Z = 12.0

    let pkt_a_cap_only = builder::try_attach_plane(vec![shared_pkt_a_wire_z12])
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let pkt_a_plug = builder::extrude(&pkt_a_cap_only, Vector3::new(0.0, 0.0, 8.0));
    let mut pkt_a_plug_faces: Vec<Face> = pkt_a_plug
        .into_boundaries()
        .into_iter()
        .next()
        .ok_or_else(|| SampleStepError::Modeling("missing pocket A plug".to_string()))?
        .into_iter()
        .collect();
    let pkt_a_top_z20 = pkt_a_plug_faces
        .pop()
        .ok_or_else(|| SampleStepError::Modeling("empty pocket A plug".to_string()))?;
    // Skip `pkt_a_plug_faces[0]` (its solid bottom cap, since we already pushed `pkt_a_floor_proto` with the blind hole!),
    // and push the inverted side walls!
    for wall_f in pkt_a_plug_faces.into_iter().skip(1) {
        shell_faces.push(wall_f.inverse());
    }
    let pkt_a_mouth_z20 = pkt_a_top_z20
        .boundaries()
        .into_iter()
        .next()
        .ok_or_else(|| SampleStepError::Modeling("missing pocket A top wire".to_string()))?;
    top_wires.push(pkt_a_mouth_z20.inverse());

    // Pocket B at Z = 12.0..20.0 (coplanar floor at Z = 12.0!)
    let pkt_b_wire_z12 = rect_wire_ccw(58.0, 24.0, 72.0, 46.0, 12.0);
    let pkt_b_floor_proto = builder::try_attach_plane(vec![pkt_b_wire_z12])
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let pkt_b_plug = builder::extrude(&pkt_b_floor_proto, Vector3::new(0.0, 0.0, 8.0));
    let mut pkt_b_faces: Vec<Face> = pkt_b_plug
        .into_boundaries()
        .into_iter()
        .next()
        .ok_or_else(|| SampleStepError::Modeling("missing pocket B plug".to_string()))?
        .into_iter()
        .collect();
    let pkt_b_top_z20 = pkt_b_faces
        .pop()
        .ok_or_else(|| SampleStepError::Modeling("empty pocket B plug".to_string()))?;
    for f in pkt_b_faces {
        shell_faces.push(f.inverse());
    }
    let pkt_b_mouth_z20 = pkt_b_top_z20
        .boundaries()
        .into_iter()
        .next()
        .ok_or_else(|| SampleStepError::Modeling("missing pocket B top wire".to_string()))?;
    top_wires.push(pkt_b_mouth_z20.inverse());

    // Attach new top Z=20 face with all 4 through-holes + both pocket openings
    let new_top_face = builder::try_attach_plane(top_wires)
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    shell_faces.push(new_top_face);

    let full_shell: Shell = shell_faces.into();
    let mut solid = Solid::try_new(vec![full_shell])
        .map_err(|e| SampleStepError::Modeling(format!("non-manifold bracket solid: {e}")))?;

    if tilted {
        solid = builder::rotated(
            &solid,
            Point3::new(0.0, 0.0, 0.0),
            Vector3::unit_x(),
            Rad(30.0_f64.to_radians()),
        );
        solid = builder::rotated(
            &solid,
            Point3::new(0.0, 0.0, 0.0),
            Vector3::unit_z(),
            Rad(25.0_f64.to_radians()),
        );
    }

    let compressed = solid.compress();
    let models: StepModels<'_, _, _, _> = [&compressed].into_iter().collect();
    let header = StepHeaderDescriptor {
        file_name: if tilted {
            "sample_multi_feature_tilted.step".to_string()
        } else {
            "sample_multi_feature_bracket.step".to_string()
        },
        time_stamp: "2026-11-17T00:00:00".to_string(),
        authors: vec!["ChipCutter".to_string()],
        organization: vec!["ChipCutter CAM".to_string()],
        organization_system: "monstertruck".to_string(),
        authorization: String::new(),
    };

    Ok(CompleteStepDisplay::new(models, header).to_string())
}

fn build_plate_step_with_scale(
    s: f64,
    prefix: SiPrefix,
    rewrite_header_to_inch: bool,
) -> std::result::Result<String, SampleStepError> {
    // Outer XY profile (CCW) on Z = 0:
    // (0, 0) -> (70, 0) -> [45° chamfer] (80, 10) -> (80, 60) -> (10, 60)
    // -> [R10 convex corner arc centered at (10, 50) passing through (10 - 10/sqrt(2), 50 + 10/sqrt(2))]
    // -> (0, 50) -> (0, 0)
    let v0 = builder::vertex(Point3::new(0.0 * s, 0.0 * s, 0.0));
    let v1 = builder::vertex(Point3::new(70.0 * s, 0.0 * s, 0.0));
    let v2 = builder::vertex(Point3::new(80.0 * s, 10.0 * s, 0.0));
    let v3 = builder::vertex(Point3::new(80.0 * s, 60.0 * s, 0.0));
    let v4 = builder::vertex(Point3::new(10.0 * s, 60.0 * s, 0.0));
    let v5 = builder::vertex(Point3::new(0.0 * s, 50.0 * s, 0.0));

    let inv_sqrt2 = std::f64::consts::FRAC_1_SQRT_2;
    let fillet_mid = Point3::new(
        (10.0 - 10.0 * inv_sqrt2) * s,
        (50.0 + 10.0 * inv_sqrt2) * s,
        0.0,
    );
    let fillet_edge = builder::try_circle_arc(&v4, &v5, fillet_mid)
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;

    let outer_wire: Wire = vec![
        builder::line(&v0, &v1),
        builder::line(&v1, &v2),
        builder::line(&v2, &v3),
        builder::line(&v3, &v4),
        fillet_edge,
        builder::line(&v5, &v0),
    ]
    .into();

    // Inner bore #1: Ø16 mm bearing hole centered at (28, 30) (CW orientation for inner hole wire)
    let bore1_seed = builder::vertex(Point3::new((28.0 + 8.0) * s, 30.0 * s, 0.0));
    let bore1_ccw: Wire = builder::revolve(
        &bore1_seed,
        Point3::new(28.0 * s, 30.0 * s, 0.0),
        Vector3::unit_z(),
        SweepAngle::Closed,
        2,
    );

    // Inner bore #2: Ø8 mm mounting hole centered at (60, 42)
    let bore2_seed = builder::vertex(Point3::new((60.0 + 4.0) * s, 42.0 * s, 0.0));
    let bore2_ccw: Wire = builder::revolve(
        &bore2_seed,
        Point3::new(60.0 * s, 42.0 * s, 0.0),
        Vector3::unit_z(),
        SweepAngle::Closed,
        2,
    );

    let bottom_face: Face =
        builder::try_attach_plane(vec![outer_wire, bore1_ccw.inverse(), bore2_ccw.inverse()])
            .map_err(|e| SampleStepError::Modeling(e.to_string()))?;

    let plate_solid: Solid = builder::extrude(&bottom_face, Vector3::new(0.0, 0.0, 16.0 * s));
    let compressed = plate_solid.compress();

    let models: StepModels<'_, _, _, _> = [&compressed].into_iter().collect();
    let _ = prefix;
    let header = StepHeaderDescriptor {
        file_name: "sample_machined_plate.step".to_string(),
        time_stamp: "2026-11-17T00:00:00".to_string(),
        authors: vec!["ChipCutter".to_string()],
        organization: vec!["ChipCutter CAM".to_string()],
        organization_system: "monstertruck".to_string(),
        authorization: String::new(),
    };
    let _ = StepMeasurementContext::default();
    let mut step_text = CompleteStepDisplay::new(models, header).to_string();

    if rewrite_header_to_inch {
        step_text = step_text.replace(
            "#12 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );",
            "#12 = ( CONVERSION_BASED_UNIT('INCH', #1200) LENGTH_UNIT() NAMED_UNIT(#1201) );\n#1200 = LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4), #1202);\n#1201 = DIMENSIONAL_EXPONENTS(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);\n#1202 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );",
        );
    }

    Ok(step_text)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DVec3, Ray3, approx_eq};

    use super::*;
    use crate::{
        CadKernel, CadPicker, MonstertruckKernel, PickFilter, PickTarget, StepUnitKind, SurfaceKind,
    };

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
                assert!(
                    matches!(face.surface, SurfaceKind::Plane { normal, .. } if normal.z > 0.99)
                );
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
        use crate::topology::{
            TopologyGraph, analyze_cylindrical_hole, find_coplanar_faces, find_same_diameter_holes,
            group_coaxial_holes, highest_adjacent_wall_top,
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
        let wall_top = highest_adjacent_wall_top(&doc, z12_floor, DVec3::Z).unwrap();
        assert!(approx_eq(wall_top, 20.0, 0.05));

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
}
