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
}
