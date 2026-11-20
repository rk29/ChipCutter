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

/// Builds a comprehensive Milestone 3 CAM verification block (`100 × 80 × 22 mm` max envelope):
/// - Base slab `[0..100] × [0..80]` (`Z = 0..14 mm`) with `R10` corner fillet,
/// - **Open-edge shoulder/step shelf** `[76..100] × [0..80]` at `Z = 14 mm` (open on 3 air edges,
///   bounded on `-X` by a `6 mm` vertical step wall rising to the `Z = 20 mm` main plateau),
/// - **4x `Ø6.0 mm` (`R = 3.0 mm`) mounting through-holes** (`Z = 0..14 mm`) on the right step shelf,
/// - **Closed pocket** `[12..46] × [16..64]` (`Z = 12..20 mm`) containing a **raised `Ø10.0 mm`
///   (`R = 5.0 mm`) central cylindrical island boss** at `(29, 40)` (`Z = 12..20 mm`),
/// - **Wide blind counterbore** `Ø18.0 mm` (`R = 9.0 mm`) at `(60, 22)` (`Z = 10..20 mm`),
/// - **Modeled `45° × 2.0 mm` beveled boss pad** `[50..70] × [44..72]` rising from `Z = 20 mm` to
///   `[52..68] × [46..70]` at `Z = 22 mm` with exact `45°` sloped planar chamfer walls.
pub fn sample_cam_torture_block_step() -> std::result::Result<String, SampleStepError> {
    // 1. Base Z = 0 outer wire with split vertices at X = 76 so the Z = 14 step and Z = 20 plateau
    //    share exact topological vertices and edges!
    let v0 = builder::vertex(Point3::new(0.0, 0.0, 0.0));
    let v_f76 = builder::vertex(Point3::new(76.0, 0.0, 0.0));
    let v1 = builder::vertex(Point3::new(100.0, 0.0, 0.0));
    let v2 = builder::vertex(Point3::new(100.0, 80.0, 0.0));
    let v_b76 = builder::vertex(Point3::new(76.0, 80.0, 0.0));
    let v3 = builder::vertex(Point3::new(10.0, 80.0, 0.0));
    let v4 = builder::vertex(Point3::new(0.0, 70.0, 0.0));

    let inv_sqrt2 = std::f64::consts::FRAC_1_SQRT_2;
    let fillet_mid = Point3::new(10.0 - 10.0 * inv_sqrt2, 70.0 + 10.0 * inv_sqrt2, 0.0);
    let fillet_edge = builder::try_circle_arc(&v3, &v4, fillet_mid)
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;

    let base_outer_z0: Wire = vec![
        builder::line(&v0, &v_f76), // [0] front-left (0..76, 0)
        builder::line(&v_f76, &v1), // [1] front-right step (76..100, 0)
        builder::line(&v1, &v2),    // [2] right step (100, 0..80)
        builder::line(&v2, &v_b76), // [3] back-right step (100..76, 80)
        builder::line(&v_b76, &v3), // [4] back-left (76..10, 80)
        fillet_edge,                // [5] R10 fillet (10,80 -> 0,70)
        builder::line(&v4, &v0),    // [6] left (0, 70..0)
    ]
    .into();

    // 4x Ø6.0 mm (R = 3.0 mm) through-holes on the right step shelf (X = 88)
    let step_holes = [(88.0, 14.0), (88.0, 32.0), (88.0, 48.0), (88.0, 66.0)];
    let mut base_wires_z0 = vec![base_outer_z0];
    for (cx, cy) in step_holes {
        let seed = builder::vertex(Point3::new(cx + 3.0, cy, 0.0));
        let w_ccw: Wire = builder::revolve(
            &seed,
            Point3::new(cx, cy, 0.0),
            Vector3::unit_z(),
            SweepAngle::Closed,
            2,
        );
        base_wires_z0.push(w_ccw.inverse());
    }

    let base_face_z0 = builder::try_attach_plane(base_wires_z0)
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let base_solid_z14 = builder::extrude(&base_face_z0, Vector3::new(0.0, 0.0, 14.0));
    let mut shell_faces: Vec<Face> = base_solid_z14
        .into_boundaries()
        .into_iter()
        .next()
        .ok_or_else(|| SampleStepError::Modeling("missing base z14 shell".to_string()))?
        .into_iter()
        .collect();

    // Pop Z = 14 top cap and split along X = 76 into:
    //  (a) Right open-step floor face at Z = 14 ([76..100] x [0..80] with the 4 through-hole mouths)
    //  (b) Left plateau base wire ([0..76] x [0..80]) extruded up from Z = 14 -> Z = 20!
    let old_z14_cap = shell_faces
        .pop()
        .ok_or_else(|| SampleStepError::Modeling("empty z14 shell".to_string()))?;
    let mut z14_wires = old_z14_cap.boundaries();
    let z14_outer: Vec<Edge> = z14_wires.remove(0).into_iter().collect();
    if z14_outer.len() != 7 {
        return Err(SampleStepError::Modeling(format!(
            "expected 7 outer edges on Z=14 cap, got {}",
            z14_outer.len()
        )));
    }
    let v_f76_z14 = z14_outer[0].back().clone();
    let v_b76_z14 = z14_outer[3].back().clone();
    let e_div_z14 = builder::line(&v_f76_z14, &v_b76_z14);

    // (a) Right open-edge step shelf at Z = 14
    let step_outer_wire_z14: Wire = vec![
        z14_outer[1].clone(),
        z14_outer[2].clone(),
        z14_outer[3].clone(),
        e_div_z14.inverse(),
    ]
    .into();
    let mut step_wires_z14 = vec![step_outer_wire_z14];
    step_wires_z14.extend(z14_wires); // the 4 through-hole inner wires at Z = 14
    let step_floor_face_z14 = builder::try_attach_plane(step_wires_z14)
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    shell_faces.push(step_floor_face_z14);

    // (b) Extrude Left plateau ([0..76] x [0..80]) from Z = 14 -> Z = 20
    let plateau_wire_z14: Wire = vec![
        z14_outer[0].clone(),
        e_div_z14,
        z14_outer[4].clone(),
        z14_outer[5].clone(),
        z14_outer[6].clone(),
    ]
    .into();
    let plateau_base_z14 = builder::try_attach_plane(vec![plateau_wire_z14])
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let plateau_solid = builder::extrude(&plateau_base_z14, Vector3::new(0.0, 0.0, 6.0)); // Z = 14 -> 20
    let mut plateau_faces: Vec<Face> = plateau_solid
        .into_boundaries()
        .into_iter()
        .next()
        .ok_or_else(|| SampleStepError::Modeling("missing plateau shell".to_string()))?
        .into_iter()
        .collect();
    let plateau_top_z20 = plateau_faces
        .pop()
        .ok_or_else(|| SampleStepError::Modeling("empty plateau shell".to_string()))?;
    // Skip `plateau_faces[0]` (its internal Z=14 bottom cap) and add its 5 vertical walls (including the X=76 step wall!)
    for wf in plateau_faces.into_iter().skip(1) {
        shell_faces.push(wf);
    }
    let mut top_z20_wires = plateau_top_z20.boundaries();

    // 2. Closed Pocket [12..46] x [16..64] at Z = 12..20 with Raised Ø10 mm (R = 5) Island Boss at (29, 40)
    let p_a = builder::vertex(Point3::new(12.0, 16.0, 12.0));
    let p_b = builder::vertex(Point3::new(46.0, 16.0, 12.0));
    let p_c = builder::vertex(Point3::new(46.0, 64.0, 12.0));
    let p_d = builder::vertex(Point3::new(12.0, 64.0, 12.0));
    let pkt_outer_z12: Wire = vec![
        builder::line(&p_a, &p_b),
        builder::line(&p_b, &p_c),
        builder::line(&p_c, &p_d),
        builder::line(&p_d, &p_a),
    ]
    .into();
    let island_seed_z12 = builder::vertex(Point3::new(29.0 + 5.0, 40.0, 12.0));
    let island_wire_z12: Wire = builder::revolve(
        &island_seed_z12,
        Point3::new(29.0, 40.0, 12.0),
        Vector3::unit_z(),
        SweepAngle::Closed,
        2,
    );
    let annular_floor_proto =
        builder::try_attach_plane(vec![pkt_outer_z12, island_wire_z12.inverse()])
            .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let annular_plug = builder::extrude(&annular_floor_proto, Vector3::new(0.0, 0.0, 8.0)); // Z = 12 -> 20
    let mut annular_faces: Vec<Face> = annular_plug
        .into_boundaries()
        .into_iter()
        .next()
        .ok_or_else(|| SampleStepError::Modeling("missing annular pocket plug".to_string()))?
        .into_iter()
        .collect();
    let annular_top_z20 = annular_faces
        .pop()
        .ok_or_else(|| SampleStepError::Modeling("empty annular plug".to_string()))?;
    let mut annular_top_wires = annular_top_z20.boundaries();
    if annular_top_wires.len() != 2 {
        return Err(SampleStepError::Modeling(
            "expected outer + island wire on annular pocket plug".to_string(),
        ));
    }
    let pkt_mouth_z20 = annular_top_wires.remove(0);
    let island_top_cw_z20 = annular_top_wires.remove(0);
    // Invert bottom + outer walls + inner island walls so normals point into solid / out of boss!
    for f in annular_faces {
        shell_faces.push(f.inverse());
    }
    // Cap top of the central Ø10mm cylindrical boss at Z = 20 (+Z outward normal)
    let island_top_cap_z20 = builder::try_attach_plane(vec![island_top_cw_z20.inverse()])
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    shell_faces.push(island_top_cap_z20);
    top_z20_wires.push(pkt_mouth_z20.inverse());

    // 3. Wide Blind Counterbore Ø18.0 mm (R = 9.0 mm) at (60, 22), Z = 10..20
    let cb_seed = builder::vertex(Point3::new(60.0 + 9.0, 22.0, 10.0));
    let cb_wire_z10: Wire = builder::revolve(
        &cb_seed,
        Point3::new(60.0, 22.0, 10.0),
        Vector3::unit_z(),
        SweepAngle::Closed,
        2,
    );
    let cb_floor_proto = builder::try_attach_plane(vec![cb_wire_z10])
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let cb_plug = builder::extrude(&cb_floor_proto, Vector3::new(0.0, 0.0, 10.0)); // Z = 10 -> 20
    let mut cb_faces: Vec<Face> = cb_plug
        .into_boundaries()
        .into_iter()
        .next()
        .ok_or_else(|| SampleStepError::Modeling("missing counterbore plug".to_string()))?
        .into_iter()
        .collect();
    let cb_top_z20 = cb_faces
        .pop()
        .ok_or_else(|| SampleStepError::Modeling("empty counterbore plug".to_string()))?;
    let cb_mouth_z20 = cb_top_z20
        .boundaries()
        .into_iter()
        .next()
        .ok_or_else(|| SampleStepError::Modeling("missing counterbore mouth".to_string()))?;
    for f in cb_faces {
        shell_faces.push(f.inverse());
    }
    top_z20_wires.push(cb_mouth_z20.inverse());

    // 4. Modeled 45° x 2.0 mm Beveled Boss Pad:
    //    Base [50..70] x [44..72] at Z = 20 -> Top [52..68] x [46..70] at Z = 22
    let b0 = builder::vertex(Point3::new(50.0, 44.0, 20.0));
    let b1 = builder::vertex(Point3::new(70.0, 44.0, 20.0));
    let b2 = builder::vertex(Point3::new(70.0, 72.0, 20.0));
    let b3 = builder::vertex(Point3::new(50.0, 72.0, 20.0));

    let t0 = builder::vertex(Point3::new(52.0, 46.0, 22.0));
    let t1 = builder::vertex(Point3::new(68.0, 46.0, 22.0));
    let t2 = builder::vertex(Point3::new(68.0, 70.0, 22.0));
    let t3 = builder::vertex(Point3::new(52.0, 70.0, 22.0));

    let eb0 = builder::line(&b0, &b1);
    let eb1 = builder::line(&b1, &b2);
    let eb2 = builder::line(&b2, &b3);
    let eb3 = builder::line(&b3, &b0);

    let et0 = builder::line(&t0, &t1);
    let et1 = builder::line(&t1, &t2);
    let et2 = builder::line(&t2, &t3);
    let et3 = builder::line(&t3, &t0);

    let ed0 = builder::line(&b0, &t0);
    let ed1 = builder::line(&b1, &t1);
    let ed2 = builder::line(&b2, &t2);
    let ed3 = builder::line(&b3, &t3);

    // 4 planar 45° chamfer side faces (CCW from outside -> outward normals tilted 45° up!)
    let chamf_front = builder::try_attach_plane(vec![
        vec![eb0.clone(), ed1.clone(), et0.inverse(), ed0.inverse()].into(),
    ])
    .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let chamf_right = builder::try_attach_plane(vec![
        vec![eb1.clone(), ed2.clone(), et1.inverse(), ed1.inverse()].into(),
    ])
    .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let chamf_back = builder::try_attach_plane(vec![
        vec![eb2.clone(), ed3.clone(), et2.inverse(), ed2.inverse()].into(),
    ])
    .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let chamf_left = builder::try_attach_plane(vec![
        vec![eb3.clone(), ed0.clone(), et3.inverse(), ed3.inverse()].into(),
    ])
    .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let boss_top_z22 = builder::try_attach_plane(vec![vec![et0, et1, et2, et3].into()])
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;

    shell_faces.push(chamf_front);
    shell_faces.push(chamf_right);
    shell_faces.push(chamf_back);
    shell_faces.push(chamf_left);
    shell_faces.push(boss_top_z22);

    let boss_footprint_ccw: Wire = vec![eb0, eb1, eb2, eb3].into();
    top_z20_wires.push(boss_footprint_ccw.inverse());

    // Finalize main plateau Z = 20 top face
    let final_top_z20 = builder::try_attach_plane(top_z20_wires)
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    shell_faces.push(final_top_z20);

    let full_shell: Shell = shell_faces.into();
    let solid = Solid::try_new(vec![full_shell])
        .map_err(|e| SampleStepError::Modeling(format!("non-manifold torture block: {e}")))?;

    let compressed = solid.compress();
    let models: StepModels<'_, _, _, _> = [&compressed].into_iter().collect();
    let header = StepHeaderDescriptor {
        file_name: "sample_cam_torture_block.step".to_string(),
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

/// Builds a `"v1 -> v2"` revised STEP pair (`09-persistence-cli.md` §2 & `02-cad-selection.md` §CAD Reload)
/// specifically designed to verify `GeometricSignature` associative re-matching:
/// - **v1 (`90 × 70 × 20 mm`):**
///   - Blind rectangular pocket `[14..46] × [16..54]` at `Z = 12.0 mm`
///   - `Ø14.0 mm` (`R = 7.0 mm`) precision bore through-hole at `(68.0, 24.0)`
///   - `2x Ø6.0 mm` (`R = 3.0 mm`) mounting through-holes at `(68.0, 48.0)` and `(68.0, 60.0)`
/// - **v2 (`92 × 70 × 20 mm`, permuted face/edge topology order):**
///   - Pocket resized & deepened to `[12..48] × [15..55]` at `Z = 11.0 mm`
///   - `Ø14.0 mm` bore shifted to `(71.0, 26.0)`
///   - First `Ø6.0 mm` mounting hole shifted to `(70.0, 47.0)`
///   - Second `Ø6.0 mm` mounting hole at `(68.0, 60.0)` **deleted** (triggers `NeedsRepick`!)
pub fn sample_revised_part_pair_step() -> std::result::Result<(String, String), SampleStepError> {
    let v1 = build_revision_block_step(
        "revision_v1.step",
        90.0,
        (14.0, 16.0, 46.0, 54.0, 12.0),
        &[(68.0, 24.0, 7.0), (68.0, 48.0, 3.0), (68.0, 60.0, 3.0)],
        false,
    )?;
    let v2 = build_revision_block_step(
        "revision_v2.step",
        92.0,
        (12.0, 15.0, 48.0, 55.0, 11.0),
        &[(71.0, 26.0, 7.0), (70.0, 47.0, 3.0)],
        true,
    )?;
    Ok((v1, v2))
}

/// Builds a revised STEP variant where `v1`'s single `Ø14.0 mm` bore at `(68.0, 24.0)` is replaced
/// by two symmetric twin `Ø14.0 mm` bores equidistant at `(68.0, 16.0)` and `(68.0, 32.0)` to test
/// deterministic ambiguity detection (`RematchOutcome::Ambiguous`).
pub fn sample_ambiguous_twin_hole_step() -> std::result::Result<String, SampleStepError> {
    build_revision_block_step(
        "revision_ambiguous_twins.step",
        90.0,
        (14.0, 16.0, 46.0, 54.0, 12.0),
        &[
            (68.0, 16.0, 7.0),
            (68.0, 32.0, 7.0),
            (68.0, 48.0, 3.0),
            (68.0, 60.0, 3.0),
        ],
        true,
    )
}

fn build_revision_block_step(
    file_name: &str,
    width_x: f64,
    pocket_x0_y0_x1_y1_z: (f64, f64, f64, f64, f64),
    through_holes_xyr: &[(f64, f64, f64)],
    permute_shell_order: bool,
) -> std::result::Result<String, SampleStepError> {
    let v0 = builder::vertex(Point3::new(0.0, 0.0, 0.0));
    let v1 = builder::vertex(Point3::new(width_x - 10.0, 0.0, 0.0));
    let v2 = builder::vertex(Point3::new(width_x, 10.0, 0.0));
    let v3 = builder::vertex(Point3::new(width_x, 70.0, 0.0));
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

    let mut bottom_wires = vec![outer_wire];
    for &(cx, cy, r) in through_holes_xyr {
        let seed = builder::vertex(Point3::new(cx + r, cy, 0.0));
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
    let mut base_faces: Vec<Face> = base_solid
        .into_boundaries()
        .into_iter()
        .next()
        .ok_or_else(|| SampleStepError::Modeling("missing extruded shell".to_string()))?
        .into_iter()
        .collect();

    let old_top_face = base_faces
        .pop()
        .ok_or_else(|| SampleStepError::Modeling("empty shell".to_string()))?;
    let mut top_wires: Vec<Wire> = old_top_face.boundaries();

    let (px0, py0, px1, py1, pz) = pocket_x0_y0_x1_y1_z;
    let pa = builder::vertex(Point3::new(px0, py0, pz));
    let pb = builder::vertex(Point3::new(px1, py0, pz));
    let pc = builder::vertex(Point3::new(px1, py1, pz));
    let pd = builder::vertex(Point3::new(px0, py1, pz));
    let pkt_wire: Wire = vec![
        builder::line(&pa, &pb),
        builder::line(&pb, &pc),
        builder::line(&pc, &pd),
        builder::line(&pd, &pa),
    ]
    .into();
    let pkt_floor_proto = builder::try_attach_plane(vec![pkt_wire])
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let pkt_plug = builder::extrude(&pkt_floor_proto, Vector3::new(0.0, 0.0, 20.0 - pz));
    let mut pkt_faces: Vec<Face> = pkt_plug
        .into_boundaries()
        .into_iter()
        .next()
        .ok_or_else(|| SampleStepError::Modeling("missing pocket plug".to_string()))?
        .into_iter()
        .collect();
    let pkt_top = pkt_faces
        .pop()
        .ok_or_else(|| SampleStepError::Modeling("empty pocket plug".to_string()))?;
    let pkt_mouth = pkt_top
        .boundaries()
        .into_iter()
        .next()
        .ok_or_else(|| SampleStepError::Modeling("missing pocket mouth".to_string()))?;
    top_wires.push(pkt_mouth.inverse());

    let new_top_face = builder::try_attach_plane(top_wires)
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;

    let mut all_faces: Vec<Face> = Vec::new();
    if permute_shell_order {
        // Deliberately emit pocket cavity faces & top face BEFORE base walls so every FaceId/LoopId/EdgeId shifts!
        all_faces.push(new_top_face);
        for pf in pkt_faces {
            all_faces.push(pf.inverse());
        }
        all_faces.extend(base_faces);
    } else {
        all_faces.extend(base_faces);
        for pf in pkt_faces {
            all_faces.push(pf.inverse());
        }
        all_faces.push(new_top_face);
    }

    let shell: Shell = all_faces.into();
    let solid =
        Solid::try_new(vec![shell]).map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let compressed = solid.compress();
    let models: StepModels<'_, _, _, _> = [&compressed].into_iter().collect();
    let header = StepHeaderDescriptor {
        file_name: file_name.to_string(),
        time_stamp: "2026-11-18T00:00:00".to_string(),
        authors: vec!["ChipCutter".to_string()],
        organization: vec!["ChipCutter CAM".to_string()],
        organization_system: "monstertruck".to_string(),
        authorization: String::new(),
    };
    Ok(CompleteStepDisplay::new(models, header).to_string())
}

fn serialize_tutorial_solid(
    solid: Solid,
    file_name: &str,
) -> std::result::Result<String, SampleStepError> {
    let compressed = solid.compress();
    let models: StepModels<'_, _, _, _> = [&compressed].into_iter().collect();
    let header = StepHeaderDescriptor {
        file_name: file_name.to_string(),
        time_stamp: "2026-11-19T00:00:00".to_string(),
        authors: vec!["ChipCutter".to_string()],
        organization: vec!["ChipCutter CAM".to_string()],
        organization_system: "monstertruck".to_string(),
        authorization: String::new(),
    };
    Ok(CompleteStepDisplay::new(models, header).to_string())
}

/// Minimal Tutorial 01 (Facing) starter part: plain flat rectangular billet (`100 × 80 × 20 mm`,
/// 6 planar faces, zero holes/pockets/bosses).
pub fn tutorial_facing_billet_step() -> std::result::Result<String, SampleStepError> {
    let v0 = builder::vertex(Point3::new(0.0, 0.0, 0.0));
    let v1 = builder::vertex(Point3::new(100.0, 0.0, 0.0));
    let v2 = builder::vertex(Point3::new(100.0, 80.0, 0.0));
    let v3 = builder::vertex(Point3::new(0.0, 80.0, 0.0));
    let outer: Wire = vec![
        builder::line(&v0, &v1),
        builder::line(&v1, &v2),
        builder::line(&v2, &v3),
        builder::line(&v3, &v0),
    ]
    .into();
    let base = builder::try_attach_plane(vec![outer])
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let solid = builder::extrude(&base, Vector3::new(0.0, 0.0, 20.0));
    serialize_tutorial_solid(solid, "01-facing-billet.step")
}

/// Minimal Tutorial 02 (2D Contour & Holding Tabs) starter part: clean flat mounting plate
/// (`100 × 70 × 12 mm`) with one `R10 mm` rounded corner and one `10 × 10 mm` 45° chamfered corner
/// (zero pockets, holes, or bosses).
pub fn tutorial_contour_plate_step() -> std::result::Result<String, SampleStepError> {
    let v0 = builder::vertex(Point3::new(0.0, 0.0, 0.0));
    let v1 = builder::vertex(Point3::new(90.0, 0.0, 0.0));
    let v2 = builder::vertex(Point3::new(100.0, 10.0, 0.0));
    let v3 = builder::vertex(Point3::new(100.0, 70.0, 0.0));
    let v4 = builder::vertex(Point3::new(10.0, 70.0, 0.0));
    let v5 = builder::vertex(Point3::new(0.0, 60.0, 0.0));

    let inv_sqrt2 = std::f64::consts::FRAC_1_SQRT_2;
    let fillet_mid = Point3::new(10.0 - 10.0 * inv_sqrt2, 60.0 + 10.0 * inv_sqrt2, 0.0);
    let fillet_edge = builder::try_circle_arc(&v4, &v5, fillet_mid)
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;

    let outer: Wire = vec![
        builder::line(&v0, &v1),
        builder::line(&v1, &v2),
        builder::line(&v2, &v3),
        builder::line(&v3, &v4),
        fillet_edge,
        builder::line(&v5, &v0),
    ]
    .into();
    let base = builder::try_attach_plane(vec![outer])
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let solid = builder::extrude(&base, Vector3::new(0.0, 0.0, 12.0));
    serialize_tutorial_solid(solid, "02-contour-plate.step")
}

/// Minimal Tutorial 03 (2D Pocket) starter part (`100 × 80 × 20 mm`) containing ONLY:
/// - Closed pocket `[12..56] × [16..64]` (`Z = 12..20 mm`) with one raised `Ø14 mm` island boss at `(34, 40)`,
/// - Open-shoulder step shelf `[74..100] × [0..80]` at `Z = 14 mm` (open on 3 air sides).
pub fn tutorial_pocket_block_step() -> std::result::Result<String, SampleStepError> {
    let v0 = builder::vertex(Point3::new(0.0, 0.0, 0.0));
    let v_f74 = builder::vertex(Point3::new(74.0, 0.0, 0.0));
    let v1 = builder::vertex(Point3::new(100.0, 0.0, 0.0));
    let v2 = builder::vertex(Point3::new(100.0, 80.0, 0.0));
    let v_b74 = builder::vertex(Point3::new(74.0, 80.0, 0.0));
    let v3 = builder::vertex(Point3::new(0.0, 80.0, 0.0));

    let base_outer_z0: Wire = vec![
        builder::line(&v0, &v_f74), // [0] front-left (0..74, 0)
        builder::line(&v_f74, &v1), // [1] front-right step (74..100, 0)
        builder::line(&v1, &v2),    // [2] right step (100, 0..80)
        builder::line(&v2, &v_b74), // [3] back-right step (100..74, 80)
        builder::line(&v_b74, &v3), // [4] back-left (74..0, 80)
        builder::line(&v3, &v0),    // [5] left (0, 80..0)
    ]
    .into();

    let base_face_z0 = builder::try_attach_plane(vec![base_outer_z0])
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let base_solid_z14 = builder::extrude(&base_face_z0, Vector3::new(0.0, 0.0, 14.0));
    let mut shell_faces: Vec<Face> = base_solid_z14
        .into_boundaries()
        .into_iter()
        .next()
        .ok_or_else(|| SampleStepError::Modeling("missing pocket base shell".to_string()))?
        .into_iter()
        .collect();

    let old_z14_cap = shell_faces
        .pop()
        .ok_or_else(|| SampleStepError::Modeling("empty z14 shell".to_string()))?;
    let mut z14_wires = old_z14_cap.boundaries();
    let z14_outer: Vec<Edge> = z14_wires.remove(0).into_iter().collect();
    if z14_outer.len() != 6 {
        return Err(SampleStepError::Modeling(format!(
            "expected 6 outer edges on Z=14 cap, got {}",
            z14_outer.len()
        )));
    }
    let v_f74_z14 = z14_outer[0].back().clone();
    let v_b74_z14 = z14_outer[3].back().clone();
    let e_div_z14 = builder::line(&v_f74_z14, &v_b74_z14);

    // Right open-shoulder shelf at Z = 14 mm
    let step_outer_z14: Wire = vec![
        z14_outer[1].clone(),
        z14_outer[2].clone(),
        z14_outer[3].clone(),
        e_div_z14.inverse(),
    ]
    .into();
    let step_floor_z14 = builder::try_attach_plane(vec![step_outer_z14])
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    shell_faces.push(step_floor_z14);

    // Left main block extruded Z = 14 -> 20 mm
    let plateau_wire_z14: Wire = vec![
        z14_outer[0].clone(),
        e_div_z14,
        z14_outer[4].clone(),
        z14_outer[5].clone(),
    ]
    .into();
    let plateau_base_z14 = builder::try_attach_plane(vec![plateau_wire_z14])
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let plateau_solid = builder::extrude(&plateau_base_z14, Vector3::new(0.0, 0.0, 6.0));
    let mut plateau_faces: Vec<Face> = plateau_solid
        .into_boundaries()
        .into_iter()
        .next()
        .ok_or_else(|| SampleStepError::Modeling("missing plateau shell".to_string()))?
        .into_iter()
        .collect();
    let plateau_top_z20 = plateau_faces
        .pop()
        .ok_or_else(|| SampleStepError::Modeling("empty plateau shell".to_string()))?;
    for wf in plateau_faces.into_iter().skip(1) {
        shell_faces.push(wf);
    }
    let mut top_z20_wires = plateau_top_z20.boundaries();

    // Closed pocket [12..56] x [16..64] at Z = 12..20 with raised Ø14 mm (R = 7 mm) island at (34, 40)
    let p_a = builder::vertex(Point3::new(12.0, 16.0, 12.0));
    let p_b = builder::vertex(Point3::new(56.0, 16.0, 12.0));
    let p_c = builder::vertex(Point3::new(56.0, 64.0, 12.0));
    let p_d = builder::vertex(Point3::new(12.0, 64.0, 12.0));
    let pkt_outer_z12: Wire = vec![
        builder::line(&p_a, &p_b),
        builder::line(&p_b, &p_c),
        builder::line(&p_c, &p_d),
        builder::line(&p_d, &p_a),
    ]
    .into();
    let island_seed_z12 = builder::vertex(Point3::new(34.0 + 7.0, 40.0, 12.0));
    let island_wire_z12: Wire = builder::revolve(
        &island_seed_z12,
        Point3::new(34.0, 40.0, 12.0),
        Vector3::unit_z(),
        SweepAngle::Closed,
        2,
    );
    let annular_floor = builder::try_attach_plane(vec![pkt_outer_z12, island_wire_z12.inverse()])
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let annular_plug = builder::extrude(&annular_floor, Vector3::new(0.0, 0.0, 8.0));
    let mut annular_faces: Vec<Face> = annular_plug
        .into_boundaries()
        .into_iter()
        .next()
        .ok_or_else(|| SampleStepError::Modeling("missing annular pocket plug".to_string()))?
        .into_iter()
        .collect();
    let annular_top_z20 = annular_faces
        .pop()
        .ok_or_else(|| SampleStepError::Modeling("empty annular plug".to_string()))?;
    let mut annular_top_wires = annular_top_z20.boundaries();
    if annular_top_wires.len() != 2 {
        return Err(SampleStepError::Modeling(
            "expected outer + island wire on annular pocket plug".to_string(),
        ));
    }
    let pkt_mouth_z20 = annular_top_wires.remove(0);
    let island_top_cw_z20 = annular_top_wires.remove(0);
    for f in annular_faces {
        shell_faces.push(f.inverse());
    }
    let island_top_cap_z20 = builder::try_attach_plane(vec![island_top_cw_z20.inverse()])
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    shell_faces.push(island_top_cap_z20);
    top_z20_wires.push(pkt_mouth_z20.inverse());

    let new_top_z20 = builder::try_attach_plane(top_z20_wires)
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    shell_faces.push(new_top_z20);

    let shell: Shell = shell_faces.into();
    let solid =
        Solid::try_new(vec![shell]).map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    serialize_tutorial_solid(solid, "03-pocket-block.step")
}

/// Minimal Tutorial 04 (Drilling & Helical Bore) starter part (`100 × 70 × 16 mm`) containing ONLY:
/// - 4x `Ø6.0 mm` corner mounting through-holes (`Z = 0..16 mm`),
/// - 1x `Ø18.0 mm` central blind counterbore (`Z = 6..16 mm`).
pub fn tutorial_drill_bore_plate_step() -> std::result::Result<String, SampleStepError> {
    let v0 = builder::vertex(Point3::new(0.0, 0.0, 0.0));
    let v1 = builder::vertex(Point3::new(100.0, 0.0, 0.0));
    let v2 = builder::vertex(Point3::new(100.0, 70.0, 0.0));
    let v3 = builder::vertex(Point3::new(0.0, 70.0, 0.0));
    let outer_z0: Wire = vec![
        builder::line(&v0, &v1),
        builder::line(&v1, &v2),
        builder::line(&v2, &v3),
        builder::line(&v3, &v0),
    ]
    .into();

    let hole_centers = [(14.0, 14.0), (86.0, 14.0), (86.0, 56.0), (14.0, 56.0)];
    let mut bottom_wires = vec![outer_z0];
    for (cx, cy) in hole_centers {
        let seed = builder::vertex(Point3::new(cx + 3.0, cy, 0.0));
        let w_ccw: Wire = builder::revolve(
            &seed,
            Point3::new(cx, cy, 0.0),
            Vector3::unit_z(),
            SweepAngle::Closed,
            2,
        );
        bottom_wires.push(w_ccw.inverse());
    }

    let base_face_z0 = builder::try_attach_plane(bottom_wires)
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let base_solid = builder::extrude(&base_face_z0, Vector3::new(0.0, 0.0, 16.0));
    let mut shell_faces: Vec<Face> = base_solid
        .into_boundaries()
        .into_iter()
        .next()
        .ok_or_else(|| SampleStepError::Modeling("missing drill plate shell".to_string()))?
        .into_iter()
        .collect();
    let old_top_z16 = shell_faces
        .pop()
        .ok_or_else(|| SampleStepError::Modeling("empty drill plate shell".to_string()))?;
    let mut top_wires = old_top_z16.boundaries();

    // Central Ø18.0 mm (R = 9.0 mm) blind counterbore at (50, 35), Z = 6..16 mm
    let cb_seed = builder::vertex(Point3::new(50.0 + 9.0, 35.0, 6.0));
    let cb_wire_z6: Wire = builder::revolve(
        &cb_seed,
        Point3::new(50.0, 35.0, 6.0),
        Vector3::unit_z(),
        SweepAngle::Closed,
        2,
    );
    let cb_floor = builder::try_attach_plane(vec![cb_wire_z6])
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let cb_plug = builder::extrude(&cb_floor, Vector3::new(0.0, 0.0, 10.0));
    let mut cb_faces: Vec<Face> = cb_plug
        .into_boundaries()
        .into_iter()
        .next()
        .ok_or_else(|| SampleStepError::Modeling("missing counterbore plug".to_string()))?
        .into_iter()
        .collect();
    let cb_top_z16 = cb_faces
        .pop()
        .ok_or_else(|| SampleStepError::Modeling("empty counterbore plug".to_string()))?;
    let cb_mouth_z16 = cb_top_z16
        .boundaries()
        .into_iter()
        .next()
        .ok_or_else(|| SampleStepError::Modeling("missing counterbore mouth".to_string()))?;
    for f in cb_faces {
        shell_faces.push(f.inverse());
    }
    top_wires.push(cb_mouth_z16.inverse());

    let new_top_z16 = builder::try_attach_plane(top_wires)
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    shell_faces.push(new_top_z16);

    let shell: Shell = shell_faces.into();
    let solid =
        Solid::try_new(vec![shell]).map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    serialize_tutorial_solid(solid, "04-drill-bore-plate.step")
}

/// Minimal Tutorial 05 (2D Adaptive Clearing & 2.5D Rest Machining) starter part (`100 × 80 × 20 mm`)
/// containing ONLY one deep `10 mm` closed sharp-corner pocket `[14..86] × [14..66]` (`Z = 10..20 mm`)
/// with a raised `Ø16 mm` central boss at `(50, 40)`—ideal for `Ø10 mm` full-depth Adaptive roughing
/// followed by `Ø3 mm` Rest Machining in the 4 sharp corners.
pub fn tutorial_adaptive_pocket_step() -> std::result::Result<String, SampleStepError> {
    let v0 = builder::vertex(Point3::new(0.0, 0.0, 0.0));
    let v1 = builder::vertex(Point3::new(100.0, 0.0, 0.0));
    let v2 = builder::vertex(Point3::new(100.0, 80.0, 0.0));
    let v3 = builder::vertex(Point3::new(0.0, 80.0, 0.0));
    let outer_z0: Wire = vec![
        builder::line(&v0, &v1),
        builder::line(&v1, &v2),
        builder::line(&v2, &v3),
        builder::line(&v3, &v0),
    ]
    .into();
    let base_face_z0 = builder::try_attach_plane(vec![outer_z0])
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let base_solid = builder::extrude(&base_face_z0, Vector3::new(0.0, 0.0, 20.0));
    let mut shell_faces: Vec<Face> = base_solid
        .into_boundaries()
        .into_iter()
        .next()
        .ok_or_else(|| SampleStepError::Modeling("missing adaptive block shell".to_string()))?
        .into_iter()
        .collect();
    let old_top_z20 = shell_faces
        .pop()
        .ok_or_else(|| SampleStepError::Modeling("empty adaptive block shell".to_string()))?;
    let mut top_wires = old_top_z20.boundaries();

    // Closed sharp-corner pocket [14..86] x [14..66] at Z = 10..20 with raised Ø16 mm (R = 8 mm) island at (50, 40)
    let p_a = builder::vertex(Point3::new(14.0, 14.0, 10.0));
    let p_b = builder::vertex(Point3::new(86.0, 14.0, 10.0));
    let p_c = builder::vertex(Point3::new(86.0, 66.0, 10.0));
    let p_d = builder::vertex(Point3::new(14.0, 66.0, 10.0));
    let pkt_outer_z10: Wire = vec![
        builder::line(&p_a, &p_b),
        builder::line(&p_b, &p_c),
        builder::line(&p_c, &p_d),
        builder::line(&p_d, &p_a),
    ]
    .into();
    let island_seed_z10 = builder::vertex(Point3::new(50.0 + 8.0, 40.0, 10.0));
    let island_wire_z10: Wire = builder::revolve(
        &island_seed_z10,
        Point3::new(50.0, 40.0, 10.0),
        Vector3::unit_z(),
        SweepAngle::Closed,
        2,
    );
    let annular_floor = builder::try_attach_plane(vec![pkt_outer_z10, island_wire_z10.inverse()])
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let annular_plug = builder::extrude(&annular_floor, Vector3::new(0.0, 0.0, 10.0));
    let mut annular_faces: Vec<Face> = annular_plug
        .into_boundaries()
        .into_iter()
        .next()
        .ok_or_else(|| SampleStepError::Modeling("missing annular pocket plug".to_string()))?
        .into_iter()
        .collect();
    let annular_top_z20 = annular_faces
        .pop()
        .ok_or_else(|| SampleStepError::Modeling("empty annular plug".to_string()))?;
    let mut annular_top_wires = annular_top_z20.boundaries();
    if annular_top_wires.len() != 2 {
        return Err(SampleStepError::Modeling(
            "expected outer + island wire on adaptive pocket plug".to_string(),
        ));
    }
    let pkt_mouth_z20 = annular_top_wires.remove(0);
    let island_top_cw_z20 = annular_top_wires.remove(0);
    for f in annular_faces {
        shell_faces.push(f.inverse());
    }
    let island_top_cap_z20 = builder::try_attach_plane(vec![island_top_cw_z20.inverse()])
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    shell_faces.push(island_top_cap_z20);
    top_z20_wires_push(&mut top_wires, pkt_mouth_z20.inverse());

    let new_top_z20 = builder::try_attach_plane(top_wires)
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    shell_faces.push(new_top_z20);

    let shell: Shell = shell_faces.into();
    let solid =
        Solid::try_new(vec![shell]).map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    serialize_tutorial_solid(solid, "05-adaptive-pocket.step")
}

fn top_z20_wires_push(wires: &mut Vec<Wire>, wire: Wire) {
    wires.push(wire);
}

/// Minimal Tutorial 06 (Edge Chamfering & 2D Curve Trace / Engraving) starter part (`100 × 70 × 18 mm`)
/// with pre-squared `Z = 18 mm` top surface containing ONLY:
/// - A modeled `45° × 1.5 mm` sloped bevel face along the front top edge (`Y = 0..1.5`, `Z = 16.5..18`),
/// - A crisp sharp `90°` unmodeled back top edge (`Y = 70, Z = 18`) for sharp-edge deburring, and
/// - A clean flat `Z = 18 mm` top face ready for associative single-line Hershey text engraving.
pub fn tutorial_chamfer_engrave_block_step() -> std::result::Result<String, SampleStepError> {
    // Extrude the YZ cross-section (a 70 × 18 mm rectangle with its top-front corner beveled 45° × 1.5 mm)
    // along +X from X = 0 to X = 100!
    // Cross-section vertices in the X = 0 plane (oriented CCW when viewed from +X so +X extrusion has outward normals):
    let v0 = builder::vertex(Point3::new(0.0, 0.0, 0.0));
    let v1 = builder::vertex(Point3::new(0.0, 0.0, 16.5));
    let v2 = builder::vertex(Point3::new(0.0, 1.5, 18.0));
    let v3 = builder::vertex(Point3::new(0.0, 70.0, 18.0));
    let v4 = builder::vertex(Point3::new(0.0, 70.0, 0.0));

    let profile_x0: Wire = vec![
        builder::line(&v0, &v1),
        builder::line(&v1, &v2), // 45° x 1.5 mm beveled front edge
        builder::line(&v2, &v3), // Z = 18.0 flat top face
        builder::line(&v3, &v4), // Y = 70.0 sharp back wall
        builder::line(&v4, &v0), // Z = 0.0 bottom face
    ]
    .into();
    // In `monstertruck`, `profile_x0` above is CW viewed from +X (normal -X); inverting it gives
    // normal (+1, 0, 0) aligned with the (+100, 0, 0) extrusion vector so all solid normals point outward!
    let end_face_x0 = builder::try_attach_plane(vec![profile_x0.inverse()])
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let solid = builder::extrude(&end_face_x0, Vector3::new(100.0, 0.0, 0.0));
    serialize_tutorial_solid(solid, "06-chamfer-engrave-block.step")
}

/// Minimal Tutorial 11 (**Indexed `3+1` Multi-Face Milling**) prismatic STEP part aligned along
/// WCS `X` (`70 × 32 × 32 mm`, `X ∈ [-35, +35]`, `Y ∈ [-16, +16]`, `Z ∈ [-16, +16]`) containing:
/// - **4 cardinal planar faces**: `+Z` (`0°`), `+Y` (`90°`), `-Z` (`180°`), and `-Y` (`270°`), and
/// - **1 generous `45°` beveled mounting facet** along the `(+Y, +Z)` corner (`Y + Z = 22`,
///   `14.14 × 70 mm`, outward normal `(0, +1/√2, +1/√2)` -> exact `45.0°` `A`-axis index angle),
///
/// allowing machinists to click any planar face in 3D and click **Align to Selected 3D Face** to
/// compute exact `0°`, `45°`, `90°`, `180°`, and `270°` rotary index angles in a single chucking.
pub fn tutorial_indexed_4side_block_step() -> std::result::Result<String, SampleStepError> {
    // YZ cross-section at X = -35 mm oriented CCW viewed from +X (normal +X, aligned with +70 X extrusion):
    //   (-16, -16) -> (+16, -16) -> (+16, +6) -> [45° bevel] (+6, +16) -> (-16, +16) -> (-16, -16)
    let v0 = builder::vertex(Point3::new(-35.0, -16.0, -16.0));
    let v1 = builder::vertex(Point3::new(-35.0, 16.0, -16.0));
    let v2 = builder::vertex(Point3::new(-35.0, 16.0, 6.0));
    let v3 = builder::vertex(Point3::new(-35.0, 6.0, 16.0));
    let v4 = builder::vertex(Point3::new(-35.0, -16.0, 16.0));

    let profile_x_neg: Wire = vec![
        builder::line(&v0, &v1), // Z = -16 (-Z face, 180°)
        builder::line(&v1, &v2), // Y = +16 (+Y face, 90°)
        builder::line(&v2, &v3), // 45° corner bevel (+Y/+Z, 45°)
        builder::line(&v3, &v4), // Z = +16 (+Z face, 0°)
        builder::line(&v4, &v0), // Y = -16 (-Y face, 270°)
    ]
    .into();
    let end_face_x_neg = builder::try_attach_plane(vec![profile_x_neg])
        .map_err(|e| SampleStepError::Modeling(e.to_string()))?;
    let solid = builder::extrude(&end_face_x_neg, Vector3::new(70.0, 0.0, 0.0));
    serialize_tutorial_solid(solid, "11-indexed-multiface-block.step")
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

    #[test]
    fn torture_block_classifies_open_step_raised_island_modeled_chamfer_and_lower_floor() {
        use crate::topology::{
            classify_planar_face_loops, extract_modeled_chamfer_strip,
            highest_adjacent_lower_floor_proj,
        };

        let step = sample_cam_torture_block_step().unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes(
                "torture_block.step",
                step.as_bytes(),
                DEFAULT_TESS_CHORD_TOL_MM,
            )
            .unwrap();
        assert_eq!(doc.bodies.len(), 1);

        // 1. Open-edge step floor at Z = 14.0 mm: 3 open air edges + 1 rising wall edge at X = 76,
        //    and its 4 Ø6mm through-hole inner loops are downward holes (0 raised islands!)
        let step_floor = doc
            .faces
            .iter()
            .find(|f| {
                matches!(
                    f.surface,
                    SurfaceKind::Plane { origin_mm, normal }
                    if normal.z > 0.99 && approx_eq(origin_mm.z, 14.0, 1e-2)
                )
            })
            .unwrap()
            .id;
        let step_cls = classify_planar_face_loops(&doc, step_floor, DVec3::Z);
        assert_eq!(step_cls.open_outer_edges.len(), 3);
        assert!(step_cls.raised_inner_loops.is_empty());

        // 2. Enclosed island pocket floor at Z = 12.0 mm: 0 open outer edges + 1 raised Ø10mm island boss!
        let island_pkt_floor = doc
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
        let pkt_cls = classify_planar_face_loops(&doc, island_pkt_floor, DVec3::Z);
        assert!(pkt_cls.open_outer_edges.is_empty());
        assert_eq!(pkt_cls.raised_inner_loops.len(), 1);

        // 3. Modeled 45° x 2.0 mm chamfer strip on the beveled boss (Z = 20..22 mm)
        let chamfer_face = doc
            .faces
            .iter()
            .find(|f| {
                matches!(
                    f.surface,
                    SurfaceKind::Plane { normal, .. }
                    if approx_eq(normal.z, std::f64::consts::FRAC_1_SQRT_2, 1e-2)
                )
            })
            .unwrap()
            .id;
        let strip = extract_modeled_chamfer_strip(&doc, chamfer_face, DVec3::Z).unwrap();
        assert!(approx_eq(
            strip.bevel_half_angle_rad,
            std::f64::consts::FRAC_PI_4,
            1e-2
        ));
        assert!(approx_eq(strip.axial_drop_mm, 2.0, 2e-2));
        assert!(approx_eq(strip.radial_width_mm, 2.0, 2e-2));
        assert!(!strip.upper_edges.is_empty());
        assert!(!strip.lower_edges.is_empty());

        // 4. Adjacent lower step floor detection below the X = 76, Z = 20 sharp edge
        let step_top_edge = doc
            .edges
            .iter()
            .find(|e| {
                approx_eq(e.aabb_mm.min.z, 20.0, 1e-2)
                    && approx_eq(e.aabb_mm.max.z, 20.0, 1e-2)
                    && approx_eq(e.aabb_mm.min.x, 76.0, 1e-2)
                    && approx_eq(e.aabb_mm.max.x, 76.0, 1e-2)
            })
            .unwrap()
            .id;
        let lower_floor =
            highest_adjacent_lower_floor_proj(&doc, &[step_top_edge], 20.0, DVec3::Z).unwrap();
        assert!(approx_eq(lower_floor, 14.0, 2e-2));
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
        let bevel_face = ch_doc
            .faces
            .iter()
            .find(|f| {
                matches!(
                    f.surface,
                    SurfaceKind::Plane { normal, .. }
                    if normal.z > 0.65 && normal.z < 0.76
                )
            })
            .unwrap()
            .id;
        let strip = crate::extract_modeled_chamfer_strip(&ch_doc, bevel_face, DVec3::Z).unwrap();
        assert!(approx_eq(
            strip.bevel_half_angle_rad,
            std::f64::consts::FRAC_PI_4,
            1e-2
        ));
        assert!(approx_eq(strip.radial_width_mm, 1.5, 2e-2));
        assert!(approx_eq(strip.axial_drop_mm, 1.5, 2e-2));
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
}
