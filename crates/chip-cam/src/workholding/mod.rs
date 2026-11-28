//! 3D Workholding fixtures, vises, toe/strap clamps, spoilboards, keep-out zones, and imported
//! fixture solids (`chip-cam`).
//!
//! All geometry is computed in exact `f64` millimeters in Setup WCS coordinates, producing
//! watertight [`TriMesh3D`] meshes with [`chip_bvh::FlatBvh`] spatial indexes and exact 2D XY
//! [`RegionSet2D`] keep-out footprints—strictly UI-agnostic with zero `egui`/`wgpu` dependencies.

mod mesh_gen;
mod model;

pub use mesh_gen::{
    resolve_fixture_body, resolve_setup_fixtures, resolve_setup_fixtures_parametric,
};
pub use model::{
    FixtureBody, FixtureKind, FixtureZRef, ResolvedFixtureSolid, ViseClampAxis, ViseFixedSide,
};

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use std::collections::HashMap;

    use chip_bvh::TriMesh3D;
    use chip_cad::{
        CadDocument, CadKernel, MeshImportOptions, MonstertruckKernel, load_mesh_cad_document,
    };
    use chip_fixtures::{sample_machined_plate_step, sample_sculpted_dome_stl_bytes};
    use chip_math::{DEFAULT_TESS_CHORD_TOL_MM, DVec2, DVec3, DisplayUnit, ParamLength, approx_eq};

    use super::*;
    use crate::{
        setup::{ResolvedSetup, Setup},
        stock::{FixedBoxXyAlign, FixedBoxZAlign, StockConfig},
        wcs::{BoxAnchor27, CamError, WcsOriginMode, WorkOffsetSlot},
    };

    /// Asserts that `mesh` is a strictly watertight, manifold, consistently outward-oriented solid:
    /// every undirected edge has exactly 2 opposite half-edges, and signed volume > 0.
    fn assert_watertight_outward_solid(mesh: &TriMesh3D, label: &str) {
        assert!(!mesh.is_empty(), "{label}: mesh is empty");
        let mut half_edges: HashMap<(u32, u32), usize> = HashMap::new();
        let mut undir_edges: HashMap<(u32, u32), usize> = HashMap::new();
        let mut signed_vol6 = 0.0_f64;

        for tri in &mesh.triangles {
            let [i0, i1, i2] = tri.indices;
            let v0 = mesh.vertices[i0 as usize];
            let v1 = mesh.vertices[i1 as usize];
            let v2 = mesh.vertices[i2 as usize];
            signed_vol6 += v0.dot(v1.cross(v2));

            for (a, b) in [(i0, i1), (i1, i2), (i2, i0)] {
                *half_edges.entry((a, b)).or_insert(0) += 1;
                let key = if a < b { (a, b) } else { (b, a) };
                *undir_edges.entry(key).or_insert(0) += 1;
            }
        }

        for (&(a, b), &count) in &undir_edges {
            assert_eq!(
                count, 2,
                "{label}: edge ({a}, {b}) has incident count {count} (expected 2 for watertight manifold)"
            );
            assert_eq!(
                half_edges.get(&(a, b)).copied().unwrap_or(0),
                1,
                "{label}: half-edge ({a}->{b}) orientation mismatch"
            );
            assert_eq!(
                half_edges.get(&(b, a)).copied().unwrap_or(0),
                1,
                "{label}: half-edge ({b}->{a}) orientation mismatch"
            );
        }

        assert!(
            signed_vol6 > 1e-3,
            "{label}: expected positive outward signed volume, got {}",
            signed_vol6 / 6.0
        );
    }

    fn make_box_setup(wx: f64, wy: f64, wz: f64) -> (Setup, ResolvedSetup) {
        let mut setup = Setup::new("s1", "Test Setup", WorkOffsetSlot::G54);
        setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(wx),
            depth_y: ParamLength::from_mm(wy),
            height_z: ParamLength::from_mm(wz),
            xy_align: FixedBoxXyAlign::CenterXy,
            z_align: FixedBoxZAlign::CenterZ,
        };
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        };
        let resolved = setup
            .resolve_immutable(&CadDocument::empty(), DisplayUnit::Millimeter)
            .unwrap();
        // Stock spans X: [-wx/2, wx/2], Y: [-wy/2, wy/2], Z: [-wz, 0]
        (setup, resolved)
    }

    #[test]
    fn vise_jaws_flat_stepped_and_parallels_are_watertight_with_exact_bounds_and_raycasts() {
        let (mut setup, resolved) = make_box_setup(120.0, 100.0, 20.0);

        // 1. Flat jaws + parallels (Y clamp axis)
        let vise_flat = FixtureBody::default_vise("fix_vise_1", "4in Kurt Vise");
        setup.fixtures = vec![vise_flat];
        let solids =
            resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter).unwrap();
        assert_eq!(solids.len(), 1);
        let s = &solids[0];
        assert_watertight_outward_solid(&s.mesh_wcs, "flat vise + parallels");
        // X: [-50, 50], Y: [-50 - 18, 50 + 18] = [-68, 68], Z: [-20 + 5 - 38, -20 + 5] = [-53, -15]
        assert!(approx_eq(s.aabb_wcs.min.x, -50.0, 1e-6));
        assert!(approx_eq(s.aabb_wcs.max.x, 50.0, 1e-6));
        assert!(approx_eq(s.aabb_wcs.min.y, -68.0, 1e-6));
        assert!(approx_eq(s.aabb_wcs.max.y, 68.0, 1e-6));
        assert!(approx_eq(s.aabb_wcs.min.z, -53.0, 1e-6));
        assert!(approx_eq(s.aabb_wcs.max.z, -15.0, 1e-6));

        // Downward ray onto +Y jaw top at (0, 59, 10) hits at z = -15.0 (t = 25.0)
        let hit_jaw = s
            .mesh_wcs
            .raycast(DVec3::new(0.0, 59.0, 10.0), DVec3::NEG_Z, f64::INFINITY)
            .unwrap();
        assert!(approx_eq(hit_jaw.point.z, -15.0, 1e-9));
        // Downward ray onto parallel inside +Y jaw at (0, 48.5, 10) hits at stock bottom z = -20.0
        let hit_par = s
            .mesh_wcs
            .raycast(DVec3::new(0.0, 48.5, 10.0), DVec3::NEG_Z, f64::INFINITY)
            .unwrap();
        assert!(approx_eq(hit_par.point.z, -20.0, 1e-9));
        // Downward ray through open vise center (0, 0, 10) misses completely
        assert!(
            s.mesh_wcs
                .raycast(DVec3::new(0.0, 0.0, 10.0), DVec3::NEG_Z, f64::INFINITY)
                .is_none()
        );
        assert!(s.footprint_2d.contains_point(DVec2::new(0.0, 59.0)));
        assert!(!s.footprint_2d.contains_point(DVec2::new(0.0, 0.0)));

        // 2. Stepped soft jaws (X clamp axis, with step shelf & no parallels, plus expression jaw_width)
        let mut stepped = FixtureBody::default_vise("fix_vise_2", "Stepped Soft Jaws");
        if let FixtureKind::ViseJaws {
            clamp_axis,
            jaw_width,
            step_shelf_depth,
            step_shelf_height,
            include_parallels,
            ..
        } = &mut stepped.kind
        {
            *clamp_axis = ViseClampAxis::X;
            *jaw_width = ParamLength {
                value_mm: 50.0,
                expr: Some("stock_y + 20 mm".to_string()),
            };
            *step_shelf_depth = ParamLength::from_mm(4.0);
            *step_shelf_height = ParamLength::from_mm(6.0);
            *include_parallels = false;
        }
        setup.fixtures = vec![stepped];
        let solids_step =
            resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter).unwrap();
        let ss = &solids_step[0];
        assert_watertight_outward_solid(&ss.mesh_wcs, "stepped soft jaws X-axis");
        // Jaw width evaluated from `stock_y + 20 mm` = 120 mm -> Y in [-60, 60]
        assert!(approx_eq(ss.aabb_wcs.min.y, -60.0, 1e-6));
        assert!(approx_eq(ss.aabb_wcs.max.y, 60.0, 1e-6));
        // X spans [-60 - 18, 60 + 18] = [-78, 78]
        assert!(approx_eq(ss.aabb_wcs.min.x, -78.0, 1e-6));
        assert!(approx_eq(ss.aabb_wcs.max.x, 78.0, 1e-6));
        // Ray on step shelf at (58, 0, 10) hits at stock bottom z = -20.0!
        let hit_shelf = ss
            .mesh_wcs
            .raycast(DVec3::new(58.0, 0.0, 10.0), DVec3::NEG_Z, f64::INFINITY)
            .unwrap();
        assert!(approx_eq(hit_shelf.point.z, -20.0, 1e-9));
    }

    #[test]
    fn toe_clamp_rotated_cardinals_is_watertight_and_catches_stud_tower() {
        let (mut setup, resolved) = make_box_setup(120.0, 100.0, 20.0);

        for (idx, (anchor, yaw_deg)) in [
            (DVec2::new(60.0, 0.0), 0.0),
            (DVec2::new(0.0, 50.0), 90.0),
            (DVec2::new(-60.0, 0.0), 180.0),
            (DVec2::new(0.0, -50.0), 270.0),
        ]
        .into_iter()
        .enumerate()
        {
            let clamp = FixtureBody::default_toe_clamp(
                format!("clamp_{idx}"),
                format!("Clamp {yaw_deg}°"),
                anchor,
                yaw_deg,
            );
            setup.fixtures = vec![clamp];
            let solids =
                resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter)
                    .unwrap();
            let s = &solids[0];
            assert_watertight_outward_solid(&s.mesh_wcs, &format!("toe clamp {yaw_deg}°"));

            // Z spans from stock bottom (-20) up to strap (10) + stud (12) = +22.0 mm!
            assert!(approx_eq(s.aabb_wcs.min.z, -20.0, 1e-6));
            assert!(approx_eq(s.aabb_wcs.max.z, 22.0, 1e-6));

            // Ray descending directly over the stud tower catches z = +22.0 mm (above strap z = +10.0 mm!)
            let dir = DVec2::new(yaw_deg.to_radians().cos(), yaw_deg.to_radians().sin());
            let stud_u = 0.45 * (45.0 - 6.0);
            let stud_xy = anchor + dir * stud_u;
            let hit_stud = s
                .mesh_wcs
                .raycast(
                    DVec3::new(stud_xy.x, stud_xy.y, 40.0),
                    DVec3::NEG_Z,
                    f64::INFINITY,
                )
                .unwrap();
            assert!(
                approx_eq(hit_stud.point.z, 22.0, 1e-6),
                "yaw={yaw_deg}: expected stud top z=22.0, got {}",
                hit_stud.point.z
            );
            assert!(s.footprint_2d.contains_point(stud_xy));
        }
    }

    #[test]
    fn spoilboard_box_cylinder_and_imported_step_stl_resolve_cleanly() {
        let (mut setup, resolved) = make_box_setup(120.0, 100.0, 20.0);

        let sb = FixtureBody::default_spoilboard("sb_1", "MDF Spoilboard");
        let mut bx = FixtureBody::default_box_keepout(
            "bx_1",
            "Rotated Stop Block",
            DVec3::new(-75.0, 0.0, -5.0),
        );
        if let FixtureKind::BoxKeepOut { yaw_deg, .. } = &mut bx.kind {
            *yaw_deg = 45.0;
        }
        let cyl = FixtureBody::default_cylinder_keepout(
            "cyl_1",
            "Dowel Pin",
            DVec2::new(70.0, 20.0),
            -20.0,
            15.0,
        );
        let mut imp_step = FixtureBody::default_imported_model("imp_1", "STEP Fixture", "fix_step");
        if let FixtureKind::ImportedModel {
            offset_wcs,
            rotate_deg,
            ..
        } = &mut imp_step.kind
        {
            *offset_wcs = [
                ParamLength::from_mm(100.0),
                ParamLength::from_mm(0.0),
                ParamLength::from_mm(-20.0),
            ];
            *rotate_deg = [0.0, 0.0, 90.0];
        }
        let imp_stl = FixtureBody::default_imported_model("imp_2", "STL Fixture", "fix_stl");

        let step_txt = sample_machined_plate_step().unwrap();
        let step_doc = MonstertruckKernel
            .load_step_bytes(
                "fixture_plate.step",
                step_txt.as_bytes(),
                DEFAULT_TESS_CHORD_TOL_MM,
            )
            .unwrap();
        let stl_bytes = sample_sculpted_dome_stl_bytes();
        let stl_doc = load_mesh_cad_document(
            "fixture_dome.stl",
            &stl_bytes,
            &MeshImportOptions::default(),
        )
        .unwrap();

        let mut docs: HashMap<&str, CadDocument> = HashMap::new();
        docs.insert("fix_step", step_doc);
        docs.insert("fix_stl", stl_doc);

        setup.fixtures = vec![sb, bx, cyl, imp_step, imp_stl];
        let solids = resolve_setup_fixtures(&setup, &resolved, DisplayUnit::Millimeter, |id| {
            docs.get(id)
        })
        .unwrap();
        assert_eq!(solids.len(), 5);

        // Spoilboard (stock X: [-60, 60] + 25 margin = [-85, 85])
        assert_watertight_outward_solid(&solids[0].mesh_wcs, "spoilboard");
        assert!(approx_eq(solids[0].aabb_wcs.min.x, -85.0, 1e-6));
        assert!(approx_eq(solids[0].aabb_wcs.max.x, 85.0, 1e-6));
        assert!(approx_eq(solids[0].aabb_wcs.max.z, -20.0, 1e-6));
        assert!(approx_eq(solids[0].aabb_wcs.min.z, -38.0, 1e-6));
        assert!(approx_eq(solids[0].allowed_overcut_mm, 0.5, 1e-9));

        // BoxKeepOut (rotated 45°)
        assert_watertight_outward_solid(&solids[1].mesh_wcs, "rotated box keep-out");
        let diag_half = 25.0 * std::f64::consts::FRAC_1_SQRT_2;
        assert!(approx_eq(solids[1].aabb_wcs.min.x, -75.0 - diag_half, 1e-6));
        assert!(approx_eq(solids[1].aabb_wcs.max.x, -75.0 + diag_half, 1e-6));
        assert!(
            solids[1]
                .footprint_2d
                .contains_point(DVec2::new(-75.0, 0.0))
        );

        // CylinderKeepOut
        assert_watertight_outward_solid(&solids[2].mesh_wcs, "cylinder keep-out");
        assert!(approx_eq(solids[2].aabb_wcs.min.z, -20.0, 1e-6));
        assert!(approx_eq(solids[2].aabb_wcs.max.z, 15.0, 1e-6));
        assert!(
            solids[2]
                .footprint_2d
                .contains_point(DVec2::new(70.0, 20.0))
        );
        assert!(
            !solids[2]
                .footprint_2d
                .contains_point(DVec2::new(80.0, 20.0))
        );

        // Imported STEP & STL fixtures
        assert!(solids[3].mesh_wcs.len() >= 12);
        assert!(solids[3].aabb_wcs.is_valid());
        assert!(!solids[3].footprint_2d.is_empty());
        assert!(solids[4].mesh_wcs.len() >= 100);
        assert!(solids[4].aabb_wcs.is_valid());
        assert!(!solids[4].footprint_2d.is_empty());
    }

    #[test]
    fn rejects_invalid_fixture_dimensions_and_missing_models() {
        let (mut setup, resolved) = make_box_setup(120.0, 100.0, 20.0);

        let mut neg_margin = FixtureBody::default_spoilboard("s1", "Bad Margin");
        neg_margin.clearance_margin = ParamLength::from_mm(-1.0);
        setup.fixtures = vec![neg_margin];
        assert!(matches!(
            resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter),
            Err(CamError::InvalidFixture { .. })
        ));

        let bad_cyl =
            FixtureBody::default_cylinder_keepout("c1", "Bad Cyl", DVec2::ZERO, 10.0, 5.0);
        setup.fixtures = vec![bad_cyl];
        assert!(matches!(
            resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter),
            Err(CamError::InvalidFixture { .. })
        ));

        let missing_imp = FixtureBody::default_imported_model("m1", "Missing", "nonexistent");
        setup.fixtures = vec![missing_imp];
        assert!(matches!(
            resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter),
            Err(CamError::InvalidFixture { .. })
        ));

        // Disabled invalid fixture is skipped cleanly
        setup.fixtures[0].enabled = false;
        let skipped =
            resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter).unwrap();
        assert!(skipped.is_empty());
    }
}
