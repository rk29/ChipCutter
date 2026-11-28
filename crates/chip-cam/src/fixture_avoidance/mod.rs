//! Full-assembly swept-cutter vs. 3D workholding fixture `FlatBvh` collision kernel and
//! automatic safe-rapid / non-cutting link routing (`M61`).
//!
//! All calculations operate in exact `f64` Setup WCS millimeters against [`ResolvedFixtureSolid`]
//! watertight triangle meshes and their [`chip_bvh::FlatBvh`] spatial indexes—strictly UI-agnostic.

mod collision;
mod routing;

use chip_math::DVec3;
pub use chip_tools::ToolAssemblyPart;
use chip_tools::{AnalyticCutterEnvelope, Tool, ToolAssemblyEnvelope};

pub use collision::{
    check_chord_fixture_collision, check_segment_fixture_collision,
    check_segment_fixture_collisions_all, check_segment_fixture_collisions_with_profile,
    check_segments_fixture_collisions_par,
};
pub use routing::route_toolpath_around_fixtures;

/// Exact full-assembly collision finding against a [`ResolvedFixtureSolid`].
#[derive(Debug, Clone, PartialEq)]
pub struct FixtureHit {
    pub fixture_id: String,
    pub fixture_name: String,
    pub contact_part: ToolAssemblyPart,
    pub penetration_mm: f64,
    pub required_lift_z_wcs: f64,
    pub hit_point_wcs: DVec3,
}

/// Summary telemetry produced by [`route_toolpath_around_fixtures`].
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct FixtureRoutingSummary {
    pub rapid_lifts_inserted: usize,
    pub max_lifted_z_wcs: f64,
}

/// Full revolved tool assembly profile (cutting flutes, non-cutting shank/neck, collet nut, and
/// tapered holder body) backed by canonical [`chip_tools::ToolAssemblyEnvelope`].
#[derive(Debug, Clone, Copy)]
pub struct ToolAssemblyProfile {
    pub assembly: ToolAssemblyEnvelope,
    pub envelope: AnalyticCutterEnvelope,
    pub cutting_radius_mm: f64,
    pub flute_length_mm: f64,
    pub neck_radius_mm: f64,
    pub neck_length_mm: f64,
    pub is_thread_mill: bool,
    pub has_relieved_neck: bool,
    pub shank_radius_mm: f64,
    pub stickout_length_mm: f64,
    pub holder_nut_radius_mm: f64,
    pub holder_body_radius_mm: f64,
    pub holder_cone_z0: f64,
    pub holder_cone_z1: f64,
}

impl ToolAssemblyProfile {
    /// Builds the full revolved assembly envelope from canonical [`ToolAssemblyEnvelope::from_tool`].
    pub fn from_tool(tool: &Tool) -> Self {
        let assembly = ToolAssemblyEnvelope::from_tool(tool);
        Self {
            assembly,
            envelope: assembly.envelope,
            cutting_radius_mm: assembly.cutting_radius_mm(),
            flute_length_mm: assembly.flute_length_mm(),
            neck_radius_mm: assembly.neck_radius_mm(),
            neck_length_mm: assembly.neck_length_mm(),
            is_thread_mill: assembly.is_thread_mill(),
            has_relieved_neck: assembly.has_relieved_neck(),
            shank_radius_mm: assembly.shank_radius_mm(),
            stickout_length_mm: assembly.stickout_length_mm,
            holder_nut_radius_mm: assembly.holder_nut_radius_mm,
            holder_body_radius_mm: assembly.holder_body_radius_mm,
            holder_cone_z0: assembly.holder_cone_z0,
            holder_cone_z1: assembly.holder_cone_z1,
        }
    }

    #[inline]
    pub fn max_assembly_radius_mm(&self) -> f64 {
        self.assembly.max_assembly_radius_mm()
    }

    /// Evaluates the collet nut / tapered holder cone radius at `dz >= stickout_length_mm`.
    #[inline]
    pub fn holder_radius_at_dz(&self, dz: f64) -> f64 {
        self.assembly.holder_radius_at_dz(dz)
    }

    /// Returns the lowest axial height `dz >= 0` above the tip where the assembly reaches radial
    /// distance `r_eff` from the spindle axis, paired with the corresponding [`ToolAssemblyPart`].
    #[inline]
    pub fn min_dz_and_part_for_radius(&self, r_eff: f64) -> Option<(f64, ToolAssemblyPart)> {
        self.assembly.min_dz_and_part_for_radius(r_eff)
    }
}

/// Physical grazing tolerance (`mm`) used when `include_clearance_margin == false` so flush tangent
/// contact (e.g. a contour finishing exactly on top of a spoilboard or flush beside a vise jaw)
/// never triggers a false collision.
pub(super) const PHYSICAL_CONTACT_EPS_MM: f64 = 5e-3;
/// Margin-mode threshold (`mm`) used for automatic safe-rapid routing.
pub(super) const MARGIN_CONTACT_EPS_MM: f64 = 1e-4;
/// Extra vertical clearance (`mm`) added on top of `max_required_lift_z` for safe-rapid bridges.
pub(super) const SAFE_RAPID_EXTRA_LIFT_MM: f64 = 2.0;

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cad::CadDocument;
    use chip_math::{DVec2, DisplayUnit, ParamLength, approx_eq};
    use chip_tools::{CoolantMode, CutterGeometry, SpindleDirection, Tool, ToolId};

    use super::*;
    use crate::{
        BoxAnchor27, FixtureBody, FixtureKind, MotionRole, ResolvedSetup, SegmentMotion3D, Setup,
        StockConfig, ToolpathBuilder, ToolpathSegment3D, WcsOriginMode, WorkOffsetSlot,
        resolve_setup_fixtures_parametric,
    };

    fn test_endmill_6mm() -> Tool {
        Tool {
            id: ToolId("t1_6mm".to_string()),
            tool_number: 1,
            length_offset_number: 1,
            diameter_offset_number: 1,
            name: "6mm Flat Endmill".to_string(),
            vendor: None,
            product_id: None,
            native_unit: DisplayUnit::Millimeter,
            spindle_direction: SpindleDirection::Cw,
            default_coolant: CoolantMode::Flood,
            stickout_length_mm: 24.0,
            geometry: CutterGeometry::FlatEndmill {
                diameter_mm: 6.0,
                flute_length_mm: 18.0,
                overall_length_mm: 55.0,
                shank_diameter_mm: 6.0,
                flutes: 3,
            },
            presets: Vec::new(),
        }
    }

    fn make_box_setup() -> (Setup, ResolvedSetup) {
        let mut setup = Setup::new("s1", "Setup 1", WorkOffsetSlot::G54);
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        };
        setup.stock = StockConfig::FixedBox {
            width_x: ParamLength::from_mm(100.0),
            depth_y: ParamLength::from_mm(100.0),
            height_z: ParamLength::from_mm(20.0),
            xy_align: crate::FixedBoxXyAlign::CenterXy,
            z_align: crate::FixedBoxZAlign::CenterZ,
        };
        let doc = CadDocument::empty();
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
        (setup, resolved)
    }

    #[test]
    fn rapid_routing_lifts_cleanly_over_toe_clamp_and_preserves_cutting_moves() {
        let (mut setup, resolved) = make_box_setup();
        // Place a ToeClamp across X = 0 (pointing +Y, anchor at Y = -16 so the tall stud tower sits directly on Y = 0!)
        let clamp =
            FixtureBody::default_toe_clamp("c1", "Center Clamp", DVec2::new(0.0, -16.0), 90.0);
        setup.fixtures.push(clamp);
        let fixtures =
            resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter).unwrap();
        let clamp_top_z = fixtures[0].aabb_wcs.max.z;
        assert!(
            clamp_top_z > 20.0,
            "expected stud top > 20 mm, got {clamp_top_z}"
        );

        let tool = test_endmill_6mm();
        let mut tb = ToolpathBuilder::new("op_1");
        tb.rapid_to(DVec3::new(-30.0, 0.0, 10.0));
        tb.linear_to(DVec3::new(-30.0, 0.0, -5.0), 300.0, MotionRole::Plunge);
        tb.linear_to(DVec3::new(-20.0, 0.0, -5.0), 800.0, MotionRole::Cut);
        tb.rapid_to(DVec3::new(-20.0, 0.0, 10.0));
        // Colliding horizontal rapid across the toe clamp at Z = 10 mm
        tb.rapid_to(DVec3::new(30.0, 0.0, 10.0));
        tb.linear_to(DVec3::new(30.0, 0.0, -5.0), 300.0, MotionRole::Plunge);
        tb.linear_to(DVec3::new(40.0, 0.0, -5.0), 800.0, MotionRole::Cut);
        let mut tp = tb.finish();

        // 1. No-op when `auto_avoid = false`
        let orig_events = tp.events.clone();
        let sum_off =
            route_toolpath_around_fixtures(&mut tp, &tool, &resolved, &fixtures, false, None);
        assert_eq!(sum_off.rapid_lifts_inserted, 0);
        assert_eq!(tp.events, orig_events);

        // 2. Active routing lifts the colliding rapid cleanly above clamp_top + clearance + 2.0
        let sum_on =
            route_toolpath_around_fixtures(&mut tp, &tool, &resolved, &fixtures, true, Some(30.0));
        assert_eq!(sum_on.rapid_lifts_inserted, 1);
        assert!(
            sum_on.max_lifted_z_wcs >= clamp_top_z + fixtures[0].clearance_margin_mm + 2.0 - 1e-4
        );
        assert_eq!(tp.rapid_lifts_inserted, 1);
        assert!(approx_eq(
            tp.max_lifted_z_wcs,
            sum_on.max_lifted_z_wcs,
            1e-9
        ));
        // Machine Z travel (30 mm) was exceeded by `max_lifted_z - stock_min_z` (~45 mm) -> warning!
        assert!(
            tp.warnings
                .iter()
                .any(|w| w.contains("exceeds machine Z travel limit")),
            "expected Z travel warning, got {:?}",
            tp.warnings
        );

        // 3. Every rapid/linear segment now passes full-assembly collision check cleanly!
        for seg in &tp.segments_wcs {
            let hit = check_segment_fixture_collision(seg, &tool, &fixtures, false);
            assert!(
                hit.is_none(),
                "expected zero fixture hits after routing, got {hit:?}"
            );
        }
    }

    #[test]
    fn holder_collision_caught_when_flute_clears_tall_jaw() {
        let (mut setup, resolved) = make_box_setup();
        // Tall box keep-out standing 8 mm away from Y = 0 (Y = 8..28, Z = -20..+35)
        // Tool Ø6 mm (flute_r = 3, shank_r = 3, stickout = 24, holder_nut_r = 12) moving along Y = 0 at Z = 0:
        // - Flute & shank are at distance 8 mm > 3 mm -> clear!
        // - Holder nut starts at Z = 0 + 24 = +24 mm with radius 12 mm > 8 mm -> penetrates 4.0 mm!
        let mut jaw = FixtureBody::default_box_keepout(
            "tall_jaw",
            "Tall Vise Jaw",
            DVec3::new(0.0, 18.0, 7.5),
        );
        jaw.kind = FixtureKind::BoxKeepOut {
            center_wcs: [
                ParamLength::from_mm(0.0),
                ParamLength::from_mm(18.0),
                ParamLength::from_mm(7.5),
            ],
            size_wcs: [
                ParamLength::from_mm(60.0),
                ParamLength::from_mm(20.0),
                ParamLength::from_mm(55.0), // Z = -20 .. +35
            ],
            yaw_deg: 0.0,
        };
        setup.fixtures.push(jaw);
        let fixtures =
            resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter).unwrap();

        let tool = test_endmill_6mm();
        let seg = ToolpathSegment3D {
            start_wcs: DVec3::new(-20.0, 0.0, 0.0),
            end_wcs: DVec3::new(20.0, 0.0, 0.0),
            motion: SegmentMotion3D::Linear {
                feed_mm_min: 800.0,
                role: MotionRole::Cut,
            },
            start_rotary_deg: 0.0,
            end_rotary_deg: 0.0,
        };
        let hit = check_segment_fixture_collision(&seg, &tool, &fixtures, false)
            .expect("holder must collide with tall jaw");
        assert_eq!(hit.fixture_id, "tall_jaw");
        assert_eq!(hit.contact_part, ToolAssemblyPart::Holder);
        assert!(
            approx_eq(hit.penetration_mm, 4.0, 0.05),
            "expected ~4.0 mm holder radial penetration, got {}",
            hit.penetration_mm
        );
    }

    #[test]
    fn spoilboard_allowed_overcut_permits_skim_and_flags_deep_plunge() {
        let (mut setup, resolved) = make_box_setup();
        let mut sb = FixtureBody::default_spoilboard("sb1", "MDF Spoilboard");
        sb.allowed_overcut = ParamLength::from_mm(0.3);
        setup.fixtures.push(sb);
        let fixtures =
            resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter).unwrap();
        let sb_top = fixtures[0].aabb_wcs.max.z; // stock min Z = -20.0

        let tool = test_endmill_6mm();
        // 0.2 mm skim cut -> allowed!
        let skim = ToolpathSegment3D {
            start_wcs: DVec3::new(-10.0, 0.0, sb_top - 0.2),
            end_wcs: DVec3::new(10.0, 0.0, sb_top - 0.2),
            motion: SegmentMotion3D::Linear {
                feed_mm_min: 600.0,
                role: MotionRole::Cut,
            },
            start_rotary_deg: 0.0,
            end_rotary_deg: 0.0,
        };
        assert!(
            check_segment_fixture_collision(&skim, &tool, &fixtures, false).is_none(),
            "0.2 mm skim into spoilboard with 0.3 mm allowance must pass cleanly"
        );

        // 1.0 mm deep cut -> triggers FixtureCollision!
        let deep = ToolpathSegment3D {
            start_wcs: DVec3::new(0.0, 0.0, sb_top + 1.0),
            end_wcs: DVec3::new(0.0, 0.0, sb_top - 1.0),
            motion: SegmentMotion3D::Linear {
                feed_mm_min: 300.0,
                role: MotionRole::Plunge,
            },
            start_rotary_deg: 0.0,
            end_rotary_deg: 0.0,
        };
        let hit = check_segment_fixture_collision(&deep, &tool, &fixtures, false)
            .expect("1.0 mm plunge into spoilboard must collide");
        assert_eq!(hit.fixture_id, "sb1");
        assert_eq!(hit.contact_part, ToolAssemblyPart::Flute);
        assert!(
            approx_eq(hit.penetration_mm, 0.7, 0.05),
            "expected ~0.7 mm excess over 0.3 mm allowance, got {}",
            hit.penetration_mm
        );
    }

    #[test]
    fn clamp_in_cut_path_is_never_mutated_by_rapid_router() {
        let (mut setup, resolved) = make_box_setup();
        setup.fixtures.push(FixtureBody::default_toe_clamp(
            "c_cut",
            "Blocking Clamp",
            DVec2::new(0.0, 0.0),
            0.0,
        ));
        let fixtures =
            resolve_setup_fixtures_parametric(&setup, &resolved, DisplayUnit::Millimeter).unwrap();
        let tool = test_endmill_6mm();

        let mut tb = ToolpathBuilder::new("op_cut");
        tb.rapid_to(DVec3::new(-25.0, 0.0, 35.0));
        tb.linear_to(DVec3::new(-25.0, 0.0, -2.0), 300.0, MotionRole::Plunge);
        // Cutting move straight through the clamp nose at Z = -2 mm!
        tb.linear_to(DVec3::new(25.0, 0.0, -2.0), 800.0, MotionRole::Cut);
        tb.rapid_to(DVec3::new(25.0, 0.0, 35.0));
        let mut tp = tb.finish();
        let orig_events = tp.events.clone();

        let summary =
            route_toolpath_around_fixtures(&mut tp, &tool, &resolved, &fixtures, true, None);
        assert_eq!(summary.rapid_lifts_inserted, 0);
        assert_eq!(
            tp.events, orig_events,
            "cutting moves must NEVER be mutated"
        );
    }
}
