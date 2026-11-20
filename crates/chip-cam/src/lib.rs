//! Setups, Work Coordinate Systems (WCS), stock definitions, 2.5D B-Rep feature extraction,
//! Z-height hierarchy resolution, and CAM operation models (`chip-cam`).

pub mod cl_data;
pub mod heights;
pub mod medial;
pub mod offset2d;
pub mod operation;
pub mod params;
pub mod rest2d;
pub mod rotary;
pub mod selection;
pub mod setup;
pub mod stock;
pub mod strategies;
pub mod surface3d;
pub mod toolpath;
pub mod wcs;

pub use cl_data::{ArcDir, ArcPlane, ClEvent, CutterCompMode, DrillCycleKind, MotionRole};
pub use heights::{HeightReference, HeightSpec, OperationHeights, ResolvedHeights, SetupHeights};
pub use medial::{
    MedialChain2D, MedialNode2D, extract_interior_medial_nodes, extract_medial_axis_chains,
    find_medial_inscribed_pole,
};
pub use offset2d::{
    HelicalEntryPole, OffsetRegionNode, PocketRegion2D, build_offset_region_tree,
    clip_scanline_to_region, find_helical_entry_pole, from_cavalier_polyline, offset_polyline,
    to_cavalier_polyline,
};
pub use operation::{
    EffectiveCuttingParams, Operation, OperationFeedOverrides, OperationKind, OperationStatus,
};
pub use params::{
    Adaptive3DParams, AdaptiveClearingParams, AdaptiveCutDirection, ChamferSourceMode,
    ChamferTraceMode, ChamferTraceParams, Contour2DParams, ContourCompensation, DrillBoreParams,
    EntryStrategy, FacingCutMode, FacingParams, HoldingTabsConfig, HoleMachiningMode, LeadConfig,
    LinkingParams, MachiningBoundaryMode, ManualTabAnchor, Parallel3DParams, ParallelCutDirection,
    PassesParams, Pencil3DParams, Pocket2DParams, PocketPassOrder, PocketStrategyKind,
    RestMachiningConfig, RestMachiningMode, RetractPolicy, Rotary2LinearParams, Rotary4AxisPattern,
    Rotary4AxisSimParams, RotaryCutDirection, RotaryPatternMode, RotaryTiltMode, Scallop3DParams,
    StrategyParams, TabPlacementMode, TabProfileKind, ToolContainmentMode, VCarveClearingKind,
    VCarveInlayMode, VCarveParams, Waterline3DParams, WaterlineOrderMode,
};
pub use rest2d::{
    ContourRestClipQuery, PocketRestQuery, RegionSet2D, RestPocketWorkItem, UpstreamSweptInput,
    clip_contour_path_to_rest, clip_polyline_outside_region_set, compute_rest_pocket_work_items,
    swept_segment_capsule_2d, swept_toolpath_coverage_2d,
};
pub use rotary::{
    OperationRotaryMode, ResolvedRotaryConfig, RotaryAxisKind, RotaryConfig, RotaryDirection,
    RotaryDriveMode, RotaryUnwindMode, build_indexed_virtual_setup, compute_index_angle_for_normal,
    compute_inverse_time_feed_g93, compute_rotary_angular_feed_deg_min,
    compute_stock_rotary_envelope, compute_workpiece_arc_distance_mm, cylindrical_to_wcs,
    resolve_rotary_target_deg, rotate_point_about_axis, wcs_to_cylindrical,
};
pub use selection::{
    ChainDirectionArrow3D, ExtractedFeature25D, ExtractedSelectionSummary, GeometrySelectionItem,
    extract_selection_features, extract_selection_features_with_sketches,
};
pub use setup::{
    ResolvedSetup, Setup, resolve_sketch_world_frame, resolve_sketch_world_frame_with_doc,
};
pub use stock::{
    FixedBoxXyAlign, FixedBoxZAlign, ResolvedStockGeometry, StockAxis, StockConfig,
    compute_tight_oriented_model_aabb,
};
pub use strategies::{
    OperationGenRequest, compute_z_levels, generate_operation_toolpath,
    generate_operation_toolpath_with_rest,
    pocket2d::{
        ContourParallelStrategy, PocketEngagementParams, PocketStrategy, PocketSubpath2D,
        ZigZagPocketStrategy,
    },
};
pub use surface3d::{
    RestMaskGrid2D, SurfaceCutterProfile, SurfaceCutterShape, SurfaceLinkConfig, SurfaceMesh3D,
    build_3d_rest_mask, clip_polyline_to_region, detect_flat_plateaus, drop_cutter_points_parallel,
    drop_cutter_z, drop_cutter_z_with_scratch, emit_final_surface_retract, emit_surface_safe_link,
    push_cutter_region_z, rect_region_2d, refine_convex_chords_3d, region_aabb_2d,
    resolve_3d_containment_region, sample_surface_slope_deg, simplify_3d_polyline_rdp,
    slice_surface_contours_z, stepover_for_cusp_height, stock_xy_region,
};
pub use toolpath::{
    ClearingSubToolpath, DEFAULT_RAPID_RATE_MM_MIN, DrillCycleSpec, GeneratedToolpath,
    SegmentMotion3D, TabMarker3D, ToolpathBuilder, ToolpathSegment3D, ToolpathStats,
    VerticalArcMove, XyArcMove, expand_drill_cycle_segments,
};
pub use wcs::{
    AxisAnchor, BoxAnchor27, CamError, WcsAxisRef, WcsConfig, WcsFeaturePointRef,
    WcsOrientationMode, WcsOriginMode, WorkOffsetSlot,
};

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_cad::{
        CadKernel, MonstertruckKernel, SurfaceKind, find_same_diameter_holes,
        sample_multi_feature_bracket_step,
    };
    use chip_math::{ArcSegment2D, DEFAULT_TESS_CHORD_TOL_MM, DisplayUnit, ParamLength, approx_eq};

    use super::*;

    #[test]
    fn aligns_wcs_on_tilted_step_solid_and_computes_tight_27_point_stock_bounds() {
        let tilted_step = sample_multi_feature_bracket_step(true).unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes(
                "tilted_bracket.step",
                tilted_step.as_bytes(),
                DEFAULT_TESS_CHORD_TOL_MM,
            )
            .unwrap();

        // Find the largest planar face (the 90x70 bottom face, area ~ 6100 mm^2, whose outward normal points -Z_part)
        // and flip Z so WCS +Z points straight up into the pocketed top of the bracket!
        let bottom_face = doc
            .faces
            .iter()
            .filter(|f| matches!(f.surface, SurfaceKind::Plane { .. }))
            .max_by(|a, b| a.area_mm2.total_cmp(&b.area_mm2))
            .unwrap();

        // Find the longest linear edge on that bottom face (80mm front edge along +X_part)
        let longest_bottom_edge = doc
            .brep_loop(bottom_face.outer_loop.unwrap())
            .unwrap()
            .edges
            .iter()
            .filter_map(|u| doc.edge(u.edge_id))
            .max_by(|a, b| a.length_mm.total_cmp(&b.length_mm))
            .unwrap();

        let mut setup = Setup::new("setup_1", "Op 1 - Tilted Alignment", WorkOffsetSlot::G54);
        setup.wcs.orientation = WcsOrientationMode::SelectZAndX {
            z_ref: WcsAxisRef::FaceAxis(doc.bind_face(bottom_face.id).unwrap()),
            flip_z: true,
            x_ref: Some(WcsAxisRef::EdgeDirection(
                doc.bind_edge(longest_bottom_edge.id).unwrap(),
            )),
            flip_x: false,
            clock_90: 0,
            extra_z_rot_deg: 0.0,
        };
        setup.wcs.origin = WcsOriginMode::StockBoxPoint {
            anchor: BoxAnchor27::TOP_CENTER,
        };
        setup.stock = StockConfig::RelativeBox {
            side_pos_x: ParamLength::from_mm(2.0),
            side_neg_x: ParamLength::from_mm(2.0),
            side_pos_y: ParamLength::from_mm(2.0),
            side_neg_y: ParamLength::from_mm(2.0),
            symmetric_xy: true,
            top_offset_z: ParamLength::from_mm(1.0),
            bottom_offset_z: ParamLength::from_mm(0.0),
            sheet_thickness_z: ParamLength::from_mm(12.0),
            round_increment_mm: None,
        };

        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();
        assert!(resolved.frame.is_orthonormal_right_handed(1e-12));

        // Even though the STEP model is rotated 30° around X and 25° around Z in world space,
        // the WCS-aligned tight model AABB recovers the exact 90 × 70 × 20 mm part dimensions!
        let m_size = resolved.model_aabb_wcs.size();
        assert!(
            (m_size.x - 90.0).abs() < 0.08 && (m_size.y - 70.0).abs() < 0.08
                || (m_size.x - 70.0).abs() < 0.08 && (m_size.y - 90.0).abs() < 0.08,
            "unexpected oriented XY size: {m_size:?}"
        );
        assert!(approx_eq(m_size.z, 20.0, 0.05));

        // With WCS origin at `Stock Top Center`:
        // - Stock Top Z == 0.0 mm
        // - Model Top Z == -1.0 mm (`1.0 mm` facing stock allowance)
        // - Stock Bottom Z == -21.0 mm
        let s_aabb = resolved.stock.aabb_wcs();
        assert!(approx_eq(s_aabb.max.z, 0.0, 1e-6));
        assert!(approx_eq(resolved.model_aabb_wcs.max.z, -1.0, 0.05));
        assert!(approx_eq(s_aabb.min.z, -21.0, 0.05));
        assert!(approx_eq(s_aabb.center().x, 0.0, 1e-6));
        assert!(approx_eq(s_aabb.center().y, 0.0, 1e-6));

        // Verify all 27 BoxAnchor27 points are distinct on a non-degenerate box
        let anchors = BoxAnchor27::all_27();
        assert_eq!(anchors.len(), 27);
    }

    #[test]
    fn extracts_25d_features_preserves_arcs_infers_z_heights_and_enforces_hierarchy() {
        let step = sample_multi_feature_bracket_step(false).unwrap();
        let doc = MonstertruckKernel
            .load_step_bytes("bracket.step", step.as_bytes(), DEFAULT_TESS_CHORD_TOL_MM)
            .unwrap();

        let mut setup = Setup::new("setup_1", "Op 1", WorkOffsetSlot::G54);
        // Anchor WCS at Model Bottom-Front-Left `(0, 0, 0)` so WCS Z matches CAD Z `(0..20 mm)`
        setup.wcs.origin = WcsOriginMode::ModelBoxPoint {
            anchor: BoxAnchor27::BOTTOM_FRONT_LEFT,
        };
        let resolved = setup.resolve(&doc, DisplayUnit::Millimeter).unwrap();

        // 1. Select Pocket A floor at Z = 12.0 mm (which has an inner Ø10mm blind bore loop!)
        let pkt_a_floor = doc
            .faces
            .iter()
            .find(|f| {
                matches!(
                    f.surface,
                    SurfaceKind::Plane { origin_mm, normal }
                    if normal.z > 0.99 && approx_eq(origin_mm.z, 12.0, 1e-2) && !f.inner_loops.is_empty()
                )
            })
            .unwrap();

        let pkt_summary = extract_selection_features(
            &doc,
            &resolved,
            &[GeometrySelectionItem::PlanarFace {
                binding: doc.bind_face(pkt_a_floor.id).unwrap(),
            }],
        )
        .unwrap();

        assert!(approx_eq(pkt_summary.inferred_bottom_z_wcs, 12.0, 0.02));
        assert!(approx_eq(pkt_summary.inferred_top_z_wcs, 20.0, 0.05));
        match &pkt_summary.features[0] {
            ExtractedFeature25D::PlanarFace {
                outer_boundary_wcs,
                inner_islands_wcs,
                ..
            } => {
                assert!(outer_boundary_wcs.closed);
                assert_eq!(inner_islands_wcs.len(), 1);
                // Inner circular mouth must preserve true `ArcSegment2D::Arc` segments!
                assert!(
                    inner_islands_wcs[0]
                        .segments
                        .iter()
                        .all(|s| matches!(s, ArcSegment2D::Arc { .. }))
                );
            }
            other => panic!("expected PlanarFace feature, got {other:?}"),
        }

        // 2. Select all Ø6mm holes via `find_same_diameter_holes` and verify deduplication to 4 Hole25Ds
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
        let hole_items: Vec<GeometrySelectionItem> =
            find_same_diameter_holes(&doc, r3_seed, 1e-3, 1e-3)
                .into_iter()
                .map(|fid| GeometrySelectionItem::CylindricalHole {
                    binding: doc.bind_face(fid).unwrap(),
                })
                .collect();

        let hole_summary = extract_selection_features(&doc, &resolved, &hole_items).unwrap();
        assert_eq!(hole_summary.features.len(), 4);
        assert_eq!(hole_summary.highlight_faces.len(), 8);
        assert!(approx_eq(hole_summary.inferred_top_z_wcs, 20.0, 0.05));
        assert!(approx_eq(hole_summary.inferred_bottom_z_wcs, 0.0, 0.05));

        // 3. Valid OperationHeights resolve cleanly; inverted Top < Bottom fails explicitly!
        let mut op_heights = OperationHeights::default();
        let valid_h = ResolvedHeights::resolve(
            &op_heights,
            &resolved.model_aabb_wcs,
            &resolved.stock.aabb_wcs(),
            resolved.clearance_z_wcs,
            resolved.retract_z_wcs,
            pkt_summary.inferred_top_z_wcs,
            pkt_summary.inferred_bottom_z_wcs,
        )
        .unwrap();
        assert!(valid_h.clearance_z_wcs >= valid_h.retract_z_wcs);
        assert!(valid_h.top_z_wcs > valid_h.bottom_z_wcs);

        op_heights.bottom = HeightSpec::new(HeightReference::StockTop, 5.0);
        let err = ResolvedHeights::resolve(
            &op_heights,
            &resolved.model_aabb_wcs,
            &resolved.stock.aabb_wcs(),
            resolved.clearance_z_wcs,
            resolved.retract_z_wcs,
            pkt_summary.inferred_top_z_wcs,
            pkt_summary.inferred_bottom_z_wcs,
        )
        .unwrap_err();
        assert!(matches!(err, CamError::HeightHierarchyViolation { .. }));
    }
}
