//! 2D Sketch Layers, Parametric 2D Primitives, CAM Typography, Snapping, DXF & SVG Vector
//! Importers, Bézier-to-Biarc Fitting, Gap-Healing, and Planar Region Classification (`chip-sketch`).

pub mod curve_fit;
pub mod dxf_import;
pub mod heal_chain;
pub mod model;
pub mod picker;
pub mod primitives;
pub mod snap;
pub mod svg_import;
pub mod typography;

pub use curve_fit::{circumcircle_through_three_points, fit_bezpath_to_arc_polylines};
pub use dxf_import::{
    DxfImportError, DxfImportOptions, DxfImportReport, import_dxf_bytes, import_dxf_path,
    sample_router_panel_dxf,
};
pub use heal_chain::{
    HealedSketchGeometry, RawImportedItem, classify_planar_regions_from_candidates,
    heal_and_classify_sketch_geometry,
};
pub use model::{
    FaceSketchOriginMode, PlanarVectorRegion, SketchAnchor, SketchDiagnosticKind, SketchEntity,
    SketchEntityEntry, SketchFaceBinding, SketchImportDiagnostic, SketchLayer, SketchPlaneRef,
    VectorAssetFormat, VectorImportUnits, VectorSourceAsset, compute_sketch_sha256_hex,
    create_default_sketch_entity,
};
pub use picker::{SketchPickHit, pick_sketch_layer_2d};
pub use primitives::{
    HolePoint2D, PointPatternSpec, PolygonEvalSpec, PolygonSizingMode, RectAnchorMode,
    RectEvalSpec, SlotLengthMode, build_circle_polyline, build_rectangle_polyline,
    build_regular_polygon_polyline, build_slot_polyline, transform_arc_polyline_2d,
};
pub use snap::{SketchSnapContext, SketchSnapResult, SnapKind, snap_sketch_point_2d};
pub use svg_import::{
    SvgImportError, SvgImportOptions, SvgImportReport, import_svg_bytes, import_svg_path,
    sample_badge_logo_svg,
};
pub use typography::{
    BUILTIN_DEJAVU_SANS_BOLD_TTF, CustomFontMap, HersheyFontKind, HersheyLayoutParams,
    SketchFontSource, SketchTextSpec, TextHAlign, TextVAlign, TtfOutlineParams,
    evaluate_hershey_strokes, evaluate_ttf_outline_regions,
};

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::{Aabb2, ArcSegment2D, DVec2, EvalContext, ParamLength, approx_eq};
    use kurbo::{BezPath, Circle as KCircle, Shape as _};

    use super::*;

    #[test]
    fn cubic_bezier_circle_fits_compact_true_circular_arcs_within_tolerance() {
        let k_circle = KCircle::new((25.0, -10.0), 15.0);
        let bez: BezPath = k_circle.to_path(1e-4);
        let polys = fit_bezpath_to_arc_polylines(&bez, 0.01);
        assert_eq!(polys.len(), 1);
        let poly = &polys[0];
        assert!(poly.closed);
        assert!(
            poly.segments.len() <= 12,
            "expected compact biarc segments, got {}",
            poly.segments.len()
        );
        assert!(
            poly.segments
                .iter()
                .all(|s| matches!(s, ArcSegment2D::Arc { .. })),
            "circle BezPath must fit exclusively to ArcSegment2D::Arc"
        );
        for i in 0..32 {
            let (pt, _) = poly.sample_at_fraction((i as f64) / 32.0).unwrap();
            let err = (pt.distance(DVec2::new(25.0, -10.0)) - 15.0).abs();
            assert!(err <= 0.012, "radial biarc error {err:.5} mm exceeded tol");
        }
    }

    #[test]
    fn dxf_importer_heals_micro_gaps_deduplicates_lines_expands_blocks_and_classifies_islands() {
        let dxf_bytes = sample_router_panel_dxf();
        let report = import_dxf_bytes(&dxf_bytes, &DxfImportOptions::default()).unwrap();
        let sk = &report.sketch;

        assert!(approx_eq(report.unit_scale_to_mm, 1.0, 1e-9));
        let aabb = sk.exact_aabb();
        assert!(approx_eq(aabb.min.x, 0.0, 1e-3));
        assert!(approx_eq(aabb.min.y, 0.0, 1e-3));
        assert!(approx_eq(aabb.max.x, 160.0, 1e-3));
        assert!(approx_eq(aabb.max.y, 100.0, 1e-3));

        assert_eq!(sk.circle_count(), 4);
        assert_eq!(sk.open_chain_count(), 0);
        assert!(
            sk.diagnostics
                .iter()
                .any(|d| d.kind == SketchDiagnosticKind::HealedEndpointGap)
        );
        assert!(
            sk.diagnostics
                .iter()
                .any(|d| d.kind == SketchDiagnosticKind::DuplicateSegmentRemoved)
        );

        let full_hit = pick_sketch_layer_2d(sk, DVec2::new(55.0, 50.0), 1.0, true).unwrap();
        match full_hit {
            SketchPickHit::Region { region_id, .. } => {
                let reg = sk.regions.iter().find(|r| r.id == region_id).unwrap();
                assert_eq!(reg.sub_layer.as_deref(), Some("POCKETS"));
                assert_eq!(reg.islands_cw.len(), 1);
            }
            other => panic!("expected Region hit at (55, 50), got {other:?}"),
        }
        let pockets_only = import_dxf_bytes(
            &dxf_bytes,
            &DxfImportOptions {
                include_layers: Some(["POCKETS".to_string()].into_iter().collect()),
                ..DxfImportOptions::default()
            },
        )
        .unwrap();
        assert_eq!(pockets_only.sketch.regions.len(), 1);
        assert_eq!(pockets_only.sketch.regions[0].islands_cw.len(), 1);
        assert!(pockets_only.sketch.regions[0].contains_point(DVec2::new(55.0, 50.0)));
        assert!(!pockets_only.sketch.regions[0].contains_point(DVec2::new(80.0, 50.0)));

        let hit =
            pick_sketch_layer_2d(&pockets_only.sketch, DVec2::new(55.0, 50.0), 1.0, true).unwrap();
        assert!(matches!(hit, SketchPickHit::Region { .. }));
    }

    #[test]
    fn svg_importer_honors_physical_mm_flattens_group_transforms_and_extracts_islands() {
        let svg = sample_badge_logo_svg();
        let report = import_svg_bytes(svg.as_bytes(), &SvgImportOptions::default()).unwrap();
        assert!(approx_eq(report.canvas_size_mm.x, 100.0, 1e-2));
        assert!(approx_eq(report.canvas_size_mm.y, 60.0, 1e-2));

        let aabb = report.sketch.exact_aabb();
        assert!(approx_eq(aabb.min.x, 4.0, 0.05));
        assert!(approx_eq(aabb.max.x, 96.0, 0.05));
        assert!(approx_eq(aabb.min.y, 4.0, 0.05));
        assert!(approx_eq(aabb.max.y, 56.0, 0.05));

        assert_eq!(report.sketch.regions.len(), 2);
        let has_arcs = report.sketch.entities.iter().any(|e| {
            e.entity
                .to_arc_polyline()
                .segments
                .iter()
                .any(|s| matches!(s, ArcSegment2D::Arc { .. }))
        });
        assert!(
            has_arcs,
            "SVG arcs/rounded rects must preserve true circular arcs"
        );
    }

    #[test]
    fn parametric_primitives_evaluate_exact_arcs_expressions_and_m7_toml_compat() {
        let mut ctx = EvalContext::default();
        ctx.set_var("stock_x", 120.0);
        ctx.set_var("stock_y", 80.0);

        let mut rect = SketchEntity::Rectangle {
            anchor: RectAnchorMode::Center,
            x: ParamLength::parse("stock_x / 2", &ctx).unwrap(),
            y: ParamLength::parse("stock_y / 2", &ctx).unwrap(),
            width: ParamLength::parse("stock_x - 20 mm", &ctx).unwrap(),
            height: ParamLength::from_mm(50.0),
            corner_radius: ParamLength::from_mm(5.0),
            rotation_deg: 0.0,
        };
        let poly = rect.to_arc_polyline();
        assert!(poly.closed);
        assert_eq!(poly.segments.len(), 8);
        let arc_count = poly
            .segments
            .iter()
            .filter(
                |s| matches!(s, ArcSegment2D::Arc { radius, .. } if approx_eq(*radius, 5.0, 1e-6)),
            )
            .count();
        assert_eq!(arc_count, 4);
        // Area of 100x50 rect with R5 fillets = 5000 - (4 - pi)*25
        let expected_area = 100.0 * 50.0 - (4.0 - std::f64::consts::PI) * 25.0;
        assert!(approx_eq(poly.signed_area(), expected_area, 1e-3));

        // Update stock_x -> reevaluate shifts center & width dynamically!
        ctx.set_var("stock_x", 160.0);
        rect.reevaluate(&ctx);
        let aabb = rect.to_arc_polyline().exact_aabb();
        assert!(approx_eq(aabb.center().x, 80.0, 1e-6));
        assert!(approx_eq(aabb.size().x, 140.0, 1e-6));

        // Stadium Slot (`OverallLength = 40`, `width = 10` -> `C2C = 30`, `2` lines + `2` R5 180° arcs)
        let slot = build_slot_polyline(SlotLengthMode::OverallLength, DVec2::ZERO, 40.0, 10.0, 0.0);
        assert_eq!(slot.segments.len(), 4);
        let expected_slot_area = 30.0 * 10.0 + std::f64::consts::PI * 25.0;
        assert!(approx_eq(slot.signed_area(), expected_slot_area, 1e-3));

        // Hex Across-Flats = 19 mm -> circumdiameter = 2 * (9.5 / cos(30°)) = 19 / (sqrt(3)/2)
        let hex = build_regular_polygon_polyline(PolygonEvalSpec {
            center_xy: DVec2::ZERO,
            sides: 6,
            sizing: PolygonSizingMode::AcrossFlats,
            diameter_mm: 19.0,
            corner_radius_mm: 0.0,
            rotation_deg: 0.0,
        });
        assert_eq!(hex.segments.len(), 6);
        let expected_circum_r = 9.5 / (30.0_f64.to_radians().cos());
        assert!(approx_eq(hex.exact_aabb().max.x, expected_circum_r, 1e-5));

        // Bolt circle 6 holes on Ø60 mm
        let bc = SketchEntity::PointPattern(PointPatternSpec::BoltCircle {
            center_x: ParamLength::from_mm(50.0),
            center_y: ParamLength::from_mm(40.0),
            bolt_circle_diameter: ParamLength::from_mm(60.0),
            hole_diameter: ParamLength::from_mm(6.5),
            count: 6,
            start_angle_deg: 0.0,
        });
        let holes = bc.evaluate_holes();
        assert_eq!(holes.len(), 6);
        assert!(approx_eq(holes[0].center_xy.x, 80.0, 1e-6));
        assert!(approx_eq(holes[0].center_xy.y, 40.0, 1e-6));
        assert!(approx_eq(holes[3].center_xy.x, 20.0, 1e-6));

        // Milestone 7 legacy `center_xy` TOML deserialization compatibility
        let legacy_toml = r#"
id = "c1"
name = "Legacy Circle"
[entity]
type = "circle"
center_xy = [15.5, 22.25]
diameter = { expr = "8", value_mm = 8.0 }
"#;
        let mut parsed: SketchEntityEntry = toml::from_str(legacy_toml).unwrap();
        assert_eq!(
            parsed.entity.circle_center_mm(),
            Some(DVec2::new(15.5, 22.25))
        );
        parsed.entity.reevaluate(&EvalContext::default());
        assert_eq!(
            parsed.entity.circle_center_mm(),
            Some(DVec2::new(15.5, 22.25))
        );
    }

    #[test]
    fn truetype_outline_text_extracts_exact_counter_islands_biarcs_and_welds_overlaps() {
        let spec = SketchTextSpec {
            content: "B8O".to_string(),
            font: SketchFontSource::BuiltinSansBold,
            cap_height: ParamLength::from_mm(20.0),
            x: ParamLength::from_mm(0.0),
            y: ParamLength::from_mm(0.0),
            h_align: TextHAlign::Left,
            v_align: TextVAlign::Baseline,
            ..SketchTextSpec::default()
        };
        let regions = spec.evaluate_outline_regions(None, None);
        // 'B' (2 counters), '8' (2 counters), 'O' (1 counter) => 3 outer glyph regions, 5 islands total!
        assert_eq!(regions.len(), 3, "expected 3 glyph regions for 'B8O'");
        let mut island_counts: Vec<usize> = regions.iter().map(|r| r.islands_cw.len()).collect();
        island_counts.sort_unstable();
        assert_eq!(island_counts, vec![1, 2, 2]);

        let aabb_y = regions
            .iter()
            .fold(Aabb2::EMPTY, |acc, r| acc.union(&r.outer_ccw.exact_aabb()));
        // Cap height = 20 mm -> top of 'B'/'O' sits near Y = +20 mm (within overshoot)!
        assert!((aabb_y.max.y - 20.0).abs() <= 1.0);

        let has_true_arcs = regions.iter().any(|r| {
            r.outer_ccw
                .segments
                .iter()
                .any(|s| matches!(s, ArcSegment2D::Arc { .. }))
        });
        assert!(
            has_true_arcs,
            "TrueType outlines must fit curved bowls into circular ArcSegment2D::Arcs"
        );

        // Negative letter spacing forces adjacent 'OO' outer contours to overlap and weld into 1 region with 2 islands!
        let welded_spec = SketchTextSpec {
            content: "OO".to_string(),
            font: SketchFontSource::BuiltinSansBold,
            cap_height: ParamLength::from_mm(20.0),
            letter_spacing: ParamLength::from_mm(-6.0),
            ..SketchTextSpec::default()
        };
        let welded = welded_spec.evaluate_outline_regions(None, None);
        assert_eq!(
            welded.len(),
            1,
            "overlapping 'OO' glyphs must boolean-union into a single outer region"
        );
        assert!(
            welded[0].islands_cw.len() >= 2,
            "welded 'OO' must preserve interior counters"
        );
    }

    #[test]
    fn hershey_single_line_fonts_produce_1_pass_strokes_and_honor_mirror_x() {
        for font in HersheyFontKind::ALL {
            let spec = SketchTextSpec {
                content: "S/N 2026-08".to_string(),
                font: SketchFontSource::Hershey { font },
                cap_height: ParamLength::from_mm(6.0),
                h_align: TextHAlign::Center,
                v_align: TextVAlign::Middle,
                ..SketchTextSpec::default()
            };
            assert!(spec.evaluate_outline_regions(None, None).is_empty());
            let strokes = spec.evaluate_polylines(None);
            assert!(
                strokes.len() >= 10,
                "expected multi-character strokes for {font:?}"
            );
            let aabb = strokes
                .iter()
                .fold(Aabb2::EMPTY, |acc, p| acc.union(&p.exact_aabb()));
            assert!(approx_eq(aabb.size().y, 6.0, 0.8));

            let mut mirrored = spec.clone();
            mirrored.h_align = TextHAlign::Left;
            let normal_aabb = mirrored
                .evaluate_polylines(None)
                .iter()
                .fold(Aabb2::EMPTY, |acc, p| acc.union(&p.exact_aabb()));
            mirrored.mirror_x = true;
            let mirror_aabb = mirrored
                .evaluate_polylines(None)
                .iter()
                .fold(Aabb2::EMPTY, |acc, p| acc.union(&p.exact_aabb()));
            assert!(normal_aabb.max.x > 10.0);
            assert!(mirror_aabb.min.x < -10.0);
        }
    }

    #[test]
    fn snap_engine_prioritizes_origin_endpoints_midpoints_centers_stock_and_grid() {
        let mut sk = SketchLayer::new("sk_1", "Snap Test", "setup_1");
        sk.entities.push(SketchEntityEntry {
            id: "rect_1".to_string(),
            name: "Rect".to_string(),
            sub_layer: None,
            is_construction: false,
            entity: SketchEntity::Rectangle {
                anchor: RectAnchorMode::FrontLeft,
                x: ParamLength::from_mm(10.0),
                y: ParamLength::from_mm(10.0),
                width: ParamLength::from_mm(40.0),
                height: ParamLength::from_mm(20.0),
                corner_radius: ParamLength::from_mm(0.0),
                rotation_deg: 0.0,
            },
        });
        let stock = Aabb2::from_points([DVec2::new(0.0, 0.0), DVec2::new(100.0, 80.0)]);
        let cad_pts = [DVec2::new(77.5, 42.25)];
        let ctx = SketchSnapContext {
            sketch: &sk,
            stock_xy_bounds: Some(stock),
            projected_cad_points: &cad_pts,
            snap_radius_mm: 2.0,
            grid_step_mm: Some(5.0),
        };

        let s_origin = snap_sketch_point_2d(DVec2::new(0.8, -0.5), &ctx);
        assert_eq!(s_origin.kind, Some(SnapKind::Origin));
        assert_eq!(s_origin.point_xy, DVec2::ZERO);

        let s_end = snap_sketch_point_2d(DVec2::new(50.6, 10.4), &ctx);
        assert_eq!(s_end.kind, Some(SnapKind::Endpoint));
        assert_eq!(s_end.point_xy, DVec2::new(50.0, 10.0));

        let s_mid = snap_sketch_point_2d(DVec2::new(30.4, 9.5), &ctx);
        assert_eq!(s_mid.kind, Some(SnapKind::Midpoint));
        assert_eq!(s_mid.point_xy, DVec2::new(30.0, 10.0));

        let s_stock = snap_sketch_point_2d(DVec2::new(99.2, 79.4), &ctx);
        assert_eq!(s_stock.kind, Some(SnapKind::StockCorner));
        assert_eq!(s_stock.point_xy, DVec2::new(100.0, 80.0));

        let s_cad = snap_sketch_point_2d(DVec2::new(78.1, 41.9), &ctx);
        assert_eq!(s_cad.kind, Some(SnapKind::CadFeature));
        assert_eq!(s_cad.point_xy, DVec2::new(77.5, 42.25));

        let s_grid = snap_sketch_point_2d(DVec2::new(63.8, 56.1), &ctx);
        assert_eq!(s_grid.kind, Some(SnapKind::Grid));
        assert_eq!(s_grid.point_xy, DVec2::new(65.0, 55.0));
    }

    #[test]
    fn milestone_33_dxf_import_caps_nested_insert_grid_expansion() {
        // 3-level nested INSERT grid (20x20 per level = 400^3 = 64M expansions) must fail cleanly
        let dxf_text = concat!(
            "0\nSECTION\n2\nBLOCKS\n",
            "0\nBLOCK\n8\n0\n2\nB0\n70\n0\n10\n0.0\n20\n0.0\n30\n0.0\n",
            "0\nLINE\n8\n0\n10\n0.0\n20\n0.0\n11\n1.0\n21\n0.0\n",
            "0\nENDBLK\n",
            "0\nBLOCK\n8\n0\n2\nB1\n70\n0\n10\n0.0\n20\n0.0\n30\n0.0\n",
            "0\nINSERT\n8\n0\n2\nB0\n10\n0.0\n20\n0.0\n70\n20\n71\n20\n44\n1.0\n45\n1.0\n",
            "0\nENDBLK\n",
            "0\nBLOCK\n8\n0\n2\nB2\n70\n0\n10\n0.0\n20\n0.0\n30\n0.0\n",
            "0\nINSERT\n8\n0\n2\nB1\n10\n0.0\n20\n0.0\n70\n20\n71\n20\n44\n1.0\n45\n1.0\n",
            "0\nENDBLK\n",
            "0\nENDSEC\n",
            "0\nSECTION\n2\nENTITIES\n",
            "0\nINSERT\n8\n0\n2\nB2\n10\n0.0\n20\n0.0\n70\n20\n71\n20\n44\n1.0\n45\n1.0\n",
            "0\nENDSEC\n0\nEOF\n",
        );
        let err = import_dxf_bytes(dxf_text.as_bytes(), &DxfImportOptions::default()).unwrap_err();
        assert!(
            matches!(err, DxfImportError::DxfParse(_)),
            "expected DxfParse complexity-limit error, got {err:?}"
        );
    }
}
