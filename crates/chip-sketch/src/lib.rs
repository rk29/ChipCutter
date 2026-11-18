//! 2D Sketch Layers, DXF & SVG Vector Importers, Bézier-to-Biarc Fitting, Gap-Healing,
//! and Planar Region Classification (`chip-sketch`).

pub mod curve_fit;
pub mod dxf_import;
pub mod heal_chain;
pub mod model;
pub mod picker;
pub mod svg_import;

pub use curve_fit::{circumcircle_through_three_points, fit_bezpath_to_arc_polylines};
pub use dxf_import::{
    DxfImportError, DxfImportOptions, DxfImportReport, import_dxf_bytes, import_dxf_path,
    sample_router_panel_dxf,
};
pub use heal_chain::{HealedSketchGeometry, RawImportedItem, heal_and_classify_sketch_geometry};
pub use model::{
    PlanarVectorRegion, SketchAnchor, SketchDiagnosticKind, SketchEntity, SketchEntityEntry,
    SketchImportDiagnostic, SketchLayer, SketchPlaneRef, VectorAssetFormat, VectorImportUnits,
    VectorSourceAsset, compute_sketch_sha256_hex,
};
pub use picker::{SketchPickHit, pick_sketch_layer_2d};
pub use svg_import::{
    SvgImportError, SvgImportOptions, SvgImportReport, import_svg_bytes, import_svg_path,
    sample_badge_logo_svg,
};

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::{ArcSegment2D, DVec2, approx_eq};
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
        // Standard 4-quadrant cubic Bézier circle fits in <= 8 true circular arcs (zero G1 faceting!)
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

        // 4x Ø6mm mounting circles preserved natively as `SketchEntity::Circle`!
        assert_eq!(sk.circle_count(), 4);
        // Zero unclosed chains left because the 0.03mm gap at (50.03, 33.0) was healed under 0.05mm tol!
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

        // Verify per-layer Even-Odd nesting on full multi-layer DXF:
        // Clicking at `(55, 50)` hits the `POCKETS` region (`[50..110] x [33..67]` minus `[70..90] x [44..56]` island)!
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

        // Picker hits circle boundary & enclosing region accurately
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

        // Outer plaque (`92x52`) encloses the R=18 ring (`depth 1`), which encloses the R=10 center (`depth 2`),
        // so we get 2 Even-Odd regions! And the emblem arcs are fitted as true circular arcs!
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
}
