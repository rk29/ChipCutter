//! Segment deduplication, R-tree (`rstar`) endpoint gap-healing chainer, and Even-Odd planar
//! region + island hierarchy classification (`11-sketches-vectors.md`).

use chip_geom2d::is_simple_closed_polyline;
use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, ParamLength};
use rstar::{AABB, PointDistance, RTree, RTreeObject};

use crate::model::{
    PlanarVectorRegion, SketchDiagnosticKind, SketchEntity, SketchEntityEntry,
    SketchImportDiagnostic,
};

/// Unhealed curve or circle emitted by a DXF or SVG parser prior to deduplication & gap-healing.
#[derive(Debug, Clone)]
pub enum RawImportedItem {
    Curve {
        sub_layer: Option<String>,
        polyline: ArcPolyline2D,
    },
    Circle {
        sub_layer: Option<String>,
        center_xy: DVec2,
        diameter_mm: f64,
    },
}

/// Output of [`heal_and_classify_sketch_geometry`].
#[derive(Debug, Clone)]
pub struct HealedSketchGeometry {
    pub entities: Vec<SketchEntityEntry>,
    pub regions: Vec<PlanarVectorRegion>,
    pub diagnostics: Vec<SketchImportDiagnostic>,
}

#[derive(Clone, Copy)]
struct EndpointNode {
    chain_idx: usize,
    is_end: bool,
    pos: [f64; 2],
}

impl RTreeObject for EndpointNode {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_point(self.pos)
    }
}

impl PointDistance for EndpointNode {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        let dx = self.pos[0] - point[0];
        let dy = self.pos[1] - point[1];
        dx * dx + dy * dy
    }
}

/// Deduplicates overlapping segments, chains open curves across micro-gaps `<= join_tolerance_mm`,
/// and classifies all closed loops into outer `CCW` [`PlanarVectorRegion`]s paired with direct `CW` islands.
pub fn heal_and_classify_sketch_geometry(
    raw_items: Vec<RawImportedItem>,
    join_tolerance_mm: f64,
) -> HealedSketchGeometry {
    let join_tol = join_tolerance_mm.max(0.0);
    let mut diagnostics = Vec::new();
    let mut circles: Vec<(Option<String>, DVec2, f64)> = Vec::new();
    let mut closed_loops: Vec<(Option<String>, ArcPolyline2D)> = Vec::new();
    let mut open_segs: Vec<(Option<String>, ArcSegment2D)> = Vec::new();

    for item in raw_items {
        match item {
            RawImportedItem::Circle {
                sub_layer,
                center_xy,
                diameter_mm,
            } => {
                if center_xy.is_finite() && diameter_mm.is_finite() && diameter_mm > 1e-4 {
                    if !circles.iter().any(|(_, c, d)| {
                        c.distance(center_xy) <= 1e-4 && (*d - diameter_mm).abs() <= 1e-4
                    }) {
                        circles.push((sub_layer, center_xy, diameter_mm));
                    } else {
                        diagnostics.push(SketchImportDiagnostic {
                            kind: SketchDiagnosticKind::DuplicateSegmentRemoved,
                            location_xy: Some(center_xy),
                            message: format!("Removed duplicate Ø{diameter_mm:.3} mm circle."),
                        });
                    }
                } else {
                    diagnostics.push(SketchImportDiagnostic {
                        kind: SketchDiagnosticKind::DegenerateSkipped,
                        location_xy: None,
                        message: "Skipped zero/invalid diameter circle.".to_string(),
                    });
                }
            }
            RawImportedItem::Curve {
                sub_layer,
                polyline,
            } => {
                let valid_segs: Vec<ArcSegment2D> = polyline
                    .segments
                    .into_iter()
                    .filter(|s| s.start().is_finite() && s.end().is_finite() && s.length() > 1e-5)
                    .collect();
                if valid_segs.is_empty() {
                    continue;
                }
                let is_already_closed = polyline.closed
                    && valid_segs
                        .first()
                        .zip(valid_segs.last())
                        .is_some_and(|(a, b)| a.start().distance(b.end()) <= 1e-3);
                if is_already_closed {
                    closed_loops.push((
                        sub_layer,
                        ArcPolyline2D {
                            segments: valid_segs,
                            closed: true,
                        },
                    ));
                } else {
                    for s in valid_segs {
                        open_segs.push((sub_layer.clone(), s));
                    }
                }
            }
        }
    }

    // 1. Deduplicate exact/reversed overlapping open segments (`1e-4 mm` tolerance)
    let mut unique_open: Vec<(Option<String>, ArcSegment2D)> = Vec::with_capacity(open_segs.len());
    for (layer, seg) in open_segs {
        let is_dup = unique_open
            .iter()
            .any(|(_, existing)| are_segments_duplicate(existing, &seg, 1e-4));
        if is_dup {
            diagnostics.push(SketchImportDiagnostic {
                kind: SketchDiagnosticKind::DuplicateSegmentRemoved,
                location_xy: Some(seg.start()),
                message: "Removed duplicate overlapping curve segment.".to_string(),
            });
        } else {
            unique_open.push((layer, seg));
        }
    }

    // 2. R-tree gap-healing chainer on open segments (`coincidence_eps = 1e-4`, `heal_tol = join_tol`)
    let chain_tol = join_tol.max(1e-4);
    let (healed_closed, remaining_open) =
        chain_open_segments(unique_open, chain_tol, &mut diagnostics);
    closed_loops.extend(healed_closed);

    // 3. Build addressable `SketchEntityEntry` list (`Circle`s + closed `Polyline`s + open `Polyline`s)
    let mut entities: Vec<SketchEntityEntry> = Vec::new();
    let mut region_candidates: Vec<(Option<String>, ArcPolyline2D)> = Vec::new();

    for (idx, (sub_layer, center_xy, diameter_mm)) in circles.into_iter().enumerate() {
        let ent = SketchEntity::new_circle(
            ParamLength::from_mm(center_xy.x),
            ParamLength::from_mm(center_xy.y),
            ParamLength::from_mm(diameter_mm),
        );
        let poly = ent.to_arc_polyline();
        region_candidates.push((sub_layer.clone(), poly));
        entities.push(SketchEntityEntry {
            id: format!("circle_{}", idx + 1),
            name: format!("Circle Ø{diameter_mm:.2} mm"),
            sub_layer,
            is_construction: false,
            entity: ent,
        });
    }

    for (idx, (sub_layer, loop_poly)) in closed_loops.into_iter().enumerate() {
        if loop_poly.perimeter() <= 1e-3 || loop_poly.signed_area().abs() <= 1e-5 {
            continue;
        }
        region_candidates.push((sub_layer.clone(), loop_poly.clone()));
        entities.push(SketchEntityEntry {
            id: format!("loop_{}", idx + 1),
            name: format!("Closed Loop {}", idx + 1),
            sub_layer,
            is_construction: false,
            entity: SketchEntity::new_polyline(loop_poly),
        });
    }

    for (idx, (sub_layer, open_poly)) in remaining_open.into_iter().enumerate() {
        if let Some(s) = open_poly.segments.first().map(ArcSegment2D::start) {
            diagnostics.push(SketchImportDiagnostic {
                kind: SketchDiagnosticKind::OpenChainRemaining,
                location_xy: Some(s),
                message: format!(
                    "Open chain #{} ({} segs, L={:.2} mm) remains unclosed.",
                    idx + 1,
                    open_poly.segments.len(),
                    open_poly.perimeter()
                ),
            });
        }
        entities.push(SketchEntityEntry {
            id: format!("chain_{}", idx + 1),
            name: format!("Open Chain {}", idx + 1),
            sub_layer,
            is_construction: false,
            entity: SketchEntity::new_polyline(open_poly),
        });
    }

    // 4. Classify closed loops into Even-Odd nested `PlanarVectorRegion`s (`outer_ccw` + direct `islands_cw`)
    let regions = classify_planar_regions_from_candidates(region_candidates);

    HealedSketchGeometry {
        entities,
        regions,
        diagnostics,
    }
}

fn are_segments_duplicate(a: &ArcSegment2D, b: &ArcSegment2D, eps: f64) -> bool {
    let same_dir = a.start().distance(b.start()) <= eps && a.end().distance(b.end()) <= eps;
    let rev_dir = a.start().distance(b.end()) <= eps && a.end().distance(b.start()) <= eps;
    if !same_dir && !rev_dir {
        return false;
    }
    match (a.eval_at(0.5), b.eval_at(0.5)) {
        (Some((ma, _)), Some((mb, _))) => ma.distance(mb) <= 2.0 * eps,
        _ => false,
    }
}

type LayeredPolyline = (Option<String>, ArcPolyline2D);

fn chain_open_segments(
    segs: Vec<(Option<String>, ArcSegment2D)>,
    join_tol: f64,
    diagnostics: &mut Vec<SketchImportDiagnostic>,
) -> (Vec<LayeredPolyline>, Vec<LayeredPolyline>) {
    let mut chains: Vec<Option<(Option<String>, Vec<ArcSegment2D>)>> =
        segs.into_iter().map(|(l, s)| Some((l, vec![s]))).collect();

    let tol2 = join_tol * join_tol;

    // Iteratively merge closest open endpoints via R-tree until no pair lies within `join_tol`
    loop {
        let mut nodes = Vec::with_capacity(chains.len() * 2);
        for (idx, slot) in chains.iter().enumerate() {
            let Some((_, c_segs)) = slot else {
                continue;
            };
            if let (Some(first), Some(last)) = (c_segs.first(), c_segs.last()) {
                let s = first.start();
                let e = last.end();
                if c_segs.len() >= 2 && s.distance_squared(e) <= 1e-10 {
                    continue;
                }
                nodes.push(EndpointNode {
                    chain_idx: idx,
                    is_end: false,
                    pos: [s.x, s.y],
                });
                nodes.push(EndpointNode {
                    chain_idx: idx,
                    is_end: true,
                    pos: [e.x, e.y],
                });
            }
        }
        if nodes.len() < 2 {
            break;
        }
        let tree = RTree::bulk_load(nodes.clone());

        let mut best_merge: Option<(usize, bool, usize, bool, f64)> = None;
        for node in &nodes {
            for nb in tree.locate_within_distance(node.pos, tol2) {
                if nb.chain_idx == node.chain_idx && nb.is_end == node.is_end {
                    continue;
                }
                // Avoid merging same single-segment start with its own end unless chain length >= 2
                if nb.chain_idx == node.chain_idx {
                    let seg_len = chains[node.chain_idx].as_ref().map_or(0, |(_, v)| v.len());
                    if seg_len < 2 {
                        continue;
                    }
                }
                let d2 = nb.distance_2(&node.pos);
                if d2 <= tol2 && best_merge.is_none_or(|(_, _, _, _, prev_d2)| d2 < prev_d2) {
                    best_merge = Some((node.chain_idx, node.is_end, nb.chain_idx, nb.is_end, d2));
                }
            }
        }

        let Some((idx_a, a_is_end, idx_b, b_is_end, d2)) = best_merge else {
            break;
        };
        let gap = d2.sqrt();

        if idx_a == idx_b {
            // Close chain `idx_a` onto itself!
            if let Some((_, ref mut c_segs)) = chains[idx_a]
                && let (Some(first), Some(last)) = (c_segs.first().copied(), c_segs.last().copied())
            {
                let start_pt = first.start();
                let end_pt = last.end();
                if gap > 1e-4 {
                    diagnostics.push(SketchImportDiagnostic {
                        kind: SketchDiagnosticKind::HealedEndpointGap,
                        location_xy: Some(end_pt),
                        message: format!("Healed {gap:.4} mm loop closure gap."),
                    });
                    c_segs.push(ArcSegment2D::Line {
                        start: end_pt,
                        end: start_pt,
                    });
                } else if gap > 1e-8 {
                    // Snap micro-epsilon endpoint directly
                    if let Some(last_mut) = c_segs.last_mut() {
                        *last_mut = snap_segment_end(*last_mut, start_pt);
                    }
                }
            }
            continue;
        }

        let Some((layer_b, mut segs_b)) = chains[idx_b].take() else {
            continue;
        };
        let Some((layer_a, mut segs_a)) = chains[idx_a].take() else {
            chains[idx_b] = Some((layer_b, segs_b));
            continue;
        };

        // Orient `segs_a` so the joining endpoint is at `segs_a.end()`, and `segs_b` so it is at `segs_b.start()`
        if !a_is_end {
            segs_a = reverse_segment_vec(&segs_a);
        }
        if b_is_end {
            segs_b = reverse_segment_vec(&segs_b);
        }

        if let (Some(a_last), Some(b_first)) = (segs_a.last().copied(), segs_b.first().copied()) {
            let pa = a_last.end();
            let pb = b_first.start();
            if gap > 1e-4 {
                diagnostics.push(SketchImportDiagnostic {
                    kind: SketchDiagnosticKind::HealedEndpointGap,
                    location_xy: Some(pa),
                    message: format!("Healed {gap:.4} mm gap between curve endpoints."),
                });
                segs_a.push(ArcSegment2D::Line { start: pa, end: pb });
            } else if gap > 1e-8
                && let Some(last_mut) = segs_a.last_mut()
            {
                *last_mut = snap_segment_end(*last_mut, pb);
            }
        }
        segs_a.extend(segs_b);
        chains[idx_a] = Some((layer_a.or(layer_b), segs_a));
    }

    let mut closed_out = Vec::new();
    let mut open_out = Vec::new();
    for slot in chains.into_iter().flatten() {
        let (layer, segs) = slot;
        let is_closed = segs.len() >= 2
            && segs
                .first()
                .zip(segs.last())
                .is_some_and(|(a, b)| a.start().distance(b.end()) <= 1e-4);
        if is_closed {
            closed_out.push((
                layer,
                ArcPolyline2D {
                    segments: segs,
                    closed: true,
                },
            ));
        } else {
            open_out.push((
                layer,
                ArcPolyline2D {
                    segments: segs,
                    closed: false,
                },
            ));
        }
    }
    (closed_out, open_out)
}

fn reverse_segment_vec(segs: &[ArcSegment2D]) -> Vec<ArcSegment2D> {
    let poly = ArcPolyline2D {
        segments: segs.to_vec(),
        closed: false,
    };
    poly.reverse().segments
}

fn snap_segment_end(seg: ArcSegment2D, new_end: DVec2) -> ArcSegment2D {
    match seg {
        ArcSegment2D::Line { start, .. } => ArcSegment2D::Line {
            start,
            end: new_end,
        },
        ArcSegment2D::Arc {
            start,
            center,
            radius,
            ccw,
            ..
        } => ArcSegment2D::Arc {
            start,
            end: new_end,
            center,
            radius,
            ccw,
        },
    }
}

/// Organizes closed candidate loops (`(sub_layer, ArcPolyline2D)`) into Even-Odd nested
/// [`PlanarVectorRegion`]s (`outer_ccw` paired with direct `islands_cw`).
pub fn classify_planar_regions_from_candidates(
    candidates: Vec<(Option<String>, ArcPolyline2D)>,
) -> Vec<PlanarVectorRegion> {
    struct LoopItem {
        sub_layer: Option<String>,
        ccw: ArcPolyline2D,
        abs_area: f64,
        sample_pt: DVec2,
    }

    // Deduplicate coincident closed loops WITHIN the same `sub_layer`
    let mut all_loops: Vec<LoopItem> = Vec::new();
    let mut layer_keys: Vec<Option<String>> = Vec::new();
    for (sub_layer, poly) in candidates {
        let ccw = poly.ensure_orientation(true);
        let abs_area = ccw.signed_area().abs();
        if abs_area <= 1e-4 {
            continue;
        }
        // Skip self-intersecting figure-8 loops by checking `chip_geom2d`'s canonical simplicity check
        if !is_simple_closed_polyline(&ccw) {
            continue;
        }
        let Some((sample_pt, _)) = ccw.sample_at_fraction(0.137) else {
            continue;
        };
        let dup = all_loops.iter().any(|existing| {
            existing.sub_layer == sub_layer
                && (existing.abs_area - abs_area).abs() <= 1e-2
                && existing.ccw.distance_to_point(sample_pt) <= 1e-3
        });
        if !dup {
            if !layer_keys.contains(&sub_layer) {
                layer_keys.push(sub_layer.clone());
            }
            all_loops.push(LoopItem {
                sub_layer,
                ccw,
                abs_area,
                sample_pt,
            });
        }
    }

    // Perform Even-Odd planar region + island classification per `sub_layer` so multi-layer CAD
    // drawings (`OUTLINE`, `POCKETS`, `HOLES`, `CUTOUTS`) and grouped SVGs (`<g id="...">`) keep
    // each layer's pockets + interior islands independent of the outer sheet perimeter, while
    // single-layer drawings (`"0"` or `None`) still nest Even-Odd across all loops!
    let mut regions = Vec::new();
    for key in layer_keys {
        let mut loops: Vec<&LoopItem> = all_loops.iter().filter(|l| l.sub_layer == key).collect();
        loops.sort_by(|a, b| b.abs_area.total_cmp(&a.abs_area));

        let n = loops.len();
        let mut parent: Vec<Option<usize>> = vec![None; n];
        let mut depth: Vec<usize> = vec![0; n];

        for i in 0..n {
            let mut direct_parent: Option<usize> = None;
            for j in (0..i).rev() {
                if loops[j].ccw.contains_point(loops[i].sample_pt) {
                    direct_parent = Some(j);
                    break;
                }
            }
            parent[i] = direct_parent;
            depth[i] = direct_parent.map_or(0, |p| depth[p] + 1);
        }

        for i in 0..n {
            if !depth[i].is_multiple_of(2) {
                continue;
            }
            let mut islands_cw = Vec::new();
            let mut isl_area_sum = 0.0;
            for child in (i + 1)..n {
                if parent[child] == Some(i) {
                    isl_area_sum += loops[child].abs_area;
                    islands_cw.push(loops[child].ccw.ensure_orientation(false));
                }
            }
            let net_area = (loops[i].abs_area - isl_area_sum).max(0.0);
            if net_area > 1e-4 {
                regions.push(PlanarVectorRegion {
                    id: format!("region_{}", regions.len() + 1),
                    sub_layer: loops[i].sub_layer.clone(),
                    outer_ccw: loops[i].ccw.clone(),
                    islands_cw,
                    area_mm2: net_area,
                });
            }
        }
    }
    regions
}
