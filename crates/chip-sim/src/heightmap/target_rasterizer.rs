use chip_cad::{BodyId, CadDocument, SurfaceKind};
use chip_cam::{
    ExtractedFeature25D, GeometrySelectionItem, ResolvedSetup, extract_selection_features,
};
use chip_math::{Aabb2, ArcPolyline2D, DVec2};
use rayon::prelude::*;

use crate::verifier::SimVerificationConfig;

use super::HeightfieldStock;

/// Returns the per-operation [`SimVerificationConfig`] after applying any intentional
/// unmodeled sharp-edge chamfer break or curve-trace engraving allowance (`mm`).
#[inline]
pub(super) fn effective_verify_config_for_op(
    base: SimVerificationConfig,
    intentional_edge_break_mm: f64,
) -> SimVerificationConfig {
    let mut cfg = base;
    cfg.gouge_tol_mm += intentional_edge_break_mm.max(0.0);
    cfg
}

impl HeightfieldStock {
    /// Rasterizes the upper `+Z` envelope of `doc` (filtered to `target_bodies`, or all bodies if empty) onto `self.z_target`.
    ///
    /// Uses exact analytic `ArcPolyline2D::contains_point` for horizontal planar faces (`n_wcs.z > 0.999`)
    /// so circular holes, counterbores, and bosses suffer **zero** polygonal chord sag, combined with
    /// exact analytic surface `z(x, y)` evaluation on sloped planar/chamfer faces and barycentric Z on freeform faces.
    pub fn rasterize_target_brep(
        &mut self,
        doc: &CadDocument,
        setup: &ResolvedSetup,
        target_bodies: &[BodyId],
    ) {
        self.z_target.fill(f32::NEG_INFINITY);
        let include_body = |bid: BodyId| target_bodies.is_empty() || target_bodies.contains(&bid);

        // 1. Collect upward-facing horizontal planar faces with exact analytic `ArcPolyline2D` boundaries
        struct AnalyticHorizFace {
            outer: ArcPolyline2D,
            inners: Vec<ArcPolyline2D>,
            aabb: Aabb2,
            z_wcs: f32,
        }
        let mut horiz_faces: Vec<AnalyticHorizFace> = Vec::new();
        let mut handled_horiz_face_ids = vec![false; doc.faces.len()];

        for face in &doc.faces {
            if !include_body(face.body_id) {
                continue;
            }
            if let SurfaceKind::Plane { normal, .. } = face.surface {
                let n_wcs = setup.frame.world_to_local_dir(normal.normalize_or_zero());
                if n_wcs.z > 0.999
                    && let Some(binding) = doc.bind_face(face.id)
                    && let Ok(summary) = extract_selection_features(
                        doc,
                        setup,
                        &[GeometrySelectionItem::PlanarFace { binding }],
                    )
                {
                    for feat in summary.features {
                        if let ExtractedFeature25D::PlanarFace {
                            source_face_id,
                            outer_boundary_wcs,
                            inner_islands_wcs,
                            face_z_wcs,
                            ..
                        } = feat
                            && source_face_id == Some(face.id)
                            && !outer_boundary_wcs.segments.is_empty()
                        {
                            let aabb = outer_boundary_wcs.exact_aabb();
                            if aabb.is_valid() {
                                horiz_faces.push(AnalyticHorizFace {
                                    outer: outer_boundary_wcs,
                                    inners: inner_islands_wcs,
                                    aabb,
                                    z_wcs: face_z_wcs as f32,
                                });
                                if (face.id.0 as usize) < handled_horiz_face_ids.len() {
                                    handled_horiz_face_ids[face.id.0 as usize] = true;
                                }
                            }
                        }
                    }
                }
            }
        }

        // Parallel row evaluation for all exact horizontal B-Rep faces
        if !horiz_faces.is_empty() {
            let nx = self.nx;
            let ox = self.origin_x_mm;
            let oy = self.origin_y_mm;
            let dx = self.dx_mm;
            let dy = self.dy_mm;

            self.z_target
                .par_chunks_mut(nx)
                .enumerate()
                .for_each(|(j, row_z)| {
                    let y = oy + (j as f64 + 0.5) * dy;
                    for hf in &horiz_faces {
                        if y < hf.aabb.min.y - 1e-6 || y > hf.aabb.max.y + 1e-6 {
                            continue;
                        }
                        let i0 = (((hf.aabb.min.x - ox) / dx).floor() as isize).max(0) as usize;
                        let i1 = (((hf.aabb.max.x - ox) / dx).ceil() as isize)
                            .clamp(0, (nx - 1) as isize) as usize;
                        for (off, z_cell) in row_z[i0..=i1].iter_mut().enumerate() {
                            if *z_cell >= hf.z_wcs {
                                continue;
                            }
                            let i = i0 + off;
                            let pt = DVec2::new(ox + (i as f64 + 0.5) * dx, y);
                            if hf.outer.contains_point(pt)
                                && !hf.inners.iter().any(|inn| inn.contains_point(pt))
                            {
                                *z_cell = (*z_cell).max(hf.z_wcs);
                            }
                        }
                    }
                });
        }

        // 2. Rasterize sloped / conical / toroidal / freeform upward-facing triangles
        for tri in &doc.render_mesh.pick_triangles {
            let fid_idx = tri.face_id.0 as usize;
            if fid_idx < handled_horiz_face_ids.len() && handled_horiz_face_ids[fid_idx] {
                continue;
            }
            let Some(face) = doc.face(tri.face_id) else {
                continue;
            };
            if !include_body(face.body_id) {
                continue;
            }

            let v0 = setup.frame.world_to_local_point(tri.v0_mm);
            let v1 = setup.frame.world_to_local_point(tri.v1_mm);
            let v2 = setup.frame.world_to_local_point(tri.v2_mm);

            let e1 = v1 - v0;
            let e2 = v2 - v0;
            let tri_n = e1.cross(e2);
            let area2_xy = tri_n.z;
            // Skip vertical or downward-facing triangles (`area2_xy.abs() <= 1e-8` is vertical in XY)
            if area2_xy.abs() <= 1e-8 {
                continue;
            }
            let n_unit = tri_n.normalize_or_zero();
            if n_unit.z <= 1e-4 {
                continue;
            }

            let analytic_plane = if let SurfaceKind::Plane { origin_mm, normal } = face.surface {
                let o_wcs = setup.frame.world_to_local_point(origin_mm);
                let n_wcs = setup.frame.world_to_local_dir(normal.normalize_or_zero());
                if n_wcs.z.abs() > 1e-4 {
                    Some((o_wcs, n_wcs))
                } else {
                    None
                }
            } else {
                None
            };

            let min_x = v0.x.min(v1.x).min(v2.x);
            let max_x = v0.x.max(v1.x).max(v2.x);
            let min_y = v0.y.min(v1.y).min(v2.y);
            let max_y = v0.y.max(v1.y).max(v2.y);

            let (Some((i0, i1)), Some((j0, j1))) = (
                self.index_span_x(min_x, max_x),
                self.index_span_y(min_y, max_y),
            ) else {
                continue;
            };

            let inv_area2 = 1.0 / area2_xy;
            for j in j0..=j1 {
                let py = self.origin_y_mm + (j as f64 + 0.5) * self.dy_mm;
                let row = j * self.nx;
                for i in i0..=i1 {
                    let px = self.origin_x_mm + (i as f64 + 0.5) * self.dx_mm;
                    let w0 = ((v1.x - px) * (v2.y - py) - (v1.y - py) * (v2.x - px)) * inv_area2;
                    let w1 = ((v2.x - px) * (v0.y - py) - (v2.y - py) * (v0.x - px)) * inv_area2;
                    let w2 = 1.0 - w0 - w1;
                    if w0 >= -1e-6 && w1 >= -1e-6 && w2 >= -1e-6 {
                        let z_val = if let Some((o_wcs, n_wcs)) = analytic_plane {
                            o_wcs.z
                                - (n_wcs.x * (px - o_wcs.x) + n_wcs.y * (py - o_wcs.y)) / n_wcs.z
                        } else {
                            w0 * v0.z + w1 * v1.z + w2 * v2.z
                        };
                        let zf = z_val as f32;
                        if zf > self.z_target[row + i] {
                            self.z_target[row + i] = zf;
                        }
                    }
                }
            }
        }

        // 3. Rasterize upward-facing triangles from imported triangle mesh bodies (`STL`/`OBJ`/`3MF`)
        for mesh in &doc.meshes {
            if !include_body(mesh.id) {
                continue;
            }
            for tri in &mesh.triangles {
                let [i0, i1, i2] = tri.indices;
                let v0 = setup
                    .frame
                    .world_to_local_point(mesh.vertices_mm[i0 as usize]);
                let v1 = setup
                    .frame
                    .world_to_local_point(mesh.vertices_mm[i1 as usize]);
                let v2 = setup
                    .frame
                    .world_to_local_point(mesh.vertices_mm[i2 as usize]);

                let tri_n = (v1 - v0).cross(v2 - v0);
                let area2_xy = tri_n.z;
                if area2_xy.abs() <= 1e-8 {
                    continue;
                }
                let n_unit = tri_n.normalize_or_zero();
                if n_unit.z <= 1e-4 {
                    continue;
                }

                let min_x = v0.x.min(v1.x).min(v2.x);
                let max_x = v0.x.max(v1.x).max(v2.x);
                let min_y = v0.y.min(v1.y).min(v2.y);
                let max_y = v0.y.max(v1.y).max(v2.y);

                let (Some((i0x, i1x)), Some((j0y, j1y))) = (
                    self.index_span_x(min_x, max_x),
                    self.index_span_y(min_y, max_y),
                ) else {
                    continue;
                };

                let inv_area2 = 1.0 / area2_xy;
                for j in j0y..=j1y {
                    let py = self.origin_y_mm + (j as f64 + 0.5) * self.dy_mm;
                    let row = j * self.nx;
                    for i in i0x..=i1x {
                        let px = self.origin_x_mm + (i as f64 + 0.5) * self.dx_mm;
                        let w0 =
                            ((v1.x - px) * (v2.y - py) - (v1.y - py) * (v2.x - px)) * inv_area2;
                        let w1 =
                            ((v2.x - px) * (v0.y - py) - (v2.y - py) * (v0.x - px)) * inv_area2;
                        let w2 = 1.0 - w0 - w1;
                        if w0 >= -1e-6 && w1 >= -1e-6 && w2 >= -1e-6 {
                            let zf = (w0 * v0.z + w1 * v1.z + w2 * v2.z) as f32;
                            if zf > self.z_target[row + i] {
                                self.z_target[row + i] = zf;
                            }
                        }
                    }
                }
            }
        }
    }
}
