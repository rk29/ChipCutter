use chip_math::DVec3;
use rayon::prelude::*;

use crate::{
    mesher_common::op_color,
    metrics::{
        SimStockColorMode, deviation_to_heatmap_rgba, signed_deviation_at_machined_endpoint,
    },
    types::{Axis3, SimColoredVertex, SimMeshOptions, SimStockMesh, UNCUT_OP_IDX},
};

use super::grid::{DexelSpan, TriDexelStock};

/// Cosine of the 38° sharp-crease threshold (`cos(38°) ≈ 0.788`) separating smooth curved
/// surfaces from prismatic 90° pocket walls, floors, and machined edges.
const CREASE_COS_THRESHOLD: f64 = 0.788;
const TAUBIN_LAMBDA: f64 = 0.45;
const TAUBIN_MU: f64 = -0.47;
const MAX_RELAX_DISP_RATIO: f64 = 0.45;

impl TriDexelStock {
    /// Extracts a 3D GPU-ready [`SimStockMesh`] from this Tri-Dexel solid using the requested
    /// visual [`SimMeshOptions`] (`color_mode` + optional crease-preserving `mesh_skinning`).
    pub fn extract_mesh_with_options(
        &self,
        target_stock: Option<&TriDexelStock>,
        options: SimMeshOptions,
    ) -> SimStockMesh {
        tessellate_tridexel_stock_mesh_with_options(self, target_stock, options)
    }
}

/// Finds the dexel span endpoint (`min_mm` if `want_min` is true, else `max_mm`) closest to
/// `target_coord` within `max_dist`, returning `(exact_coord, op_idx)`.
#[inline]
fn nearest_ray_endpoint(
    spans: &[DexelSpan],
    target_coord: f64,
    want_min: bool,
    max_dist: f64,
) -> Option<(f64, u32)> {
    let mut best: Option<(f64, f64, u32)> = None;
    for s in spans {
        let (c, op) = if want_min {
            (s.min_mm, s.min_op_idx)
        } else {
            (s.max_mm, s.max_op_idx)
        };
        let d = (c - target_coord).abs();
        if d <= max_dist && best.is_none_or(|(bd, _, _)| d < bd) {
            best = Some((d, c, op));
        }
    }
    best.map(|(_, c, op)| (c, op))
}

/// Evaluates a crease-aware smooth surface normal on the dominant dexel plane of `n_face` so
/// curved cylinders, spheres, domes, and helical flutes shade seamlessly across X/Y/Z face seams
/// while sharp 90° edges fall back crisply to `n_face`.
#[inline]
fn crease_aware_smooth_normal(stock: &TriDexelStock, center_wcs: DVec3, n_face: DVec3) -> DVec3 {
    let ax = n_face.x.abs();
    let ay = n_face.y.abs();
    let az = n_face.z.abs();
    let dom_ax = if ax >= ay && ax >= az {
        Axis3::X
    } else if ay >= az {
        Axis3::Y
    } else {
        Axis3::Z
    };

    let u_ax = dom_ax.u_axis();
    let v_ax = dom_ax.v_axis();
    let plane = stock.plane(dom_ax);
    let du = plane.du_mm;
    let dv = plane.dv_mm;
    let dw = stock.pitch(dom_ax);
    let nu = plane.nu as isize;
    let nv = plane.nv as isize;

    let diu = (((u_ax.get(center_wcs) - plane.origin_u_mm) / du).floor() as isize)
        .clamp(0, (nu - 1).max(0));
    let div = (((v_ax.get(center_wcs) - plane.origin_v_mm) / dv).floor() as isize)
        .clamp(0, (nv - 1).max(0));
    let cw = dom_ax.get(center_wcs);
    let dom_want_min = dom_ax.get(n_face) < 0.0;
    let max_d = 1.35 * dw;

    let query = |qu: isize, qv: isize, ref_w: f64| -> Option<f64> {
        if qu < 0 || qv < 0 || qu >= nu || qv >= nv {
            return None;
        }
        nearest_ray_endpoint(
            &plane.ray_spans(qu as usize, qv as usize),
            ref_w,
            dom_want_min,
            max_d,
        )
        .map(|(c, _)| c)
    };

    let Some(wc) = query(diu, div, cw) else {
        return n_face;
    };
    let ql = query(diu - 1, div, wc);
    let qr = query(diu + 1, div, wc);
    let qd = query(diu, div - 1, wc);
    let qu = query(diu, div + 1, wc);

    let dw_du = match (ql, qr) {
        (Some(wl), Some(wr)) => (wr - wl) / (2.0 * du),
        (None, Some(wr)) => (wr - wc) / du,
        (Some(wl), None) => (wc - wl) / du,
        (None, None) => 0.0,
    };
    let dw_dv = match (qd, qu) {
        (Some(wd), Some(wu)) => (wu - wd) / (2.0 * dv),
        (None, Some(wu)) => (wu - wc) / dv,
        (Some(wd), None) => (wc - wd) / dv,
        (None, None) => 0.0,
    };

    let n_uvw = if dom_want_min {
        DVec3::new(dw_du, dw_dv, -1.0)
    } else {
        DVec3::new(-dw_du, -dw_dv, 1.0)
    };
    let n_smooth = dom_ax
        .from_uvw(n_uvw.x, n_uvw.y, n_uvw.z)
        .normalize_or_zero();
    if n_smooth.dot(n_face) >= 0.72 {
        n_smooth
    } else {
        n_face
    }
}

/// Extracts a watertight, sharp-feature-preserving 3D [`SimStockMesh`] from a [`TriDexelStock`]
/// using axis-decoupled Dual-Vertex Surface Nets.
pub fn tessellate_tridexel_stock_mesh(stock: &TriDexelStock, color_by_op: bool) -> SimStockMesh {
    let mode = if color_by_op {
        SimStockColorMode::ByOperation
    } else {
        SimStockColorMode::Metallic
    };
    tessellate_tridexel_stock_mesh_colored(stock, None, mode)
}

/// Extracts a watertight, sharp-feature-preserving 3D [`SimStockMesh`] from a [`TriDexelStock`]
/// with selectable [`SimStockColorMode`] (`Metallic`, `ByOperation`, or signed `ErrorHeatmap`
/// comparing against `target_stock`).
pub fn tessellate_tridexel_stock_mesh_colored(
    stock: &TriDexelStock,
    target_stock: Option<&TriDexelStock>,
    color_mode: SimStockColorMode,
) -> SimStockMesh {
    tessellate_tridexel_stock_mesh_with_options(
        stock,
        target_stock,
        SimMeshOptions::from(color_mode),
    )
}

/// Extracts a watertight, sharp-feature-preserving 3D [`SimStockMesh`] from a [`TriDexelStock`]
/// using visual [`SimMeshOptions`] (`color_mode` + optional crease-preserving `mesh_skinning`).
pub fn tessellate_tridexel_stock_mesh_with_options(
    stock: &TriDexelStock,
    target_stock: Option<&TriDexelStock>,
    options: SimMeshOptions,
) -> SimStockMesh {
    let color_mode = options.color_mode;
    let color_by_op = matches!(color_mode, SimStockColorMode::ByOperation);
    let [nx, ny, nz] = stock.dims;
    if nx == 0 || ny == 0 || nz == 0 {
        return SimStockMesh::default();
    }

    // 1. Build padded occupancy volume `(nx + 2) × (ny + 2) × (nz + 2)` in parallel via 3-axis majority vote
    let pnx = nx + 2;
    let pny = ny + 2;
    let pnz = nz + 2;
    let mut occ = vec![false; pnx * pny * pnz];
    let occ_slice = pnx * pny;
    occ[occ_slice..(nz + 1) * occ_slice]
        .par_chunks_mut(occ_slice)
        .enumerate()
        .for_each(|(k, slice)| {
            for j in 0..ny {
                let row = (j + 1) * pnx;
                for i in 0..nx {
                    if stock.is_voxel_solid(i, j, k) {
                        slice[row + i + 1] = true;
                    }
                }
            }
        });
    let occ_idx = |ijk: [usize; 3]| -> usize { (ijk[2] * pny + ijk[1]) * pnx + ijk[0] };

    // 2. Compute dual vertex positions on corners `(0..=nx) × (0..=ny) × (0..=nz)` in parallel
    let dnx = nx + 1;
    let dny = ny + 1;
    let dnz = nz + 1;
    let dual_idx = |ijk: [usize; 3]| -> usize { (ijk[2] * dny + ijk[1]) * dnx + ijk[0] };
    let mut dual_pos: Vec<[f32; 3]> = vec![[0.0; 3]; dnx * dny * dnz];

    let dual_slice = dnx * dny;
    dual_pos
        .par_chunks_mut(dual_slice)
        .enumerate()
        .for_each(|(dk, slice)| {
            for dj in 0..dny {
                let row = dj * dnx;
                for di in 0..dnx {
                    // Check if the 8 surrounding padded voxels have a sign change
                    let mut any_in = false;
                    let mut any_out = false;
                    for oz in 0..=1 {
                        for oy in 0..=1 {
                            for ox in 0..=1 {
                                if occ[occ_idx([di + ox, dj + oy, dk + oz])] {
                                    any_in = true;
                                } else {
                                    any_out = true;
                                }
                            }
                        }
                    }
                    if !(any_in && any_out) {
                        continue;
                    }

                    let d_ijk = [di, dj, dk];
                    let mut pos = [0.0_f32; 3];

                    // Axis-generic dual-vertex crossing accumulation across X, Y, and Z!
                    for axis in Axis3::ALL {
                        let ax_i = axis.index();
                        let u_ax = axis.u_axis();
                        let v_ax = axis.v_axis();
                        let nu = stock.dim(u_ax);
                        let nv = stock.dim(v_ax);
                        let du_c = u_ax.get_usize(d_ijk);
                        let dv_c = v_ax.get_usize(d_ijk);
                        let dw = axis.get_usize(d_ijk);
                        let pitch_w = stock.pitch(axis);
                        let base_w = axis.get(stock.origin_wcs) + (dw as f64) * pitch_w;
                        let max_dw = 1.05 * pitch_w;
                        let plane = stock.plane(axis);

                        let mut w_sum = 0.0;
                        let mut w_cnt = 0u32;
                        for ov in 0..=1usize {
                            let Some(iv) = (dv_c + ov).checked_sub(1) else {
                                continue;
                            };
                            if iv >= nv {
                                continue;
                            }
                            for ou in 0..=1usize {
                                let Some(iu) = (du_c + ou).checked_sub(1) else {
                                    continue;
                                };
                                if iu >= nu {
                                    continue;
                                }
                                let lo = occ[occ_idx(axis.from_uvw_usize(iu + 1, iv + 1, dw))];
                                let hi = occ[occ_idx(axis.from_uvw_usize(iu + 1, iv + 1, dw + 1))];
                                if lo != hi {
                                    let want_min = !lo && hi;
                                    if let Some((wc, _)) = nearest_ray_endpoint(
                                        &plane.ray_spans(iu, iv),
                                        base_w,
                                        want_min,
                                        max_dw,
                                    ) {
                                        w_sum += wc;
                                        w_cnt += 1;
                                    }
                                }
                            }
                        }

                        let coord = if w_cnt > 0 {
                            (w_sum / f64::from(w_cnt))
                                .clamp(base_w - 0.55 * pitch_w, base_w + 0.55 * pitch_w)
                        } else {
                            base_w
                        };
                        pos[ax_i] = coord as f32;
                    }

                    slice[row + di] = pos;
                }
            }
        });

    if !options.mesh_skinning {
        // Crisp path (unchanged behavior): emit outward-oriented quads with a single normal per quad
        let mut mesh = SimStockMesh {
            vertices: Vec::with_capacity(nx * ny * 4),
            indices: Vec::with_capacity(nx * ny * 6),
        };

        let push_quad = |mesh: &mut SimStockMesh,
                         p0: [f32; 3],
                         p1: [f32; 3],
                         p2: [f32; 3],
                         p3: [f32; 3],
                         fallback_n: [f32; 3],
                         color: [f32; 4]| {
            let v0 = DVec3::new(f64::from(p0[0]), f64::from(p0[1]), f64::from(p0[2]));
            let v1 = DVec3::new(f64::from(p1[0]), f64::from(p1[1]), f64::from(p1[2]));
            let v2 = DVec3::new(f64::from(p2[0]), f64::from(p2[1]), f64::from(p2[2]));
            let v3 = DVec3::new(f64::from(p3[0]), f64::from(p3[1]), f64::from(p3[2]));
            let n_raw = (v2 - v0).cross(v3 - v1);
            let n_len = n_raw.length();
            let n_face = if n_len > 1e-10 {
                n_raw / n_len
            } else {
                DVec3::new(
                    f64::from(fallback_n[0]),
                    f64::from(fallback_n[1]),
                    f64::from(fallback_n[2]),
                )
            };
            let center_wcs = 0.25 * (v0 + v1 + v2 + v3);
            let n_smooth = crease_aware_smooth_normal(stock, center_wcs, n_face);
            let normal = [n_smooth.x as f32, n_smooth.y as f32, n_smooth.z as f32];
            let base = mesh.vertices.len() as u32;
            mesh.vertices.extend_from_slice(&[
                SimColoredVertex {
                    pos_mm: p0,
                    normal,
                    color_rgba: color,
                },
                SimColoredVertex {
                    pos_mm: p1,
                    normal,
                    color_rgba: color,
                },
                SimColoredVertex {
                    pos_mm: p2,
                    normal,
                    color_rgba: color,
                },
                SimColoredVertex {
                    pos_mm: p3,
                    normal,
                    color_rgba: color,
                },
            ]);
            mesh.indices
                .extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
        };

        for axis in [Axis3::Z, Axis3::X, Axis3::Y] {
            let u_ax = axis.u_axis();
            let v_ax = axis.v_axis();
            let nu = stock.dim(u_ax);
            let nv = stock.dim(v_ax);
            let dw_len = stock.dim(axis) + 1;
            let pitch_w = stock.pitch(axis);
            let max_dw = 1.05 * pitch_w;
            let origin_w = axis.get(stock.origin_wcs);
            let plane = stock.plane(axis);
            let target_plane = target_stock.map(|t| t.plane(axis));
            let is_side = axis.is_side_axis();
            let corners = axis.quad_corner_offsets();
            let pos_n = axis.unit_vec_f32();
            let neg_n = [-pos_n[0], -pos_n[1], -pos_n[2]];

            for dw in 0..dw_len {
                let base_w = origin_w + (dw as f64) * pitch_w;
                for iv in 0..nv {
                    for iu in 0..nu {
                        let lo = occ[occ_idx(axis.from_uvw_usize(iu + 1, iv + 1, dw))];
                        let hi = occ[occ_idx(axis.from_uvw_usize(iu + 1, iv + 1, dw + 1))];
                        if lo == hi {
                            continue;
                        }
                        let want_min = !lo && hi;
                        let hit = nearest_ray_endpoint(
                            &plane.ray_spans(iu, iv),
                            base_w,
                            want_min,
                            max_dw,
                        );
                        let (coord_w, op) = hit.unwrap_or((base_w, UNCUT_OP_IDX));
                        let col = match (color_mode, target_plane) {
                            (
                                SimStockColorMode::ErrorHeatmap {
                                    tolerance_mm,
                                    max_scale_mm,
                                },
                                Some(tp),
                            ) => {
                                let dev = signed_deviation_at_machined_endpoint(
                                    &tp.ray_spans(iu, iv),
                                    coord_w,
                                    want_min,
                                );
                                deviation_to_heatmap_rgba(dev, tolerance_mm, max_scale_mm)
                            }
                            _ => op_color(op, color_by_op, is_side),
                        };

                        let a = dual_pos[dual_idx(axis.from_uvw_usize(
                            iu + corners[0].0,
                            iv + corners[0].1,
                            dw,
                        ))];
                        let b = dual_pos[dual_idx(axis.from_uvw_usize(
                            iu + corners[1].0,
                            iv + corners[1].1,
                            dw,
                        ))];
                        let c = dual_pos[dual_idx(axis.from_uvw_usize(
                            iu + corners[2].0,
                            iv + corners[2].1,
                            dw,
                        ))];
                        let d = dual_pos[dual_idx(axis.from_uvw_usize(
                            iu + corners[3].0,
                            iv + corners[3].1,
                            dw,
                        ))];
                        if lo && !hi {
                            push_quad(&mut mesh, a, b, c, d, pos_n, col);
                        } else {
                            push_quad(&mut mesh, a, d, c, b, neg_n, col);
                        }
                    }
                }
            }
        }

        return mesh;
    }

    // 3. Mesh Skinning path: collect indexed Surface Nets quads + compact active dual vertices
    struct RawQuad {
        verts: [u32; 4],
        fallback_n: [f32; 3],
        color: [f32; 4],
    }

    let mut dual_to_compact = vec![u32::MAX; dnx * dny * dnz];
    let mut orig_pos: Vec<DVec3> = Vec::with_capacity(nx * ny);
    let mut is_boundary: Vec<bool> = Vec::with_capacity(nx * ny);
    let mut raw_quads: Vec<RawQuad> = Vec::with_capacity(nx * ny);

    let mut intern_vert = |ijk: [usize; 3]| -> u32 {
        let d_id = dual_idx(ijk);
        let existing = dual_to_compact[d_id];
        if existing != u32::MAX {
            return existing;
        }
        let cid = orig_pos.len() as u32;
        dual_to_compact[d_id] = cid;
        let p = dual_pos[d_id];
        orig_pos.push(DVec3::new(
            f64::from(p[0]),
            f64::from(p[1]),
            f64::from(p[2]),
        ));
        let bnd = ijk[0] == 0
            || ijk[0] == nx
            || ijk[1] == 0
            || ijk[1] == ny
            || ijk[2] == 0
            || ijk[2] == nz;
        is_boundary.push(bnd);
        cid
    };

    for axis in [Axis3::Z, Axis3::X, Axis3::Y] {
        let u_ax = axis.u_axis();
        let v_ax = axis.v_axis();
        let nu = stock.dim(u_ax);
        let nv = stock.dim(v_ax);
        let dw_len = stock.dim(axis) + 1;
        let pitch_w = stock.pitch(axis);
        let max_dw = 1.05 * pitch_w;
        let origin_w = axis.get(stock.origin_wcs);
        let plane = stock.plane(axis);
        let target_plane = target_stock.map(|t| t.plane(axis));
        let is_side = axis.is_side_axis();
        let corners = axis.quad_corner_offsets();
        let pos_n = axis.unit_vec_f32();
        let neg_n = [-pos_n[0], -pos_n[1], -pos_n[2]];

        for dw in 0..dw_len {
            let base_w = origin_w + (dw as f64) * pitch_w;
            for iv in 0..nv {
                for iu in 0..nu {
                    let lo = occ[occ_idx(axis.from_uvw_usize(iu + 1, iv + 1, dw))];
                    let hi = occ[occ_idx(axis.from_uvw_usize(iu + 1, iv + 1, dw + 1))];
                    if lo == hi {
                        continue;
                    }
                    let want_min = !lo && hi;
                    let hit =
                        nearest_ray_endpoint(&plane.ray_spans(iu, iv), base_w, want_min, max_dw);
                    let (coord_w, op) = hit.unwrap_or((base_w, UNCUT_OP_IDX));
                    let col = match (color_mode, target_plane) {
                        (
                            SimStockColorMode::ErrorHeatmap {
                                tolerance_mm,
                                max_scale_mm,
                            },
                            Some(tp),
                        ) => {
                            let dev = signed_deviation_at_machined_endpoint(
                                &tp.ray_spans(iu, iv),
                                coord_w,
                                want_min,
                            );
                            deviation_to_heatmap_rgba(dev, tolerance_mm, max_scale_mm)
                        }
                        _ => op_color(op, color_by_op, is_side),
                    };

                    let va =
                        intern_vert(axis.from_uvw_usize(iu + corners[0].0, iv + corners[0].1, dw));
                    let vb =
                        intern_vert(axis.from_uvw_usize(iu + corners[1].0, iv + corners[1].1, dw));
                    let vc =
                        intern_vert(axis.from_uvw_usize(iu + corners[2].0, iv + corners[2].1, dw));
                    let vd =
                        intern_vert(axis.from_uvw_usize(iu + corners[3].0, iv + corners[3].1, dw));
                    let (verts, fallback_n) = if lo && !hi {
                        ([va, vb, vc, vd], pos_n)
                    } else {
                        ([va, vd, vc, vb], neg_n)
                    };
                    raw_quads.push(RawQuad {
                        verts,
                        fallback_n,
                        color: col,
                    });
                }
            }
        }
    }

    let num_verts = orig_pos.len();
    let num_quads = raw_quads.len();
    if num_verts == 0 || num_quads == 0 {
        return SimStockMesh::default();
    }

    // 4. Build CSR vertex -> incident quads adjacency
    let mut deg = vec![0u32; num_verts];
    for q in &raw_quads {
        for &v in &q.verts {
            deg[v as usize] += 1;
        }
    }
    let mut offsets = vec![0usize; num_verts + 1];
    for v in 0..num_verts {
        offsets[v + 1] = offsets[v] + deg[v] as usize;
    }
    let mut vert_quads = vec![0u32; offsets[num_verts]];
    let mut write_pos = offsets[..num_verts].to_vec();
    for (qi, q) in raw_quads.iter().enumerate() {
        for &v in &q.verts {
            let vi = v as usize;
            vert_quads[write_pos[vi]] = qi as u32;
            write_pos[vi] += 1;
        }
    }

    // 5. Compute initial geometric quad normals & pin outer stock boundaries + sharp creases
    let init_quad_n: Vec<DVec3> = raw_quads
        .par_iter()
        .map(|q| {
            let v0 = orig_pos[q.verts[0] as usize];
            let v1 = orig_pos[q.verts[1] as usize];
            let v2 = orig_pos[q.verts[2] as usize];
            let v3 = orig_pos[q.verts[3] as usize];
            let n_raw = (v2 - v0).cross(v3 - v1);
            let len_sq = n_raw.length_squared();
            if len_sq > 1e-20 {
                n_raw / len_sq.sqrt()
            } else {
                DVec3::new(
                    f64::from(q.fallback_n[0]),
                    f64::from(q.fallback_n[1]),
                    f64::from(q.fallback_n[2]),
                )
            }
        })
        .collect();

    let pinned: Vec<bool> = (0..num_verts)
        .into_par_iter()
        .map(|v| {
            if is_boundary[v] {
                return true;
            }
            let inc = &vert_quads[offsets[v]..offsets[v + 1]];
            for a in 0..inc.len() {
                let na = init_quad_n[inc[a] as usize];
                for &qb in &inc[(a + 1)..] {
                    if na.dot(init_quad_n[qb as usize]) < CREASE_COS_THRESHOLD {
                        return true;
                    }
                }
            }
            false
        })
        .collect();

    // 6. Run 2 passes of volume-preserving Taubin (λ = 0.45, μ = -0.47) relaxation on unpinned
    //    smooth vertices, clamping total displacement from original dual vertex to <= 0.45 * pitch
    let min_pitch = stock.pitch_mm.x.min(stock.pitch_mm.y).min(stock.pitch_mm.z);
    let max_disp = MAX_RELAX_DISP_RATIO * min_pitch;
    let max_disp_sq = max_disp * max_disp;

    let mut cur_pos = orig_pos.clone();
    let mut next_pos = orig_pos.clone();

    for _pass in 0..2 {
        for &factor in &[TAUBIN_LAMBDA, TAUBIN_MU] {
            next_pos.par_iter_mut().enumerate().for_each(|(v, out_p)| {
                if pinned[v] {
                    *out_p = orig_pos[v];
                    return;
                }
                let inc = &vert_quads[offsets[v]..offsets[v + 1]];
                let mut sum = DVec3::ZERO;
                let mut cnt = 0u32;
                let vu = v as u32;
                for &qi in inc {
                    let qv = raw_quads[qi as usize].verts;
                    for ci in 0..4 {
                        if qv[ci] == vu {
                            sum += cur_pos[qv[(ci + 1) & 3] as usize];
                            sum += cur_pos[qv[(ci + 3) & 3] as usize];
                            cnt += 2;
                            break;
                        }
                    }
                }
                if cnt == 0 {
                    *out_p = cur_pos[v];
                    return;
                }
                let avg = sum / f64::from(cnt);
                let mut cand = cur_pos[v] + (avg - cur_pos[v]) * factor;
                let delta = cand - orig_pos[v];
                let d_sq = delta.length_squared();
                if d_sq > max_disp_sq {
                    cand = orig_pos[v] + delta * (max_disp / d_sq.sqrt());
                }
                *out_p = cand;
            });
            std::mem::swap(&mut cur_pos, &mut next_pos);
        }
    }

    // 7. Recompute area-weighted quad normals from relaxed positions
    let quad_normals: Vec<(DVec3, DVec3)> = raw_quads
        .par_iter()
        .map(|q| {
            let v0 = cur_pos[q.verts[0] as usize];
            let v1 = cur_pos[q.verts[1] as usize];
            let v2 = cur_pos[q.verts[2] as usize];
            let v3 = cur_pos[q.verts[3] as usize];
            let area_n = (v2 - v0).cross(v3 - v1);
            let len_sq = area_n.length_squared();
            if len_sq > 1e-20 {
                (area_n, area_n / len_sq.sqrt())
            } else {
                let fb = DVec3::new(
                    f64::from(q.fallback_n[0]),
                    f64::from(q.fallback_n[1]),
                    f64::from(q.fallback_n[2]),
                );
                (fb * 1e-6, fb)
            }
        })
        .collect();

    // 8. Compute crease-aware per-corner smooth normals `(n0, n1, n2, n3)` and emit SimStockMesh
    let mut vertices = vec![
        SimColoredVertex {
            pos_mm: [0.0; 3],
            normal: [0.0, 0.0, 1.0],
            color_rgba: [0.0; 4],
        };
        num_quads * 4
    ];
    vertices
        .par_chunks_mut(4)
        .enumerate()
        .for_each(|(qi, out_v)| {
            let q = &raw_quads[qi];
            let (_, qn) = quad_normals[qi];
            for ci in 0..4 {
                let v = q.verts[ci] as usize;
                let inc = &vert_quads[offsets[v]..offsets[v + 1]];
                let mut n_sum = DVec3::ZERO;
                for &adj_qi in inc {
                    let (adj_area_n, adj_unit_n) = quad_normals[adj_qi as usize];
                    if adj_unit_n.dot(qn) >= CREASE_COS_THRESHOLD {
                        n_sum += adj_area_n;
                    }
                }
                let n_len_sq = n_sum.length_squared();
                let nc = if n_len_sq > 1e-20 {
                    n_sum / n_len_sq.sqrt()
                } else {
                    qn
                };
                let p = cur_pos[v];
                out_v[ci] = SimColoredVertex {
                    pos_mm: [p.x as f32, p.y as f32, p.z as f32],
                    normal: [nc.x as f32, nc.y as f32, nc.z as f32],
                    color_rgba: q.color,
                };
            }
        });

    let mut indices = Vec::with_capacity(num_quads * 6);
    for qi in 0..num_quads {
        let base = (qi as u32) * 4;
        indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }

    SimStockMesh { vertices, indices }
}
