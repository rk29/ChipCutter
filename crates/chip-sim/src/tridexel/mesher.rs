use chip_math::DVec3;
use rayon::prelude::*;

use crate::{
    cutter_profile::SimColoredVertex,
    heightmap::{SimStockMesh, UNCUT_OP_IDX},
    mesher_common::op_color,
    metrics::{
        SimStockColorMode, deviation_to_heatmap_rgba, signed_deviation_at_machined_endpoint,
    },
};

use super::grid::{DexelSpan, TriDexelStock};

/// Finds the dexel span endpoint (`min_mm` if `want_min` is true, else `max_mm`) closest to
/// `target_coord` within `max_dist`, returning `(exact_coord, op_idx)`.
#[inline]
fn nearest_ray_endpoint(
    spans: &[DexelSpan],
    target_coord: f64,
    want_min: bool,
    max_dist: f64,
) -> Option<(f64, u16)> {
    let mut best: Option<(f64, f64, u16)> = None;
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

/// Extracts a watertight, sharp-feature-preserving 3D [`SimStockMesh`] from a [`TriDexelStock`]
/// using axis-decoupled Dual-Vertex Surface Nets.
///
/// - Flat horizontal planes, vertical walls, and 90° pocket/slot corners stay razor-sharp because
///   each coordinate `(x, y, z)` of a dual vertex snaps independently to the exact `X`/`Y`/`Z`
///   dexel endpoints crossing that dual cell.
/// - Curved ball-nose, chamfer, and undercut surfaces interpolate smoothly from sub-voxel dexel
///   endpoints with exact geometric face normals.
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
    let color_by_op = matches!(color_mode, SimStockColorMode::ByOperation);
    let nx = stock.nx;
    let ny = stock.ny;
    let nz = stock.nz;
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
    let occ_idx = |ix: usize, iy: usize, iz: usize| -> usize { (iz * pny + iy) * pnx + ix };

    // 2. Compute dual vertex positions on corners `(0..=nx) × (0..=ny) × (0..=nz)` in parallel
    let dnx = nx + 1;
    let dny = ny + 1;
    let dnz = nz + 1;
    let dual_idx = |di: usize, dj: usize, dk: usize| -> usize { (dk * dny + dj) * dnx + di };
    let mut dual_pos: Vec<[f32; 3]> = vec![[0.0; 3]; dnx * dny * dnz];

    let max_dx = 1.05 * stock.dx_mm;
    let max_dy = 1.05 * stock.dy_mm;
    let max_dz = 1.05 * stock.dz_mm;

    let dual_slice = dnx * dny;
    dual_pos
        .par_chunks_mut(dual_slice)
        .enumerate()
        .for_each(|(dk, slice)| {
            let base_z = stock.origin_mm.z + (dk as f64) * stock.dz_mm;
            for dj in 0..dny {
                let base_y = stock.origin_mm.y + (dj as f64) * stock.dy_mm;
                let row = dj * dnx;
                for di in 0..dnx {
                    // Check if the 8 surrounding padded voxels have a sign change
                    let mut any_in = false;
                    let mut any_out = false;
                    for oz in 0..=1 {
                        for oy in 0..=1 {
                            for ox in 0..=1 {
                                if occ[occ_idx(di + ox, dj + oy, dk + oz)] {
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

                    let base_x = stock.origin_mm.x + (di as f64) * stock.dx_mm;
                    let mut x_sum = 0.0;
                    let mut x_cnt = 0u32;
                    let mut y_sum = 0.0;
                    let mut y_cnt = 0u32;
                    let mut z_sum = 0.0;
                    let mut z_cnt = 0u32;

                    // Z-ray crossings around `(di, dj)` at level `dk`
                    for oy in 0..=1usize {
                        let Some(j) = (dj + oy).checked_sub(1) else {
                            continue;
                        };
                        if j >= ny {
                            continue;
                        }
                        for ox in 0..=1usize {
                            let Some(i) = (di + ox).checked_sub(1) else {
                                continue;
                            };
                            if i >= nx {
                                continue;
                            }
                            let lo = occ[occ_idx(i + 1, j + 1, dk)];
                            let hi = occ[occ_idx(i + 1, j + 1, dk + 1)];
                            if lo != hi {
                                let want_min = !lo && hi;
                                if let Some((zc, _)) = nearest_ray_endpoint(
                                    stock.z_rays.ray(i, j),
                                    base_z,
                                    want_min,
                                    max_dz,
                                ) {
                                    z_sum += zc;
                                    z_cnt += 1;
                                }
                            }
                        }
                    }

                    // X-ray crossings around `(dj, dk)` at plane `di`
                    for oz in 0..=1usize {
                        let Some(k) = (dk + oz).checked_sub(1) else {
                            continue;
                        };
                        if k >= nz {
                            continue;
                        }
                        for oy in 0..=1usize {
                            let Some(j) = (dj + oy).checked_sub(1) else {
                                continue;
                            };
                            if j >= ny {
                                continue;
                            }
                            let lo = occ[occ_idx(di, j + 1, k + 1)];
                            let hi = occ[occ_idx(di + 1, j + 1, k + 1)];
                            if lo != hi {
                                let want_min = !lo && hi;
                                if let Some((xc, _)) = nearest_ray_endpoint(
                                    stock.x_rays.ray(j, k),
                                    base_x,
                                    want_min,
                                    max_dx,
                                ) {
                                    x_sum += xc;
                                    x_cnt += 1;
                                }
                            }
                        }
                    }

                    // Y-ray crossings around `(di, dk)` at plane `dj`
                    for oz in 0..=1usize {
                        let Some(k) = (dk + oz).checked_sub(1) else {
                            continue;
                        };
                        if k >= nz {
                            continue;
                        }
                        for ox in 0..=1usize {
                            let Some(i) = (di + ox).checked_sub(1) else {
                                continue;
                            };
                            if i >= nx {
                                continue;
                            }
                            let lo = occ[occ_idx(i + 1, dj, k + 1)];
                            let hi = occ[occ_idx(i + 1, dj + 1, k + 1)];
                            if lo != hi {
                                let want_min = !lo && hi;
                                if let Some((yc, _)) = nearest_ray_endpoint(
                                    stock.y_rays.ray(i, k),
                                    base_y,
                                    want_min,
                                    max_dy,
                                ) {
                                    y_sum += yc;
                                    y_cnt += 1;
                                }
                            }
                        }
                    }

                    let vx = if x_cnt > 0 {
                        (x_sum / f64::from(x_cnt))
                            .clamp(base_x - 0.55 * stock.dx_mm, base_x + 0.55 * stock.dx_mm)
                    } else {
                        base_x
                    };
                    let vy = if y_cnt > 0 {
                        (y_sum / f64::from(y_cnt))
                            .clamp(base_y - 0.55 * stock.dy_mm, base_y + 0.55 * stock.dy_mm)
                    } else {
                        base_y
                    };
                    let vz = if z_cnt > 0 {
                        (z_sum / f64::from(z_cnt))
                            .clamp(base_z - 0.55 * stock.dz_mm, base_z + 0.55 * stock.dz_mm)
                    } else {
                        base_z
                    };
                    slice[row + di] = [vx as f32, vy as f32, vz as f32];
                }
            }
        });

    // 3. Emit outward-oriented quads on all sign-changing voxel boundaries
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
        // Diagonal cross product handles non-planar quads cleanly
        let n_raw = (v2 - v0).cross(v3 - v1);
        let n_len = n_raw.length();
        let normal = if n_len > 1e-10 {
            let u = n_raw / n_len;
            [u.x as f32, u.y as f32, u.z as f32]
        } else {
            fallback_n
        };
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

    // Z-axis boundary quads (`+Z` and `-Z`)
    for dk in 0..dnz {
        let base_z = stock.origin_mm.z + (dk as f64) * stock.dz_mm;
        for j in 0..ny {
            for i in 0..nx {
                let lo = occ[occ_idx(i + 1, j + 1, dk)];
                let hi = occ[occ_idx(i + 1, j + 1, dk + 1)];
                if lo == hi {
                    continue;
                }
                let want_min = !lo && hi;
                let hit_z = nearest_ray_endpoint(stock.z_rays.ray(i, j), base_z, want_min, max_dz);
                let (coord_z, op) = hit_z.unwrap_or((base_z, UNCUT_OP_IDX));
                let col = match (color_mode, target_stock) {
                    (
                        SimStockColorMode::ErrorHeatmap {
                            tolerance_mm,
                            max_scale_mm,
                        },
                        Some(tgt),
                    ) => {
                        let dev = signed_deviation_at_machined_endpoint(
                            tgt.z_rays.ray(i, j),
                            coord_z,
                            want_min,
                        );
                        deviation_to_heatmap_rgba(dev, tolerance_mm, max_scale_mm)
                    }
                    _ => op_color(op, color_by_op, false),
                };
                let a = dual_pos[dual_idx(i, j, dk)];
                let b = dual_pos[dual_idx(i + 1, j, dk)];
                let c = dual_pos[dual_idx(i + 1, j + 1, dk)];
                let d = dual_pos[dual_idx(i, j + 1, dk)];
                if lo && !hi {
                    push_quad(&mut mesh, a, b, c, d, [0.0, 0.0, 1.0], col);
                } else {
                    push_quad(&mut mesh, a, d, c, b, [0.0, 0.0, -1.0], col);
                }
            }
        }
    }

    // X-axis boundary quads (`+X` and `-X`)
    for di in 0..dnx {
        let base_x = stock.origin_mm.x + (di as f64) * stock.dx_mm;
        for k in 0..nz {
            for j in 0..ny {
                let lo = occ[occ_idx(di, j + 1, k + 1)];
                let hi = occ[occ_idx(di + 1, j + 1, k + 1)];
                if lo == hi {
                    continue;
                }
                let want_min = !lo && hi;
                let hit_x = nearest_ray_endpoint(stock.x_rays.ray(j, k), base_x, want_min, max_dx);
                let (coord_x, op) = hit_x.unwrap_or((base_x, UNCUT_OP_IDX));
                let col = match (color_mode, target_stock) {
                    (
                        SimStockColorMode::ErrorHeatmap {
                            tolerance_mm,
                            max_scale_mm,
                        },
                        Some(tgt),
                    ) => {
                        let dev = signed_deviation_at_machined_endpoint(
                            tgt.x_rays.ray(j, k),
                            coord_x,
                            want_min,
                        );
                        deviation_to_heatmap_rgba(dev, tolerance_mm, max_scale_mm)
                    }
                    _ => op_color(op, color_by_op, true),
                };
                let a = dual_pos[dual_idx(di, j, k)];
                let b = dual_pos[dual_idx(di, j + 1, k)];
                let c = dual_pos[dual_idx(di, j + 1, k + 1)];
                let d = dual_pos[dual_idx(di, j, k + 1)];
                if lo && !hi {
                    push_quad(&mut mesh, a, b, c, d, [1.0, 0.0, 0.0], col);
                } else {
                    push_quad(&mut mesh, a, d, c, b, [-1.0, 0.0, 0.0], col);
                }
            }
        }
    }

    // Y-axis boundary quads (`+Y` and `-Y`)
    for dj in 0..dny {
        let base_y = stock.origin_mm.y + (dj as f64) * stock.dy_mm;
        for k in 0..nz {
            for i in 0..nx {
                let lo = occ[occ_idx(i + 1, dj, k + 1)];
                let hi = occ[occ_idx(i + 1, dj + 1, k + 1)];
                if lo == hi {
                    continue;
                }
                let want_min = !lo && hi;
                let hit_y = nearest_ray_endpoint(stock.y_rays.ray(i, k), base_y, want_min, max_dy);
                let (coord_y, op) = hit_y.unwrap_or((base_y, UNCUT_OP_IDX));
                let col = match (color_mode, target_stock) {
                    (
                        SimStockColorMode::ErrorHeatmap {
                            tolerance_mm,
                            max_scale_mm,
                        },
                        Some(tgt),
                    ) => {
                        let dev = signed_deviation_at_machined_endpoint(
                            tgt.y_rays.ray(i, k),
                            coord_y,
                            want_min,
                        );
                        deviation_to_heatmap_rgba(dev, tolerance_mm, max_scale_mm)
                    }
                    _ => op_color(op, color_by_op, true),
                };
                let a = dual_pos[dual_idx(i, dj, k)];
                let b = dual_pos[dual_idx(i, dj, k + 1)];
                let c = dual_pos[dual_idx(i + 1, dj, k + 1)];
                let d = dual_pos[dual_idx(i + 1, dj, k)];
                if lo && !hi {
                    push_quad(&mut mesh, a, b, c, d, [0.0, 1.0, 0.0], col);
                } else {
                    push_quad(&mut mesh, a, d, c, b, [0.0, -1.0, 0.0], col);
                }
            }
        }
    }

    mesh
}
