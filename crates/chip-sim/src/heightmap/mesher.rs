use crate::{
    cutter_profile::SimColoredVertex,
    metrics::{SimStockColorMode, deviation_to_heatmap_rgba},
};

use super::{HeightfieldStock, SimStockMesh, UNCUT_OP_IDX};

/// Distinct operation color palette for **Color Stock by Operation / Tool** (`06-simulation.md` §3).
pub const OPERATION_STOCK_PALETTE: [[f32; 4]; 8] = [
    [0.34, 0.66, 0.94, 1.0], // Azure (Op 1 - e.g. Facing)
    [0.24, 0.80, 0.62, 1.0], // Mint Emerald (Op 2 - Pocket)
    [0.92, 0.64, 0.26, 1.0], // Warm Amber (Op 3 - Step)
    [0.72, 0.48, 0.94, 1.0], // Violet (Op 4 - Bore)
    [0.94, 0.44, 0.56, 1.0], // Coral Rose (Op 5 - Drill)
    [0.28, 0.80, 0.86, 1.0], // Cyan Teal (Op 6 - Contour)
    [0.86, 0.80, 0.30, 1.0], // Citrine (Op 7 - Chamfer)
    [0.58, 0.78, 0.40, 1.0], // Lime Sage (Op 8+)
];

const RAW_STOCK_TOP_COLOR: [f32; 4] = [0.58, 0.62, 0.67, 1.0];
const RAW_STOCK_SIDE_COLOR: [f32; 4] = [0.50, 0.54, 0.59, 1.0];
const MACHINED_METAL_COLOR: [f32; 4] = [0.76, 0.80, 0.85, 1.0];

#[inline]
fn stock_cell_color(op_idx: u16, color_by_op: bool, is_side_wall: bool) -> [f32; 4] {
    if op_idx == UNCUT_OP_IDX {
        if is_side_wall {
            RAW_STOCK_SIDE_COLOR
        } else {
            RAW_STOCK_TOP_COLOR
        }
    } else if color_by_op {
        OPERATION_STOCK_PALETTE[(op_idx as usize) % OPERATION_STOCK_PALETTE.len()]
    } else {
        MACHINED_METAL_COLOR
    }
}

/// Triangulates a [`HeightfieldStock`] into a crisp-walled 3D solid mesh with greedy X-row quad
/// merging on flat regions, sharp vertical step risers on pocket/boss/tab walls, and outer side walls.
pub fn tessellate_stock_mesh(stock: &HeightfieldStock, color_by_op: bool) -> SimStockMesh {
    let mode = if color_by_op {
        SimStockColorMode::ByOperation
    } else {
        SimStockColorMode::Metallic
    };
    tessellate_stock_mesh_colored(stock, mode)
}

/// Triangulates a [`HeightfieldStock`] with selectable [`SimStockColorMode`] (`Metallic`,
/// `ByOperation`, or `ErrorHeatmap` comparing `z_stock` against `z_target`).
pub fn tessellate_stock_mesh_colored(
    stock: &HeightfieldStock,
    color_mode: SimStockColorMode,
) -> SimStockMesh {
    let color_by_op = matches!(color_mode, SimStockColorMode::ByOperation);
    const MAX_VISUAL_DIM: usize = 512;
    let stride = (stock.nx.max(stock.ny)).div_ceil(MAX_VISUAL_DIM).max(1);
    let vx = stock.nx / stride;
    let vy = stock.ny / stride;
    if vx == 0 || vy == 0 {
        return SimStockMesh::default();
    }

    let vdx = (stock.nx as f64 * stock.dx_mm) / (vx as f64);
    let vdy = (stock.ny as f64 * stock.dy_mm) / (vy as f64);

    // Downsample (stride == 1 is a direct copy of active cell heights)
    let vtotal = vx * vy;
    let mut v_active = vec![false; vtotal];
    let mut v_ztop = vec![f32::NEG_INFINITY; vtotal];
    let mut v_zbot = vec![f32::NEG_INFINITY; vtotal];
    let mut v_ztgt = vec![f32::NEG_INFINITY; vtotal];
    let mut v_op = vec![UNCUT_OP_IDX; vtotal];

    for vj in 0..vy {
        let sj = (vj * stride + stride / 2).min(stock.ny - 1);
        for vi in 0..vx {
            let si = (vi * stride + stride / 2).min(stock.nx - 1);
            let sidx = sj * stock.nx + si;
            let vidx = vj * vx + vi;
            let zt = stock.z_stock[sidx];
            let zb = stock.z_bottom[sidx];
            if stock.in_stock[sidx] && zt > zb + 1e-4 {
                v_active[vidx] = true;
                v_ztop[vidx] = zt;
                v_zbot[vidx] = zb;
                v_ztgt[vidx] = stock.z_target[sidx];
                v_op[vidx] = stock.last_op_idx[sidx];
            }
        }
    }

    let cell_top_color = |vidx: usize, is_side: bool| -> [f32; 4] {
        match color_mode {
            SimStockColorMode::Metallic => stock_cell_color(v_op[vidx], false, is_side),
            SimStockColorMode::ByOperation => stock_cell_color(v_op[vidx], true, is_side),
            SimStockColorMode::ErrorHeatmap {
                tolerance_mm,
                max_scale_mm,
            } => {
                let zt = v_ztgt[vidx];
                if zt.is_finite() && zt > v_zbot[vidx] + 1e-5 {
                    let dev = f64::from(v_ztop[vidx]) - f64::from(zt);
                    deviation_to_heatmap_rgba(dev, tolerance_mm, max_scale_mm)
                } else {
                    // Outside target silhouette -> uncut excess stock
                    deviation_to_heatmap_rgba(max_scale_mm, tolerance_mm, max_scale_mm)
                }
            }
        }
    };
    let is_heatmap = matches!(color_mode, SimStockColorMode::ErrorHeatmap { .. });

    let mut mesh = SimStockMesh {
        vertices: Vec::with_capacity(vtotal / 2),
        indices: Vec::with_capacity(vtotal),
    };

    let push_quad = |mesh: &mut SimStockMesh,
                     p0: [f32; 3],
                     p1: [f32; 3],
                     p2: [f32; 3],
                     p3: [f32; 3],
                     normal: [f32; 3],
                     color: [f32; 4]| {
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

    // 1. Greedy X-run merged top faces + smooth sloped normals on chamfers/ball grooves
    let slope_max_dz = (1.6 * vdx.max(vdy)) as f32;
    for vj in 0..vy {
        let y0 = (stock.origin_y_mm + (vj as f64) * vdy) as f32;
        let y1 = (stock.origin_y_mm + ((vj + 1) as f64) * vdy) as f32;
        let row = vj * vx;

        let mut vi = 0usize;
        while vi < vx {
            let idx = row + vi;
            if !v_active[idx] {
                vi += 1;
                continue;
            }
            let z0 = v_ztop[idx];
            let op0 = v_op[idx];

            // Compute local finite-difference normal to detect sloped chamfer/ball surfaces vs flat floors
            let zl = if vi > 0 && v_active[idx - 1] && (v_ztop[idx - 1] - z0).abs() <= slope_max_dz
            {
                v_ztop[idx - 1]
            } else {
                z0
            };
            let zr =
                if vi + 1 < vx && v_active[idx + 1] && (v_ztop[idx + 1] - z0).abs() <= slope_max_dz
                {
                    v_ztop[idx + 1]
                } else {
                    z0
                };
            let zd =
                if vj > 0 && v_active[idx - vx] && (v_ztop[idx - vx] - z0).abs() <= slope_max_dz {
                    v_ztop[idx - vx]
                } else {
                    z0
                };
            let zu = if vj + 1 < vy
                && v_active[idx + vx]
                && (v_ztop[idx + vx] - z0).abs() <= slope_max_dz
            {
                v_ztop[idx + vx]
            } else {
                z0
            };

            let dz_dx = (zr - zl) / (2.0 * vdx as f32);
            let dz_dy = (zu - zd) / (2.0 * vdy as f32);
            let is_flat = dz_dx.abs() < 1e-3 && dz_dy.abs() < 1e-3;

            let col = cell_top_color(idx, false);
            let mut run_end = vi + 1;
            if is_flat {
                while run_end < vx {
                    let r_idx = row + run_end;
                    let same_col = if is_heatmap {
                        let rc = cell_top_color(r_idx, false);
                        (rc[0] - col[0]).abs() < 0.05
                            && (rc[1] - col[1]).abs() < 0.05
                            && (rc[2] - col[2]).abs() < 0.05
                    } else if color_by_op {
                        v_op[r_idx] == op0
                    } else {
                        (v_op[r_idx] == UNCUT_OP_IDX) == (op0 == UNCUT_OP_IDX)
                    };
                    if !v_active[r_idx] || (v_ztop[r_idx] - z0).abs() > 2e-4 || !same_col {
                        break;
                    }
                    run_end += 1;
                }
            }

            let normal = if is_flat {
                [0.0, 0.0, 1.0]
            } else {
                let len = (dz_dx * dz_dx + dz_dy * dz_dy + 1.0).sqrt();
                [-dz_dx / len, -dz_dy / len, 1.0 / len]
            };

            let x0 = (stock.origin_x_mm + (vi as f64) * vdx) as f32;
            let x1 = (stock.origin_x_mm + (run_end as f64) * vdx) as f32;
            push_quad(
                &mut mesh,
                [x0, y0, z0],
                [x1, y0, z0],
                [x1, y1, z0],
                [x0, y1, z0],
                normal,
                col,
            );

            vi = run_end;
        }
    }

    // 2. Crisp vertical step risers & outer stock/hole walls
    // -X / +X vertical risers (normal `[-1, 0, 0]` and `[+1, 0, 0]`) — row-major for cache locality
    for vj in 0..vy {
        let y0 = (stock.origin_y_mm + (vj as f64) * vdy) as f32;
        let y1 = (stock.origin_y_mm + ((vj + 1) as f64) * vdy) as f32;
        let row = vj * vx;
        for vi in 0..=vx {
            let x = (stock.origin_x_mm + (vi as f64) * vdx) as f32;

            let left = if vi > 0 && v_active[row + (vi - 1)] {
                Some((
                    v_zbot[row + (vi - 1)],
                    v_ztop[row + (vi - 1)],
                    row + (vi - 1),
                ))
            } else {
                None
            };
            let right = if vi < vx && v_active[row + vi] {
                Some((v_zbot[row + vi], v_ztop[row + vi], row + vi))
            } else {
                None
            };

            match (left, right) {
                (Some((lb, lt, lidx)), None) => {
                    let col = cell_top_color(lidx, true);
                    push_quad(
                        &mut mesh,
                        [x, y0, lb],
                        [x, y1, lb],
                        [x, y1, lt],
                        [x, y0, lt],
                        [1.0, 0.0, 0.0],
                        col,
                    );
                }
                (None, Some((rb, rt, ridx))) => {
                    let col = cell_top_color(ridx, true);
                    push_quad(
                        &mut mesh,
                        [x, y1, rb],
                        [x, y0, rb],
                        [x, y0, rt],
                        [x, y1, rt],
                        [-1.0, 0.0, 0.0],
                        col,
                    );
                }
                (Some((lb, lt, lidx)), Some((rb, rt, ridx))) => {
                    if lt > rt + 2e-4 {
                        let z_lo = rt.max(lb);
                        let col = cell_top_color(ridx, false);
                        push_quad(
                            &mut mesh,
                            [x, y0, z_lo],
                            [x, y1, z_lo],
                            [x, y1, lt],
                            [x, y0, lt],
                            [1.0, 0.0, 0.0],
                            col,
                        );
                    } else if rt > lt + 2e-4 {
                        let z_lo = lt.max(rb);
                        let col = cell_top_color(lidx, false);
                        push_quad(
                            &mut mesh,
                            [x, y1, z_lo],
                            [x, y0, z_lo],
                            [x, y0, rt],
                            [x, y1, rt],
                            [-1.0, 0.0, 0.0],
                            col,
                        );
                    }
                }
                (None, None) => {}
            }
        }
    }

    // -Y / +Y vertical risers (normal `[0, -1, 0]` and `[0, +1, 0]`)
    for vj in 0..=vy {
        let y = (stock.origin_y_mm + (vj as f64) * vdy) as f32;
        for vi in 0..vx {
            let x0 = (stock.origin_x_mm + (vi as f64) * vdx) as f32;
            let x1 = (stock.origin_x_mm + ((vi + 1) as f64) * vdx) as f32;

            let front = if vj > 0 && v_active[(vj - 1) * vx + vi] {
                let idx = (vj - 1) * vx + vi;
                Some((v_zbot[idx], v_ztop[idx], idx))
            } else {
                None
            };
            let back = if vj < vy && v_active[vj * vx + vi] {
                let idx = vj * vx + vi;
                Some((v_zbot[idx], v_ztop[idx], idx))
            } else {
                None
            };

            match (front, back) {
                (Some((fb, ft, fidx)), None) => {
                    let col = cell_top_color(fidx, true);
                    push_quad(
                        &mut mesh,
                        [x1, y, fb],
                        [x0, y, fb],
                        [x0, y, ft],
                        [x1, y, ft],
                        [0.0, 1.0, 0.0],
                        col,
                    );
                }
                (None, Some((bb, bt, bidx))) => {
                    let col = cell_top_color(bidx, true);
                    push_quad(
                        &mut mesh,
                        [x0, y, bb],
                        [x1, y, bb],
                        [x1, y, bt],
                        [x0, y, bt],
                        [0.0, -1.0, 0.0],
                        col,
                    );
                }
                (Some((fb, ft, fidx)), Some((bb, bt, bidx))) => {
                    if ft > bt + 2e-4 {
                        let z_lo = bt.max(fb);
                        let col = cell_top_color(bidx, false);
                        push_quad(
                            &mut mesh,
                            [x1, y, z_lo],
                            [x0, y, z_lo],
                            [x0, y, ft],
                            [x1, y, ft],
                            [0.0, 1.0, 0.0],
                            col,
                        );
                    } else if bt > ft + 2e-4 {
                        let z_lo = ft.max(bb);
                        let col = cell_top_color(fidx, false);
                        push_quad(
                            &mut mesh,
                            [x0, y, z_lo],
                            [x1, y, z_lo],
                            [x1, y, bt],
                            [x0, y, bt],
                            [0.0, -1.0, 0.0],
                            col,
                        );
                    }
                }
                (None, None) => {}
            }
        }
    }

    // 3. Greedy X-run merged bottom faces (`normal = [0, 0, -1]`) so stock mesh is a closed solid from below
    for vj in 0..vy {
        let y0 = (stock.origin_y_mm + (vj as f64) * vdy) as f32;
        let y1 = (stock.origin_y_mm + ((vj + 1) as f64) * vdy) as f32;
        let row = vj * vx;
        let mut vi = 0usize;
        while vi < vx {
            let idx = row + vi;
            if !v_active[idx] {
                vi += 1;
                continue;
            }
            let zb0 = v_zbot[idx];
            let mut run_end = vi + 1;
            while run_end < vx
                && v_active[row + run_end]
                && (v_zbot[row + run_end] - zb0).abs() <= 2e-4
            {
                run_end += 1;
            }
            let x0 = (stock.origin_x_mm + (vi as f64) * vdx) as f32;
            let x1 = (stock.origin_x_mm + (run_end as f64) * vdx) as f32;
            push_quad(
                &mut mesh,
                [x0, y1, zb0],
                [x1, y1, zb0],
                [x1, y0, zb0],
                [x0, y0, zb0],
                [0.0, 0.0, -1.0],
                RAW_STOCK_SIDE_COLOR,
            );
            vi = run_end;
        }
    }

    mesh
}
