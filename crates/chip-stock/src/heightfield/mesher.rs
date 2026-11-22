use crate::{
    mesher_common::{RAW_STOCK_SIDE_COLOR, op_color as stock_cell_color},
    metrics::{SimStockColorMode, deviation_to_heatmap_rgba},
    types::{SimColoredVertex, SimStockMesh, UNCUT_OP_IDX},
};

use super::HeightfieldStock;

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
    let ox = stock.origin_wcs.x;
    let oy = stock.origin_wcs.y;
    let oz = stock.origin_wcs.z;
    let z_wcs = |z_local: f32| -> f32 { (oz + f64::from(z_local)) as f32 };

    // Downsample (stride == 1 is a direct copy of active cell heights in stock-local f32)
    let vtotal = vx * vy;
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
            if zb.is_finite() && zt > zb + 1e-4 {
                v_ztop[vidx] = zt;
                v_zbot[vidx] = zb;
                v_ztgt[vidx] = stock.z_target[sidx];
                v_op[vidx] = stock.last_op_idx[sidx];
            }
        }
    }
    let v_active = |idx: usize| -> bool { v_zbot[idx].is_finite() };

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
        let y0 = (oy + (vj as f64) * vdy) as f32;
        let y1 = (oy + ((vj + 1) as f64) * vdy) as f32;
        let row = vj * vx;

        let mut vi = 0usize;
        while vi < vx {
            let idx = row + vi;
            if !v_active(idx) {
                vi += 1;
                continue;
            }
            let z0 = v_ztop[idx];
            let op0 = v_op[idx];

            let zl = if vi > 0 && v_active(idx - 1) && (v_ztop[idx - 1] - z0).abs() <= slope_max_dz
            {
                v_ztop[idx - 1]
            } else {
                z0
            };
            let zr =
                if vi + 1 < vx && v_active(idx + 1) && (v_ztop[idx + 1] - z0).abs() <= slope_max_dz
                {
                    v_ztop[idx + 1]
                } else {
                    z0
                };
            let zd =
                if vj > 0 && v_active(idx - vx) && (v_ztop[idx - vx] - z0).abs() <= slope_max_dz {
                    v_ztop[idx - vx]
                } else {
                    z0
                };
            let zu = if vj + 1 < vy
                && v_active(idx + vx)
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
                    if !v_active(r_idx) || (v_ztop[r_idx] - z0).abs() > 2e-4 || !same_col {
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

            let x0 = (ox + (vi as f64) * vdx) as f32;
            let x1 = (ox + (run_end as f64) * vdx) as f32;
            let z0_w = z_wcs(z0);
            push_quad(
                &mut mesh,
                [x0, y0, z0_w],
                [x1, y0, z0_w],
                [x1, y1, z0_w],
                [x0, y1, z0_w],
                normal,
                col,
            );

            vi = run_end;
        }
    }

    // 2. Crisp vertical step risers & outer stock/hole walls
    // -X / +X vertical risers
    for vj in 0..vy {
        let y0 = (oy + (vj as f64) * vdy) as f32;
        let y1 = (oy + ((vj + 1) as f64) * vdy) as f32;
        let row = vj * vx;
        for vi in 0..=vx {
            let x = (ox + (vi as f64) * vdx) as f32;

            let left = if vi > 0 && v_active(row + (vi - 1)) {
                let lidx = row + (vi - 1);
                Some((v_zbot[lidx], v_ztop[lidx], lidx))
            } else {
                None
            };
            let right = if vi < vx && v_active(row + vi) {
                let ridx = row + vi;
                Some((v_zbot[ridx], v_ztop[ridx], ridx))
            } else {
                None
            };

            match (left, right) {
                (Some((lb, lt, lidx)), None) => {
                    let col = cell_top_color(lidx, true);
                    let (lb_w, lt_w) = (z_wcs(lb), z_wcs(lt));
                    push_quad(
                        &mut mesh,
                        [x, y0, lb_w],
                        [x, y1, lb_w],
                        [x, y1, lt_w],
                        [x, y0, lt_w],
                        [1.0, 0.0, 0.0],
                        col,
                    );
                }
                (None, Some((rb, rt, ridx))) => {
                    let col = cell_top_color(ridx, true);
                    let (rb_w, rt_w) = (z_wcs(rb), z_wcs(rt));
                    push_quad(
                        &mut mesh,
                        [x, y1, rb_w],
                        [x, y0, rb_w],
                        [x, y0, rt_w],
                        [x, y1, rt_w],
                        [-1.0, 0.0, 0.0],
                        col,
                    );
                }
                (Some((lb, lt, lidx)), Some((rb, rt, ridx))) => {
                    if lt > rt + 2e-4 {
                        let z_lo = z_wcs(rt.max(lb));
                        let lt_w = z_wcs(lt);
                        let col = cell_top_color(ridx, false);
                        push_quad(
                            &mut mesh,
                            [x, y0, z_lo],
                            [x, y1, z_lo],
                            [x, y1, lt_w],
                            [x, y0, lt_w],
                            [1.0, 0.0, 0.0],
                            col,
                        );
                    } else if rt > lt + 2e-4 {
                        let z_lo = z_wcs(lt.max(rb));
                        let rt_w = z_wcs(rt);
                        let col = cell_top_color(lidx, false);
                        push_quad(
                            &mut mesh,
                            [x, y1, z_lo],
                            [x, y0, z_lo],
                            [x, y0, rt_w],
                            [x, y1, rt_w],
                            [-1.0, 0.0, 0.0],
                            col,
                        );
                    }
                }
                (None, None) => {}
            }
        }
    }

    // -Y / +Y vertical risers
    for vj in 0..=vy {
        let y = (oy + (vj as f64) * vdy) as f32;
        for vi in 0..vx {
            let x0 = (ox + (vi as f64) * vdx) as f32;
            let x1 = (ox + ((vi + 1) as f64) * vdx) as f32;

            let front = if vj > 0 && v_active((vj - 1) * vx + vi) {
                let idx = (vj - 1) * vx + vi;
                Some((v_zbot[idx], v_ztop[idx], idx))
            } else {
                None
            };
            let back = if vj < vy && v_active(vj * vx + vi) {
                let idx = vj * vx + vi;
                Some((v_zbot[idx], v_ztop[idx], idx))
            } else {
                None
            };

            match (front, back) {
                (Some((fb, ft, fidx)), None) => {
                    let col = cell_top_color(fidx, true);
                    let (fb_w, ft_w) = (z_wcs(fb), z_wcs(ft));
                    push_quad(
                        &mut mesh,
                        [x1, y, fb_w],
                        [x0, y, fb_w],
                        [x0, y, ft_w],
                        [x1, y, ft_w],
                        [0.0, 1.0, 0.0],
                        col,
                    );
                }
                (None, Some((bb, bt, bidx))) => {
                    let col = cell_top_color(bidx, true);
                    let (bb_w, bt_w) = (z_wcs(bb), z_wcs(bt));
                    push_quad(
                        &mut mesh,
                        [x0, y, bb_w],
                        [x1, y, bb_w],
                        [x1, y, bt_w],
                        [x0, y, bt_w],
                        [0.0, -1.0, 0.0],
                        col,
                    );
                }
                (Some((fb, ft, fidx)), Some((bb, bt, bidx))) => {
                    if ft > bt + 2e-4 {
                        let z_lo = z_wcs(bt.max(fb));
                        let ft_w = z_wcs(ft);
                        let col = cell_top_color(bidx, false);
                        push_quad(
                            &mut mesh,
                            [x1, y, z_lo],
                            [x0, y, z_lo],
                            [x0, y, ft_w],
                            [x1, y, ft_w],
                            [0.0, 1.0, 0.0],
                            col,
                        );
                    } else if bt > ft + 2e-4 {
                        let z_lo = z_wcs(ft.max(bb));
                        let bt_w = z_wcs(bt);
                        let col = cell_top_color(fidx, false);
                        push_quad(
                            &mut mesh,
                            [x0, y, z_lo],
                            [x1, y, z_lo],
                            [x1, y, bt_w],
                            [x0, y, bt_w],
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
        let y0 = (oy + (vj as f64) * vdy) as f32;
        let y1 = (oy + ((vj + 1) as f64) * vdy) as f32;
        let row = vj * vx;
        let mut vi = 0usize;
        while vi < vx {
            let idx = row + vi;
            if !v_active(idx) {
                vi += 1;
                continue;
            }
            let zb0 = v_zbot[idx];
            let mut run_end = vi + 1;
            while run_end < vx
                && v_active(row + run_end)
                && (v_zbot[row + run_end] - zb0).abs() <= 2e-4
            {
                run_end += 1;
            }
            let x0 = (ox + (vi as f64) * vdx) as f32;
            let x1 = (ox + (run_end as f64) * vdx) as f32;
            let zb0_w = z_wcs(zb0);
            push_quad(
                &mut mesh,
                [x0, y1, zb0_w],
                [x1, y1, zb0_w],
                [x1, y0, zb0_w],
                [x0, y0, zb0_w],
                [0.0, 0.0, -1.0],
                RAW_STOCK_SIDE_COLOR,
            );
            vi = run_end;
        }
    }

    mesh
}
