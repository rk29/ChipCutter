use chip_math::DVec3;

use crate::{
    types::{SimColoredVertex, SimMeshOptions, SimStockMesh},
    visual::{RAW_STOCK_SIDE_COLOR, RAW_STOCK_TOP_COLOR, op_color},
    visual::{SimStockColorMode, deviation_to_heatmap_rgba},
};

use super::CylindricalStock;

const CYL_CREASE_COS: f64 = 0.788;
const CYL_TAUBIN_LAMBDA: f64 = 0.45;
const CYL_TAUBIN_MU: f64 = -0.47;

impl CylindricalStock {
    /// Tessellates this 4th-axis cylindrical stock using visual [`SimMeshOptions`] (`color_mode`
    /// + optional crease-preserving radial `mesh_skinning`).
    pub fn extract_mesh_with_options(
        &self,
        options: SimMeshOptions,
        rotary_angle_deg: f64,
    ) -> SimStockMesh {
        tessellate_cylindrical_stock_mesh_with_options(self, options, rotary_angle_deg)
    }
}

/// Tessellates a [`CylindricalStock`] into a smooth, watertight 3D [`SimStockMesh`] (including
/// both end caps) optionally rotated live around the rotary axis by `rotary_angle_deg`.
pub fn tessellate_cylindrical_stock_mesh(
    stock: &CylindricalStock,
    color_by_op: bool,
    rotary_angle_deg: f64,
) -> SimStockMesh {
    let mode = if color_by_op {
        SimStockColorMode::ByOperation
    } else {
        SimStockColorMode::Metallic
    };
    tessellate_cylindrical_stock_mesh_colored(stock, mode, rotary_angle_deg)
}

/// Tessellates a [`CylindricalStock`] into a smooth, watertight 3D [`SimStockMesh`] with
/// selectable [`SimStockColorMode`] (`Metallic`, `ByOperation`, or signed `ErrorHeatmap`
/// comparing `r_stock` against `r_target`).
pub fn tessellate_cylindrical_stock_mesh_colored(
    stock: &CylindricalStock,
    color_mode: SimStockColorMode,
    rotary_angle_deg: f64,
) -> SimStockMesh {
    tessellate_cylindrical_stock_mesh_with_options(
        stock,
        SimMeshOptions::from(color_mode),
        rotary_angle_deg,
    )
}

/// Tessellates a [`CylindricalStock`] into a watertight 3D [`SimStockMesh`] using visual
/// [`SimMeshOptions`] (`color_mode` + optional crease-preserving radial Taubin `mesh_skinning`).
pub fn tessellate_cylindrical_stock_mesh_with_options(
    stock: &CylindricalStock,
    options: SimMeshOptions,
    rotary_angle_deg: f64,
) -> SimStockMesh {
    let color_mode = options.color_mode;
    let nu = stock.nu;
    let nt = stock.ntheta;
    if nu < 2 || nt < 3 {
        return SimStockMesh::default();
    }

    let step_u = (nu / 180).max(1);
    let step_t = (nt / 180).max(1);
    let su = nu.div_ceil(step_u);
    let st = nt.div_ceil(step_t);

    let heatmap_active = matches!(color_mode, SimStockColorMode::ErrorHeatmap { .. })
        && stock.r_target.iter().any(|&r| r >= 0.0 && r.is_finite());

    let sample_r = |iu: usize, it: usize| -> f64 {
        let cu = (iu * step_u).min(nu - 1);
        let ct = (it * step_t) % nt;
        f64::from(stock.r_stock[cu * nt + ct].max(0.0))
    };
    let sample_color = |iu: usize, it: usize| -> [f32; 4] {
        let cu = (iu * step_u).min(nu - 1);
        let ct = (it * step_t) % nt;
        let idx = cu * nt + ct;
        match color_mode {
            SimStockColorMode::Metallic => op_color(stock.last_op_idx[idx], false, false),
            SimStockColorMode::ByOperation => op_color(stock.last_op_idx[idx], true, false),
            SimStockColorMode::ErrorHeatmap {
                tolerance_mm,
                max_scale_mm,
            } => {
                if !heatmap_active {
                    return RAW_STOCK_TOP_COLOR;
                }
                let r_tgt = stock.r_target[idx];
                if r_tgt >= 0.0 && r_tgt.is_finite() {
                    let dev = f64::from(stock.r_stock[idx].max(0.0)) - f64::from(r_tgt);
                    deviation_to_heatmap_rgba(dev, tolerance_mm, max_scale_mm)
                } else {
                    deviation_to_heatmap_rgba(max_scale_mm, tolerance_mm, max_scale_mm)
                }
            }
        }
    };

    let mut r_orig: Vec<f64> = Vec::with_capacity(su * st);
    let mut colors: Vec<[f32; 4]> = Vec::with_capacity(su * st);
    for iu in 0..su {
        for it in 0..st {
            r_orig.push(sample_r(iu, it));
            colors.push(sample_color(iu, it));
        }
    }

    let r_final = if options.mesh_skinning {
        let du_eff = stock.du * (step_u as f64);
        let cliff_dr = 1.5 * du_eff;
        let max_disp = 0.45 * du_eff;

        let mut pinned = vec![true; su * st];
        for iu in 1..su.saturating_sub(1) {
            for it in 0..st {
                let idx = iu * st + it;
                let r0 = r_orig[idx];
                if r0 <= 1e-4 {
                    continue;
                }
                let t_prev = (it + st - 1) % st;
                let t_next = (it + 1) % st;
                let nbs = [
                    (iu - 1) * st + it,
                    (iu + 1) * st + it,
                    iu * st + t_prev,
                    iu * st + t_next,
                ];
                let steep = nbs.iter().any(|&ni| (r_orig[ni] - r0).abs() > cliff_dr);
                if !steep {
                    pinned[idx] = false;
                }
            }
        }

        let mut r_cur = r_orig.clone();
        let mut r_next = r_orig.clone();
        for _pass in 0..2 {
            for &factor in &[CYL_TAUBIN_LAMBDA, CYL_TAUBIN_MU] {
                for iu in 0..su {
                    for it in 0..st {
                        let idx = iu * st + it;
                        if pinned[idx] {
                            r_next[idx] = r_orig[idx];
                            continue;
                        }
                        let r0_orig = r_orig[idx];
                        let t_prev = (it + st - 1) % st;
                        let t_next = (it + 1) % st;
                        let nbs = [
                            (iu - 1) * st + it,
                            (iu + 1) * st + it,
                            iu * st + t_prev,
                            iu * st + t_next,
                        ];
                        let mut sum = 0.0;
                        let mut cnt = 0.0;
                        for ni in nbs {
                            if (r_orig[ni] - r0_orig).abs() <= cliff_dr {
                                sum += r_cur[ni];
                                cnt += 1.0;
                            }
                        }
                        if cnt > 0.0 {
                            let avg = sum / cnt;
                            let cand = r_cur[idx] + factor * (avg - r_cur[idx]);
                            r_next[idx] =
                                cand.clamp(r0_orig - max_disp, r0_orig + max_disp).max(0.0);
                        } else {
                            r_next[idx] = r_cur[idx];
                        }
                    }
                }
                std::mem::swap(&mut r_cur, &mut r_next);
            }
        }
        r_cur
    } else {
        r_orig
    };

    let mut positions: Vec<DVec3> = Vec::with_capacity(su * st);
    for iu in 0..su {
        let u = if iu + 1 == su {
            stock.u_max
        } else if iu == 0 {
            stock.u_min
        } else {
            stock.u_min + ((iu * step_u) as f64 + 0.5) * stock.du
        };
        for it in 0..st {
            let theta = (it * step_t) as f64 * stock.dtheta;
            let r = r_final[iu * st + it];
            let p_part = stock.rotary.cylindrical_to_wcs(u, theta, r);
            let p_rot = if rotary_angle_deg.abs() > 1e-6 {
                stock.rotary.rotate_point_wcs(p_part, rotary_angle_deg)
            } else {
                p_part
            };
            positions.push(p_rot);
        }
    }

    let mut vertices: Vec<SimColoredVertex>;
    let mut indices: Vec<u32>;

    if !options.mesh_skinning {
        let mut normals = vec![DVec3::ZERO; su * st];
        indices = Vec::with_capacity((su - 1) * st * 6 + st * 6);

        for iu in 0..(su - 1) {
            for it in 0..st {
                let it_next = (it + 1) % st;
                let i00 = iu * st + it;
                let i10 = (iu + 1) * st + it;
                let i11 = (iu + 1) * st + it_next;
                let i01 = iu * st + it_next;

                let p00 = positions[i00];
                let p10 = positions[i10];
                let p11 = positions[i11];
                let p01 = positions[i01];

                let n1 = (p10 - p00).cross(p11 - p00);
                let n2 = (p11 - p00).cross(p01 - p00);
                normals[i00] += n1 + n2;
                normals[i10] += n1;
                normals[i11] += n1 + n2;
                normals[i01] += n2;

                indices.extend_from_slice(&[
                    i00 as u32, i10 as u32, i11 as u32, i00 as u32, i11 as u32, i01 as u32,
                ]);
            }
        }

        vertices = Vec::with_capacity(su * st + 2 * (st + 1));
        for i in 0..(su * st) {
            let p = positions[i];
            let n = normals[i].normalize_or_zero();
            vertices.push(SimColoredVertex {
                pos_mm: [p.x as f32, p.y as f32, p.z as f32],
                normal: [n.x as f32, n.y as f32, n.z as f32],
                color_rgba: colors[i],
            });
        }
    } else {
        // Crease-aware per-corner smooth normals so sharp turned shoulders stay crisp while
        // curved/helical bands shade smoothly.
        let q_rows = su - 1;
        let mut quad_area_n = vec![DVec3::ZERO; q_rows * st];
        let mut quad_unit_n = vec![DVec3::ZERO; q_rows * st];
        for iu in 0..q_rows {
            for it in 0..st {
                let it_next = (it + 1) % st;
                let p00 = positions[iu * st + it];
                let p10 = positions[(iu + 1) * st + it];
                let p11 = positions[(iu + 1) * st + it_next];
                let p01 = positions[iu * st + it_next];
                let n1 = (p10 - p00).cross(p11 - p00);
                let n2 = (p11 - p00).cross(p01 - p00);
                let an = n1 + n2;
                let qidx = iu * st + it;
                quad_area_n[qidx] = an;
                quad_unit_n[qidx] = an.normalize_or_zero();
            }
        }

        let corner_normal = |vu: usize, vt: usize, qn: DVec3| -> [f32; 3] {
            let t_prev = (vt + st - 1) % st;
            let mut sum = DVec3::ZERO;
            let u_start = vu.saturating_sub(1);
            let u_end = vu.min(q_rows - 1);
            for qu in u_start..=u_end {
                for qt in [t_prev, vt] {
                    let qidx = qu * st + qt;
                    let adj_un = quad_unit_n[qidx];
                    if adj_un.dot(qn) >= CYL_CREASE_COS {
                        sum += quad_area_n[qidx];
                    }
                }
            }
            let n = if sum.length_squared() > 1e-20 {
                sum.normalize()
            } else {
                qn
            };
            [n.x as f32, n.y as f32, n.z as f32]
        };

        vertices = Vec::with_capacity(q_rows * st * 4 + 2 * (st + 1));
        indices = Vec::with_capacity(q_rows * st * 6 + st * 6);
        for iu in 0..q_rows {
            for it in 0..st {
                let it_next = (it + 1) % st;
                let qn = quad_unit_n[iu * st + it];
                let corners = [(iu, it), (iu + 1, it), (iu + 1, it_next), (iu, it_next)];
                let base = vertices.len() as u32;
                for (vu, vt) in corners {
                    let vi = vu * st + vt;
                    let p = positions[vi];
                    vertices.push(SimColoredVertex {
                        pos_mm: [p.x as f32, p.y as f32, p.z as f32],
                        normal: corner_normal(vu, vt, qn),
                        color_rgba: colors[vi],
                    });
                }
                indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
            }
        }
    }

    let u_vec = stock.rotary.axis_unit();
    for (is_max_end, ring_iu, cap_u, cap_normal) in [
        (false, 0usize, stock.u_min, -u_vec),
        (true, su - 1, stock.u_max, u_vec),
    ] {
        let c_part = stock.rotary.cylindrical_to_wcs(cap_u, 0.0, 0.0);
        let c_rot = if rotary_angle_deg.abs() > 1e-6 {
            stock.rotary.rotate_point_wcs(c_part, rotary_angle_deg)
        } else {
            c_part
        };
        let center_color = if heatmap_active {
            let mut acc = [0.0_f32; 4];
            for it in 0..st {
                let c = colors[ring_iu * st + it];
                for ch in 0..4 {
                    acc[ch] += c[ch];
                }
            }
            let inv = 1.0 / (st as f32);
            [acc[0] * inv, acc[1] * inv, acc[2] * inv, 1.0]
        } else {
            RAW_STOCK_SIDE_COLOR
        };
        let center_idx = vertices.len() as u32;
        vertices.push(SimColoredVertex {
            pos_mm: [c_rot.x as f32, c_rot.y as f32, c_rot.z as f32],
            normal: [
                cap_normal.x as f32,
                cap_normal.y as f32,
                cap_normal.z as f32,
            ],
            color_rgba: center_color,
        });

        let ring_base = vertices.len() as u32;
        for it in 0..st {
            let p = positions[ring_iu * st + it];
            let ring_color = if heatmap_active {
                colors[ring_iu * st + it]
            } else {
                RAW_STOCK_SIDE_COLOR
            };
            vertices.push(SimColoredVertex {
                pos_mm: [p.x as f32, p.y as f32, p.z as f32],
                normal: [
                    cap_normal.x as f32,
                    cap_normal.y as f32,
                    cap_normal.z as f32,
                ],
                color_rgba: ring_color,
            });
        }
        for it in 0..st {
            let a = ring_base + it as u32;
            let b = ring_base + ((it + 1) % st) as u32;
            if is_max_end {
                indices.extend_from_slice(&[center_idx, b, a]);
            } else {
                indices.extend_from_slice(&[center_idx, a, b]);
            }
        }
    }

    SimStockMesh { vertices, indices }
}
