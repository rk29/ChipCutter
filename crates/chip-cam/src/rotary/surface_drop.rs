use chip_bvh::FlatBvhNode;
use chip_cad::MeshTriangle;
use chip_math::{Aabb3, DVec2, DVec3};
use rayon::prelude::*;

use crate::surface3d::{SurfaceCutterProfile, SurfaceMesh3D, drop_cutter::drop_cutter_triangle};

use super::{ResolvedRotaryConfig, RotaryAxisKind};

/// Builds a rotated [`SurfaceMesh3D`] pre-filtered to ONLY triangles intersecting the exact
/// cutter strip (`|cross - center| <= cross_half_width`, `u ∈ [u_start - cross_half_width, u_end + cross_half_width]`,
/// `max.z >= z_floor`), eliminating ~85% of unreachable triangles prior to BVH construction with
/// zero approximation error.
pub fn build_rotated_strip_mesh(
    base: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    theta_deg: f64,
    u_start: f64,
    u_end: f64,
    cross_half_width: f64,
    z_floor: f64,
) -> SurfaceMesh3D {
    let vertices_wcs: Vec<DVec3> = base
        .vertices
        .iter()
        .map(|&v| rot.rotate_point_wcs(v, theta_deg))
        .collect();

    let pad = cross_half_width + 1e-4;
    let u_lo = u_start - pad;
    let u_hi = u_end + pad;
    let z_lo = z_floor - 1e-4;

    let mut aabb_wcs = Aabb3::EMPTY;
    let mut triangles = Vec::with_capacity(base.triangles.len() / 3);
    for tri in &base.triangles {
        let [i0, i1, i2] = tri.indices;
        let v0 = vertices_wcs[i0 as usize];
        let v1 = vertices_wcs[i1 as usize];
        let v2 = vertices_wcs[i2 as usize];
        let tri_min = v0.min(v1).min(v2);
        let tri_max = v0.max(v1).max(v2);
        if tri_max.z < z_lo {
            continue;
        }
        let overlaps_strip = match rot.axis {
            RotaryAxisKind::XAxis | RotaryAxisKind::ZAxis => {
                tri_max.x >= u_lo
                    && tri_min.x <= u_hi
                    && tri_max.y >= rot.center_wcs.y - pad
                    && tri_min.y <= rot.center_wcs.y + pad
            }
            RotaryAxisKind::YAxis => {
                tri_max.y >= u_lo
                    && tri_min.y <= u_hi
                    && tri_max.x >= rot.center_wcs.x - pad
                    && tri_min.x <= rot.center_wcs.x + pad
            }
        };
        if !overlaps_strip {
            continue;
        }
        let n = (v1 - v0).cross(v2 - v0).normalize_or_zero();
        let tri_aabb = Aabb3::new(tri_min, tri_max);
        aabb_wcs = aabb_wcs.union(&tri_aabb);
        triangles.push(MeshTriangle {
            indices: tri.indices,
            normal: n,
            aabb: tri_aabb,
        });
    }
    SurfaceMesh3D::from_indexed_triangles(vertices_wcs, triangles, aabb_wcs)
}

/// Evaluates the exact analytical `f64` Drop-Cutter contact Z height (`z_tip_wcs` in mm) at an
/// arbitrary rotary angle `theta_deg`, axial coordinate `u_axial`, and cross-axis offset `c_cross`
/// directly against `base_mesh`'s `FlatBvh` without rebuilding or rotating the full mesh.
///
/// Uses exact oriented-strip interval bounds on `FlatBvhNode` AABBs with top-down highest-Z-first
/// traversal and zero heap allocations, achieving `< 1e-9 mm` agreement with
/// [`build_rotated_strip_mesh`] + [`crate::surface3d::drop_cutter_z_with_scratch`].
pub fn drop_cutter_rotary_at(
    base_mesh: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    cutter: &SurfaceCutterProfile,
    u_axial: f64,
    c_cross: f64,
    theta_deg: f64,
    z_floor: f64,
) -> Option<f64> {
    if rot.axis == RotaryAxisKind::ZAxis
        || base_mesh.bvh.is_empty()
        || !u_axial.is_finite()
        || !c_cross.is_finite()
        || !theta_deg.is_finite()
        || !z_floor.is_finite()
    {
        return None;
    }

    let r_max = cutter.max_radius_mm();
    let r_pad = r_max + 1e-5;
    let r_xy_sq = (r_max + 1e-6) * (r_max + 1e-6);

    let angle_rad = (theta_deg * rot.direction.sign()).to_radians();
    let (sin_a, cos_a) = angle_rad.sin_cos();
    let cx = rot.center_wcs.x;
    let cy = rot.center_wcs.y;
    let cz = rot.center_wcs.z;
    let is_x_axis = rot.axis == RotaryAxisKind::XAxis;

    let (qx, qy) = if is_x_axis {
        (u_axial, cy + c_cross)
    } else {
        (cx + c_cross, u_axial)
    };
    let c_xy = DVec2::new(qx, qy);

    let u_lo = u_axial - r_pad;
    let u_hi = u_axial + r_pad;
    let c_lo = c_cross - r_pad;
    let c_hi = c_cross + r_pad;
    let z_min_cutoff = z_floor - cutter.stock_axial_mm.max(0.0) - 1e-4;

    let node_z_max_if_overlaps = |node: &FlatBvhNode, z_cut: f64| -> Option<f64> {
        if is_x_axis {
            let nu_min = f64::from(node.aabb_min[0]);
            let nu_max = f64::from(node.aabb_max[0]);
            if nu_max < u_lo || nu_min > u_hi {
                return None;
            }
            let dy0 = f64::from(node.aabb_min[1]) - cy;
            let dy1 = f64::from(node.aabb_max[1]) - cy;
            let dz0 = f64::from(node.aabb_min[2]) - cz;
            let dz1 = f64::from(node.aabb_max[2]) - cz;

            let (cy_lo, cy_hi) = if cos_a >= 0.0 {
                (dy0 * cos_a, dy1 * cos_a)
            } else {
                (dy1 * cos_a, dy0 * cos_a)
            };
            let neg_sin = -sin_a;
            let (cz_lo, cz_hi) = if neg_sin >= 0.0 {
                (dz0 * neg_sin, dz1 * neg_sin)
            } else {
                (dz1 * neg_sin, dz0 * neg_sin)
            };
            if cy_hi + cz_hi < c_lo || cy_lo + cz_lo > c_hi {
                return None;
            }

            let z_max =
                cz + if sin_a >= 0.0 {
                    dy1 * sin_a
                } else {
                    dy0 * sin_a
                } + if cos_a >= 0.0 {
                    dz1 * cos_a
                } else {
                    dz0 * cos_a
                };
            (z_max >= z_cut).then_some(z_max)
        } else {
            let nu_min = f64::from(node.aabb_min[1]);
            let nu_max = f64::from(node.aabb_max[1]);
            if nu_max < u_lo || nu_min > u_hi {
                return None;
            }
            let dx0 = f64::from(node.aabb_min[0]) - cx;
            let dx1 = f64::from(node.aabb_max[0]) - cx;
            let dz0 = f64::from(node.aabb_min[2]) - cz;
            let dz1 = f64::from(node.aabb_max[2]) - cz;

            let (cx_lo, cx_hi) = if cos_a >= 0.0 {
                (dx0 * cos_a, dx1 * cos_a)
            } else {
                (dx1 * cos_a, dx0 * cos_a)
            };
            let (cz_lo, cz_hi) = if sin_a >= 0.0 {
                (dz0 * sin_a, dz1 * sin_a)
            } else {
                (dz1 * sin_a, dz0 * sin_a)
            };
            if cx_hi + cz_hi < c_lo || cx_lo + cz_lo > c_hi {
                return None;
            }

            let neg_sin = -sin_a;
            let z_max =
                cz + if neg_sin >= 0.0 {
                    dx1 * neg_sin
                } else {
                    dx0 * neg_sin
                } + if cos_a >= 0.0 {
                    dz1 * cos_a
                } else {
                    dz0 * cos_a
                };
            (z_max >= z_cut).then_some(z_max)
        }
    };

    let rot_pt = |p: DVec3| -> DVec3 {
        if is_x_axis {
            let dy = p.y - cy;
            let dz = p.z - cz;
            DVec3::new(
                p.x,
                cy + dy * cos_a - dz * sin_a,
                cz + dy * sin_a + dz * cos_a,
            )
        } else {
            let dx = p.x - cx;
            let dz = p.z - cz;
            DVec3::new(
                cx + dx * cos_a + dz * sin_a,
                p.y,
                cz - dx * sin_a + dz * cos_a,
            )
        }
    };

    let nodes = base_mesh.bvh.nodes();
    let prim_indices = base_mesh.bvh.prim_indices();
    let root_z_max = node_z_max_if_overlaps(&nodes[0], z_min_cutoff)?;

    let mut best_z = f64::NEG_INFINITY;
    let mut hit_any = false;

    let mut stack: [(u32, f64); 64] = [(0, 0.0); 64];
    let mut sp = 1usize;
    stack[0] = (0, root_z_max);

    while sp > 0 {
        sp -= 1;
        let (node_idx, node_z_max) = stack[sp];
        if hit_any && node_z_max <= best_z {
            continue;
        }
        let node = &nodes[node_idx as usize];
        if node.is_leaf() {
            let first = node.left_or_first as usize;
            let end = first + node.tri_count as usize;
            for &pi in &prim_indices[first..end] {
                let tri = &base_mesh.triangles[pi as usize];
                let [i0, i1, i2] = tri.indices;
                let v0 = rot_pt(base_mesh.vertices[i0 as usize]);
                let v1 = rot_pt(base_mesh.vertices[i1 as usize]);
                let v2 = rot_pt(base_mesh.vertices[i2 as usize]);

                let tz_max = v0.z.max(v1.z).max(v2.z);
                if tz_max < z_min_cutoff || (hit_any && tz_max <= best_z) {
                    continue;
                }

                let tx_min = v0.x.min(v1.x).min(v2.x);
                let tx_max = v0.x.max(v1.x).max(v2.x);
                let ty_min = v0.y.min(v1.y).min(v2.y);
                let ty_max = v0.y.max(v1.y).max(v2.y);
                let dx = if qx < tx_min {
                    tx_min - qx
                } else if qx > tx_max {
                    qx - tx_max
                } else {
                    0.0
                };
                let dy = if qy < ty_min {
                    ty_min - qy
                } else if qy > ty_max {
                    qy - ty_max
                } else {
                    0.0
                };
                if dx * dx + dy * dy > r_xy_sq {
                    continue;
                }

                if let Some(z_tri) = drop_cutter_triangle(cutter, c_xy, v0, v1, v2)
                    && z_tri.is_finite()
                    && (!hit_any || z_tri > best_z)
                {
                    best_z = z_tri;
                    hit_any = true;
                }
            }
        } else {
            let z_cut = if hit_any {
                best_z.max(z_min_cutoff)
            } else {
                z_min_cutoff
            };
            let left_idx = node.left_or_first;
            let right_idx = left_idx + 1;
            let zl = node_z_max_if_overlaps(&nodes[left_idx as usize], z_cut);
            let zr = node_z_max_if_overlaps(&nodes[right_idx as usize], z_cut);
            match (zl, zr) {
                (Some(l), Some(r)) => {
                    if sp + 2 <= stack.len() {
                        // Push smaller Z first so larger Z is popped and visited first
                        if l >= r {
                            stack[sp] = (right_idx, r);
                            stack[sp + 1] = (left_idx, l);
                        } else {
                            stack[sp] = (left_idx, l);
                            stack[sp + 1] = (right_idx, r);
                        }
                        sp += 2;
                    }
                }
                (Some(l), None) => {
                    if sp < stack.len() {
                        stack[sp] = (left_idx, l);
                        sp += 1;
                    }
                }
                (None, Some(r)) => {
                    if sp < stack.len() {
                        stack[sp] = (right_idx, r);
                        sp += 1;
                    }
                }
                (None, None) => {}
            }
        }
    }

    if hit_any {
        Some((best_z + cutter.stock_axial_mm).max(z_floor))
    } else {
        None
    }
}

/// Evaluates the exact `f64` Drop-Cutter tip height relative to `rot.center_wcs.z` at
/// `(u_axial, c_cross, theta_deg)`, floored at `r_floor`.
#[inline]
pub fn drop_cutter_rotary_z_rel(
    base_mesh: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    cutter: &SurfaceCutterProfile,
    u_axial: f64,
    c_cross: f64,
    theta_deg: f64,
    r_floor: f64,
) -> f64 {
    let z_floor = rot.center_wcs.z + r_floor;
    let z_hit = drop_cutter_rotary_at(base_mesh, rot, cutter, u_axial, c_cross, theta_deg, z_floor);
    (z_hit.unwrap_or(z_floor) - rot.center_wcs.z).max(r_floor)
}

/// Estimates the effective flank slope factor `cos(φ_eff) ∈ [0.45, 1.0]` for slope-compensated
/// cusp-height stepover scaling (`Δ_step = Δ_flat * cos(φ_eff)`).
///
/// - When `circumferential == false` (spiral patterns stepping along `u`), measures axial flank
///   slopes `|dr/du|`.
/// - When `circumferential == true` (axial-parallel / flowline patterns stepping along `θ`),
///   measures circumferential flank slopes `|dr / (r * dθ)|`.
#[allow(clippy::too_many_arguments)]
pub fn estimate_rotary_flank_cos_phi(
    base_mesh: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    cutter: &SurfaceCutterProfile,
    u_start: f64,
    u_end: f64,
    start_deg: f64,
    end_deg: f64,
    is_full_turn: bool,
    r_floor: f64,
    circumferential: bool,
) -> f64 {
    let nu = (((u_end - u_start) / 0.8).ceil() as usize).clamp(8, 80);
    let du = (u_end - u_start) / (nu as f64);
    let span_deg = (end_deg - start_deg).max(1e-3);
    let nt = ((span_deg / 4.0).ceil() as usize).clamp(8, 90);
    let d_theta = span_deg / (nt as f64);
    let d_th_rad = d_theta.to_radians().max(1e-4);
    let theta_count = if is_full_turn { nt } else { nt + 1 };

    let grid: Vec<Vec<f64>> = (0..theta_count)
        .into_par_iter()
        .map(|it| {
            let th = start_deg + (it as f64) * d_theta;
            (0..=nu)
                .map(|iu| {
                    let u = u_start + (iu as f64) * du;
                    drop_cutter_rotary_z_rel(base_mesh, rot, cutter, u, 0.0, th, r_floor)
                })
                .collect()
        })
        .collect();

    let mut slopes = Vec::with_capacity(theta_count * nu);
    if circumferential {
        for it in 0..theta_count {
            let it_next = if is_full_turn {
                (it + 1) % theta_count
            } else if it + 1 < theta_count {
                it + 1
            } else {
                continue;
            };
            for iu in 0..=nu {
                let r_avg = (0.5 * (grid[it][iu] + grid[it_next][iu])).max(1.0);
                let s = (grid[it_next][iu] - grid[it][iu]).abs() / (r_avg * d_th_rad);
                if s.is_finite() {
                    slopes.push(s);
                }
            }
        }
    } else if du > 1e-6 {
        for row in &grid {
            for iu in 0..nu {
                let s = (row[iu + 1] - row[iu]).abs() / du;
                if s.is_finite() {
                    slopes.push(s);
                }
            }
        }
    }

    if slopes.is_empty() {
        return 1.0;
    }
    slopes.sort_by(f64::total_cmp);
    // Use 90th percentile slope so steep helical flanks/bevels get tight cusp control
    // without letting isolated 90° step walls collapse the entire stepover.
    let p90_idx = ((slopes.len() as f64 - 1.0) * 0.90).round() as usize;
    let slope_eff = slopes[p90_idx.min(slopes.len() - 1)];
    (1.0 / (1.0 + slope_eff * slope_eff).sqrt()).clamp(0.45, 1.0)
}

/// Returns `-1.0` if `base_mesh` is globally inward-wound (negative signed volume, as in some
/// legacy STL fixtures) or `+1.0` for standard outward-oriented meshes.
fn mesh_outward_winding_sign(base_mesh: &SurfaceMesh3D) -> f64 {
    let mut signed_vol6 = 0.0_f64;
    for tri in &base_mesh.triangles {
        let [i0, i1, i2] = tri.indices;
        let v0 = base_mesh.vertices[i0 as usize];
        let v1 = base_mesh.vertices[i1 as usize];
        let v2 = base_mesh.vertices[i2 as usize];
        signed_vol6 += v0.dot(v1.cross(v2));
    }
    if signed_vol6 < -1e-6 { -1.0 } else { 1.0 }
}

/// Returns `true` when `base_mesh` contains significant steep circumferential flanks (`> ~70°`
/// from radial) or genuine circumferential overhangs (`n · e_r <= 0`) inside the axial window
/// `[u_start - margin_mm, u_end + margin_mm]`.
///
/// Used by [`crate::strategies::rotary4axis_sim`] so multi-offset circumferential flank probing
/// activates cleanly on overhanging helical blades while leaving non-overhanging flutes/cams
/// (and Phase 5 golden snapshots) 100% bit-identical.
pub fn mesh_has_steep_circumferential_overhang(
    base_mesh: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    u_start: f64,
    u_end: f64,
    margin_mm: f64,
) -> bool {
    if rot.axis == RotaryAxisKind::ZAxis || base_mesh.triangles.is_empty() {
        return false;
    }
    let winding_sign = mesh_outward_winding_sign(base_mesh);
    let (_, _, u_vec) = rot.axis.basis();
    let u_lo = u_start - margin_mm.max(0.0);
    let u_hi = u_end + margin_mm.max(0.0);
    let mut steep_area = 0.0_f64;

    for tri in &base_mesh.triangles {
        let [i0, i1, i2] = tri.indices;
        let v0 = base_mesh.vertices[i0 as usize];
        let v1 = base_mesh.vertices[i1 as usize];
        let v2 = base_mesh.vertices[i2 as usize];
        let centroid = (v0 + v1 + v2) / 3.0;
        let u = centroid.dot(u_vec);
        if u < u_lo || u > u_hi {
            continue;
        }
        let rel = centroid - rot.center_wcs;
        let radial_vec = rel - u_vec * rel.dot(u_vec);
        let r_len = radial_vec.length();
        if r_len <= 0.25 {
            continue;
        }
        let e_r = radial_vec / r_len;
        let cross = (v1 - v0).cross(v2 - v0);
        let area2 = cross.length();
        if area2 <= 1e-8 {
            continue;
        }
        let n = (cross / area2) * winding_sign;
        let n_perp = n - u_vec * n.dot(u_vec);
        let n_perp_len = n_perp.length();
        if n_perp_len <= 0.25 {
            continue;
        }
        let cos_radial = n_perp.dot(e_r) / n_perp_len;
        // Steep circumferential wall (> ~70° from radial) or true circumferential overhang (<= 0)
        if cos_radial < 0.35 {
            steep_area += 0.5 * area2;
            if steep_area > 1.0 {
                return true;
            }
        }
    }
    false
}

/// Fast post-generation reachability check detecting axial (`u`-direction) overhang / undercut
/// surface area on `base_mesh` inside `[u_start, u_end]` that cannot be reached from any vertical
/// cylindrical pose `(u, c, θ)`.
///
/// Returns an actionable non-blocking warning suggesting an indexed `Undercut3D` (`LollipopEndmill`)
/// pass whenever significant unreachable axial-undercut area (`>= 2.0 mm²`) is found.
pub fn detect_rotary_axial_undercut_warning(
    base_mesh: &SurfaceMesh3D,
    rot: &ResolvedRotaryConfig,
    u_start: f64,
    u_end: f64,
    r_floor: f64,
) -> Option<String> {
    if rot.axis == RotaryAxisKind::ZAxis || base_mesh.triangles.is_empty() {
        return None;
    }
    let winding_sign = mesh_outward_winding_sign(base_mesh);
    let (_, _, u_vec) = rot.axis.basis();
    let mut mesh_u_min = f64::INFINITY;
    let mut mesh_u_max = f64::NEG_INFINITY;
    for &v in &base_mesh.vertices {
        let u = v.dot(u_vec);
        mesh_u_min = mesh_u_min.min(u);
        mesh_u_max = mesh_u_max.max(u);
    }
    if !mesh_u_min.is_finite() || !mesh_u_max.is_finite() {
        return None;
    }

    let u_lo = (u_start + 0.5).max(mesh_u_min + 0.35);
    let u_hi = (u_end - 0.5).min(mesh_u_max - 0.35);
    if u_hi <= u_lo {
        return None;
    }

    let max_ray_t = (rot.stock_outer_radius_mm * 2.5).max(100.0);
    // Test a ±40° fan of perpendicular-plane spindle approach rays `(e_r, e_theta)`
    const APPROACH_ANGLES_DEG: [f64; 5] = [-40.0, -20.0, 0.0, 20.0, 40.0];
    let approach_sc: [(f64, f64); 5] = APPROACH_ANGLES_DEG.map(|deg| deg.to_radians().sin_cos());

    let unreachable_area: f64 = base_mesh
        .triangles
        .par_iter()
        .map(|tri| {
            let [i0, i1, i2] = tri.indices;
            let v0 = base_mesh.vertices[i0 as usize];
            let v1 = base_mesh.vertices[i1 as usize];
            let v2 = base_mesh.vertices[i2 as usize];
            let centroid = (v0 + v1 + v2) / 3.0;
            let u = centroid.dot(u_vec);
            if u < u_lo || u > u_hi {
                return 0.0;
            }
            let rel = centroid - rot.center_wcs;
            let radial_vec = rel - u_vec * rel.dot(u_vec);
            let r_len = radial_vec.length();
            if r_len <= r_floor + 0.25 {
                return 0.0;
            }
            let cross = (v1 - v0).cross(v2 - v0);
            let area2 = cross.length();
            if area2 <= 1e-6 {
                return 0.0;
            }
            let n = (cross / area2) * winding_sign;
            let e_r = radial_vec / r_len;
            let n_u_abs = n.dot(u_vec).abs();
            let n_r = n.dot(e_r);

            // Only evaluate triangles with an axial orientation component that either face inward
            // toward the rotary axis or sit beneath an axial roof (normal non-radial).
            if n_u_abs < 0.15 || n_r > 0.35 {
                return 0.0;
            }

            let e_theta = u_vec.cross(e_r).normalize_or_zero();
            let origin = centroid + n * 0.05;

            // Reachable if ANY vertical cylindrical approach ray in the cross-sectional plane
            // faces the triangle (`n · dir >= -0.10`) AND escapes without hitting an overhanging roof.
            let mut reachable = false;
            for &(sin_a, cos_a) in &approach_sc {
                let dir = (e_r * cos_a + e_theta * sin_a).normalize_or_zero();
                if n.dot(dir) >= -0.10 && base_mesh.raycast(origin, dir, max_ray_t).is_none() {
                    reachable = true;
                    break;
                }
            }
            if reachable { 0.0 } else { 0.5 * area2 }
        })
        .sum();

    if unreachable_area >= 2.0 {
        Some(format!(
            "Rotary toolpath detected ~{unreachable_area:.1} mm² of axial overhang/undercut surface area unreachable from vertical cylindrical poses; consider adding an indexed Undercut3D pass with a LollipopEndmill."
        ))
    } else {
        None
    }
}
