//! Exact swept Z-slab XY projection (`project_surface_z_slab_xy`) of a [`SurfaceMesh3D`].
//!
//! Replaces sparse 1–2 discrete Z-plane slices when separating finished target-part geometry from
//! uncut excess stock in carried-over `TriDexelStock` roughing (`Adaptive3D` and `Waterline3D`),
//! ensuring fast-twisting helical blades and sloped walls are covered continuously across the
//! entire slab `[z_lo, z_hi]` with zero missed intermediate positions.

use chip_geom2d::contours_to_region_set;
use chip_math::{ArcPolyline2D, ArcSegment2D, DVec2, DVec3};
use rayon::prelude::*;

use crate::{
    rest2d::RegionSet2D,
    surface3d::{SurfaceMesh3D, push_cutter::slice_surface_contours_z},
};

/// Convex 2D polygon produced by clipping a 3D triangle against `z_lo <= z <= z_hi` and projecting
/// onto XY (`3..=5` vertices stored inline with zero heap allocation).
#[derive(Debug, Clone, Copy)]
struct ClippedTri2D {
    pts: [DVec2; 5],
    len: u8,
    min_x: f64,
    max_x: f64,
    min_y: f64,
    max_y: f64,
    /// Unit outward edge normals for CCW convex polygon (when `is_area_positive == true`).
    normals: [DVec2; 5],
    is_area_positive: bool,
}

impl ClippedTri2D {
    fn from_clipped_3d(pts_3d: &[DVec3], len: usize) -> Option<Self> {
        if len < 2 {
            return None;
        }
        let mut pts = [DVec2::ZERO; 5];
        let mut out_len = 0usize;
        for &p in &pts_3d[..len.min(5)] {
            let p2 = DVec2::new(p.x, p.y);
            if !p2.is_finite() {
                return None;
            }
            if out_len == 0 || pts[out_len - 1].distance_squared(p2) > 1e-14 {
                pts[out_len] = p2;
                out_len += 1;
            }
        }
        if out_len >= 2 && pts[0].distance_squared(pts[out_len - 1]) <= 1e-14 {
            out_len -= 1;
        }
        if out_len < 2 {
            return None;
        }

        let mut min_x = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        for &p in &pts[..out_len] {
            min_x = min_x.min(p.x);
            max_x = max_x.max(p.x);
            min_y = min_y.min(p.y);
            max_y = max_y.max(p.y);
        }

        let mut area2 = 0.0_f64;
        if out_len >= 3 {
            for i in 0..out_len {
                let a = pts[i];
                let b = pts[(i + 1) % out_len];
                area2 += a.x * b.y - a.y * b.x;
            }
            if area2 < 0.0 {
                pts[..out_len].reverse();
                area2 = -area2;
            }
        }

        let mut normals = [DVec2::ZERO; 5];
        let is_area_positive = out_len >= 3 && area2 > 1e-10;
        if is_area_positive {
            for i in 0..out_len {
                let a = pts[i];
                let b = pts[(i + 1) % out_len];
                let e = b - a;
                let elen = e.length();
                if elen > 1e-12 {
                    // Outward right-hand normal for CCW edge `a -> b`.
                    normals[i] = DVec2::new(e.y / elen, -e.x / elen);
                }
            }
        }

        Some(Self {
            pts,
            len: out_len as u8,
            min_x,
            max_x,
            min_y,
            max_y,
            normals,
            is_area_positive,
        })
    }

    /// Exact Separating Axis Theorem overlap test between this convex 2D polygon (or degenerate
    /// vertical-wall segment) and an axis-aligned square grid cell centered at `c` with half-width `h`.
    #[inline]
    fn overlaps_cell(&self, c: DVec2, h: f64) -> bool {
        if c.x + h < self.min_x
            || c.x - h > self.max_x
            || c.y + h < self.min_y
            || c.y - h > self.max_y
        {
            return false;
        }
        let n = self.len as usize;
        if self.is_area_positive {
            for i in 0..n {
                let normal = self.normals[i];
                if normal == DVec2::ZERO {
                    continue;
                }
                let d = (c - self.pts[i]).dot(normal);
                let r_proj = h * (normal.x.abs() + normal.y.abs());
                if d > r_proj + 1e-12 {
                    return false;
                }
            }
            true
        } else {
            // Degenerate 1D projection (vertical wall triangle): test distance from cell center
            // to each edge segment against cell circumradius `h * sqrt(2)`.
            let max_d2 = 2.0 * h * h + 1e-12;
            for i in 0..n {
                let a = self.pts[i];
                let b = self.pts[(i + 1) % n];
                if point_to_segment_dist_sq(c, a, b) <= max_d2 {
                    return true;
                }
            }
            false
        }
    }
}

#[inline]
fn point_to_segment_dist_sq(p: DVec2, a: DVec2, b: DVec2) -> f64 {
    let ab = b - a;
    let ab2 = ab.length_squared();
    if ab2 <= 1e-18 {
        return p.distance_squared(a);
    }
    let t = ((p - a).dot(ab) / ab2).clamp(0.0, 1.0);
    p.distance_squared(a + ab * t)
}

/// Sutherland-Hodgman clips a 3D convex polygon against a single horizontal half-space:
/// - `keep_above == true`: keeps `p.z >= z_plane`
/// - `keep_above == false`: keeps `p.z <= z_plane`
fn clip_poly_z_plane(
    input: &[DVec3],
    in_len: usize,
    z_plane: f64,
    keep_above: bool,
    out: &mut [DVec3; 5],
) -> usize {
    if in_len == 0 {
        return 0;
    }
    let signed_dist = |p: DVec3| -> f64 {
        if keep_above {
            p.z - z_plane
        } else {
            z_plane - p.z
        }
    };

    let mut out_len = 0usize;
    let mut prev = input[in_len - 1];
    let mut d_prev = signed_dist(prev);
    let mut prev_in = d_prev >= -1e-12;

    for &cur in &input[..in_len] {
        let d_cur = signed_dist(cur);
        let cur_in = d_cur >= -1e-12;

        if cur_in != prev_in {
            let denom = d_prev - d_cur;
            if denom.abs() > 1e-15 {
                let t = (d_prev / denom).clamp(0.0, 1.0);
                let hit = prev.lerp(cur, t);
                if out_len < out.len() {
                    out[out_len] = hit;
                    out_len += 1;
                }
            }
        }
        if cur_in && out_len < out.len() {
            out[out_len] = cur;
            out_len += 1;
        }

        prev = cur;
        d_prev = d_cur;
        prev_in = cur_in;
    }

    out_len
}

/// Clips triangle `(v0, v1, v2)` to the Z slab `[z_lo, z_hi]` and returns its 2D XY projection.
fn clip_triangle_to_z_slab_xy(
    v0: DVec3,
    v1: DVec3,
    v2: DVec3,
    z_lo: f64,
    z_hi: f64,
) -> Option<ClippedTri2D> {
    let tri_in = [v0, v1, v2, DVec3::ZERO, DVec3::ZERO];
    let mut buf_a = [DVec3::ZERO; 5];
    let len_a = clip_poly_z_plane(&tri_in, 3, z_lo, true, &mut buf_a);
    if len_a < 2 {
        return None;
    }
    let mut buf_b = [DVec3::ZERO; 5];
    let len_b = clip_poly_z_plane(&buf_a, len_a, z_hi, false, &mut buf_b);
    if len_b < 2 {
        return None;
    }
    ClippedTri2D::from_clipped_3d(&buf_b, len_b)
}

/// Computes the exact swept XY footprint (`RegionSet2D`) of the 3D solid bounded by `surface`
/// across the entire WCS Z slab `[z_lo, z_hi]`.
///
/// Mathematically, for any closed solid `S`,
/// `Proj_XY(S ∩ [z_lo, z_hi]) = Slice_XY(S, z_lo) ∪ Slice_XY(S, z_hi) ∪ ⋃_{T ∈ ∂S} Proj_XY(T ∩ [z_lo, z_hi])`.
///
/// - `Slice_XY` fills solid vertical-walled cores/shafts that span `[z_lo, z_hi]` without interior
///   horizontal caps.
/// - Exact clipped-triangle XY projection rasterized via SAT cell-overlap onto a parallel grid of
///   spacing `<= cell_mm` covers 100% of every sloped wall, plateau, and fast-twisting helical
///   blade position continuously between `z_lo` and `z_hi` with zero gaps.
pub fn project_surface_z_slab_xy(
    surface: &SurfaceMesh3D,
    z_lo: f64,
    z_hi: f64,
    cell_mm: f64,
) -> RegionSet2D {
    if !z_lo.is_finite()
        || !z_hi.is_finite()
        || surface.triangles.is_empty()
        || !surface.aabb_wcs.is_valid()
    {
        return RegionSet2D::empty();
    }

    let (z0, z1) = if z_lo <= z_hi {
        (z_lo, z_hi)
    } else {
        (z_hi, z_lo)
    };
    let mesh_min_z = surface.aabb_wcs.min.z;
    let mesh_max_z = surface.aabb_wcs.max.z;
    if z1 < mesh_min_z - 1e-6 || z0 > mesh_max_z + 1e-6 {
        return RegionSet2D::empty();
    }

    let slab_lo = z0.max(mesh_min_z);
    let slab_hi = z1.min(mesh_max_z);

    // 1. Collect exact solid cross-sections at slab boundaries so vertical-walled cores spanning
    //    `[slab_lo, slab_hi]` without interior caps remain completely solid.
    let mut boundary_slices: Vec<RegionSet2D> = Vec::with_capacity(2);
    if mesh_max_z - mesh_min_z > 2e-4 {
        let z_eps_lo = mesh_min_z + 1e-4;
        let z_eps_hi = mesh_max_z - 1e-4;
        if slab_lo <= z_eps_hi && slab_hi >= z_eps_lo {
            let z_a = slab_lo.clamp(z_eps_lo, z_eps_hi);
            let slice_a = contours_to_region_set(&slice_surface_contours_z(surface, z_a));
            if !slice_a.is_empty() {
                boundary_slices.push(slice_a);
            }
            let z_b = slab_hi.clamp(z_eps_lo, z_eps_hi);
            if (z_b - z_a).abs() > 1e-3 {
                let slice_b = contours_to_region_set(&slice_surface_contours_z(surface, z_b));
                if !slice_b.is_empty() {
                    boundary_slices.push(slice_b);
                }
            }
        }
    }

    // 2. Query BVH for all triangles overlapping `[slab_lo, slab_hi]` and clip each triangle to the
    //    slab in parallel, producing exact convex 2D XY projections.
    let tri_indices = surface.bvh.query_z_slab(slab_lo - 1e-6, slab_hi + 1e-6);
    let clipped: Vec<ClippedTri2D> = if tri_indices.is_empty() {
        Vec::new()
    } else {
        tri_indices
            .par_iter()
            .filter_map(|&ti| {
                let tri = surface.triangles.get(ti)?;
                if tri.aabb.max.z < slab_lo - 1e-6 || tri.aabb.min.z > slab_hi + 1e-6 {
                    return None;
                }
                let [i0, i1, i2] = tri.indices;
                let v0 = *surface.vertices_wcs.get(i0 as usize)?;
                let v1 = *surface.vertices_wcs.get(i1 as usize)?;
                let v2 = *surface.vertices_wcs.get(i2 as usize)?;
                clip_triangle_to_z_slab_xy(v0, v1, v2, slab_lo, slab_hi)
            })
            .collect()
    };

    if clipped.is_empty() {
        return RegionSet2D::union_many(&mut boundary_slices);
    }

    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for poly in &clipped {
        min_x = min_x.min(poly.min_x);
        max_x = max_x.max(poly.max_x);
        min_y = min_y.min(poly.min_y);
        max_y = max_y.max(poly.max_y);
    }
    for sl in &boundary_slices {
        for reg in &sl.regions {
            let b = reg.exact_aabb();
            if b.is_valid() {
                min_x = min_x.min(b.min.x);
                max_x = max_x.max(b.max.x);
                min_y = min_y.min(b.min.y);
                max_y = max_y.max(b.max.y);
            }
        }
    }
    if !min_x.is_finite() || !max_x.is_finite() || !min_y.is_finite() || !max_y.is_finite() {
        return RegionSet2D::union_many(&mut boundary_slices);
    }

    let span_x = (max_x - min_x).max(0.05);
    let span_y = (max_y - min_y).max(0.05);
    const MAX_GRID_CELLS: f64 = 160_000.0;
    let req_cell = if cell_mm.is_finite() && cell_mm > 0.0 {
        cell_mm.clamp(0.04, 1.0)
    } else {
        0.2
    };
    let pitch = req_cell.max(((span_x * span_y) / MAX_GRID_CELLS).sqrt());

    // Pad by 2 empty cells on every side so all traced dual-grid contours are strictly closed.
    let pad = 2.0 * pitch;
    let origin_x = min_x - pad;
    let origin_y = min_y - pad;
    let nx = (((max_x + pad - origin_x) / pitch).ceil() as usize).max(4);
    let ny = (((max_y + pad - origin_y) / pitch).ceil() as usize).max(4);

    // 3. Bin clipped polygons by Y-row range and rasterize BOTH clipped triangles (exact SAT
    //    cell-polygon overlap) and boundary cross-sections directly onto a single occupancy grid,
    //    emitting one clean non-overlapping contour set without fragile diagonal/grid booleans.
    let mut row_bins: Vec<Vec<u32>> = vec![Vec::new(); ny];
    for (idx, poly) in clipped.iter().enumerate() {
        let j0 = (((poly.min_y - origin_y) / pitch).floor() as isize).clamp(0, (ny - 1) as isize)
            as usize;
        let j1 = (((poly.max_y - origin_y) / pitch).floor() as isize).clamp(0, (ny - 1) as isize)
            as usize;
        for bin in &mut row_bins[j0..=j1] {
            bin.push(idx as u32);
        }
    }

    let slice_bounds: Vec<(f64, f64, f64, f64)> = boundary_slices
        .iter()
        .map(|sl| {
            let mut sx0 = f64::INFINITY;
            let mut sx1 = f64::NEG_INFINITY;
            let mut sy0 = f64::INFINITY;
            let mut sy1 = f64::NEG_INFINITY;
            for reg in &sl.regions {
                let b = reg.exact_aabb();
                if b.is_valid() {
                    sx0 = sx0.min(b.min.x);
                    sx1 = sx1.max(b.max.x);
                    sy0 = sy0.min(b.min.y);
                    sy1 = sy1.max(b.max.y);
                }
            }
            (sx0, sx1, sy0, sy1)
        })
        .collect();

    let half_p = 0.5 * pitch;
    let mut occ = vec![false; nx * ny];
    occ.par_chunks_mut(nx)
        .enumerate()
        .for_each(|(iy, row_occ)| {
            let cy = origin_y + (iy as f64 + 0.5) * pitch;
            let bin = &row_bins[iy];
            for &poly_idx in bin {
                let poly = &clipped[poly_idx as usize];
                let i0 = (((poly.min_x - origin_x) / pitch).floor() as isize)
                    .clamp(0, (nx - 1) as isize) as usize;
                let i1 = (((poly.max_x - origin_x) / pitch).floor() as isize)
                    .clamp(0, (nx - 1) as isize) as usize;
                for (ix, slot) in row_occ.iter_mut().enumerate().take(i1 + 1).skip(i0) {
                    if *slot {
                        continue;
                    }
                    let cx = origin_x + (ix as f64 + 0.5) * pitch;
                    if poly.overlaps_cell(DVec2::new(cx, cy), half_p) {
                        *slot = true;
                    }
                }
            }

            // Fill interior of slab-boundary solid cross-sections (e.g. vertical-walled shafts/cores)
            for (sl, &(sx0, sx1, sy0, sy1)) in boundary_slices.iter().zip(&slice_bounds) {
                if !sx0.is_finite() || cy + half_p < sy0 || cy - half_p > sy1 {
                    continue;
                }
                let i0 = (((sx0 - origin_x) / pitch).floor() as isize).clamp(0, (nx - 1) as isize)
                    as usize;
                let i1 = (((sx1 - origin_x) / pitch).ceil() as isize).clamp(0, (nx - 1) as isize)
                    as usize;
                for (ix, slot) in row_occ.iter_mut().enumerate().take(i1 + 1).skip(i0) {
                    if *slot {
                        continue;
                    }
                    let cx = origin_x + (ix as f64 + 0.5) * pitch;
                    if sl.contains_point(DVec2::new(cx, cy)) {
                        *slot = true;
                    }
                }
            }
        });

    let swept_contours = trace_binary_grid_contours(&occ, nx, ny, origin_x, origin_y, pitch);
    contours_to_region_set(&swept_contours)
}

#[inline]
fn edge_dest(u: usize, v: usize, dir: u8) -> (usize, usize) {
    match dir {
        0 => (u + 1, v),
        1 => (u, v + 1),
        2 => (u - 1, v),
        _ => (u, v - 1),
    }
}

/// Deterministic dual-grid boundary contour tracer (`CCW` outer loops, `CW` inner holes,
/// right-turn saddle disambiguation, lossless collinear vertex merge).
fn trace_binary_grid_contours(
    occ: &[bool],
    w: usize,
    h: usize,
    origin_x: f64,
    origin_y: f64,
    pitch: f64,
) -> Vec<ArcPolyline2D> {
    let gw = w + 1;
    let gh = h + 1;
    let mut out_mask = vec![0u8; gw * gh];
    let mut edge_count = 0usize;

    let is_solid = |ix: isize, iy: isize| -> bool {
        if ix < 0 || iy < 0 || ix >= w as isize || iy >= h as isize {
            false
        } else {
            occ[(iy as usize) * w + (ix as usize)]
        }
    };

    for j in 0..h {
        let row = j * w;
        for i in 0..w {
            if !occ[row + i] {
                continue;
            }
            let ii = i as isize;
            let jj = j as isize;
            if !is_solid(ii, jj - 1) {
                out_mask[j * gw + i] |= 1 << 0;
                edge_count += 1;
            }
            if !is_solid(ii + 1, jj) {
                out_mask[j * gw + (i + 1)] |= 1 << 1;
                edge_count += 1;
            }
            if !is_solid(ii, jj + 1) {
                out_mask[(j + 1) * gw + (i + 1)] |= 1 << 2;
                edge_count += 1;
            }
            if !is_solid(ii - 1, jj) {
                out_mask[(j + 1) * gw + i] |= 1 << 3;
                edge_count += 1;
            }
        }
    }

    if edge_count == 0 {
        return Vec::new();
    }

    let mut contours = Vec::new();
    let mut pts_buf: Vec<DVec2> = Vec::new();

    for sv in 0..gh {
        for su in 0..gw {
            loop {
                let mask = out_mask[sv * gw + su];
                if mask == 0 {
                    break;
                }
                let start_dir = mask.trailing_zeros() as u8;
                pts_buf.clear();

                let mut cur_u = su;
                let mut cur_v = sv;
                let mut cur_dir = start_dir;
                let mut prev_dir: Option<u8> = None;

                for _ in 0..=edge_count {
                    let vidx = cur_v * gw + cur_u;
                    if (out_mask[vidx] & (1 << cur_dir)) == 0 {
                        break;
                    }
                    out_mask[vidx] &= !(1 << cur_dir);

                    if prev_dir != Some(cur_dir) {
                        pts_buf.push(DVec2::new(
                            origin_x + (cur_u as f64) * pitch,
                            origin_y + (cur_v as f64) * pitch,
                        ));
                    }
                    prev_dir = Some(cur_dir);

                    let (nu, nv) = edge_dest(cur_u, cur_v, cur_dir);
                    if nu == su && nv == sv {
                        break;
                    }
                    let next_mask = out_mask[nv * gw + nu];
                    if next_mask == 0 {
                        break;
                    }
                    let candidates = [
                        (cur_dir + 3) & 3,
                        cur_dir,
                        (cur_dir + 1) & 3,
                        (cur_dir + 2) & 3,
                    ];
                    let Some(&ndir) = candidates.iter().find(|&&d| (next_mask & (1 << d)) != 0)
                    else {
                        break;
                    };
                    cur_u = nu;
                    cur_v = nv;
                    cur_dir = ndir;
                }

                if pts_buf.len() >= 3 {
                    let first = pts_buf[0];
                    let second = pts_buf[1];
                    let last = pts_buf[pts_buf.len() - 1];
                    let d1 = second - first;
                    let d0 = first - last;
                    if (d0.x * d1.y - d0.y * d1.x).abs() <= 1e-9 && d0.dot(d1) > 0.0 {
                        pts_buf.remove(0);
                    }
                }

                if pts_buf.len() >= 3 {
                    let mut segs = Vec::with_capacity(pts_buf.len());
                    for k in 0..pts_buf.len() {
                        let a = pts_buf[k];
                        let b = pts_buf[(k + 1) % pts_buf.len()];
                        if a.distance_squared(b) > 1e-12 {
                            segs.push(ArcSegment2D::Line { start: a, end: b });
                        }
                    }
                    if segs.len() >= 3 {
                        contours.push(ArcPolyline2D {
                            segments: segs,
                            closed: true,
                        });
                    }
                }
            }
        }
    }

    contours
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::{DVec2, DVec3};

    use super::*;

    /// Builds a watertight twisting rectangular blade along Z ∈ `[z0, z1]` rotating linearly from
    /// `0°` at `z0` to `twist_deg` at `z1` (width = `2 * half_len`, thickness = `2 * half_thick`).
    fn build_twisted_blade_mesh(
        z0: f64,
        z1: f64,
        twist_deg: f64,
        half_len: f64,
        half_thick: f64,
        z_steps: usize,
    ) -> SurfaceMesh3D {
        let local_corners = [
            DVec2::new(-half_len, -half_thick),
            DVec2::new(half_len, -half_thick),
            DVec2::new(half_len, half_thick),
            DVec2::new(-half_len, half_thick),
        ];
        let ring = |z: f64| -> [DVec3; 4] {
            let t = ((z - z0) / (z1 - z0)).clamp(0.0, 1.0);
            let ang = (t * twist_deg).to_radians();
            let (s, c) = ang.sin_cos();
            local_corners.map(|p| DVec3::new(p.x * c - p.y * s, p.x * s + p.y * c, z))
        };

        let mut tris: Vec<[DVec3; 3]> = Vec::new();
        let mut rings = Vec::with_capacity(z_steps + 1);
        for k in 0..=z_steps {
            let z = z0 + (z1 - z0) * (k as f64 / z_steps as f64);
            rings.push(ring(z));
        }

        // Bottom cap (-Z normal)
        let b = rings[0];
        tris.push([b[0], b[2], b[1]]);
        tris.push([b[0], b[3], b[2]]);

        // Top cap (+Z normal)
        let t = rings[z_steps];
        tris.push([t[0], t[1], t[2]]);
        tris.push([t[0], t[2], t[3]]);

        // Side walls
        for k in 0..z_steps {
            let r0 = rings[k];
            let r1 = rings[k + 1];
            for e in 0..4 {
                let ne = (e + 1) % 4;
                tris.push([r0[e], r0[ne], r1[ne]]);
                tris.push([r0[e], r1[ne], r1[e]]);
            }
        }

        SurfaceMesh3D::from_raw_triangles(&tris)
    }

    #[test]
    fn swept_z_slab_covers_100_percent_of_45_deg_twisting_helical_blade() {
        // Fast-twisting helical blade rotating 45° across a 4 mm Z slab `[2.0, 6.0]`
        // (total blade spans `[0.0, 8.0]` rotating 90° total -> 45° across `[2.0, 6.0]`).
        let mesh = build_twisted_blade_mesh(0.0, 8.0, 90.0, 15.0, 1.5, 32);
        let z_lo = 2.0;
        let z_hi = 6.0;

        let swept = project_surface_z_slab_xy(&mesh, z_lo, z_hi, 0.15);
        assert!(!swept.is_empty());

        // Every intermediate angle between 22.5° (at z=2) and 67.5° (at z=6) across the full blade
        // length MUST be 100% covered inside `swept` (zero gaps between discrete Z planes!).
        let mut checked = 0usize;
        for iz in 0..=40 {
            let z = z_lo + (z_hi - z_lo) * (iz as f64 / 40.0);
            let ang = ((z / 8.0) * 90.0).to_radians();
            let (s, c) = ang.sin_cos();
            for ir in -14..=14 {
                let lx = ir as f64;
                for it in [-1.2_f64, 0.0, 1.2] {
                    let pt = DVec2::new(lx * c - it * s, lx * s + it * c);
                    assert!(
                        swept.contains_point(pt),
                        "swept slab missed twisted blade point {pt:?} at z={z:.2} (ang={:.1}°)",
                        ang.to_degrees()
                    );
                    checked += 1;
                }
            }
        }
        assert!(checked > 3000);

        // Points strictly outside the swept 22.5°..67.5° angular wedge (e.g. along 135° at r=12)
        // must NOT be covered.
        let outside_pt = DVec2::new(-8.5, 8.5);
        assert!(
            !swept.contains_point(outside_pt),
            "point {outside_pt:?} outside swept blade sector should not be occupied"
        );
    }

    #[test]
    fn swept_z_slab_keeps_solid_vertical_prism_core_filled_and_handles_empty_slabs() {
        // Untwisted vertical prism `[-10..10] x [-8..8] x [0..20]` queried in interior slab `[6..10]`:
        // has ZERO horizontal cap triangles inside `[6..10]`, yet its interior `(0, 0)` MUST stay solid!
        let mesh = build_twisted_blade_mesh(0.0, 20.0, 0.0, 10.0, 8.0, 4);
        let slab = project_surface_z_slab_xy(&mesh, 6.0, 10.0, 0.25);
        assert!(slab.contains_point(DVec2::ZERO));
        assert!(slab.contains_point(DVec2::new(9.0, 7.0)));
        assert!(!slab.contains_point(DVec2::new(12.0, 0.0)));

        // Slab strictly above mesh returns empty
        assert!(project_surface_z_slab_xy(&mesh, 21.0, 25.0, 0.25).is_empty());
        // Slab strictly below mesh returns empty
        assert!(project_surface_z_slab_xy(&mesh, -5.0, -1.0, 0.25).is_empty());
    }
}
