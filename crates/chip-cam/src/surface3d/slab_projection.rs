//! Exact swept Z-slab XY projection (`project_surface_z_slab_xy`) of a [`SurfaceMesh3D`].
//!
//! Replaces sparse 1–2 discrete Z-plane slices when separating finished target-part geometry from
//! uncut excess stock in carried-over `TriDexelStock` roughing (`Adaptive3D` and `Waterline3D`),
//! ensuring fast-twisting helical blades and sloped walls are covered continuously across the
//! entire slab `[z_lo, z_hi]` with zero missed intermediate positions.

use chip_bvh::{TriMesh3D, clip_triangle_to_z_slab};
use chip_geom2d::contours_to_region_set;
use chip_math::{DVec2, DVec3, point_to_segment_dist_sq_2d, trace_binary_grid_contours};
use rayon::prelude::*;

use crate::{rest2d::RegionSet2D, surface3d::push_cutter::slice_surface_contours_z};

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
                if point_to_segment_dist_sq_2d(c, a, b) <= max_d2 {
                    return true;
                }
            }
            false
        }
    }
}

/// Clips triangle `(v0, v1, v2)` to the Z slab `[z_lo, z_hi]` and returns its 2D XY projection.
fn clip_triangle_to_z_slab_xy(
    v0: DVec3,
    v1: DVec3,
    v2: DVec3,
    z_lo: f64,
    z_hi: f64,
) -> Option<ClippedTri2D> {
    let mut clipped = [DVec3::ZERO; 7];
    let len = clip_triangle_to_z_slab(v0, v1, v2, z_lo, z_hi, &mut clipped);
    if len < 2 {
        return None;
    }
    ClippedTri2D::from_clipped_3d(&clipped, len)
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
    surface: &(impl AsRef<TriMesh3D> + ?Sized),
    z_lo: f64,
    z_hi: f64,
    cell_mm: f64,
) -> RegionSet2D {
    let surface = surface.as_ref();
    if !z_lo.is_finite()
        || !z_hi.is_finite()
        || surface.triangles.is_empty()
        || !surface.bounds.is_valid()
    {
        return RegionSet2D::empty();
    }

    let (z0, z1) = if z_lo <= z_hi {
        (z_lo, z_hi)
    } else {
        (z_hi, z_lo)
    };
    let mesh_min_z = surface.bounds.min.z;
    let mesh_max_z = surface.bounds.max.z;
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
                let v0 = *surface.vertices.get(i0 as usize)?;
                let v1 = *surface.vertices.get(i1 as usize)?;
                let v2 = *surface.vertices.get(i2 as usize)?;
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

    let swept_contours =
        trace_binary_grid_contours(&occ, nx, ny, origin_x, origin_y, pitch, pitch, 0.0);
    contours_to_region_set(&swept_contours)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::{DVec2, DVec3};

    use super::*;
    use crate::surface3d::SurfaceMesh3D;

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
