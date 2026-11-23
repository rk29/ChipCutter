//! Rigid-frame volumetric carryover transfer (`Setup i [G54] -> Setup j [G55]`) and pure-CPU
//! rest-stock occupancy queries on [`TriDexelStock`].

use std::collections::BTreeMap;

use chip_bvh::TriMesh3D;
use chip_math::{Aabb2, Aabb3, ArcPolyline2D, ArcSegment2D, DVec2, DVec3, RigidFrame3};
use rayon::prelude::*;

use crate::types::{Axis3, SimError, SimResolution, UNCUT_OP_IDX};

use super::{
    grid::{
        DexelPlane, DexelSpan, FlatDexelSpan, INLINE_DEXEL_SLOTS, MIN_DEXEL_SPAN_MM, TriDexelStock,
        query_overlap_length,
    },
    mesher::tessellate_tridexel_stock_mesh,
};

/// Exact signed cardinal axis permutation representation of a rigid rotation `R = dst^{-1} * src`.
///
/// For each destination axis `d ∈ {0, 1, 2}`:
/// `p_dst[d] = dst_sign[d] * p_src[src_for_dst[d]] + t_dst[d]`.
#[derive(Debug, Clone, Copy)]
struct CardinalPermutation {
    src_for_dst: [Axis3; 3],
    dst_sign: [i8; 3],
    t_dst: DVec3,
}

/// Detects whether `M_src_to_dst = dst_frame^{-1} * src_frame` has a rotation matrix whose columns
/// are signed cardinal unit axes within `1e-6` (covers all `90°` / `180°` flip and 6-sided setups).
fn detect_cardinal_permutation(
    src_frame: RigidFrame3,
    dst_frame: RigidFrame3,
) -> Option<CardinalPermutation> {
    const CARDINAL_TOL: f64 = 1e-6;
    let cols = [
        dst_frame.world_to_local_dir(src_frame.x_axis),
        dst_frame.world_to_local_dir(src_frame.y_axis),
        dst_frame.world_to_local_dir(src_frame.z_axis),
    ];

    let mut used_dst = [false; 3];
    let mut src_for_dst = [Axis3::X; 3];
    let mut dst_sign = [1_i8; 3];

    for (c_idx, col) in cols.iter().enumerate() {
        let comps = [col.x, col.y, col.z];
        let mut best_d = 0usize;
        let mut best_abs = comps[0].abs();
        for (d, &v) in comps.iter().enumerate().skip(1) {
            if v.abs() > best_abs {
                best_abs = v.abs();
                best_d = d;
            }
        }
        if (best_abs - 1.0).abs() > CARDINAL_TOL {
            return None;
        }
        for (d, &v) in comps.iter().enumerate() {
            if d != best_d && v.abs() > CARDINAL_TOL {
                return None;
            }
        }
        if used_dst[best_d] {
            return None;
        }
        used_dst[best_d] = true;
        src_for_dst[best_d] = Axis3::ALL[c_idx];
        dst_sign[best_d] = if comps[best_d] >= 0.0 { 1 } else { -1 };
    }

    let t_dst = dst_frame.world_to_local_point(src_frame.origin_mm);
    Some(CardinalPermutation {
        src_for_dst,
        dst_sign,
        t_dst,
    })
}

impl TriDexelStock {
    /// Transfers this machined volumetric solid from `src_frame` (Setup `i` WCS) into `dst_frame`
    /// (Setup `j` WCS).
    ///
    /// - **Fast exact path** (`90°`/`180°` flips and 6-sided setups): when `dst_frame^{-1} * src_frame`
    ///   is a signed cardinal axis permutation, permutes `(X, Y, Z)` dexel planes directly, reverses
    ///   ray ordering and negates/shifts interval endpoints where axis sign is `-1`, preserving exact
    ///   volume (`|V_dst - V_src| < 1e-3 mm³`) and `op_idx` provenance tags with zero resampling blur.
    /// - **General oblique path**: extracts the watertight `TriDexel` surface mesh, transforms
    ///   vertices into `dst_frame` WCS, builds a `FlatBvh` (`TriMesh3D`), and re-voxelizes onto a
    ///   fresh [`TriDexelStock`] at `resolution`.
    pub fn transform_to_frame(
        &self,
        src_frame: RigidFrame3,
        dst_frame: RigidFrame3,
        resolution: SimResolution,
    ) -> Result<Self, SimError> {
        if !src_frame.is_orthonormal_right_handed(1e-4)
            || !dst_frame.is_orthonormal_right_handed(1e-4)
        {
            return Err(SimError::InvalidStockBounds);
        }
        if let Some(perm) = detect_cardinal_permutation(src_frame, dst_frame) {
            return Ok(self.transform_cardinal_exact(perm));
        }
        self.transform_oblique_bvh(src_frame, dst_frame, resolution)
    }

    /// Zero-blur, zero-volume-drift rigid transfer for signed cardinal axis permutations.
    fn transform_cardinal_exact(&self, perm: CardinalPermutation) -> Self {
        let src_aabb = self.aabb_wcs();

        let mut dst_dims = [0usize; 3];
        let mut dst_pitch = DVec3::ZERO;
        let mut dst_origin = DVec3::ZERO;

        for d_idx in 0..3 {
            let src_ax = perm.src_for_dst[d_idx];
            let s = perm.dst_sign[d_idx];
            let t_d = Axis3::ALL[d_idx].get(perm.t_dst);
            dst_dims[d_idx] = self.dim(src_ax);
            let pitch = self.pitch(src_ax);
            let orig = if s > 0 {
                src_ax.get(src_aabb.min) + t_d
            } else {
                -src_ax.get(src_aabb.max) + t_d
            };
            match d_idx {
                0 => {
                    dst_pitch.x = pitch;
                    dst_origin.x = orig;
                }
                1 => {
                    dst_pitch.y = pitch;
                    dst_origin.y = orig;
                }
                _ => {
                    dst_pitch.z = pitch;
                    dst_origin.z = orig;
                }
            }
        }

        let build_plane = |dst_ax: Axis3| -> DexelPlane {
            let du_ax = dst_ax.u_axis();
            let dv_ax = dst_ax.v_axis();

            let d_idx = dst_ax.index();
            let du_idx = du_ax.index();
            let dv_idx = dv_ax.index();

            let src_w_ax = perm.src_for_dst[d_idx];
            let src_u_for_du = perm.src_for_dst[du_idx];
            let src_v_for_dv = perm.src_for_dst[dv_idx];

            let w_sign = perm.dst_sign[d_idx];
            let u_sign = perm.dst_sign[du_idx];
            let v_sign = perm.dst_sign[dv_idx];
            let t_w = dst_ax.get(perm.t_dst);

            let nu = dst_dims[du_idx];
            let nv = dst_dims[dv_idx];
            let mut dst_plane = DexelPlane::new(
                dst_ax,
                nu,
                nv,
                du_ax.get(dst_origin),
                dv_ax.get(dst_origin),
                dst_ax.get(dst_origin),
                du_ax.get(dst_pitch),
                dv_ax.get(dst_pitch),
            );

            let src_plane = self.plane(src_w_ax);
            let src_su_ax = src_w_ax.u_axis();
            let swap_uv = src_su_ax != src_u_for_du;
            debug_assert!(
                (!swap_uv && src_w_ax.v_axis() == src_v_for_dv)
                    || (swap_uv && src_su_ax == src_v_for_dv && src_w_ax.v_axis() == src_u_for_du)
            );

            let src_dim_du = self.dim(src_u_for_du);
            let src_dim_dv = self.dim(src_v_for_dv);
            let dst_origin_w = dst_plane.origin_w_mm;
            let row_slot_stride = nu * INLINE_DEXEL_SLOTS;

            let overflow_entries: Vec<(u32, Vec<DexelSpan>)> = dst_plane
                .slots
                .par_chunks_mut(row_slot_stride)
                .zip(dst_plane.counts.par_chunks_mut(nu))
                .enumerate()
                .map(|(iv_dst, (row_slots, row_counts))| {
                    let mut row_ov: Vec<(u32, Vec<DexelSpan>)> = Vec::new();
                    let mut mapped: Vec<DexelSpan> = Vec::with_capacity(INLINE_DEXEL_SLOTS);
                    let src_idx_dv = if v_sign > 0 {
                        iv_dst
                    } else {
                        src_dim_dv - 1 - iv_dst
                    };
                    let row_base = iv_dst * nu;

                    for (iu_dst, count_slot) in row_counts.iter_mut().enumerate().take(nu) {
                        let src_idx_du = if u_sign > 0 {
                            iu_dst
                        } else {
                            src_dim_du - 1 - iu_dst
                        };
                        let (iu_src, iv_src) = if swap_uv {
                            (src_idx_dv, src_idx_du)
                        } else {
                            (src_idx_du, src_idx_dv)
                        };

                        let src_spans = src_plane.ray_spans(iu_src, iv_src);
                        if src_spans.is_empty() {
                            *count_slot = 0;
                            continue;
                        }

                        mapped.clear();
                        if w_sign > 0 {
                            for &s in src_spans.iter() {
                                mapped.push(DexelSpan::new(
                                    s.min_mm + t_w,
                                    s.max_mm + t_w,
                                    s.min_op_idx,
                                    s.max_op_idx,
                                ));
                            }
                        } else {
                            for &s in src_spans.iter().rev() {
                                mapped.push(DexelSpan::new(
                                    -s.max_mm + t_w,
                                    -s.min_mm + t_w,
                                    s.max_op_idx,
                                    s.min_op_idx,
                                ));
                            }
                        }

                        let base = iu_dst * INLINE_DEXEL_SLOTS;
                        if mapped.len() <= INLINE_DEXEL_SLOTS {
                            let mut written = 0usize;
                            for &s in &mapped {
                                let flat = FlatDexelSpan::from_wcs(s, dst_origin_w);
                                if flat.max_local > flat.min_local {
                                    row_slots[base + written] = flat;
                                    written += 1;
                                }
                            }
                            for k in written..INLINE_DEXEL_SLOTS {
                                row_slots[base + k] = FlatDexelSpan::ZERO;
                            }
                            *count_slot = written as u32;
                        } else {
                            for k in 0..INLINE_DEXEL_SLOTS {
                                row_slots[base + k] =
                                    FlatDexelSpan::from_wcs(mapped[k], dst_origin_w);
                            }
                            *count_slot = mapped.len() as u32;
                            row_ov.push(((row_base + iu_dst) as u32, mapped.clone()));
                        }
                    }
                    row_ov
                })
                .reduce(Vec::new, |mut a, mut b| {
                    a.append(&mut b);
                    a
                });

            dst_plane.overflow = overflow_entries.into_iter().collect::<BTreeMap<_, _>>();
            dst_plane
        };

        Self {
            dims: dst_dims,
            origin_wcs: dst_origin,
            pitch_mm: dst_pitch,
            planes: [
                build_plane(Axis3::X),
                build_plane(Axis3::Y),
                build_plane(Axis3::Z),
            ],
            rotary: None,
        }
    }

    /// General oblique rigid-frame transfer: extracts watertight surface mesh, transforms vertices
    /// into `dst_frame` WCS, builds a `FlatBvh` (`TriMesh3D`), and re-voxelizes via parallel BVH
    /// raycasting across all 3 orthogonal dexel planes.
    fn transform_oblique_bvh(
        &self,
        src_frame: RigidFrame3,
        dst_frame: RigidFrame3,
        resolution: SimResolution,
    ) -> Result<Self, SimError> {
        let map_pt = |p_src: DVec3| -> DVec3 {
            dst_frame.world_to_local_point(src_frame.local_to_world_point(p_src))
        };

        let mesh = tessellate_tridexel_stock_mesh(self, false);
        if mesh.indices.len() < 3 || mesh.vertices.is_empty() {
            let src_b = self.aabb_wcs();
            let mut dst_b = Aabb3::EMPTY;
            for sx in [src_b.min.x, src_b.max.x] {
                for sy in [src_b.min.y, src_b.max.y] {
                    for sz in [src_b.min.z, src_b.max.z] {
                        dst_b.extend_point(map_pt(DVec3::new(sx, sy, sz)));
                    }
                }
            }
            return Self::empty_for_bounds(dst_b, resolution);
        }

        let transformed_verts: Vec<DVec3> = mesh
            .vertices
            .par_iter()
            .map(|v| {
                map_pt(DVec3::new(
                    f64::from(v.pos_mm[0]),
                    f64::from(v.pos_mm[1]),
                    f64::from(v.pos_mm[2]),
                ))
            })
            .collect();

        let mut raw_tris: Vec<[DVec3; 3]> = Vec::with_capacity(mesh.indices.len() / 3);
        let mut dst_aabb = Aabb3::EMPTY;
        for tri in mesh.indices.as_chunks::<3>().0 {
            let v0 = transformed_verts[tri[0] as usize];
            let v1 = transformed_verts[tri[1] as usize];
            let v2 = transformed_verts[tri[2] as usize];
            dst_aabb.extend_point(v0);
            dst_aabb.extend_point(v1);
            dst_aabb.extend_point(v2);
            raw_tris.push([v0, v1, v2]);
        }

        if raw_tris.is_empty() || !dst_aabb.is_valid() {
            return Err(SimError::InvalidStockBounds);
        }

        let bvh_mesh = TriMesh3D::from_raw_triangles(&raw_tris);
        let mut dst = Self::empty_for_bounds(dst_aabb, resolution)?;

        const JITTER_U: f64 = 1.371_91e-7;
        const JITTER_V: f64 = 2.718_28e-7;

        for axis in Axis3::ALL {
            let plane = dst.plane_mut(axis);
            let nu = plane.nu;
            let origin_u = plane.origin_u_mm;
            let origin_v = plane.origin_v_mm;
            let origin_w = plane.origin_w_mm;
            let du = plane.du_mm;
            let dv = plane.dv_mm;
            let start_w = origin_w - 1.0;
            let ray_dir = axis.unit_vec();
            let row_slot_stride = nu * INLINE_DEXEL_SLOTS;

            let overflow_entries: Vec<(u32, Vec<DexelSpan>)> = plane
                .slots
                .par_chunks_mut(row_slot_stride)
                .zip(plane.counts.par_chunks_mut(nu))
                .enumerate()
                .map(|(iv, (row_slots, row_counts))| {
                    let mut row_ov: Vec<(u32, Vec<DexelSpan>)> = Vec::new();
                    let mut hits = Vec::with_capacity(8);
                    let mut clean: Vec<(f64, bool)> = Vec::with_capacity(8);
                    let mut spans: Vec<DexelSpan> = Vec::with_capacity(INLINE_DEXEL_SLOTS);
                    let rv = origin_v + (iv as f64 + 0.5) * dv + JITTER_V;
                    let row_base = iv * nu;

                    for (iu, count_slot) in row_counts.iter_mut().enumerate().take(nu) {
                        let ru = origin_u + (iu as f64 + 0.5) * du + JITTER_U;
                        let ray_origin = axis.from_uvw(ru, rv, start_w);
                        bvh_mesh.raycast_all_hits(ray_origin, ray_dir, f64::INFINITY, &mut hits);
                        if hits.len() < 2 {
                            *count_slot = 0;
                            continue;
                        }

                        clean.clear();
                        for h in &hits {
                            let w = start_w + h.t;
                            let entering = h.normal.dot(ray_dir) < 0.0;
                            if let Some(&(prev_w, prev_ent)) = clean.last()
                                && (w - prev_w).abs() <= 1e-6
                            {
                                if entering == prev_ent {
                                    // Shared edge between two identically-oriented triangles: keep one.
                                    continue;
                                } else {
                                    // Zero-thickness silhouette graze: cancel the pair.
                                    clean.pop();
                                    continue;
                                }
                            }
                            clean.push((w, entering));
                        }

                        spans.clear();
                        let pairs = clean.len() / 2;
                        for p in 0..pairs {
                            let a = clean[2 * p].0;
                            let b = clean[2 * p + 1].0;
                            if b - a > MIN_DEXEL_SPAN_MM {
                                spans.push(DexelSpan::new(a, b, UNCUT_OP_IDX, UNCUT_OP_IDX));
                            }
                        }

                        let base = iu * INLINE_DEXEL_SLOTS;
                        if spans.len() <= INLINE_DEXEL_SLOTS {
                            let mut written = 0usize;
                            for &s in &spans {
                                let flat = FlatDexelSpan::from_wcs(s, origin_w);
                                if flat.max_local > flat.min_local {
                                    row_slots[base + written] = flat;
                                    written += 1;
                                }
                            }
                            for k in written..INLINE_DEXEL_SLOTS {
                                row_slots[base + k] = FlatDexelSpan::ZERO;
                            }
                            *count_slot = written as u32;
                        } else {
                            for k in 0..INLINE_DEXEL_SLOTS {
                                row_slots[base + k] = FlatDexelSpan::from_wcs(spans[k], origin_w);
                            }
                            *count_slot = spans.len() as u32;
                            row_ov.push(((row_base + iu) as u32, spans.clone()));
                        }
                    }
                    row_ov
                })
                .reduce(Vec::new, |mut a, mut b| {
                    a.append(&mut b);
                    a
                });

            plane.overflow = overflow_entries.into_iter().collect::<BTreeMap<_, _>>();
        }

        Ok(dst)
    }

    /// Returns `true` if any Z-ray contains at least `min_thickness_mm` of solid material inside
    /// the WCS Z slab `[z_lo, z_hi]`.
    pub fn has_material_in_z_slab(&self, z_lo: f64, z_hi: f64, min_thickness_mm: f64) -> bool {
        if !z_lo.is_finite() || !z_hi.is_finite() || z_hi <= z_lo + 1e-9 {
            return false;
        }
        let aabb = self.aabb_wcs();
        if z_hi <= aabb.min.z + 1e-9 || z_lo >= aabb.max.z - 1e-9 {
            return false;
        }
        let thresh = min_thickness_mm.max(1e-6);
        let z_plane = self.z_plane();
        let total = z_plane.ray_count();
        for ray_idx in 0..total {
            if z_plane.counts[ray_idx] == 0 {
                continue;
            }
            let spans = z_plane.ray_spans_by_index(ray_idx);
            if query_overlap_length(&spans, z_lo, z_hi) >= thresh {
                return true;
            }
        }
        false
    }

    /// Returns the highest solid WCS Z coordinate across all Z-rays overlapping `aabb2`
    /// (`None` if no solid material remains inside `aabb2`).
    pub fn max_stock_z_in_xy_aabb(&self, aabb2: Aabb2) -> Option<f64> {
        if !aabb2.is_valid() {
            return None;
        }
        let z_plane = self.z_plane();
        let (i0, i1) = z_plane.index_span_u(aabb2.min.x, aabb2.max.x)?;
        let (j0, j1) = z_plane.index_span_v(aabb2.min.y, aabb2.max.y)?;

        let mut best: Option<f64> = None;
        for j in j0..=j1 {
            let row = j * z_plane.nu;
            for i in i0..=i1 {
                let ray_idx = row + i;
                if z_plane.counts[ray_idx] == 0 {
                    continue;
                }
                let spans = z_plane.ray_spans_by_index(ray_idx);
                if let Some(last) = spans.last()
                    && best.is_none_or(|b| last.max_mm > b)
                {
                    best = Some(last.max_mm);
                }
            }
        }
        best
    }

    /// Extracts closed 2D contour polygons (`CCW` outer loops, `CW` inner hole loops) covering all
    /// XY cells that still contain solid material inside the WCS Z slab `[z_lo, z_hi]`, optionally
    /// inflated outward by Euclidean radius `cell_pad_mm`.
    ///
    /// Output loops are non-self-intersecting and directly consumable by
    /// `chip_geom2d::contours_to_region_set` / `RegionSet2D` boolean intersection for rest-aware
    /// Op-2 roughing and facing.
    pub fn extract_remaining_material_regions_in_z_slab(
        &self,
        z_lo: f64,
        z_hi: f64,
        cell_pad_mm: f64,
    ) -> Vec<ArcPolyline2D> {
        if !z_lo.is_finite() || !z_hi.is_finite() || z_hi <= z_lo + 1e-9 {
            return Vec::new();
        }
        let z_plane = self.z_plane();
        let nx = z_plane.nu;
        let ny = z_plane.nv;
        if nx == 0 || ny == 0 {
            return Vec::new();
        }
        let dx = z_plane.du_mm;
        let dy = z_plane.dv_mm;
        let pad_mm = if cell_pad_mm.is_finite() && cell_pad_mm > 1e-6 {
            cell_pad_mm
        } else {
            0.0
        };
        let eff_pad = if pad_mm > 0.0 {
            pad_mm + 0.5 * dx.max(dy)
        } else {
            0.0
        };
        let pad_x = if eff_pad > 0.0 {
            (eff_pad / dx).ceil() as usize + 1
        } else {
            0
        };
        let pad_y = if eff_pad > 0.0 {
            (eff_pad / dy).ceil() as usize + 1
        } else {
            0
        };

        let gnx = nx + 2 * pad_x;
        let gny = ny + 2 * pad_y;
        let mut occ = vec![false; gnx * gny];
        let mut any_solid = false;

        for j in 0..ny {
            let src_row = j * nx;
            let dst_row = (j + pad_y) * gnx + pad_x;
            for i in 0..nx {
                let ray_idx = src_row + i;
                if z_plane.counts[ray_idx] == 0 {
                    continue;
                }
                let spans = z_plane.ray_spans_by_index(ray_idx);
                if query_overlap_length(&spans, z_lo, z_hi) > 1e-4 {
                    occ[dst_row + i] = true;
                    any_solid = true;
                }
            }
        }

        if !any_solid {
            return Vec::new();
        }

        if eff_pad > 0.0 {
            dilate_mask_euclidean(&mut occ, gnx, gny, dx, dy, eff_pad);
        }

        let origin_x = z_plane.origin_u_mm - (pad_x as f64) * dx;
        let origin_y = z_plane.origin_v_mm - (pad_y as f64) * dy;
        let simplify_tol = if eff_pad > 0.0 {
            0.22 * dx.min(dy)
        } else {
            0.0
        };

        trace_binary_grid_contours(&occ, gnx, gny, origin_x, origin_y, dx, dy, simplify_tol)
    }
}

/// Exact `O(W * H)` separable squared Euclidean distance transform (Felzenszwalb & Huttenlocher)
/// for smooth circular morphological dilation of a 2D occupancy grid.
fn dilate_mask_euclidean(occ: &mut [bool], w: usize, h: usize, dx: f64, dy: f64, radius_mm: f64) {
    const INF: f64 = 1e30;
    let r2 = radius_mm * radius_mm;

    // Pass 1: exact 1D vertical distance along each column (in mm, then squared).
    let mut g_sq = vec![INF; w * h];
    for x in 0..w {
        let mut d = INF;
        for y in 0..h {
            let idx = y * w + x;
            if occ[idx] {
                d = 0.0;
            } else if d < INF {
                d += dy;
            }
            g_sq[idx] = d;
        }
        let mut d = INF;
        for y in (0..h).rev() {
            let idx = y * w + x;
            if occ[idx] {
                d = 0.0;
            } else if d < INF {
                d += dy;
            }
            let best = g_sq[idx].min(d);
            g_sq[idx] = if best < INF * 0.5 { best * best } else { INF };
        }
    }

    // Pass 2: exact 1D lower envelope of parabolas along each row.
    let mut v = vec![0usize; w];
    let mut z = vec![0.0_f64; w + 1];
    let dx2 = dx * dx;

    for y in 0..h {
        let row = y * w;
        let mut k = 0usize;
        let mut has_any = false;
        for q in 0..w {
            let fq = g_sq[row + q];
            if fq >= INF * 0.5 {
                continue;
            }
            if !has_any {
                v[0] = q;
                z[0] = f64::NEG_INFINITY;
                z[1] = f64::INFINITY;
                k = 0;
                has_any = true;
                continue;
            }
            let qf = q as f64;
            loop {
                let vk = v[k];
                let vkf = vk as f64;
                let fvk = g_sq[row + vk];
                let s = ((fq + qf * qf * dx2) - (fvk + vkf * vkf * dx2)) / (2.0 * dx2 * (qf - vkf));
                if s <= z[k] {
                    if k == 0 {
                        v[0] = q;
                        z[0] = f64::NEG_INFINITY;
                        z[1] = f64::INFINITY;
                        break;
                    }
                    k -= 1;
                } else {
                    k += 1;
                    v[k] = q;
                    z[k] = s;
                    z[k + 1] = f64::INFINITY;
                    break;
                }
            }
        }
        if !has_any {
            continue;
        }
        let mut cur = 0usize;
        for q in 0..w {
            let qf = q as f64;
            while z[cur + 1] < qf {
                cur += 1;
            }
            let vk = v[cur];
            let ddx = (q as f64 - vk as f64) * dx;
            let dist_sq = ddx * ddx + g_sq[row + vk];
            if dist_sq <= r2 {
                occ[row + q] = true;
            }
        }
    }
}

/// Directed grid step: `0 = +X`, `1 = +Y`, `2 = -X`, `3 = -Y`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct DirectedEdge {
    u: usize,
    v: usize,
    dir: u8,
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

/// Traces all closed boundary loops around `occ` (`w × h`) on the dual corner grid `(w+1) × (h+1)`.
///
/// Solid lies strictly to the left of every directed edge, so outer boundaries are strictly `CCW`
/// and interior holes are strictly `CW`, with saddle corners resolved deterministically via
/// right-turn priority (guaranteeing non-self-intersecting simple loops).
#[allow(clippy::too_many_arguments)]
fn trace_binary_grid_contours(
    occ: &[bool],
    w: usize,
    h: usize,
    origin_x: f64,
    origin_y: f64,
    dx: f64,
    dy: f64,
    simplify_tol: f64,
) -> Vec<ArcPolyline2D> {
    let gw = w + 1;
    let gh = h + 1;
    // Bitmask of unused outgoing edges per dual grid vertex `(u, v)`: bits 0..4 for dirs 0..4
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
            // Bottom edge `(i, j) -> (i+1, j)` (dir 0: +X) if south neighbor empty
            if !is_solid(ii, jj - 1) {
                out_mask[j * gw + i] |= 1 << 0;
                edge_count += 1;
            }
            // Right edge `(i+1, j) -> (i+1, j+1)` (dir 1: +Y) if east neighbor empty
            if !is_solid(ii + 1, jj) {
                out_mask[j * gw + (i + 1)] |= 1 << 1;
                edge_count += 1;
            }
            // Top edge `(i+1, j+1) -> (i, j+1)` (dir 2: -X) if north neighbor empty
            if !is_solid(ii, jj + 1) {
                out_mask[(j + 1) * gw + (i + 1)] |= 1 << 2;
                edge_count += 1;
            }
            // Left edge `(i, j+1) -> (i, j)` (dir 3: -Y) if west neighbor empty
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
                let start = DirectedEdge {
                    u: su,
                    v: sv,
                    dir: start_dir,
                };

                pts_buf.clear();
                let mut cur_u = start.u;
                let mut cur_v = start.v;
                let mut cur_dir = start.dir;
                let mut prev_dir: Option<u8> = None;

                // Safety bound prevents any infinite loop even on malformed input.
                for _ in 0..=edge_count {
                    let vidx = cur_v * gw + cur_u;
                    if (out_mask[vidx] & (1 << cur_dir)) == 0 {
                        break;
                    }
                    out_mask[vidx] &= !(1 << cur_dir);

                    // Emit vertex only when direction changes (lossless collinear merge!).
                    if prev_dir != Some(cur_dir) {
                        pts_buf.push(DVec2::new(
                            origin_x + (cur_u as f64) * dx,
                            origin_y + (cur_v as f64) * dy,
                        ));
                    }
                    prev_dir = Some(cur_dir);

                    let (nu, nv) = edge_dest(cur_u, cur_v, cur_dir);
                    if nu == start.u && nv == start.v {
                        break;
                    }
                    let next_mask = out_mask[nv * gw + nu];
                    if next_mask == 0 {
                        break;
                    }
                    // Turn priority: right turn `(cur_dir + 3) % 4`, straight `cur_dir`, left `(cur_dir + 1) % 4`.
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

                // Collapse collinear seam if first and last edges share the same direction
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

                let final_pts = if simplify_tol > 1e-6 && pts_buf.len() > 12 {
                    simplify_closed_polygon_rdp(&pts_buf, simplify_tol)
                } else {
                    pts_buf.clone()
                };

                if final_pts.len() >= 3 {
                    let mut segs = Vec::with_capacity(final_pts.len());
                    for k in 0..final_pts.len() {
                        let a = final_pts[k];
                        let b = final_pts[(k + 1) % final_pts.len()];
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

/// Lightweight closed-loop Douglas-Peucker simplification preserving corner fidelity within `tol`.
fn simplify_closed_polygon_rdp(pts: &[DVec2], tol: f64) -> Vec<DVec2> {
    let n = pts.len();
    if n <= 4 {
        return pts.to_vec();
    }
    // Split closed loop at the pair of farthest vertices (index 0 and farthest from 0)
    let p0 = pts[0];
    let mut split = 1usize;
    let mut best_d2 = -1.0_f64;
    for (i, &p) in pts.iter().enumerate().skip(1) {
        let d2 = p.distance_squared(p0);
        if d2 > best_d2 {
            best_d2 = d2;
            split = i;
        }
    }
    if split == 0 || split >= n - 1 {
        return pts.to_vec();
    }

    let mut half_a: Vec<DVec2> = pts[..=split].to_vec();
    let mut half_b: Vec<DVec2> = Vec::with_capacity(n - split + 1);
    half_b.extend_from_slice(&pts[split..]);
    half_b.push(pts[0]);

    let tol2 = tol * tol;
    let simp_a = rdp_open(&mut half_a, tol2);
    let simp_b = rdp_open(&mut half_b, tol2);

    let mut out = Vec::with_capacity(simp_a.len() + simp_b.len());
    out.extend_from_slice(&simp_a);
    if simp_b.len() > 2 {
        out.extend_from_slice(&simp_b[1..simp_b.len() - 1]);
    }
    if out.len() >= 3 { out } else { pts.to_vec() }
}

fn rdp_open(pts: &mut [DVec2], tol2: f64) -> Vec<DVec2> {
    let n = pts.len();
    if n <= 2 {
        return pts.to_vec();
    }
    let mut keep = vec![false; n];
    keep[0] = true;
    keep[n - 1] = true;
    let mut stack = vec![(0usize, n - 1)];

    while let Some((lo, hi)) = stack.pop() {
        if hi <= lo + 1 {
            continue;
        }
        let a = pts[lo];
        let b = pts[hi];
        let ab = b - a;
        let ab_len2 = ab.length_squared();
        let mut max_d2 = -1.0_f64;
        let mut max_i = lo;
        for (i, &p) in pts.iter().enumerate().take(hi).skip(lo + 1) {
            let d2 = if ab_len2 <= 1e-18 {
                p.distance_squared(a)
            } else {
                let t = ((p - a).dot(ab) / ab_len2).clamp(0.0, 1.0);
                p.distance_squared(a + ab * t)
            };
            if d2 > max_d2 {
                max_d2 = d2;
                max_i = i;
            }
        }
        if max_d2 > tol2 {
            keep[max_i] = true;
            stack.push((lo, max_i));
            stack.push((max_i, hi));
        }
    }

    pts.iter()
        .enumerate()
        .filter_map(|(i, &p)| keep[i].then_some(p))
        .collect()
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use chip_math::{Aabb3, DVec3, RigidFrame3};
    use chip_tools::{CutterGeometry, default_starter_tool_library};

    use crate::{
        SweepChord, SweepMotion, SweepToolSpec, TriDexelStock,
        types::{Axis3, RadialCutterProfile, SimResolution, StockShapeSpec, StockVerifyTolerances},
    };

    fn rotated_x_frame(angle_deg: f64) -> RigidFrame3 {
        let a = (-angle_deg).to_radians();
        let (s, c) = a.sin_cos();
        RigidFrame3 {
            origin_mm: DVec3::ZERO,
            x_axis: DVec3::X,
            y_axis: DVec3::new(0.0, c, s),
            z_axis: DVec3::new(0.0, -s, c),
        }
    }

    #[test]
    fn rotary_frame_transfer_preserves_volume_and_overhangs_for_cardinal_and_arbitrary_angles() {
        // Ø40 mm x 60 mm cylindrical stock along X, with a deep slot machined on the +Z/+Y quadrant
        // creating an asymmetric solid with vertical overhangs when rotated around X.
        let spec = StockShapeSpec::Cylinder {
            axis: Axis3::X,
            center_wcs: DVec3::ZERO,
            radius_mm: 20.0,
            min_axial_mm: -30.0,
            max_axial_mm: 30.0,
            aabb_wcs: Aabb3::new(
                DVec3::new(-30.0, -20.0, -20.0),
                DVec3::new(30.0, 20.0, 20.0),
            ),
        };
        let mut stock = TriDexelStock::from_stock_spec(&spec, SimResolution::Standard).unwrap();

        let mut tool = default_starter_tool_library().tools[0].clone();
        tool.geometry = CutterGeometry::FlatEndmill {
            diameter_mm: 8.0,
            flute_length_mm: 25.0,
            overall_length_mm: 50.0,
            shank_diameter_mm: 8.0,
            flutes: 3,
        };
        let tool_spec = SweepToolSpec {
            profile: RadialCutterProfile::from_tool(&tool),
            is_flat_endmill: true,
        };
        let chord = SweepChord {
            motion: SweepMotion::Linear {
                p0: DVec3::new(-22.0, 12.0, 4.0),
                p1: DVec3::new(22.0, 12.0, 4.0),
                rotary_deg_0: 0.0,
                rotary_deg_1: 0.0,
            },
            op_idx: 0,
            is_rapid: false,
            profile_idx: 0,
            verify_tol: StockVerifyTolerances::default(),
        };
        let _ = stock.sweep_chords(&[chord], &[tool_spec], None, false);

        let src_vol = stock.compute_volume_mm3();
        assert!(src_vol > 10_000.0);

        let src_frame = RigidFrame3::IDENTITY;

        // 1. Cardinal rotary angles (90°, 180°, 270°): exact zero-blur permutation fast path!
        for angle_deg in [90.0_f64, 180.0, 270.0] {
            let dst_frame = rotated_x_frame(angle_deg);
            let dst = stock
                .transform_to_frame(src_frame, dst_frame, SimResolution::Standard)
                .unwrap();
            let vol_err_pct = ((dst.compute_volume_mm3() - src_vol).abs() / src_vol) * 100.0;
            assert!(
                vol_err_pct < 0.01,
                "Cardinal {angle_deg}° transfer volume error {vol_err_pct:.4}% exceeded 0.01%"
            );
            // Verify solid interior point on opposite side `(0, -12, 0)` transferred cleanly!
            let mapped = dst_frame.world_to_local_point(DVec3::new(0.0, -12.0, 0.0));
            assert!(
                dst.has_material_in_z_slab(mapped.z - 1.5, mapped.z + 1.5, 0.2),
                "Cardinal {angle_deg}° transfer lost solid material at {mapped:?}"
            );
        }

        // 2. Arbitrary oblique rotary angles (60°, 120°, 240°): watertight BVH re-voxelization!
        for angle_deg in [60.0_f64, 120.0, 240.0] {
            let dst_frame = rotated_x_frame(angle_deg);
            let dst = stock
                .transform_to_frame(src_frame, dst_frame, SimResolution::Standard)
                .unwrap();
            let vol_err_pct = ((dst.compute_volume_mm3() - src_vol).abs() / src_vol) * 100.0;
            assert!(
                vol_err_pct < 0.5,
                "Oblique {angle_deg}° transfer volume error {vol_err_pct:.4}% exceeded 0.5%"
            );
            // Verify uncut lobe at `(0, -12, 8)` still exists in rotated frame
            let mapped_solid = dst_frame.world_to_local_point(DVec3::new(0.0, -12.0, 8.0));
            let regs = dst.extract_remaining_material_regions_in_z_slab(
                mapped_solid.z - 1.0,
                mapped_solid.z + 1.0,
                0.0,
            );
            assert!(
                !regs.is_empty(),
                "Oblique {angle_deg}° transfer lost solid overhang region around {mapped_solid:?}"
            );
        }
    }
}
