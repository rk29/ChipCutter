use std::{collections::BTreeMap, ops::Deref};

use chip_math::{Aabb3, DVec2, DVec3};
use rayon::prelude::*;

use crate::types::{
    AnalyticHorizFace, Axis3, RotaryStockSpec, SimError, SimResolution, StockShapeSpec,
    UNCUT_OP_IDX,
};

/// Minimum retained dexel interval thickness (`mm`) below which a sliver is discarded.
pub(crate) const MIN_DEXEL_SPAN_MM: f64 = 1e-6;

/// Number of inline [`FlatDexelSpan`] slots per ray in [`DexelPlane::slots`] (covers >99.9% of
/// machined rays inline without any heap side-table lookup; rays with `> 4` spans transparently
/// spill into `DexelPlane::overflow`).
pub const INLINE_DEXEL_SLOTS: usize = 4;

const UNCUT_OP_TAG_U16: u16 = u16::MAX;

/// Packs `(min_op_idx, max_op_idx)` (`UNCUT_OP_IDX` = `u32::MAX`) into a single WGSL `u32`.
#[inline]
pub const fn pack_span_ops(min_op: u32, max_op: u32) -> u32 {
    let lo = if min_op == UNCUT_OP_IDX || min_op >= UNCUT_OP_TAG_U16 as u32 {
        UNCUT_OP_TAG_U16 as u32
    } else {
        min_op
    };
    let hi = if max_op == UNCUT_OP_IDX || max_op >= UNCUT_OP_TAG_U16 as u32 {
        UNCUT_OP_TAG_U16 as u32
    } else {
        max_op
    };
    lo | (hi << 16)
}

/// Unpacks `min_op_idx` from a packed [`FlatDexelSpan::op_packed`] tag.
#[inline]
pub const fn unpack_min_op(op_packed: u32) -> u32 {
    let lo = (op_packed & 0xFFFF) as u16;
    if lo == UNCUT_OP_TAG_U16 {
        UNCUT_OP_IDX
    } else {
        lo as u32
    }
}

/// Unpacks `max_op_idx` from a packed [`FlatDexelSpan::op_packed`] tag.
#[inline]
pub const fn unpack_max_op(op_packed: u32) -> u32 {
    let hi = (op_packed >> 16) as u16;
    if hi == UNCUT_OP_TAG_U16 {
        UNCUT_OP_IDX
    } else {
        hi as u32
    }
}

/// 16-byte GPU-ready storage-buffer span entry (`#[repr(C)] bytemuck::Pod + Zeroable`) storing
/// stock-local `f32` interval endpoints relative to [`DexelPlane::origin_w_mm`].
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct FlatDexelSpan {
    pub min_local: f32,
    pub max_local: f32,
    pub op_packed: u32,
    pub _pad: u32,
}

impl FlatDexelSpan {
    pub const ZERO: Self = Self {
        min_local: 0.0,
        max_local: 0.0,
        op_packed: 0,
        _pad: 0,
    };

    #[inline]
    pub fn from_wcs(span: DexelSpan, origin_w_mm: f64) -> Self {
        Self {
            min_local: (span.min_mm - origin_w_mm) as f32,
            max_local: (span.max_mm - origin_w_mm) as f32,
            op_packed: pack_span_ops(span.min_op_idx, span.max_op_idx),
            _pad: 0,
        }
    }

    #[inline]
    pub fn to_wcs(self, origin_w_mm: f64) -> DexelSpan {
        DexelSpan {
            min_mm: origin_w_mm + f64::from(self.min_local),
            max_mm: origin_w_mm + f64::from(self.max_local),
            min_op_idx: unpack_min_op(self.op_packed),
            max_op_idx: unpack_max_op(self.op_packed),
        }
    }
}

/// A single solid 1D material interval `[min_mm, max_mm]` along an orthogonal dexel ray in WCS,
/// recording which operation last machined each endpoint (`UNCUT_OP_IDX` = raw stock).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DexelSpan {
    pub min_mm: f64,
    pub max_mm: f64,
    pub min_op_idx: u32,
    pub max_op_idx: u32,
}

impl DexelSpan {
    pub const ZERO: Self = Self {
        min_mm: 0.0,
        max_mm: 0.0,
        min_op_idx: UNCUT_OP_IDX,
        max_op_idx: UNCUT_OP_IDX,
    };

    #[inline]
    pub const fn new(min_mm: f64, max_mm: f64, min_op_idx: u32, max_op_idx: u32) -> Self {
        Self {
            min_mm,
            max_mm,
            min_op_idx,
            max_op_idx,
        }
    }

    #[inline]
    pub fn len_mm(self) -> f64 {
        (self.max_mm - self.min_mm).max(0.0)
    }
}

/// Zero-allocation read view of a single ray's sorted [`DexelSpan`]s (stack-allocated for
/// `<= INLINE_DEXEL_SLOTS` spans; direct slice borrow from `overflow` when `> INLINE_DEXEL_SLOTS`).
#[derive(Debug, Clone)]
pub enum RaySpans<'a> {
    Inline {
        spans: [DexelSpan; INLINE_DEXEL_SLOTS],
        len: usize,
    },
    Overflow(&'a [DexelSpan]),
}

impl Deref for RaySpans<'_> {
    type Target = [DexelSpan];

    #[inline]
    fn deref(&self) -> &[DexelSpan] {
        match self {
            Self::Inline { spans, len } => &spans[..*len],
            Self::Overflow(slice) => slice,
        }
    }
}

/// Subtracts the cut interval `[cut_min, cut_max]` from a sorted, disjoint ray list `spans`,
/// maintaining sorted non-overlapping invariants and returning total removed length (`mm`).
///
/// Splitting an interior span into two pieces naturally represents true 3D undercuts, horizontal
/// cross-holes, and multi-interval cavities.
pub fn subtract_dexel_interval(
    spans: &mut Vec<DexelSpan>,
    cut_min: f64,
    cut_max: f64,
    op_idx: u32,
) -> f64 {
    if spans.is_empty() || cut_max <= cut_min + 1e-12 {
        return 0.0;
    }
    let mut removed = 0.0;
    let mut i = 0;
    while i < spans.len() {
        let s = spans[i];
        if cut_max <= s.min_mm {
            break;
        }
        if cut_min >= s.max_mm {
            i += 1;
            continue;
        }
        let ov_min = cut_min.max(s.min_mm);
        let ov_max = cut_max.min(s.max_mm);
        let ov = (ov_max - ov_min).max(0.0);
        if ov <= 1e-12 {
            i += 1;
            continue;
        }
        removed += ov;

        let keep_lo = ov_min - s.min_mm > MIN_DEXEL_SPAN_MM;
        let keep_hi = s.max_mm - ov_max > MIN_DEXEL_SPAN_MM;

        match (keep_lo, keep_hi) {
            (false, false) => {
                spans.remove(i);
            }
            (true, false) => {
                spans[i].max_mm = ov_min;
                spans[i].max_op_idx = op_idx;
                i += 1;
            }
            (false, true) => {
                spans[i].min_mm = ov_max;
                spans[i].min_op_idx = op_idx;
                i += 1;
            }
            (true, true) => {
                spans[i].max_mm = ov_min;
                spans[i].max_op_idx = op_idx;
                spans.insert(
                    i + 1,
                    DexelSpan::new(ov_max, s.max_mm, op_idx, s.max_op_idx),
                );
                i += 2;
            }
        }
    }
    removed
}

/// Computes the exact 1D overlap length (`mm`) between sorted disjoint `spans` and `[q_min, q_max]`.
#[inline]
pub fn query_overlap_length(spans: &[DexelSpan], q_min: f64, q_max: f64) -> f64 {
    if spans.is_empty() || q_max <= q_min {
        return 0.0;
    }
    let mut total = 0.0;
    for s in spans {
        if q_max <= s.min_mm {
            break;
        }
        if q_min >= s.max_mm {
            continue;
        }
        let lo = q_min.max(s.min_mm);
        let hi = q_max.min(s.max_mm);
        if hi > lo {
            total += hi - lo;
        }
    }
    total
}

/// Computes the exact 1D penetration depth (`mm`) of interval `[q_min, q_max]` into `target_spans`,
/// returning `(penetration_depth_mm, worst_coord_mm)` if the cut intrudes deeper than `skin_mm`
/// inside any target span.
#[inline]
pub fn query_target_gouge_depth(
    target_spans: &[DexelSpan],
    q_min: f64,
    q_max: f64,
    skin_mm: f64,
) -> Option<(f64, f64)> {
    if target_spans.is_empty() || q_max <= q_min {
        return None;
    }
    let mut best: Option<(f64, f64)> = None;
    for ts in target_spans {
        if q_max <= ts.min_mm {
            break;
        }
        if q_min >= ts.max_mm {
            continue;
        }
        let eff_min = ts.min_mm + skin_mm;
        let eff_max = ts.max_mm - skin_mm;
        if eff_max <= eff_min {
            continue;
        }
        let ov_lo = q_min.max(eff_min);
        let ov_hi = q_max.min(eff_max);
        if ov_hi > ov_lo {
            let depth = ov_hi - ov_lo;
            let coord = if q_min > eff_min && q_max >= eff_max {
                ov_lo
            } else if q_max < eff_max && q_min <= eff_min {
                ov_hi
            } else {
                0.5 * (ov_lo + ov_hi)
            };
            if best.is_none_or(|(b, _)| depth > b) {
                best = Some((depth, coord));
            }
        }
    }
    best
}

/// Exact 1D set difference length `len(a \ b)` between two sorted disjoint span lists (`O(|a|+|b|)`).
#[inline]
pub fn spans_difference_length(a: &[DexelSpan], b: &[DexelSpan]) -> f64 {
    if a.is_empty() {
        return 0.0;
    }
    if b.is_empty() {
        return a.iter().map(|s| s.len_mm()).sum();
    }
    let mut total = 0.0;
    let mut j = 0usize;
    for sa in a {
        let mut cur = sa.min_mm;
        let end = sa.max_mm;
        while j < b.len() && b[j].max_mm <= cur {
            j += 1;
        }
        let mut k = j;
        while k < b.len() && b[k].min_mm < end {
            if b[k].min_mm > cur {
                total += b[k].min_mm - cur;
            }
            cur = cur.max(b[k].max_mm);
            if cur >= end {
                break;
            }
            k += 1;
        }
        if cur < end {
            total += end - cur;
        }
    }
    total
}

/// Tests whether coordinate `coord` lies inside any solid span of `spans`.
#[inline]
pub fn spans_contain_coord(spans: &[DexelSpan], coord: f64) -> bool {
    for s in spans {
        if coord < s.min_mm {
            return false;
        }
        if coord <= s.max_mm {
            return true;
        }
    }
    false
}

/// 2D grid of parallel dexel rays along a single coordinate axis, backed by a GPU-ready
/// flat fixed-`K` inline span pool (`slots`, `counts`) plus a rare `overflow` side-table for rays
/// exceeding `INLINE_DEXEL_SLOTS` spans.
#[derive(Debug, Clone)]
pub struct DexelPlane {
    pub axis: Axis3,
    pub nu: usize,
    pub nv: usize,
    pub origin_u_mm: f64,
    pub origin_v_mm: f64,
    /// WCS origin along `axis` (`w_axis`) used as zero reference for [`FlatDexelSpan`].
    pub origin_w_mm: f64,
    pub du_mm: f64,
    pub dv_mm: f64,
    /// Contiguous flat pool of `nu * nv * INLINE_DEXEL_SLOTS` 16-byte spans (`1` heap allocation).
    pub slots: Vec<FlatDexelSpan>,
    /// Active span count per ray (`nu * nv`, `1` heap allocation). Values `<= INLINE_DEXEL_SLOTS`
    /// live inline in `slots`; values `> INLINE_DEXEL_SLOTS` live in `overflow`.
    pub counts: Vec<u32>,
    /// Rare side-table keyed by flat ray index `ray_idx = (v * nu + u) as u32` for rays with
    /// `> INLINE_DEXEL_SLOTS` spans (e.g., >4-tooth multi-form internal thread mills).
    pub overflow: BTreeMap<u32, Vec<DexelSpan>>,
}

impl DexelPlane {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        axis: Axis3,
        nu: usize,
        nv: usize,
        origin_u_mm: f64,
        origin_v_mm: f64,
        origin_w_mm: f64,
        du_mm: f64,
        dv_mm: f64,
    ) -> Self {
        let total_rays = nu * nv;
        Self {
            axis,
            nu,
            nv,
            origin_u_mm,
            origin_v_mm,
            origin_w_mm,
            du_mm,
            dv_mm,
            slots: vec![FlatDexelSpan::ZERO; total_rays * INLINE_DEXEL_SLOTS],
            counts: vec![0; total_rays],
            overflow: BTreeMap::new(),
        }
    }

    #[inline]
    pub fn ray_count(&self) -> usize {
        self.nu * self.nv
    }

    #[inline]
    pub fn overflow_ray_count(&self) -> usize {
        self.overflow.len()
    }

    /// Reads the sorted spans of ray `(u, v)` without heap allocation.
    #[inline]
    pub fn ray_spans(&self, u: usize, v: usize) -> RaySpans<'_> {
        self.ray_spans_by_index(v * self.nu + u)
    }

    /// Reads the sorted spans of flat ray index `ray_idx` without heap allocation.
    #[inline]
    pub fn ray_spans_by_index(&self, ray_idx: usize) -> RaySpans<'_> {
        let cnt = self.counts[ray_idx] as usize;
        if cnt == 0 {
            return RaySpans::Inline {
                spans: [DexelSpan::ZERO; INLINE_DEXEL_SLOTS],
                len: 0,
            };
        }
        if cnt <= INLINE_DEXEL_SLOTS {
            let base = ray_idx * INLINE_DEXEL_SLOTS;
            let mut spans = [DexelSpan::ZERO; INLINE_DEXEL_SLOTS];
            for k in 0..cnt {
                spans[k] = self.slots[base + k].to_wcs(self.origin_w_mm);
            }
            RaySpans::Inline { spans, len: cnt }
        } else if let Some(ov) = self.overflow.get(&(ray_idx as u32)) {
            RaySpans::Overflow(ov.as_slice())
        } else {
            RaySpans::Inline {
                spans: [DexelSpan::ZERO; INLINE_DEXEL_SLOTS],
                len: 0,
            }
        }
    }

    /// Replaces all spans on ray `(u, v)`, automatically storing `<= INLINE_DEXEL_SLOTS` inline
    /// and spilling `> INLINE_DEXEL_SLOTS` into `overflow` (or migrating back out of `overflow`).
    pub fn set_ray_spans(&mut self, u: usize, v: usize, spans: &[DexelSpan]) {
        let ray_idx = v * self.nu + u;
        let base = ray_idx * INLINE_DEXEL_SLOTS;
        if spans.len() <= INLINE_DEXEL_SLOTS {
            let mut written = 0usize;
            for &s in spans {
                let flat = FlatDexelSpan::from_wcs(s, self.origin_w_mm);
                if flat.max_local > flat.min_local {
                    self.slots[base + written] = flat;
                    written += 1;
                }
            }
            for k in written..INLINE_DEXEL_SLOTS {
                self.slots[base + k] = FlatDexelSpan::ZERO;
            }
            if self.counts[ray_idx] as usize > INLINE_DEXEL_SLOTS {
                self.overflow.remove(&(ray_idx as u32));
            }
            self.counts[ray_idx] = written as u32;
        } else {
            for k in 0..INLINE_DEXEL_SLOTS {
                self.slots[base + k] = FlatDexelSpan::from_wcs(spans[k], self.origin_w_mm);
            }
            self.counts[ray_idx] = spans.len() as u32;
            self.overflow.insert(ray_idx as u32, spans.to_vec());
        }
    }

    /// Sets every ray in the plane to a single identical span (`O(nu * nv)` flat fill).
    pub fn fill_all_rays(&mut self, span: DexelSpan) {
        self.overflow.clear();
        let flat = FlatDexelSpan::from_wcs(span, self.origin_w_mm);
        if flat.max_local <= flat.min_local {
            self.clear_all();
            return;
        }
        self.counts.fill(1);
        for chunk in self.slots.as_chunks_mut::<INLINE_DEXEL_SLOTS>().0 {
            chunk[0] = flat;
            for slot in &mut chunk[1..] {
                *slot = FlatDexelSpan::ZERO;
            }
        }
    }

    /// Clears a single ray `(u, v)`.
    pub fn clear_ray(&mut self, u: usize, v: usize) {
        let ray_idx = v * self.nu + u;
        if self.counts[ray_idx] as usize > INLINE_DEXEL_SLOTS {
            self.overflow.remove(&(ray_idx as u32));
        }
        self.counts[ray_idx] = 0;
        let base = ray_idx * INLINE_DEXEL_SLOTS;
        for k in 0..INLINE_DEXEL_SLOTS {
            self.slots[base + k] = FlatDexelSpan::ZERO;
        }
    }

    /// Clears all rays in the plane.
    pub fn clear_all(&mut self) {
        self.counts.fill(0);
        self.slots.fill(FlatDexelSpan::ZERO);
        self.overflow.clear();
    }

    /// Subtracts `[cut_min, cut_max]` from ray `(u, v)` and returns the removed length (`mm`).
    pub fn subtract_ray_interval(
        &mut self,
        u: usize,
        v: usize,
        cut_min: f64,
        cut_max: f64,
        op_idx: u32,
    ) -> f64 {
        let ray_idx = v * self.nu + u;
        if self.counts[ray_idx] == 0 {
            return 0.0;
        }
        let mut buf: Vec<DexelSpan> = self.ray_spans_by_index(ray_idx).to_vec();
        let rem = subtract_dexel_interval(&mut buf, cut_min, cut_max, op_idx);
        if rem > 0.0 {
            self.set_ray_spans(u, v, &buf);
        }
        rem
    }

    #[inline]
    pub fn center_u(&self, u: usize) -> f64 {
        self.origin_u_mm + (u as f64 + 0.5) * self.du_mm
    }

    #[inline]
    pub fn center_v(&self, v: usize) -> f64 {
        self.origin_v_mm + (v as f64 + 0.5) * self.dv_mm
    }

    #[inline]
    pub fn index_span_u(&self, min_u: f64, max_u: f64) -> Option<(usize, usize)> {
        if max_u < self.origin_u_mm || min_u > self.origin_u_mm + (self.nu as f64) * self.du_mm {
            return None;
        }
        let u0 = (((min_u - self.origin_u_mm) / self.du_mm).floor() as isize).max(0) as usize;
        let u1 = (((max_u - self.origin_u_mm) / self.du_mm).ceil() as isize)
            .clamp(0, (self.nu - 1) as isize) as usize;
        (u0 <= u1).then_some((u0, u1))
    }

    #[inline]
    pub fn index_span_v(&self, min_v: f64, max_v: f64) -> Option<(usize, usize)> {
        if max_v < self.origin_v_mm || min_v > self.origin_v_mm + (self.nv as f64) * self.dv_mm {
            return None;
        }
        let v0 = (((min_v - self.origin_v_mm) / self.dv_mm).floor() as isize).max(0) as usize;
        let v1 = (((max_v - self.origin_v_mm) / self.dv_mm).ceil() as isize)
            .clamp(0, (self.nv - 1) as isize) as usize;
        (v0 <= v1).then_some((v0, v1))
    }

    /// Total solid volume (`mm³`) integrated across this orthogonal ray plane.
    pub fn volume_mm3(&self) -> f64 {
        let cell_area = self.du_mm * self.dv_mm;
        let total_len: f64 = self
            .counts
            .par_iter()
            .enumerate()
            .map(|(ray_idx, &cnt)| {
                let c = cnt as usize;
                if c == 0 {
                    0.0
                } else if c <= INLINE_DEXEL_SLOTS {
                    let base = ray_idx * INLINE_DEXEL_SLOTS;
                    let mut sum = 0.0;
                    for k in 0..c {
                        let s = self.slots[base + k];
                        sum += f64::from((s.max_local - s.min_local).max(0.0));
                    }
                    sum
                } else if let Some(ov) = self.overflow.get(&(ray_idx as u32)) {
                    ov.iter().map(|s| s.len_mm()).sum()
                } else {
                    0.0
                }
            })
            .sum();
        total_len * cell_area
    }
}

/// Helper used by Rayon parallel row loops to load ray `u` of a row into `scratch`.
#[inline]
pub(super) fn load_row_ray_into(
    u: usize,
    ray_idx: u32,
    row_slots: &[FlatDexelSpan],
    row_counts: &[u32],
    overflow: &BTreeMap<u32, Vec<DexelSpan>>,
    origin_w_mm: f64,
    scratch: &mut Vec<DexelSpan>,
) -> bool {
    scratch.clear();
    let cnt = row_counts[u] as usize;
    if cnt == 0 {
        return false;
    }
    if cnt <= INLINE_DEXEL_SLOTS {
        let base = u * INLINE_DEXEL_SLOTS;
        for k in 0..cnt {
            scratch.push(row_slots[base + k].to_wcs(origin_w_mm));
        }
        false
    } else {
        if let Some(ov) = overflow.get(&ray_idx) {
            scratch.extend_from_slice(ov);
        }
        true
    }
}

/// Helper used by Rayon parallel row loops to commit `spans` back into `row_slots`/`row_counts`
/// and record any rare overflow table delta (`None` = delete from overflow, `Some(vec)` = insert).
#[inline]
#[allow(clippy::too_many_arguments)]
pub(super) fn write_row_ray_from(
    u: usize,
    ray_idx: u32,
    row_slots: &mut [FlatDexelSpan],
    row_counts: &mut [u32],
    origin_w_mm: f64,
    spans: &[DexelSpan],
    was_overflow: bool,
    overflow_deltas: &mut Vec<(u32, Option<Vec<DexelSpan>>)>,
) {
    let base = u * INLINE_DEXEL_SLOTS;
    if spans.len() <= INLINE_DEXEL_SLOTS {
        let mut written = 0usize;
        for &s in spans {
            let flat = FlatDexelSpan::from_wcs(s, origin_w_mm);
            if flat.max_local > flat.min_local {
                row_slots[base + written] = flat;
                written += 1;
            }
        }
        for k in written..INLINE_DEXEL_SLOTS {
            row_slots[base + k] = FlatDexelSpan::ZERO;
        }
        row_counts[u] = written as u32;
        if was_overflow {
            overflow_deltas.push((ray_idx, None));
        }
    } else {
        for k in 0..INLINE_DEXEL_SLOTS {
            row_slots[base + k] = FlatDexelSpan::from_wcs(spans[k], origin_w_mm);
        }
        row_counts[u] = spans.len() as u32;
        overflow_deltas.push((ray_idx, Some(spans.to_vec())));
    }
}

/// Full-3D 3-axis orthogonal **Tri-Dexel** volumetric stock solid in Setup WCS coordinates.
///
/// Parameterized symmetrically across `planes: [DexelPlane; 3]` indexed by [`Axis3`]:
/// - `planes[Axis3::X.index()]`: `(Y, Z)` grid storing solid `X` spans
/// - `planes[Axis3::Y.index()]`: `(X, Z)` grid storing solid `Y` spans
/// - `planes[Axis3::Z.index()]`: `(X, Y)` grid storing solid `Z` spans
#[derive(Debug, Clone)]
pub struct TriDexelStock {
    pub dims: [usize; 3],
    pub origin_wcs: DVec3,
    pub pitch_mm: DVec3,
    pub planes: [DexelPlane; 3],
    /// Optional 4th-axis rotary coordinate frame when simulating a rotary Setup in volumetric
    /// 3D Tri-Dexel mode (`None` for standard 3-axis Setups).
    pub rotary: Option<RotaryStockSpec>,
}

impl TriDexelStock {
    /// Maximum single-axis voxel dimension for Tri-Dexel at each [`SimResolution`] tier.
    pub const fn max_axis_dim_for_resolution(resolution: SimResolution) -> usize {
        match resolution {
            SimResolution::Standard => 160,
            SimResolution::Fine => 288,
            SimResolution::Ultra => 448,
        }
    }

    #[inline]
    pub const fn nx(&self) -> usize {
        self.dims[0]
    }

    #[inline]
    pub const fn ny(&self) -> usize {
        self.dims[1]
    }

    #[inline]
    pub const fn nz(&self) -> usize {
        self.dims[2]
    }

    #[inline]
    pub const fn dx_mm(&self) -> f64 {
        self.pitch_mm.x
    }

    #[inline]
    pub const fn dy_mm(&self) -> f64 {
        self.pitch_mm.y
    }

    #[inline]
    pub const fn dz_mm(&self) -> f64 {
        self.pitch_mm.z
    }

    #[inline]
    pub const fn dim(&self, axis: Axis3) -> usize {
        self.dims[axis.index()]
    }

    #[inline]
    pub const fn pitch(&self, axis: Axis3) -> f64 {
        axis.get(self.pitch_mm)
    }

    #[inline]
    pub const fn plane(&self, axis: Axis3) -> &DexelPlane {
        &self.planes[axis.index()]
    }

    #[inline]
    pub fn plane_mut(&mut self, axis: Axis3) -> &mut DexelPlane {
        &mut self.planes[axis.index()]
    }

    #[inline]
    pub const fn x_plane(&self) -> &DexelPlane {
        &self.planes[0]
    }

    #[inline]
    pub const fn y_plane(&self) -> &DexelPlane {
        &self.planes[1]
    }

    #[inline]
    pub const fn z_plane(&self) -> &DexelPlane {
        &self.planes[2]
    }

    /// Allocates an empty Tri-Dexel lattice matching the exact bounds and resolution.
    pub fn empty_for_bounds(aabb_wcs: Aabb3, resolution: SimResolution) -> Result<Self, SimError> {
        if !aabb_wcs.is_valid() {
            return Err(SimError::InvalidStockBounds);
        }
        let size = aabb_wcs.size();
        if size.x <= 1e-4 || size.y <= 1e-4 || size.z <= 1e-4 {
            return Err(SimError::InvalidStockBounds);
        }

        let max_dim = Self::max_axis_dim_for_resolution(resolution);
        let longest = size.x.max(size.y).max(size.z);
        let target_pitch = longest / (max_dim as f64);

        let nx = ((size.x / target_pitch).round() as usize).clamp(4, max_dim);
        let ny = ((size.y / target_pitch).round() as usize).clamp(4, max_dim);
        let nz = ((size.z / target_pitch).round() as usize).clamp(4, max_dim);

        let dims = [nx, ny, nz];
        let pitch_mm = DVec3::new(size.x / nx as f64, size.y / ny as f64, size.z / nz as f64);
        let origin_wcs = aabb_wcs.min;

        let make_plane = |axis: Axis3| {
            let u_ax = axis.u_axis();
            let v_ax = axis.v_axis();
            DexelPlane::new(
                axis,
                u_ax.get_usize(dims),
                v_ax.get_usize(dims),
                u_ax.get(origin_wcs),
                v_ax.get(origin_wcs),
                axis.get(origin_wcs),
                u_ax.get(pitch_mm),
                v_ax.get(pitch_mm),
            )
        };

        Ok(Self {
            dims,
            origin_wcs,
            pitch_mm,
            planes: [
                make_plane(Axis3::X),
                make_plane(Axis3::Y),
                make_plane(Axis3::Z),
            ],
            rotary: None,
        })
    }

    /// Allocates an empty Tri-Dexel lattice with identical grid bounds and resolution to `template`.
    pub fn empty_like(template: &Self) -> Self {
        let make_plane = |axis: Axis3| {
            let p = template.plane(axis);
            DexelPlane::new(
                axis,
                p.nu,
                p.nv,
                p.origin_u_mm,
                p.origin_v_mm,
                p.origin_w_mm,
                p.du_mm,
                p.dv_mm,
            )
        };
        Self {
            dims: template.dims,
            origin_wcs: template.origin_wcs,
            pitch_mm: template.pitch_mm,
            planes: [
                make_plane(Axis3::X),
                make_plane(Axis3::Y),
                make_plane(Axis3::Z),
            ],
            rotary: None,
        }
    }

    /// Initializes raw stock (`Box` or `Cylinder`) symmetrically across all 3 orthogonal dexel planes.
    pub fn from_stock_spec(
        stock: &StockShapeSpec,
        resolution: SimResolution,
    ) -> Result<Self, SimError> {
        let aabb = stock.aabb_wcs();
        let mut grid = Self::empty_for_bounds(aabb, resolution)?;

        match *stock {
            StockShapeSpec::Box { aabb_wcs } => {
                for axis in Axis3::ALL {
                    let span = DexelSpan::new(
                        axis.get(aabb_wcs.min),
                        axis.get(aabb_wcs.max),
                        UNCUT_OP_IDX,
                        UNCUT_OP_IDX,
                    );
                    grid.plane_mut(axis).fill_all_rays(span);
                }
            }
            StockShapeSpec::Cylinder {
                axis: cyl_axis,
                center_wcs,
                radius_mm,
                min_axial_mm,
                max_axial_mm,
                ..
            } => {
                let r2 = radius_mm * radius_mm;
                for plane_axis in Axis3::ALL {
                    let u_ax = plane_axis.u_axis();
                    let v_ax = plane_axis.v_axis();
                    let plane = grid.plane_mut(plane_axis);
                    let nu = plane.nu;
                    let nv = plane.nv;
                    for iv in 0..nv {
                        let v = plane.center_v(iv);
                        for iu in 0..nu {
                            let u = plane.center_u(iu);
                            let span_opt = if plane_axis == cyl_axis {
                                let du = u - u_ax.get(center_wcs);
                                let dv = v - v_ax.get(center_wcs);
                                (du * du + dv * dv <= r2).then_some((min_axial_mm, max_axial_mm))
                            } else {
                                let (axial_c, perp_c, perp_center) = if u_ax == cyl_axis {
                                    (u, v, v_ax.get(center_wcs))
                                } else {
                                    (v, u, u_ax.get(center_wcs))
                                };
                                let dp = perp_c - perp_center;
                                if axial_c >= min_axial_mm
                                    && axial_c <= max_axial_mm
                                    && dp * dp <= r2
                                {
                                    let dw = (r2 - dp * dp).max(0.0).sqrt();
                                    let cw = plane_axis.get(center_wcs);
                                    Some((cw - dw, cw + dw))
                                } else {
                                    None
                                }
                            };
                            if let Some((a, b)) = span_opt
                                && b - a > MIN_DEXEL_SPAN_MM
                            {
                                plane.set_ray_spans(
                                    iu,
                                    iv,
                                    &[DexelSpan::new(a, b, UNCUT_OP_IDX, UNCUT_OP_IDX)],
                                );
                            }
                        }
                    }
                }
            }
        }

        Ok(grid)
    }

    /// Voxelizes target 3D solid geometry (pure WCS triangles + optional analytic horizontal
    /// `±Z` faces) onto a Tri-Dexel lattice matching `template` (zero `chip-cad` dependency).
    pub fn from_target_geometry(
        tris_wcs_all: &[[DVec3; 3]],
        tris_wcs_z: &[[DVec3; 3]],
        horiz_z_faces: &[AnalyticHorizFace],
        template: &Self,
    ) -> Self {
        let mut target = Self::empty_like(template);
        if tris_wcs_all.is_empty() && horiz_z_faces.is_empty() {
            return target;
        }
        for axis in Axis3::ALL {
            match axis {
                Axis3::Z => rasterize_triangles_onto_plane(
                    target.plane_mut(Axis3::Z),
                    tris_wcs_z,
                    horiz_z_faces,
                ),
                Axis3::X | Axis3::Y => {
                    rasterize_triangles_onto_plane(target.plane_mut(axis), tris_wcs_all, &[]);
                }
            }
        }
        target
    }

    /// Stock bounding box in Setup WCS coordinates.
    #[inline]
    pub fn aabb_wcs(&self) -> Aabb3 {
        Aabb3::new(
            self.origin_wcs,
            self.origin_wcs
                + DVec3::new(
                    self.dims[0] as f64 * self.pitch_mm.x,
                    self.dims[1] as f64 * self.pitch_mm.y,
                    self.dims[2] as f64 * self.pitch_mm.z,
                ),
        )
    }

    /// Mean cell pitch (`mm`) across the three axes.
    #[inline]
    pub fn mean_pitch_mm(&self) -> f64 {
        (self.pitch_mm.x + self.pitch_mm.y + self.pitch_mm.z) / 3.0
    }

    /// Per-axis solid volumes `(vol_x, vol_y, vol_z)` in `mm³`.
    pub fn per_axis_volumes_mm3(&self) -> (f64, f64, f64) {
        (
            self.planes[0].volume_mm3(),
            self.planes[1].volume_mm3(),
            self.planes[2].volume_mm3(),
        )
    }

    /// Tri-Dexel averaged solid volume (`(vol_x + vol_y + vol_z) / 3.0`) in `mm³`.
    pub fn compute_volume_mm3(&self) -> f64 {
        let (vx, vy, vz) = self.per_axis_volumes_mm3();
        (vx + vy + vz) / 3.0
    }

    /// Samples the highest solid `Z` coordinate at WCS `(x, y)` (`None` if empty at `(x, y)`).
    pub fn sample_top_z(&self, x_wcs: f64, y_wcs: f64) -> Option<f64> {
        if x_wcs < self.origin_wcs.x || y_wcs < self.origin_wcs.y {
            return None;
        }
        let i = ((x_wcs - self.origin_wcs.x) / self.pitch_mm.x).floor() as isize;
        let j = ((y_wcs - self.origin_wcs.y) / self.pitch_mm.y).floor() as isize;
        if i < 0 || j < 0 || (i as usize) >= self.dims[0] || (j as usize) >= self.dims[1] {
            return None;
        }
        self.z_plane()
            .ray_spans(i as usize, j as usize)
            .last()
            .map(|s| s.max_mm)
    }

    /// Computes the maximum solid Z-ray overlap length (`mm`) inside `[z_lo, z_hi]` across all
    /// Z-rays within `radius_mm` of `(cx, cy)` (always including the exact center cell).
    pub fn query_vertical_plunge_overlap(
        &self,
        cx: f64,
        cy: f64,
        radius_mm: f64,
        z_lo: f64,
        z_hi: f64,
    ) -> f64 {
        if z_hi <= z_lo + 1e-9 {
            return 0.0;
        }
        let z_plane = self.z_plane();
        let mut max_ov = 0.0_f64;
        let r = radius_mm.max(0.0);
        if let (Some((i0, i1)), Some((j0, j1))) = (
            z_plane.index_span_u(cx - r, cx + r),
            z_plane.index_span_v(cy - r, cy + r),
        ) {
            let r2 = r * r;
            for j in j0..=j1 {
                let py = z_plane.center_v(j);
                let dy = py - cy;
                for i in i0..=i1 {
                    let px = z_plane.center_u(i);
                    let dx = px - cx;
                    if dx * dx + dy * dy <= r2 + 1e-9 {
                        let spans = z_plane.ray_spans(i, j);
                        let ov = query_overlap_length(&spans, z_lo, z_hi);
                        if ov > max_ov {
                            max_ov = ov;
                        }
                    }
                }
            }
        }
        // Always include the exact center cell even when `radius_mm < dx_mm`
        if cx >= self.origin_wcs.x && cy >= self.origin_wcs.y {
            let ci = ((cx - self.origin_wcs.x) / self.pitch_mm.x).floor() as isize;
            let cj = ((cy - self.origin_wcs.y) / self.pitch_mm.y).floor() as isize;
            if ci >= 0 && cj >= 0 && (ci as usize) < self.dims[0] && (cj as usize) < self.dims[1] {
                let spans = z_plane.ray_spans(ci as usize, cj as usize);
                let ov = query_overlap_length(&spans, z_lo, z_hi);
                if ov > max_ov {
                    max_ov = ov;
                }
            }
        }
        max_ov
    }

    /// Returns `true` if voxel center `(i, j, k)` is solid by majority vote across the 3 orthogonal
    /// dexel planes (eliminates single-axis grazing artifacts and guarantees watertight meshing).
    #[inline]
    pub fn is_voxel_solid(&self, i: usize, j: usize, k: usize) -> bool {
        let ijk = [i, j, k];
        let center = DVec3::new(
            self.origin_wcs.x + (i as f64 + 0.5) * self.pitch_mm.x,
            self.origin_wcs.y + (j as f64 + 0.5) * self.pitch_mm.y,
            self.origin_wcs.z + (k as f64 + 0.5) * self.pitch_mm.z,
        );
        let mut votes = 0u8;
        for axis in Axis3::ALL {
            let iu = axis.u_axis().get_usize(ijk);
            let iv = axis.v_axis().get_usize(ijk);
            let coord = axis.get(center);
            if spans_contain_coord(&self.plane(axis).ray_spans(iu, iv), coord) {
                votes += 1;
            }
        }
        votes >= 2
    }
}

/// Rasterizes a closed (or mostly closed) triangle soup onto a single orthogonal [`DexelPlane`]
/// using deterministic sub-micron ray-center jitter so rays never hit shared triangle edges/vertices.
fn rasterize_triangles_onto_plane(
    plane: &mut DexelPlane,
    tris_wcs: &[[DVec3; 3]],
    horiz_z_faces: &[AnalyticHorizFace],
) {
    let axis = plane.axis;
    let u_ax = axis.u_axis();
    let v_ax = axis.v_axis();

    // Tiny deterministic irrational jitter prevents exact edge/vertex ties on axis-aligned boxes
    const JITTER_U: f64 = 1.371_91e-7;
    const JITTER_V: f64 = 2.718_28e-7;

    let nu = plane.nu;
    let nv = plane.nv;
    let mut hits: Vec<Vec<(f64, i8)>> = vec![Vec::new(); nu * nv];

    // Exact analytic horizontal B-Rep face hits on Z-plane (immune to multi-hole ear-clipping)
    if axis == Axis3::Z && !horiz_z_faces.is_empty() {
        let ox = plane.origin_u_mm;
        let oy = plane.origin_v_mm;
        let dx = plane.du_mm;
        let dy = plane.dv_mm;
        hits.par_chunks_mut(nu)
            .enumerate()
            .for_each(|(iv, row_hits)| {
                let rv = oy + (iv as f64 + 0.5) * dy + JITTER_V;
                for hf in horiz_z_faces {
                    if rv < hf.aabb.min.y - 1e-6 || rv > hf.aabb.max.y + 1e-6 {
                        continue;
                    }
                    let iu0 = (((hf.aabb.min.x - ox) / dx).floor() as isize).max(0) as usize;
                    let iu1 = (((hf.aabb.max.x - ox) / dx).ceil() as isize)
                        .clamp(0, (nu - 1) as isize) as usize;
                    for (off, cell_hits) in row_hits[iu0..=iu1].iter_mut().enumerate() {
                        let iu = iu0 + off;
                        let ru = ox + (iu as f64 + 0.5) * dx + JITTER_U;
                        let pt = DVec2::new(ru, rv);
                        if hf.outer.contains_point(pt)
                            && !hf.inners.iter().any(|inn| inn.contains_point(pt))
                        {
                            cell_hits.push((hf.z_wcs, hf.dir_sign));
                        }
                    }
                }
            });
    }

    for &[p0, p1, p2] in tris_wcs {
        let u0 = u_ax.get(p0);
        let v0 = v_ax.get(p0);
        let w0 = axis.get(p0);
        let u1 = u_ax.get(p1);
        let v1 = v_ax.get(p1);
        let w1 = axis.get(p1);
        let u2 = u_ax.get(p2);
        let v2 = v_ax.get(p2);
        let w2 = axis.get(p2);

        let area2 = (u1 - u0) * (v2 - v0) - (u2 - u0) * (v1 - v0);
        if area2.abs() <= 1e-13 {
            continue;
        }
        let inv_area2 = 1.0 / area2;

        let n = (p1 - p0).cross(p2 - p0);
        let nw = axis.get(n);
        let dir_sign: i8 = if nw < 0.0 { 1 } else { -1 };

        let min_u = u0.min(u1).min(u2);
        let max_u = u0.max(u1).max(u2);
        let min_v = v0.min(v1).min(v2);
        let max_v = v0.max(v1).max(v2);

        let (Some((iu0, iu1)), Some((iv0, iv1))) = (
            plane.index_span_u(min_u, max_u),
            plane.index_span_v(min_v, max_v),
        ) else {
            continue;
        };

        for iv in iv0..=iv1 {
            let rv = plane.center_v(iv) + JITTER_V;
            let row = iv * nu;
            for iu in iu0..=iu1 {
                let ru = plane.center_u(iu) + JITTER_U;
                let a = ((u1 - ru) * (v2 - rv) - (u2 - ru) * (v1 - rv)) * inv_area2;
                let b = ((u2 - ru) * (v0 - rv) - (u0 - ru) * (v2 - rv)) * inv_area2;
                let c = 1.0 - a - b;
                if a >= 0.0 && b >= 0.0 && c >= 0.0 {
                    let w = a * w0 + b * w1 + c * w2;
                    if w.is_finite() {
                        hits[row + iu].push((w, dir_sign));
                    }
                }
            }
        }
    }

    // Parallel commit into flat fixed-K span pool + rare overflow side-table
    plane.overflow.clear();
    let origin_w = plane.origin_w_mm;
    let row_slot_stride = nu * INLINE_DEXEL_SLOTS;

    let overflow_entries: Vec<(u32, Vec<DexelSpan>)> = plane
        .slots
        .par_chunks_mut(row_slot_stride)
        .zip(plane.counts.par_chunks_mut(nu))
        .zip(hits.par_chunks_mut(nu))
        .enumerate()
        .map(|(iv, ((row_slots, row_counts), row_hits))| {
            let mut row_ov: Vec<(u32, Vec<DexelSpan>)> = Vec::new();
            let mut clean_w: Vec<f64> = Vec::new();
            let mut ray_spans: Vec<DexelSpan> = Vec::new();
            let row_base = iv * nu;

            for iu in 0..nu {
                let cell_hits = &mut row_hits[iu];
                if cell_hits.len() < 2 {
                    row_counts[iu] = 0;
                    continue;
                }
                cell_hits.sort_by(|a, b| a.0.total_cmp(&b.0));

                clean_w.clear();
                for &(w, _) in cell_hits.iter() {
                    if clean_w.last().is_none_or(|&prev| (w - prev).abs() > 1e-8) {
                        clean_w.push(w);
                    }
                }

                ray_spans.clear();
                let pairs = clean_w.len() / 2;
                for p in 0..pairs {
                    let a = clean_w[2 * p];
                    let b = clean_w[2 * p + 1];
                    if b - a > MIN_DEXEL_SPAN_MM {
                        ray_spans.push(DexelSpan::new(a, b, UNCUT_OP_IDX, UNCUT_OP_IDX));
                    }
                }

                let base = iu * INLINE_DEXEL_SLOTS;
                if ray_spans.len() <= INLINE_DEXEL_SLOTS {
                    let mut written = 0usize;
                    for &s in &ray_spans {
                        let flat = FlatDexelSpan::from_wcs(s, origin_w);
                        if flat.max_local > flat.min_local {
                            row_slots[base + written] = flat;
                            written += 1;
                        }
                    }
                    for k in written..INLINE_DEXEL_SLOTS {
                        row_slots[base + k] = FlatDexelSpan::ZERO;
                    }
                    row_counts[iu] = written as u32;
                } else {
                    for k in 0..INLINE_DEXEL_SLOTS {
                        row_slots[base + k] = FlatDexelSpan::from_wcs(ray_spans[k], origin_w);
                    }
                    row_counts[iu] = ray_spans.len() as u32;
                    row_ov.push(((row_base + iu) as u32, ray_spans.clone()));
                }
            }
            row_ov
        })
        .reduce(Vec::new, |mut a, mut b| {
            a.append(&mut b);
            a
        });

    for (ray_idx, vec) in overflow_entries {
        plane.overflow.insert(ray_idx, vec);
    }
}
