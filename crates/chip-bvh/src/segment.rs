//! Flat binary BVH over 3D line segments (`SegmentBvh`) for `O(log N)` edge and sketch picking.

use chip_math::{Aabb3, DVec3, Ray3, SegmentHit};

use crate::bvh::{FlatBvh, FlatBvhNode, aabb_overlaps_f64, closest_point_on_segment};

/// Canonical `f64` 3D line segment tagged with a caller-defined `user_id` (e.g., `EdgeId.0`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BvhSegment {
    pub a: DVec3,
    pub b: DVec3,
    pub user_id: u32,
}

/// Closest ray-to-segment pick hit returned by [`SegmentBvh::pick_ray`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SegmentPickHit {
    pub seg_index: usize,
    pub user_id: u32,
    pub score: f64,
    pub hit: SegmentHit,
}

/// Exact `f64` closest point on a segment returned by [`SegmentBvh::closest_point`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SegmentClosestHit {
    pub seg_index: usize,
    pub user_id: u32,
    pub point: DVec3,
    pub seg_u: f64,
    pub distance: f64,
}

/// Flat-array SAH BVH over 3D line segments for sub-millisecond viewport edge/wire picking.
#[derive(Debug, Clone, Default)]
pub struct SegmentBvh {
    segments: Vec<BvhSegment>,
    aabbs: Vec<Aabb3>,
    bvh: FlatBvh,
}

impl SegmentBvh {
    /// Builds a flat binary BVH from `segments`. Non-finite segments are filtered out.
    pub fn build(segments: Vec<BvhSegment>) -> Self {
        let mut valid_segs = Vec::with_capacity(segments.len());
        let mut aabbs = Vec::with_capacity(segments.len());
        for seg in segments {
            if !seg.a.is_finite() || !seg.b.is_finite() {
                continue;
            }
            let aabb = Aabb3::new(seg.a.min(seg.b), seg.a.max(seg.b));
            valid_segs.push(seg);
            aabbs.push(aabb);
        }
        let bvh = FlatBvh::build(&aabbs);
        Self {
            segments: valid_segs,
            aabbs,
            bvh,
        }
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.segments.is_empty()
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.segments.len()
    }

    #[inline]
    pub fn bounds(&self) -> Aabb3 {
        self.bvh.bounds()
    }

    #[inline]
    pub fn nodes(&self) -> &[FlatBvhNode] {
        self.bvh.nodes()
    }

    #[inline]
    pub fn segments(&self) -> &[BvhSegment] {
        &self.segments
    }

    /// Finds the highest-priority segment within perpendicular distance `radius_mm` of `ray`
    /// and ray depth `<= max_ray_t` (for occlusion against solid faces).
    ///
    /// Uses the canonical picking score `hit.distance_mm + hit.ray_t * 1e-3` (preferring closer
    /// screen-space hits and shallower ray depths) and dynamically tightens the search cylinder
    /// as better hits are discovered.
    pub fn pick_ray(&self, ray: &Ray3, radius_mm: f64, max_ray_t: f64) -> Option<SegmentPickHit> {
        if self.segments.is_empty()
            || !radius_mm.is_finite()
            || radius_mm < 0.0
            || max_ray_t.is_nan()
            || max_ray_t < 0.0
        {
            return None;
        }

        let mut best: Option<SegmentPickHit> = None;

        self.bvh
            .traverse_ray_inflated(ray.origin, ray.dir, radius_mm, max_ray_t, |seg_idx| {
                let seg = self.segments.get(seg_idx)?;
                let hit = ray.closest_to_segment(seg.a, seg.b);
                if hit.distance_mm <= radius_mm && hit.ray_t <= max_ray_t {
                    let score = hit.distance_mm + hit.ray_t * 1e-3;
                    if best.is_none_or(|b| score < b.score) {
                        best = Some(SegmentPickHit {
                            seg_index: seg_idx,
                            user_id: seg.user_id,
                            score,
                            hit,
                        });
                        // Since `hit.ray_t >= 0`, any future winning segment must have
                        // `distance_mm <= score`, so `score` is a strictly conservative radius bound.
                        return Some(score);
                    }
                }
                None
            });

        best
    }

    /// Finds the exact `f64` closest point across all indexed 3D segments to `point`.
    pub fn closest_point(&self, point: DVec3) -> Option<SegmentClosestHit> {
        self.bvh
            .traverse_closest_point(point, |seg_idx, _best_d2| {
                let seg = self.segments.get(seg_idx)?;
                let (q, d2, seg_u) = closest_point_on_segment(point, seg.a, seg.b);
                Some((
                    d2,
                    SegmentClosestHit {
                        seg_index: seg_idx,
                        user_id: seg.user_id,
                        point: q,
                        seg_u,
                        distance: d2.sqrt(),
                    },
                ))
            })
            .map(|(_, hit)| hit)
    }

    /// Appends all segment indices whose exact `f64` AABB overlaps `aabb` into `out`.
    pub fn query_aabb_into(&self, aabb: &Aabb3, out: &mut Vec<usize>) {
        out.clear();
        let mut candidates = Vec::new();
        self.bvh.query_aabb_into(aabb, &mut candidates);
        for idx in candidates {
            if let Some(seg_aabb) = self.aabbs.get(idx)
                && aabb_overlaps_f64(seg_aabb, aabb)
            {
                out.push(idx);
            }
        }
    }
}
