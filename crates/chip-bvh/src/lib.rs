//! UI-agnostic flat-array binned-SAH BVH and canonical `f64` triangle/segment spatial indexes
//! (`chip-bvh`).
//!
//! Provides GPU-uploadable 32-byte [`FlatBvhNode`] and 48-byte [`GpuTriangle`] layouts alongside
//! exact `f64` `O(log N)` CPU queries:
//! - Front-to-back raycasting (`raycast`, `raycast_all_hits`)
//! - Exact point-to-triangle distance (`closest_point`)
//! - Drop-Cutter vertical cylinder queries (`query_xy_circle_into`)
//! - Push-Cutter / Waterline Z-slab queries (`query_z_slab_into`)
//! - 3D line-segment picking (`SegmentBvh`)

#![deny(unsafe_code)]

pub mod bvh;
pub mod mesh;
pub mod segment;

pub use bvh::{
    ClosestPointHit, FlatBvh, FlatBvhNode, GpuTriangle, RayHit, clip_triangle_to_z_slab,
    closest_point_on_segment, closest_point_on_triangle, closest_point_on_triangle_at_z,
    ray_triangle_intersect,
};
pub use mesh::{Tri3D, TriMesh3D};
pub use segment::{BvhSegment, SegmentBvh, SegmentClosestHit, SegmentPickHit};
