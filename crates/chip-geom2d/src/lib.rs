#![deny(unsafe_code)]
//! UI-agnostic 2D exact arc-polyline geometry kernel: parallel offsetting, planar boolean region
//! sets, segment-Voronoi medial axis extraction, and Ramer–Douglas–Peucker polyline simplification.

pub mod medial;
pub mod offset;
pub mod region;
pub mod simplify;

pub use medial::{
    MedialChain2D, MedialNode2D, extract_interior_medial_nodes, extract_medial_axis_chains,
    find_medial_inscribed_pole,
};
pub use offset::{
    HelicalEntryPole, OffsetRegionNode, PocketRegion2D, build_offset_region_tree,
    clip_scanline_to_region, find_helical_entry_pole, from_cavalier_polyline,
    is_simple_closed_polyline, offset_polyline, to_cavalier_polyline,
    try_union_overlapping_ccw_loops,
};
pub use region::{
    RegionSet2D, clip_polyline_by_region_set, clip_polyline_outside_region_set,
    contours_to_region_set, swept_segment_capsule_2d,
};
pub use simplify::{simplify_3d_polyline_rdp, simplify_rdp_2d};
